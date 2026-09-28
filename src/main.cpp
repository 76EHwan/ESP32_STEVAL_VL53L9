// ---------------------------------------------------------------------------
// VL53L9CX 측거 — ST 드라이버(BSD-3-Clause) ESP32 포팅
//
// 데이터 경로는 MIPI CSI-2 가 아니라 I2C 다.
//   vl53l9_hw_config_t.output_interface 를 true(I3C/시리얼)로 두면 프레임이
//   레지스터 공간으로 나오고 0x1800 창에서 읽힌다. ESP32 에 CSI 수신기가
//   없어도 측거 데이터를 받을 수 있는 이유다.
//
// 프레임 버퍼 레이아웃 (vl53l9_get_frame 기준):
//   [0          .. res*2)    depth      zone 당 uint16, 리틀엔디안
//   [res*2      .. res*4)    amplitude
//   [res*4      .. res*6)    ambient
//   [res*6      .. +res/2)   DSS LUT 인덱스
//   [...        .. +100)     status line
// ---------------------------------------------------------------------------

#include <Arduino.h>

#include "board_config.h"
#include "i2c_diag.h"
#include "vl53l9_esp32.h"

extern "C" {
#include "vl53l9.h"
#include "vl53l9_platform.h"   // vl53l9_read / vl53l9_write (청크 검증용)
}

// 1 = 같은 STEVAL-VL53L9 에서 동작이 확인된 ESP32-P4 구현
//     (kamibukuro5656/VL53L9CX_ESP32-P4_USB_ROS2) 의 기본 설정을 그대로 재현한다.
//     이 설정으로도 폴트가 나면 호스트 코드 차이는 배제된다.
#define REF_REPLICA       0

#if REF_REPLICA
#define PROF_SYNC         VL53L9_SYNC_AUTONOMOUS
#define PROF_POWER        VL53L9_POWER_REGULAR
#define PROF_CONTEXT      VL53L9_CONTEXT_SHORT
#define PROF_FRAME_PERIOD 10000UL              // 10 ms
#define PROF_BINNING      2                    // 54x42
#define PROF_EXPOSURE_MS  4
#define PROF_DSS_OFF      1
#else
// ST 의 AR_PRECISION 프로파일 값. sync 만 예제와 같이 MANUAL 로 덮어쓴다.
#define PROF_SYNC         VL53L9_SYNC_MANUAL
#define PROF_POWER        VL53L9_POWER_REGULAR   // ULTRA_LOW 는 I3C 웨이크 전제로 보임
#define PROF_CONTEXT      VL53L9_CONTEXT_SHORT
#define PROF_FRAME_PERIOD (1000000UL / 30UL)   // 30 fps
#define PROF_BINNING      12                   // 8x8 (64존). binning 2 는 start 가 60ms 를 넘긴다
#define PROF_EXPOSURE_MS  10
#define PROF_DSS_OFF      0
#endif

// 1 = 드라이버를 건드리기 전에 I2C 버스부터 진단한다.
// 기본 0: 진단은 주소만 보내는 빈 트랜잭션(프로브)을 수십 번 보내는데, VL53L9CX 는
// 이를 지원하지 않고 이후 전송을 NACK 할 수 있다. 배선을 의심할 때만 켠다.
#define RUN_I2C_DIAG      0

// 1 = init 전에 0x1800(패치 영역)에 4KB 를 써서 청크 분할 전송을 검증한다.
// 플랫폼 계층을 바꿨을 때만 켠다. 참고 구현에는 없는 쓰기라 평소에는 끈다.
#define RUN_CHUNK_TEST    0

// 1 = CSI2 로 출력만 돌려놓고 측거 파이프라인만 시험한다 (프레임 수신은 불가).
//
// I3C 모드는 파이프라인이 시작되기 전에 0x0F00 으로 죽어 frame_counter 가
// 0 에 머문다. 그래서 ref amplitude 가 기록되지 않아 VCSEL 발광 여부를
// 판정할 수 없다. CSI2 모드만 frame_counter 가 올라가며 ref amp 를 채운다.
#if REF_REPLICA
#define USE_CSI_BISECT    1
#else
#define USE_CSI_BISECT    0
#endif

// 1 = 전류 측정 모드. 대기/측거를 10초씩 교대해 소비 전류 차이를 재게 한다.
//
// 실측 결과 판별력이 없었다. 폴트 직후 STANDBY 로 떨어져 STREAMING 체류
// 시간이 짧고, 펌웨어가 9단계에서 중단되어 정상 부품이라도 발광 전류가
// 거의 안 나온다. 두 구간 차이가 멀티미터 분해능 아래다.
#define POWER_TEST_MODE   0

#define FRAME_BUF_MAX     15000                // binning 2 기준 14842B

static vl53l9_esp32_dev_t g_dev;
static uint8_t            g_frame[FRAME_BUF_MAX];
static uint16_t           g_frame_size = 0;
static uint16_t           g_w = 0, g_h = 0;

static void dumpStatus(const char *when);

static bool binningToWH(uint8_t b, uint16_t *w, uint16_t *h) {
  switch (b) {
    case 2:  *w = 54; *h = 42; return true;
    case 4:  *w = 24; *h = 24; return true;
    case 6:  *w = 18; *h = 14; return true;
    case 8:  *w = 12; *h = 10; return true;
    case 12: *w =  8; *h =  8; return true;
    case 24: *w =  4; *h =  4; return true;
    default: return false;
  }
}

static void banner(const char *title) {
  Serial.println();
  Serial.println(F("==================================================="));
  Serial.print(F("  "));
  Serial.println(title);
  Serial.println(F("==================================================="));
}

static const char *errText(int e) {
  switch (e) {
    case VL53L9_ERROR_NONE:              return "성공";
    case VL53L9_ERROR_PLATFORM:          return "플랫폼(I2C) 오류";
    case VL53L9_ERROR_INVALID_PARAM:     return "잘못된 인자";
    case VL53L9_ERROR_INVALID_STATE:     return "잘못된 상태";
    case VL53L9_ERROR_INVALID_OPERATION: return "잘못된 동작";
    case VL53L9_ERROR_TIMEOUT:           return "타임아웃";
    case VL53L9_ERROR_INTERNAL:          return "내부 오류";
    default:                             return "알 수 없음";
  }
}

#define STEP(call, what)                                                      \
  do {                                                                        \
    const int _e = (call);                                                    \
    Serial.printf("  %-36s %s\n", what, errText(_e));                         \
    if (_e != VL53L9_ERROR_NONE) { Serial.println(F("  중단.")); return; }    \
  } while (0)

static void enableSensor() {
  if (PIN_XSHUT < 0) return;
  pinMode(PIN_XSHUT, OUTPUT);
  digitalWrite(PIN_XSHUT, LOW);
  delay(10);
  digitalWrite(PIN_XSHUT, HIGH);
  delay(200);
}

static void dumpStatus(const char *when) {
  vl53l9_status_t st;
  const int e = vl53l9_get_status(&g_dev, &st);
  if (e != VL53L9_ERROR_NONE) {
    Serial.printf("  [%s] get_status 실패: %s\n", when, errText(e));
    return;
  }
  const char *fsmName = (st.fsm == 0) ? "NONE"
                      : (st.fsm == 1) ? "READY_TO_BOOT"
                      : (st.fsm == 2) ? "STANDBY"
                      : (st.fsm == 3) ? "STREAMING" : "?";
  Serial.printf("  [%s] fsm=0x%02X(%s) command_err=0x%02X firmware=0x%04X\n",
                when, st.fsm, fsmName, st.command, st.firmware);
  Serial.printf("    error: vhv_ov=%u vhv_uv=%u spad_ovl=%u hvboost=%u "
                "sof_blank=%u pll_lock=%u ref_array=%u internal_fw=%u\n",
                st.error.vhv_overvoltage, st.error.vhv_undervoltage,
                st.error.spad_supply_overload, st.error.hvboost_limit,
                st.error.sof_outside_blanking, st.error.pll_lock,
                st.error.ref_array, st.error.internal_fw);
  Serial.printf("    laser_driver[0..4] = %02X %02X %02X %02X %02X\n",
                st.laser_driver[0], st.laser_driver[1], st.laser_driver[2],
                st.laser_driver[3], st.laser_driver[4]);

  uint8_t sl[100];
  if (vl53l9_read(&g_dev, 0x0028, sl, sizeof(sl)) != VL53L9_ERROR_NONE) return;
  #define U16(off) ((uint16_t)(sl[(off)] | ((uint16_t)sl[(off) + 1] << 8)))
  #define U32(off) ((uint32_t)(sl[(off)] | ((uint32_t)sl[(off)+1] << 8) | \
                               ((uint32_t)sl[(off)+2] << 16) | ((uint32_t)sl[(off)+3] << 24)))
  Serial.printf("    frame_counter=%lu  temperature=%u  ldd_temp=%u\n",
                (unsigned long)U32(0), U16(4), U16(6));
  Serial.printf("    ref LONG   ch1 amp=%u dist=%u | ch2 amp=%u dist=%u\n",
                U16(36), U16(38), U16(40), U16(42));
  Serial.printf("    ref SHORT  ch1 amp=%u dist=%u | ch2 amp=%u dist=%u\n",
                U16(44), U16(46), U16(48), U16(50));
  Serial.printf("    frame %ux%u  error_code=0x%04X error_status=0x%02X\n",
                U16(52), U16(54), U16(60), sl[62]);
  #undef U16
  #undef U32
}

#if RUN_I2C_DIAG
static void hardwareChecklist(void) {
  banner("!! I2C 응답 없음 — 하드웨어 점검 순서");
  Serial.println(F(
    "  소프트웨어로 더 확인할 것이 없다. 센서가 주소 자체에 ACK 하지 않는다.\n\n"
    "  1) R25 / R24  (가장 유력)\n"
    "     R25 제거 + R24 장착 상태여야 Y1(12MHz) 이 AP_CLK 로 들어간다.\n\n"
    "  2) J3 점퍼 (HOST_IOVDD 3V3 핀 확인)\n\n"
    "  3) 전원 레일 (AVDD 2.8V / IOVDD 1.8V / DVDD 1.2V)\n\n"
    "  4) XSHUT 배선 및 로직 레벨\n\n"
    "  5) 센서 사망 의심 (이전 U4 리워크 이력 참고)"));
}
#endif

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(500);

  banner("VL53L9CX 측거 (ST 드라이버 ESP32 포팅)");
  Serial.println(F("  보드: ESP32 DevKit v1"));
  Serial.printf("  I2C %lu Hz, SDA=GPIO%d, SCL=GPIO%d, XSHUT=GPIO%d\n",
                (unsigned long)I2C_FREQ_HZ, PIN_SDA, PIN_SCL, PIN_XSHUT);

  g_dev.address = TOF_I2C_ADDR_7BIT;

#if RUN_I2C_DIAG
  if (!i2c_diag_run(TOF_I2C_ADDR_7BIT)) {
    hardwareChecklist();
    g_frame_size = 0;
    return;
  }
  Serial.printf("\n  0x%02X ACK 확인. 드라이버 경로로 진행한다.\n", TOF_I2C_ADDR_7BIT);
#endif

  enableSensor();
  if (!vl53l9_esp32_bus_begin()) {
    Serial.println(F("  !! Wire 초기화 실패. 중단."));
    g_frame_size = 0;
    return;
  }

#if RUN_CHUNK_TEST
  banner("[사전] 청크 쓰기 검증");
  {
    const uint32_t N = 4096; 
    static uint8_t tx[4096], rx[4096];
    for (uint32_t i = 0; i < N; i++) tx[i] = (uint8_t)(i * 7u + 3u);

    int we = vl53l9_write(&g_dev, 0x1800, tx, N);
    int re = vl53l9_read(&g_dev, 0x1800, rx, N);
    Serial.printf("  write %lu B -> %d,  read back -> %d\n", (unsigned long)N, we, re);

    uint32_t bad = 0; int32_t first = -1;
    for (uint32_t i = 0; i < N; i++) {
      if (tx[i] != rx[i]) { bad++; if (first < 0) first = (int32_t)i; }
    }
    if (bad == 0) {
      Serial.println(F("  일치. 청크 분할 쓰기 정상."));
    } else {
      Serial.printf("  !! 불일치 %lu / %lu, 첫 위치 오프셋 %ld\n", (unsigned long)bad, (unsigned long)N, (long)first);
    }
  }
#endif

  banner("초기화");

  STEP(vl53l9_init(&g_dev), "vl53l9_init (패치 설치 포함)");

  uint32_t id = 0;
  STEP(vl53l9_get_device_id(&g_dev, &id), "vl53l9_get_device_id");
  Serial.printf("    device id = 0x%08lX\n", (unsigned long)id);

  banner("[진단] 캘리브레이션 + PLL(고속 클럭 전환)");
  {
    static uint8_t calib[VL53L9_CALIB_DATA_SIZE];
    const int ce = vl53l9_get_calib_data(&g_dev, calib);
    Serial.printf("  vl53l9_get_calib_data -> %s\n", errText(ce));
    if (ce == VL53L9_ERROR_NONE) {
      uint32_t nz = 0, ff = 0;
      uint32_t fnv = 2166136261UL;   // FNV-1a. 다이별 OTP 라 소자 지문으로 쓴다
      for (uint32_t i = 0; i < VL53L9_CALIB_DATA_SIZE; i++) {
        if (calib[i]) nz++;
        if (calib[i] == 0xFF) ff++;
        fnv = (fnv ^ calib[i]) * 16777619UL;
      }
      Serial.printf("  %u 바이트 중 non-zero %lu, 0xFF %lu\n",
                    VL53L9_CALIB_DATA_SIZE, (unsigned long)nz, (unsigned long)ff);
      Serial.printf("  calib 지문(FNV-1a) = %08lX\n", (unsigned long)fnv);
    }
  }

  banner("프로파일 적용 (ST AR_PRECISION)");

  Serial.printf("  sync=%s  period=%lu us  binning=%d  exposure=%d ms  DSS=%s\n",
                PROF_SYNC == VL53L9_SYNC_MANUAL ? "MANUAL" : "AUTONOMOUS",
                (unsigned long)PROF_FRAME_PERIOD, PROF_BINNING, PROF_EXPOSURE_MS,
                PROF_DSS_OFF ? "끔" : "기본");
  STEP(vl53l9_set_sync_mode(&g_dev, PROF_SYNC),       "vl53l9_set_sync_mode");
  STEP(vl53l9_set_power_mode(&g_dev, PROF_POWER),     "vl53l9_set_power_mode (REGULAR)");
  STEP(vl53l9_set_frame_period(&g_dev, PROF_FRAME_PERIOD), "vl53l9_set_frame_period");
  STEP(vl53l9_set_context(&g_dev, PROF_CONTEXT),      "vl53l9_set_context (SHORT)");
  STEP(vl53l9_set_binning(&g_dev, PROF_CONTEXT, PROF_BINNING), "vl53l9_set_binning");
#if PROF_DSS_OFF
  STEP(vl53l9_set_dss_mode(&g_dev, PROF_CONTEXT, 0),  "vl53l9_set_dss_mode (끔)");
#endif
  STEP(vl53l9_set_exposure(&g_dev, PROF_CONTEXT, PROF_EXPOSURE_MS), "vl53l9_set_exposure");

  STEP(vl53l9_get_raw_buffer_size(PROF_BINNING, &g_frame_size), "vl53l9_get_raw_buffer_size");
  if (!binningToWH(PROF_BINNING, &g_w, &g_h) || g_frame_size > FRAME_BUF_MAX) {
    Serial.println(F("  binning 설정이 버퍼와 맞지 않는다."));
    g_frame_size = 0;
    return;
  }
  Serial.printf("    binning %d -> %ux%u (%u존), 프레임 %u 바이트\n",
                PROF_BINNING, g_w, g_h, g_w * g_h, g_frame_size);

  // 출력 인터페이스는 프로파일 "다음"에 정한다. 동작 확인된 ESP32-P4 구현과
  // ST 프로파일 헬퍼가 이 순서를 쓴다.
  vl53l9_hw_config_t hw;
  STEP(vl53l9_get_hw_config(&g_dev, &hw), "vl53l9_get_hw_config");
  Serial.printf("    output_interface (기본) = %s\n", hw.output_interface ? "I3C(시리얼)" : "CSI2");
  hw.signaling_mode = true;          // 인터럽트 패드 사용 (I2C 호스트라 IBI 불가)
#if USE_CSI_BISECT
  {
    // 동작 확인된 ESP32-P4 구현과 같은 값. 한 줄 100B, depth/amp/ambient + DSS 를 담는 높이.
    const uint32_t px = (uint32_t)g_w * g_h;
    hw.output_interface            = false;
    hw.interrupt_pad_mode          = true;
    hw.csi_status_line_force_width = false;
    hw.csi_data_rate               = 1000UL * 1000000UL;
    hw.csi_virtual_channel         = 0;
    hw.csi_status_line_datatype    = 0x2A;
    hw.csi_frame_datatype          = 0x2A;
    hw.csi_frame_width             = 100;
    hw.csi_frame_height            = (uint16_t)((px * 6U + px / 2U + 99U) / 100U);
  }
  STEP(vl53l9_set_hw_config(&g_dev, hw), "vl53l9_set_hw_config (-> CSI2)");
  Serial.println(F("  ** 이분법 모드: CSI2 로 측거만 시험한다 (프레임 수신 불가) **"));
#else
  hw.output_interface = true;
  STEP(vl53l9_set_hw_config(&g_dev, hw), "vl53l9_set_hw_config (-> I3C)");
#endif

  {
    // AUTONOMOUS 는 start 명령 완료가 60ms 를 넘길 수 있다 (참고 구현도 TIMEOUT 을
    // 허용하고 STREAMING 을 따로 기다린다). 그래서 여기서는 실패로 끊지 않는다.
    const int se = vl53l9_start(&g_dev);
    Serial.printf("  %-36s %s\n", "vl53l9_start", errText(se));
  }
  delay(100);
  dumpStatus("start 직후");

  banner("측거 시작");
}

void loop() {
  if (g_frame_size == 0) { delay(2000); return; }

#if POWER_TEST_MODE
  // ---------------------------------------------------------------------
  // 전류 측정 모드 — VCSEL 이 실제로 전류를 끄는지 본다.
  //
  // 레지스터에는 "레이저 드라이버 고장" 플래그가 없다. 그래서 발광 여부를
  // 전류로 직접 확인한다. 대기(STANDBY)와 연속 측거를 10초씩 교대하며,
  // 두 구간의 소비 전류 차이를 멀티미터로 읽으면 된다.
  //
  //   차이 있음 -> VCSEL 이 전류를 끈다. B1 은 붙어 있다
  //   차이 없음 -> VBAT_LDD 경로가 죽었다
  //
  // 단발 프레임은 노출이 10ms 뿐이라 멀티미터가 못 잡는다. 측거 구간에서는
  // 폴트로 STANDBY 에 떨어질 때마다 즉시 재시작해 듀티를 최대로 올린다.
  // ---------------------------------------------------------------------
  {
    static int8_t  phase = -1;            // -1 미초기화 / 0 대기 / 1 측거
    static uint32_t phaseStart = 0;
    const uint32_t PHASE_MS = 10000;

    if (phase < 0) {
      vl53l9_stop(&g_dev);
      phase = 0;
      phaseStart = millis();
      Serial.println(F("\n>>> [대기] STANDBY — 기준 전류를 읽으세요 (10초)"));
    } else if (millis() - phaseStart >= PHASE_MS) {
      phase = (phase == 0) ? 1 : 0;
      phaseStart = millis();
      if (phase == 1) {
        Serial.println(F("\n>>> [측거] 연속 프레임 — 전류를 읽으세요 (10초)"));
      } else {
        vl53l9_stop(&g_dev);
        Serial.println(F("\n>>> [대기] STANDBY — 기준 전류를 읽으세요 (10초)"));
      }
    }

    if (phase == 1) {
      vl53l9_status_t st;
      if (vl53l9_get_status(&g_dev, &st) == VL53L9_ERROR_NONE && st.fsm != 3) {
        vl53l9_stop(&g_dev);
        delay(2);
        vl53l9_start(&g_dev);
        delay(2);
      }
      vl53l9_trigger_frame(&g_dev);
      delay(12);                          // 노출 10ms 를 덮는다
    } else {
      delay(50);
    }
    return;
  }
#endif

#if USE_CSI_BISECT
  // CSI-2 모드 디버깅 로직 (현재는 비활성화됨)
  uint32_t fc0 = 0, fc1 = 0;
  vl53l9_read32(&g_dev, 0x0028, &fc0);
  int e = vl53l9_trigger_frame(&g_dev);
  delay(300);
  vl53l9_read32(&g_dev, 0x0028, &fc1);
  dumpStatus("트리거 300ms 후");
  delay(1200);
#else
  // -------------------------------------------------------------------------
  // 수정됨: 하드코딩된 대기 시간을 제거하고 타임아웃 기반의 동적 폴링 구조 적용
  // -------------------------------------------------------------------------
  int e = vl53l9_trigger_frame(&g_dev);
  if (e != VL53L9_ERROR_NONE) {
    Serial.printf("trigger_frame 실패: %s\n", errText(e));
    dumpStatus("trigger 실패");
    delay(2000);
    return;
  }

  uint8_t ready = 0;
  uint32_t start_ms = millis();
  
  // 프레임 데이터가 준비될 때까지 최대 500ms 동안 폴링
  while (!ready && (millis() - start_ms < 500)) {
    e = vl53l9_poll_frame(&g_dev, &ready);
    if (e != VL53L9_ERROR_NONE) {
      Serial.printf("  poll_frame 통신 에러: %s\n", errText(e));
      break;
    }
    if (!ready) {
      delay(5); // I2C 버스 부하를 줄이기 위한 짧은 대기
    }
  }

  if (!ready) {
    Serial.println("poll_frame 타임아웃 (센서가 데이터를 준비하지 않음)");
    dumpStatus("폴링 타임아웃 상태");
    delay(1500);
    return;
  }

  // 데이터 준비가 확인되면 프레임 버퍼 읽어오기
  e = vl53l9_get_frame(&g_dev, g_frame, g_frame_size);
  if (e != VL53L9_ERROR_NONE) {
    Serial.printf("get_frame 실패: %s\n", errText(e));
    delay(1500);
    return;
  }

  // Depth 데이터 파싱 및 시리얼 모니터 출력
  const uint16_t *depth = (const uint16_t *)g_frame;
  Serial.printf("\n--- depth %ux%u, 중앙 %u mm ---\n",
                g_w, g_h, depth[(g_h / 2) * g_w + (g_w / 2)]);
  for (uint16_t y = 0; y < g_h; y++) {
    Serial.print("  ");
    for (uint16_t x = 0; x < g_w; x++) {
      Serial.printf("%6u", depth[y * g_w + x]);
    }
    Serial.println();
  }
  
  delay(33); // 30fps(약 33ms) 주기에 맞춘 대기
#endif
}