// ---------------------------------------------------------------------------
// VL53L9CX 측거 — ST 드라이버(BSD-3-Clause) ESP32 포팅
//
// 데이터 경로는 MIPI CSI-2 가 아니라 I2C 다.
//   vl53l9_hw_config_t.output_interface 를 true(I3C/시리얼)로 두면 프레임이
//   레지스터 공간으로 나오고 0x1800 창에서 읽힌다. ESP32 에 CSI 수신기가
//   없어도 측거 데이터를 받을 수 있는 이유다.
//
// 프레임 버퍼 레이아웃 (vl53l9_get_frame 기준):
//   [0          .. res*2)     depth      zone 당 uint16, 리틀엔디안
//   [res*2      .. res*4)     amplitude
//   [res*4      .. res*6)     ambient
//   [res*6      .. +res/2)    DSS LUT 인덱스
//   [...        .. +100)      status line
//
// 설정 순서는 ST 예제의 vl53l9_utils_set_profile() 을 공개 API 로 그대로
// 재현한 것이다 (그 함수는 SLA0111 인 vl53l9_interface.h 에 의존해서 가져올
// 수 없다. 하는 일은 공개 API 호출 나열이라 재현이 어렵지 않다).
//
// 전제 조건: R25 제거 / R24 장착 (board_config.h 참고).
//
// === 현재 상태: 센서 하드웨어 이슈로 측거 불가 ===
//
// 소프트웨어 경로는 끝까지 동작한다.
//   init(패치 9865B 설치) / device id 0x53334C39 / 설정 6종 / start -> STREAMING
//   그리고 CSI2 모드에서는 프레임 카운터가 실제로 증가한다.
//
// 그런데 측거가 실패하고 FW 가 STANDBY 로 되돌아간다. 상태 라인을 읽어보면
// 원인이 명확하다.
//
//   ref LONG   ch1 amp=0 dist=0     ch2 amp=0 dist=0
//   ref SHORT  ch1 amp=0 dist=149   ch2 amp=0 dist=149
//   temperature=32   ldd_temp=0
//
// VCSEL 2채널 모두 기준 진폭이 0 이다. 기준 SPAD 배열이 레이저 빛을 전혀
// 받지 못한다. 같은 프레임에서 다이 온도는 32도로 정상 보고되므로 디지털·
// 아날로그 코어는 살아 있다. 레이저만 안 켜진다.
//
// 전원은 확인했다. VBAT_LDD(P3V3)를 C6/C7 에서 측정해 3.3V 정상,
// AVDD 2.8V / DVDD 1.2V / IOVDD 1.8V 모두 정상. PLL 도 동작한다
// (get_calib_data 가 COMMAND_SWITCH_TO_FAST_CLOCK 을 거쳐 성공).
//
// 남은 가능성은 LGA 볼 개방 또는 센서 내부 손상이고, 둘 다 소프트웨어로
// 구분할 수 없다. 이 보드는 EEPROM(U4)을 떼어낸 리워크 이력이 있다.
//
// 자세한 경위와 배제한 가설 전체는 docs/vl53l9cx-i2c-map.md,
// ST 문의용 정리는 docs/st-community-question.md 참고.
//
// 하드웨어가 정상이면 이 코드는 그대로 depth 프레임을 출력한다.
// ---------------------------------------------------------------------------

#include <Arduino.h>

#include "board_config.h"
#include "vl53l9_esp32.h"

extern "C" {
#include "vl53l9.h"
#include "vl53l9_platform.h"   // vl53l9_read / vl53l9_write (청크 검증용)
}

// ST 의 AR_PRECISION 프로파일 값. sync 만 예제와 같이 MANUAL 로 덮어쓴다.
#define PROF_POWER        VL53L9_POWER_REGULAR   // ULTRA_LOW 는 I3C 웨이크 전제로 보임
#define PROF_CONTEXT      VL53L9_CONTEXT_SHORT
#define PROF_FRAME_PERIOD (1000000UL / 30UL)   // 30 fps
#define PROF_BINNING      8                    // 12x10. binning 2 는 start 가 60ms 를 넘긴다
#define PROF_EXPOSURE_MS  10

#define USE_CSI_BISECT    0      // 1 = CSI2 로 측거만 시험하는 진단 모드
                                 //     (프레임 수신 불가. REF_ARRAY 에러를 드러낸다)

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

  // ---------------------------------------------------------------------
  // 상태 라인 100바이트를 통째로 읽어 기준 채널 진폭을 본다.
  //
  // vl53l9_utils.h 의 프레임 메타데이터 구조체는 SENSOR_STATUS(0x0028)
  // 영역의 직접 오버레이다. 레지스터 맵으로 검증됨:
  //   frame_counter  offset 0    = REGADDR_FRAME_COUNTER (base + 0)
  //   temperature    offset 4    = REGADDR_TEMPERATURE   (base + 0x04)
  //   error_code     offset 60   = REGADDR_ERROR_CODE    (base + 0x3C)
  //
  // ref_amplitude 는 VCSEL 이 쏜 빛을 내부 기준 경로로 받은 세기다.
  //   0 에 가까움 -> 레이저가 안 나오거나 기준 경로가 막혔다 (하드웨어)
  //   유의미한 값 -> 레이저는 나온다. 다른 이유로 REF_ARRAY 가 선 것
  // ---------------------------------------------------------------------
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

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(500);

  banner("VL53L9CX 측거 (ST 드라이버 ESP32 포팅)");

  g_dev.address = TOF_I2C_ADDR_7BIT;
  enableSensor();
  vl53l9_esp32_bus_begin();
  Serial.printf("  I2C %lu Hz, SDA=GPIO%d, SCL=GPIO%d, XSHUT=GPIO%d\n",
                (unsigned long)I2C_FREQ_HZ, PIN_SDA, PIN_SCL, PIN_XSHUT);

  // ---------------------------------------------------------------------
  // 플랫폼 계층 청크 분할 쓰기 검증
  //
  // 펌웨어 패치는 9865바이트이고 Wire 버퍼보다 커서 여러 청크로 쪼개 보낸다.
  // 이 경로가 미묘하게 틀리면 패치가 손상된 채 올라가고, 부팅은 되지만
  // 측거 코드가 돌 때 죽는다. init 전에 먼저 확인한다.
  //
  // 대상은 0x1800 (패치 영역). 어차피 init 이 곧 덮어쓰므로 안전하다.
  // ---------------------------------------------------------------------
  banner("[사전] 청크 쓰기 검증");
  {
    const uint32_t N = 4096;                       // 여러 청크에 걸치게
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
      Serial.printf("  !! 불일치 %lu / %lu, 첫 위치 오프셋 %ld (0x%04lX)\n",
                    (unsigned long)bad, (unsigned long)N,
                    (long)first, (unsigned long)(0x1800 + first));
      Serial.printf("     기대 %02X  실제 %02X\n", tx[first], rx[first]);
    }
  }

  banner("초기화");

  // 부팅 + 펌웨어 패치(9865B) 설치
  STEP(vl53l9_init(&g_dev), "vl53l9_init (패치 설치 포함)");

  uint32_t id = 0;
  STEP(vl53l9_get_device_id(&g_dev, &id), "vl53l9_get_device_id");
  Serial.printf("    device id = 0x%08lX\n", (unsigned long)id);

  // 출력 인터페이스 확인만 한다.
  //
  // 기본값이 이미 I3C(시리얼)라 되쓸 필요가 없다. 그리고 set_hw_config 는
  // output_interface 뿐 아니라 CSI 관련 필드(data_rate, frame_width/height,
  // virtual_channel, datatype, signaling_mode ...)를 한꺼번에 되쓴다.
  // ST 의 i3c 예제도 이 함수를 호출하지 않는다. 건드리지 않는다.
  // ---------------------------------------------------------------------
  // [이분법] 출력 인터페이스를 CSI2 로 바꿔서 측거 파이프라인만 시험한다.
  //
  // MIPI 데이터를 받자는 게 아니다. ESP32 에는 CSI 수신기가 없다.
  // 목적은 "FW 가 폴트를 내는가" 하나다. 프레임 카운터(0x0028)만 본다.
  //
  //   폴트 없이 카운터 증가 -> 측거 파이프라인 정상. 문제는 시리얼 출력 한정
  //   똑같이 internal_fw 폴트 -> 출력 방식과 무관. 측거 자체의 문제
  // ---------------------------------------------------------------------
  vl53l9_hw_config_t hw;
  STEP(vl53l9_get_hw_config(&g_dev, &hw), "vl53l9_get_hw_config");
  Serial.printf("    output_interface (기본) = %s\n", hw.output_interface ? "I3C(시리얼)" : "CSI2");
#if USE_CSI_BISECT
  hw.output_interface = false;   // CSI2
  STEP(vl53l9_set_hw_config(&g_dev, hw), "vl53l9_set_hw_config (-> CSI2)");
  Serial.println(F("  ** 이분법 모드: CSI2 로 측거만 시험한다 (프레임 수신 불가) **"));
#endif

  // ---------------------------------------------------------------------
  // [진단] 캘리브레이션 읽기 = PLL + 온칩 캘리브레이션 동시 점검
  //
  // get_calib_data 는 내부적으로 COMMAND_SWITCH_TO_FAST_CLOCK 을 보내
  // PLL 을 켜고 시스템 클럭을 고속으로 바꾼 뒤, OTP 미러 2332바이트를 읽고
  // 다시 외부 클럭으로 되돌린다. 측거가 쓰는 것과 같은 PLL 이다.
  //
  //   실패 -> PLL/클럭 문제. 측거 폴트의 원인일 가능성이 크다
  //   성공 -> PLL 정상. 캘리브레이션 내용으로 U4 리워크 영향도 볼 수 있다
  // ---------------------------------------------------------------------
  banner("[진단] 캘리브레이션 + PLL(고속 클럭 전환)");
  {
    static uint8_t calib[VL53L9_CALIB_DATA_SIZE];
    const int ce = vl53l9_get_calib_data(&g_dev, calib);
    Serial.printf("  vl53l9_get_calib_data -> %s\n", errText(ce));
    if (ce == VL53L9_ERROR_NONE) {
      uint32_t nz = 0, ff = 0;
      for (uint32_t i = 0; i < VL53L9_CALIB_DATA_SIZE; i++) {
        if (calib[i]) nz++;
        if (calib[i] == 0xFF) ff++;
      }
      Serial.printf("  %u 바이트 중 non-zero %lu, 0xFF %lu\n",
                    VL53L9_CALIB_DATA_SIZE, (unsigned long)nz, (unsigned long)ff);
      Serial.print(F("  앞 32B: "));
      for (int i = 0; i < 32; i++) Serial.printf("%02X ", calib[i]);
      Serial.println();
      if (nz == 0) Serial.println(F("  !! 전부 0. 캘리브레이션이 비어 있다."));
    } else {
      Serial.println(F("  !! 실패. 고속 클럭 전환(PLL) 또는 버스트 읽기 문제."));
    }
    dumpStatus("calib 직후");
  }

  banner("프로파일 적용 (ST AR_PRECISION)");

  STEP(vl53l9_set_power_mode(&g_dev, PROF_POWER),   "vl53l9_set_power_mode (REGULAR)");
  STEP(vl53l9_set_frame_period(&g_dev, PROF_FRAME_PERIOD),
                                                     "vl53l9_set_frame_period (30fps)");
  STEP(vl53l9_set_context(&g_dev, PROF_CONTEXT),     "vl53l9_set_context (SHORT)");
  STEP(vl53l9_set_binning(&g_dev, PROF_CONTEXT, PROF_BINNING), "vl53l9_set_binning");
  STEP(vl53l9_set_exposure(&g_dev, PROF_CONTEXT, PROF_EXPOSURE_MS),
                                                     "vl53l9_set_exposure (10ms)");
  STEP(vl53l9_set_sync_mode(&g_dev, VL53L9_SYNC_MANUAL), "vl53l9_set_sync_mode (MANUAL)");

  STEP(vl53l9_get_raw_buffer_size(PROF_BINNING, &g_frame_size), "vl53l9_get_raw_buffer_size");
  if (!binningToWH(PROF_BINNING, &g_w, &g_h) || g_frame_size > FRAME_BUF_MAX) {
    Serial.println(F("  binning 설정이 버퍼와 맞지 않는다."));
    g_frame_size = 0;
    return;
  }
  Serial.printf("    binning %d -> %ux%u (%u존), 프레임 %u 바이트\n",
                PROF_BINNING, g_w, g_h, g_w * g_h, g_frame_size);

  // start 전 실제 설정값을 읽어본다. FW 가 +0ms 에 설정을 거부하므로
  // 무엇이 비어 있는지 보는 게 핵심이다.
  banner("[진단] start 직전 설정 레지스터");
  {
    uint8_t b[0x100];
    if (vl53l9_read(&g_dev, 0x0460, b, 0xC0) == VL53L9_ERROR_NONE) {
      for (uint16_t off = 0; off < 0xC0; off += 16) {
        bool any = false;
        for (int k = 0; k < 16; k++) if (b[off + k]) { any = true; break; }
        if (!any) continue;
        Serial.printf("  %04X  ", 0x0460 + off);
        for (int k = 0; k < 16; k++) Serial.printf("%02X ", b[off + k]);
        Serial.println();
      }
    }
    uint8_t sn = 0xAA;
    vl53l9_read8(&g_dev, 0x04CC, &sn);          // STREAM_STEP_NUMBER (SHORT)
    Serial.printf("  STREAM_STEP_NUMBER(SHORT, 0x04CC) = %u\n", sn);
    uint32_t shots0 = 0;
    vl53l9_read32(&g_dev, 0x0504, &shots0);     // NB_SHOT_STEP(1, SHORT)
    Serial.printf("  NB_SHOT_STEP(1,SHORT, 0x0504) = %lu\n", (unsigned long)shots0);
  }

  STEP(vl53l9_start(&g_dev), "vl53l9_start");
  delay(100);
  dumpStatus("start 직후");

  banner("측거 시작");
}

void loop() {
  if (g_frame_size == 0) { delay(2000); return; }

#if USE_CSI_BISECT
  // CSI2 모드에서는 프레임을 못 받는다. FW 가 죽는지, 프레임 카운터가
  // 올라가는지만 본다.
  uint32_t fc0 = 0, fc1 = 0;
  vl53l9_read32(&g_dev, 0x0028, &fc0);            // FRAME_COUNTER

  int e = vl53l9_trigger_frame(&g_dev);
  Serial.printf("trigger_frame -> %s\n", errText(e));

  delay(300);

  vl53l9_read32(&g_dev, 0x0028, &fc1);
  Serial.printf("  FRAME_COUNTER %lu -> %lu  (증가 %ld)\n",
                (unsigned long)fc0, (unsigned long)fc1, (long)(fc1 - fc0));
  dumpStatus("트리거 300ms 후");
  Serial.println(F("---------------------------------------------------"));
  delay(1200);
  return;
#else
  int e = vl53l9_trigger_frame(&g_dev);
  if (e != VL53L9_ERROR_NONE) {
    Serial.printf("trigger_frame 실패: %s\n", errText(e));
    dumpStatus("trigger 실패");
    delay(2000);
    return;
  }
  delay(200);
  dumpStatus("무통신 200ms 후");
  uint8_t ready = 0;
  e = vl53l9_poll_frame(&g_dev, &ready);
  Serial.printf("  poll_frame -> %s, ready=%u\n", errText(e), ready);
  if (!ready) { delay(1500); return; }
  e = vl53l9_get_frame(&g_dev, g_frame, g_frame_size);
  if (e != VL53L9_ERROR_NONE) {
    Serial.printf("get_frame 실패: %s\n", errText(e));
    delay(1500);
    return;
  }
  const uint16_t *depth = (const uint16_t *)g_frame;
  Serial.printf("\n--- depth %ux%u, 중앙 %u mm ---\n",
                g_w, g_h, depth[(g_h / 2) * g_w + (g_w / 2)]);
  for (uint16_t y = 0; y < g_h; y++) {
    Serial.print("  ");
    for (uint16_t x = 0; x < g_w; x++) Serial.printf("%6u", depth[y * g_w + x]);
    Serial.println();
  }
  delay(1000);
#endif
}
