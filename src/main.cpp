// ---------------------------------------------------------------------------
// VL53L9CX 측거 — ESP32 DevKit v1 + STEVAL-VL53L9, 순수 I2C
//
// 드라이버는 src/vl53l9cx.{h,cpp}. 설정 기본값은 STEVAL-VL53L9 + Pi 5 에서
// 검증된 VanBruce/vl53l9cx-python 의 configure() 기본값과 같다.
// ---------------------------------------------------------------------------

#include <Arduino.h>
#include <Wire.h>

#include "board_config.h"
#include "vl53l9cx.h"

using namespace vl53l9cx;

// ---- 측거 설정 ----
#define CFG_RESOLUTION   "54x42"          // 54x42 24x20 18x14 12x10 8x6 4x4
#define CFG_CONTEXT      CONTEXT_SHORT
#define CFG_POWER        POWER_ULTRA_LOW  // ST 예제 AR precision 프로파일
#define CFG_SYNC         SYNC_MANUAL
#define CFG_EXPOSURE_MS  10
#define CFG_PERIOD_US    100000UL         // AUTONOMOUS 에서만 의미가 있다

// 이만큼의 프레임마다 깊이 맵을 문자로 찍는다 (0 = 안 찍음)
#define ASCII_MAP_EVERY  5

constexpr size_t kWireBuffer = 256;
constexpr size_t kFrameMax   = 14842;     // 54x42 기준

static Device g_dev(Wire, TOF_I2C_ADDR_7BIT, PIN_XSHUT);
static const Resolution *g_res = nullptr;
static uint8_t g_frame[kFrameMax];
static bool g_ready = false;

static const char *fwErrorName(uint16_t code);
static void printLddFlags(const uint8_t ldd[5]);

static void banner(const char *title) {
  Serial.println();
  Serial.println(F("==================================================="));
  Serial.printf("  %s\n", title);
  Serial.println(F("==================================================="));
}

static bool step(Err e, const char *what) {
  Serial.printf("  %-28s %s\n", what, errName(e));
  return e == Err::Ok;
}

static void dumpStatus(const char *when) {
  Status st;
  const Err e = g_dev.readStatus(&st);
  if (e != Err::Ok) {
    Serial.printf("  [%s] 상태 읽기 실패: %s\n", when, errName(e));
    return;
  }
  const uint8_t es = st.error_status;
  Serial.printf("  [%s] fsm=%u(%s) command_err=0x%02X error_code=0x%04X error_status=0x%02X\n",
                when, st.fsm, fsmName(st.fsm), st.command_error, st.error_code, es);
  Serial.printf("    error_code 0x%04X = %s\n", st.error_code, fwErrorName(st.error_code));
  Serial.printf("    fw=%u ref_array=%u pll=%u sof=%u i_limit=%u spad_ovl=%u vhv_uv=%u vhv_ov=%u\n",
                (es >> 7) & 1, (es >> 6) & 1, (es >> 5) & 1, (es >> 4) & 1,
                (es >> 3) & 1, (es >> 2) & 1, (es >> 1) & 1, es & 1);
  Serial.printf("    ldd_status = %02X %02X %02X %02X %02X   frame_counter=%lu  temp=%u  ldd_temp=%u\n",
                st.ldd_status[0], st.ldd_status[1], st.ldd_status[2], st.ldd_status[3],
                st.ldd_status[4], (unsigned long)st.frame_counter, st.temperature,
                st.ldd_temperature);
  printLddFlags(st.ldd_status);
  Serial.printf("    ref LONG  amp %u/%u dist %u/%u | ref SHORT amp %u/%u dist %u/%u\n",
                st.ref_long_amp[0], st.ref_long_amp[1], st.ref_long_dist[0], st.ref_long_dist[1],
                st.ref_short_amp[0], st.ref_short_amp[1], st.ref_short_dist[0],
                st.ref_short_dist[1]);
}

// UM3683 Table 18. LDD_ERROR_STATUS_1..5 (0x0067..0x006B) 의 비트 이름
static const char *const kLddFlags[5][8] = {
    {"skin_safety(PD 상한 초과)", "dif_removal(PD 하한 미만)", "power_high", "temp_high",
     "temp_low", "ton(APC gate 너무 김)", "toff(APC gate 너무 짧음)", nullptr},
    {"max_pulse_num", "freq_cross_check", "bandgap", "ldvcc_too_high", "ldvcc_too_low",
     "ldvcc_short_to_gnd", "ldout_short_to_ldvcc_ch0", "ldout_short_to_ldgnd_ch0"},
    {"oc_det(과전류)", "oc_det_self_test", "trace_short_to_vdda", "trace_short_to_vddio",
     "trace_short_to_gnd", "trace_open", "trace_self_test", "als_det"},
    {"vddio_under", "vdda_under", "vddio_over", "vdda_over", "watchdog_ch0", "crc_check",
     "oc_clamp", "lvds_hiz(LVDS short/open)"},
    {"ldout_short_to_ldvcc_ch1", "ldout_short_to_ldgnd_ch1", "watchdog_ch1", nullptr, nullptr,
     nullptr, nullptr, nullptr},
};

// UM3683 Table 17 중 이 보드에서 실제로 본 코드
static const char *fwErrorName(uint16_t code) {
  switch (code) {
    case 0x0000: return "NO_ERROR";
    case 0x0008: return "TOP_ERROR_LDD_TIMEOUT (LDD 가 safe mode, 재부팅 필요)";
    case 0x000D: return "TOP_ERROR_LDD_SAFETY";
    case 0x0903: return "CSI2TX_ERROR_UNDERFLOW";
    case 0x0F00: return "CABDT_ERROR_LDD_FAULT (레이저 드라이버 에러 확인)";
    case 0x0F05: return "CABDT_ERROR_VHV_TIMEOUT";
    default:     return "UM3683 Table 17 참고";
  }
}

static void printLddFlags(const uint8_t ldd[5]) {
  bool any = false;
  for (int reg = 0; reg < 5; reg++) {
    for (int bit = 0; bit < 8; bit++) {
      if (!(ldd[reg] & (1u << bit))) continue;
      const char *name = kLddFlags[reg][bit];
      Serial.printf("      LDD_ERROR_STATUS_%d bit%d: %s\n", reg + 1, bit, name ? name : "(예약)");
      any = true;
    }
  }
  if (!any) Serial.println(F("      레이저 드라이버 에러 플래그 없음"));
}

// UM3683 2.4: 레이저 안전 에러 후 LDD 는 safe mode 에 들어간다. start() 만 다시
// 하면 LDD_TIMEOUT(0x0008) 이 난다. XSHUT 부터 전부 다시 올려야 한다.
static Err rebootAndStart() {
  Err e = g_dev.powerOn();
  if (e == Err::Ok) e = g_dev.loadFirmware();
  if (e == Err::Ok) e = g_dev.boot();
  if (e == Err::Ok) e = g_dev.configure(*g_res, CFG_CONTEXT, CFG_POWER, CFG_SYNC,
                                        CFG_EXPOSURE_MS, CFG_PERIOD_US);
  if (e == Err::Ok) e = g_dev.start();
  return e;
}

static void printFrame(const FrameView &f, uint32_t n) {
  uint32_t valid = 0, amp_sum = 0;
  uint16_t dmin = 0xFFFF, dmax = 0;
  for (int r = 0; r < g_res->rows; r++) {
    for (int c = 0; c < g_res->cols; c++) {
      if (!f.valid(r, c)) continue;
      const uint16_t d = f.depthMm(r, c);
      valid++;
      amp_sum += f.amplitude(r, c);
      if (d < dmin) dmin = d;
      if (d > dmax) dmax = d;
    }
  }
  const int cr = g_res->rows / 2, cc = g_res->cols / 2;
  const uint32_t zones = (uint32_t)g_res->rows * g_res->cols;
  Serial.printf("frame #%lu (fc=%lu)  중앙 %u mm%s  유효 %lu/%lu  범위 %u..%u mm  평균 amp %lu\n",
                (unsigned long)n, (unsigned long)f.frameCounter(), f.depthMm(cr, cc),
                f.valid(cr, cc) ? "" : "(무효)", (unsigned long)valid, (unsigned long)zones,
                valid ? dmin : 0, valid ? dmax : 0,
                valid ? (unsigned long)(amp_sum / valid) : 0UL);

  if (ASCII_MAP_EVERY == 0 || (n % ASCII_MAP_EVERY) != 0) return;
  // 0..9 = 0..4.5m (500mm 간격), '+' = 그 이상, ' ' = 무효
  for (int r = 0; r < g_res->rows; r++) {
    char line[64];
    int k = 0;
    for (int c = 0; c < g_res->cols && k < 62; c++) {
      if (!f.valid(r, c)) { line[k++] = ' '; continue; }
      const uint16_t d = f.depthMm(r, c);
      line[k++] = (d >= 5000) ? '+' : (char)('0' + d / 500);
    }
    line[k] = 0;
    Serial.printf("  |%s|\n", line);
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(300);
  banner("VL53L9CX 측거 (순수 I2C, vl53l9cx-python 포팅)");
  Serial.printf("  SDA=%d SCL=%d %lu Hz  XSHUT=%d  SYNC_IN=%d\n", PIN_SDA, PIN_SCL,
                (unsigned long)I2C_FREQ_HZ, PIN_XSHUT, PIN_SYNC_IN);

  if (PIN_SYNC_IN >= 0) {                     // 플로팅 금지: HIGH 로 고정
    pinMode(PIN_SYNC_IN, OUTPUT);
    digitalWrite(PIN_SYNC_IN, HIGH);
  }
  if (PIN_INTR >= 0) pinMode(PIN_INTR, INPUT_PULLUP);

  // Wire 전에 선 상태를 본다. SDA/SCL 풀업(R7/R8)은 STEVAL 쪽 HOST_IOVDD 에서 온다.
  // 둘 다 0 이면 STEVAL 전원이 없거나 GND 가 ESP32 와 공통이 아니다.
  pinMode(PIN_SDA, INPUT);
  pinMode(PIN_SCL, INPUT);
  delay(5);
  Serial.printf("  선 상태 (내부 풀업 끔): SDA=%d SCL=%d  %s\n", digitalRead(PIN_SDA),
                digitalRead(PIN_SCL),
                (digitalRead(PIN_SDA) && digitalRead(PIN_SCL))
                    ? "정상 (외부 풀업 있음)"
                    : "!! LOW: STEVAL 전원 / 공통 GND / J3 확인");

  // 버퍼를 먼저 키운다. 실패하면 64B 청크 쓰기와 240B 읽기가 조용히 잘린다.
  if (Wire.setBufferSize(kWireBuffer) != kWireBuffer) {
    Serial.println(F("  !! Wire.setBufferSize 실패. 중단."));
    return;
  }
  if (!Wire.begin(PIN_SDA, PIN_SCL, I2C_FREQ_HZ)) {
    Serial.println(F("  !! Wire.begin 실패. 중단."));
    return;
  }
  Wire.setTimeOut(100);

  g_res = findResolution(CFG_RESOLUTION);
  if (g_res == nullptr || g_res->frameBytes() > kFrameMax) {
    Serial.println(F("  !! 해상도 설정 오류. 중단."));
    return;
  }

  banner("부팅");
  if (!step(g_dev.powerOn(), "powerOn (XSHUT, ROM 부팅)")) return;
  Serial.printf("    model id = 0x%08lX\n", (unsigned long)g_dev.modelId());
  if (!step(g_dev.loadFirmware(), "loadFirmware (패치 9865B)")) return;
  Serial.printf("    업로드 %lu ms\n", (unsigned long)g_dev.firmwareLoadMs());
  if (!step(g_dev.boot(), "boot")) { dumpStatus("boot 실패"); return; }
  Serial.printf("    patch %u.%u\n", g_dev.patchMajor(), g_dev.patchMinor());

  banner("설정");
  Serial.printf("  %s  ctx=%s  power=%s  sync=%s  exposure=%d ms  period=%lu us\n",
                g_res->name, CFG_CONTEXT == CONTEXT_SHORT ? "SHORT" : "LONG",
                CFG_POWER == POWER_ULTRA_LOW ? "ULTRA_LOW" : CFG_POWER == POWER_LOW ? "LOW" : "REGULAR",
                CFG_SYNC == SYNC_MANUAL ? "MANUAL" : CFG_SYNC == SYNC_AUTONOMOUS ? "AUTONOMOUS" : "SLAVE",
                CFG_EXPOSURE_MS, (unsigned long)CFG_PERIOD_US);
  if (!step(g_dev.configure(*g_res, CFG_CONTEXT, CFG_POWER, CFG_SYNC, CFG_EXPOSURE_MS,
                            CFG_PERIOD_US), "configure")) return;
  Serial.printf("    프레임 %u B\n", (unsigned)g_res->frameBytes());

  if (!step(g_dev.start(), "start")) { dumpStatus("start 실패"); return; }
  dumpStatus("start 직후");

  banner("측거");
  g_ready = true;
}

void loop() {
  static uint32_t n = 0;
  static uint32_t fails = 0;
  if (!g_ready) { delay(1000); return; }

  Err e = Err::Ok;
  if (CFG_SYNC == SYNC_MANUAL) e = g_dev.triggerFrame();
  if (e == Err::Ok) e = g_dev.waitFrame(1000);
  if (e == Err::Ok) e = g_dev.readFrame(g_frame, g_res->frameBytes());

  if (e == Err::Ok) {
    fails = 0;
    printFrame(FrameView(g_frame, *g_res), ++n);
    return;
  }

  Serial.printf("프레임 실패: %s\n", errName(e));
  dumpStatus("실패");
  // 폴트 후에는 XSHUT 부터 전부 다시 올려 재현되는지 본다 (UM3683 2.4).
  if (++fails <= 3) {
    Serial.printf("  재부팅 후 재시작 %lu/3: %s\n", (unsigned long)fails, errName(rebootAndStart()));
  }
  delay(1500);
}
