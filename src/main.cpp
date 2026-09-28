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
  Serial.printf("    fw=%u ref_array=%u pll=%u sof=%u i_limit=%u spad_ovl=%u vhv_uv=%u vhv_ov=%u\n",
                (es >> 7) & 1, (es >> 6) & 1, (es >> 5) & 1, (es >> 4) & 1,
                (es >> 3) & 1, (es >> 2) & 1, (es >> 1) & 1, es & 1);
  Serial.printf("    ldd_status = %02X %02X %02X %02X %02X   frame_counter=%lu  temp=%u  ldd_temp=%u\n",
                st.ldd_status[0], st.ldd_status[1], st.ldd_status[2], st.ldd_status[3],
                st.ldd_status[4], (unsigned long)st.frame_counter, st.temperature,
                st.ldd_temperature);
  Serial.printf("    ref LONG  amp %u/%u dist %u/%u | ref SHORT amp %u/%u dist %u/%u\n",
                st.ref_long_amp[0], st.ref_long_amp[1], st.ref_long_dist[0], st.ref_long_dist[1],
                st.ref_short_amp[0], st.ref_short_amp[1], st.ref_short_dist[0],
                st.ref_short_dist[1]);
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
  // FW 폴트로 STANDBY 에 떨어졌으면 몇 번 다시 시작해 재현되는지 본다.
  Status st;
  if (++fails <= 3 && g_dev.readStatus(&st) == Err::Ok && st.fsm == FSM_STANDBY) {
    Serial.printf("  재시작 시도 %lu/3: %s\n", (unsigned long)fails, errName(g_dev.start()));
  }
  delay(1500);
}
