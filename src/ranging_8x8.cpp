// ---------------------------------------------------------------------------
// VL53L9CX 8x8 depth 스트리밍 — 평범한 I2C 만으로
//
// MIPI CSI-2 도 I3C 도 쓰지 않는다. output_interface 기본값이 시리얼이라
// 프레임이 레지스터 공간(0x1800)으로 나오고, 그걸 400 kHz I2C 로 읽는다.
//
// === binning 12 = 8x8 ===
//
// lib/vl53l9/vl53l9.c:73
//     #define FRAME_SIZE_BINNING_12 (8U * 8U) // cropped: 8 * 6
//
// raw 해상도는 8x8 = 64존이다. 주석의 "cropped: 8*6" 은 CSI 출력 프레이밍에
// 적용되는 crop 이고 (set_binning 이 x_size=8, y_size=6, y_offset=1 로
// CROP_PARAMS 를 쓴다), 시리얼 경로로 읽는 raw 버퍼는 8x8 그대로다.
//
//     RAW_BUFFER_SIZE(12) = 64*3*2 + 32 + 100 = 516 바이트
//
// 다만 crop 이 시리얼 프레임에도 영향을 주는지는 실측으로 확인된 바 없다.
// 그래서 이 스케치는 표를 믿지 않고 프레임 자체의 메타데이터에 적힌
// 해상도를 읽어서 쓴다 (아래 FRAME_W/FRAME_H 디코드).
//
// === 처리율 ===
//
// 516 B / 400 kHz ~= 11.6 ms. 30 fps 가 버스에 여유 있게 들어간다.
// (참고: binning 2 는 14842 B = 0.33 s 라 3 fps 가 한계다)
//
// === 현재 상태 ===
//
// 이 보드는 VCSEL 이 발광하지 않아 depth 가 전부 0 으로 나온다. 원인은
// 호스트나 설정이 아니다 (docs/vl53l9cx-i2c-map.md 참고). 이 스케치는
// 하드웨어가 정상이면 그대로 8x8 프레임을 출력한다.
//
// 레이저가 살아 있는지는 매 프레임 찍는 ref_amp 로 바로 보인다.
//   ref_amp = 0        -> 레이저 미발광. 하드웨어 문제
//   ref_amp > 0        -> 레이저 정상. depth 를 믿어도 된다
// ---------------------------------------------------------------------------

#include <Arduino.h>

#include "board_config.h"
#include "vl53l9_esp32.h"

extern "C" {
#include "vl53l9.h"
#include "vl53l9_platform.h"
}

// --- 프로파일 -------------------------------------------------------------
#define PROF_CONTEXT      VL53L9_CONTEXT_SHORT
#define PROF_BINNING      12                     // 8x8, 64존, 516 B
#define PROF_POWER        VL53L9_POWER_REGULAR
#define PROF_FRAME_PERIOD (1000000UL / 30UL)     // 30 fps
#define PROF_EXPOSURE_MS  10

#define FRAME_READY_TIMEOUT_MS  200
#define PRINT_PERIOD_MS         500              // 그리드 출력 간격

// binning 12 의 raw 레이아웃. 위 주석의 산식 그대로다.
#define ZONES        64
#define PLANE_BYTES  (ZONES * 2)                 // 128
#define OFF_DEPTH    0
#define OFF_AMP      (PLANE_BYTES)               // 128
#define OFF_AMBIENT  (PLANE_BYTES * 2)           // 256
#define OFF_DSS      (PLANE_BYTES * 3)           // 384
#define DSS_BYTES    (ZONES / 2)                 // 32
#define OFF_STATUS   (OFF_DSS + DSS_BYTES)       // 416
#define STATUS_BYTES 100
#define FRAME_BYTES  (OFF_STATUS + STATUS_BYTES) // 516

// 상태 라인(100B) 내부 오프셋. vl53l9_utils.h 의 메타데이터 구조체가
// SENSOR_STATUS(0x0028) 영역의 직접 오버레이라는 사실에서 나온다.
#define SL_FRAME_COUNTER  0
#define SL_TEMPERATURE    4
#define SL_LDD_TEMP       6
#define SL_REF_LONG_CH1   36
#define SL_REF_SHORT_CH1  44
#define SL_FRAME_W        52
#define SL_FRAME_H        54
#define SL_ERROR_CODE     60
#define SL_ERROR_STATUS   62

static vl53l9_esp32_dev_t g_dev;
static uint8_t  g_frame[FRAME_BYTES];
static uint16_t g_frame_size = 0;
static uint16_t g_w = 8, g_h = 8;      // 메타데이터로 덮어쓴다
static bool     g_ready = false;
static uint32_t g_lastPrint = 0;
static bool     g_warnedDims = false;

// ---------------------------------------------------------------------------

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
    Serial.printf("  %-38s %s\n", what, errText(_e));                         \
    if (_e != VL53L9_ERROR_NONE) return;                                      \
  } while (0)

static void enableSensor() {
  if (PIN_XSHUT < 0) return;
  pinMode(PIN_XSHUT, OUTPUT);
  digitalWrite(PIN_XSHUT, LOW);
  delay(10);
  digitalWrite(PIN_XSHUT, HIGH);
  delay(200);
}

static inline uint16_t u16at(const uint8_t *p, uint16_t off) {
  return (uint16_t)(p[off] | ((uint16_t)p[off + 1] << 8));
}
static inline uint32_t u32at(const uint8_t *p, uint16_t off) {
  return (uint32_t)p[off] | ((uint32_t)p[off + 1] << 8) |
         ((uint32_t)p[off + 2] << 16) | ((uint32_t)p[off + 3] << 24);
}

// ---------------------------------------------------------------------------

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(500);

  Serial.println();
  Serial.println(F("==================================================="));
  Serial.println(F("  VL53L9CX 8x8 depth over I2C"));
  Serial.println(F("==================================================="));

  g_dev.address = TOF_I2C_ADDR_7BIT;
  enableSensor();
  vl53l9_esp32_bus_begin();
  Serial.printf("  I2C %lu Hz, SDA=GPIO%d, SCL=GPIO%d, XSHUT=GPIO%d\n",
                (unsigned long)I2C_FREQ_HZ, PIN_SDA, PIN_SCL, PIN_XSHUT);

  STEP(vl53l9_init(&g_dev), "vl53l9_init (패치 9865B 설치)");

  uint32_t id = 0;
  STEP(vl53l9_get_device_id(&g_dev, &id), "vl53l9_get_device_id");
  Serial.printf("    device id = 0x%08lX\n", (unsigned long)id);

  STEP(vl53l9_set_power_mode(&g_dev, PROF_POWER),   "vl53l9_set_power_mode (REGULAR)");
  STEP(vl53l9_set_frame_period(&g_dev, PROF_FRAME_PERIOD),
                                                    "vl53l9_set_frame_period (30fps)");
  STEP(vl53l9_set_context(&g_dev, PROF_CONTEXT),    "vl53l9_set_context (SHORT)");
  STEP(vl53l9_set_binning(&g_dev, PROF_CONTEXT, PROF_BINNING),
                                                    "vl53l9_set_binning (12 -> 8x8)");
  STEP(vl53l9_set_exposure(&g_dev, PROF_CONTEXT, PROF_EXPOSURE_MS),
                                                    "vl53l9_set_exposure (10ms)");
  STEP(vl53l9_set_sync_mode(&g_dev, VL53L9_SYNC_MANUAL), "vl53l9_set_sync_mode (MANUAL)");

  STEP(vl53l9_get_raw_buffer_size(PROF_BINNING, &g_frame_size),
                                                    "vl53l9_get_raw_buffer_size");
  Serial.printf("    프레임 %u 바이트 (기대 %u)\n", g_frame_size, (unsigned)FRAME_BYTES);
  if (g_frame_size != FRAME_BYTES) {
    Serial.println(F("  !! 드라이버가 돌려준 크기가 이 파일의 레이아웃 상수와 다르다."));
    Serial.println(F("     PROF_BINNING 을 바꿨다면 ZONES/OFF_* 도 같이 고쳐야 한다."));
    g_frame_size = 0;
    return;
  }

  const uint32_t bits = (uint32_t)g_frame_size * 9UL;   // 8N1 + ACK 근사
  Serial.printf("    %lu Hz 기준 프레임당 약 %lu ms (최대 %lu fps)\n",
                (unsigned long)I2C_FREQ_HZ,
                (unsigned long)(bits * 1000UL / I2C_FREQ_HZ),
                (unsigned long)(I2C_FREQ_HZ / bits));

  STEP(vl53l9_start(&g_dev), "vl53l9_start");
  delay(50);

  g_ready = true;
  Serial.println(F("\n  측거 시작.\n"));
}

void loop() {
  if (!g_ready) { delay(2000); return; }

  int e = vl53l9_trigger_frame(&g_dev);
  if (e != VL53L9_ERROR_NONE) {
    Serial.printf("trigger_frame 실패: %s\n", errText(e));
    delay(1000);
    return;
  }

  // INTR 핀 대신 폴링. 배선이 있으면 PIN_INTR 인터럽트로 바꿔도 된다.
  uint8_t ready = 0;
  const uint32_t t0 = millis();
  do {
    e = vl53l9_poll_frame(&g_dev, &ready);
    if (e != VL53L9_ERROR_NONE) {
      Serial.printf("poll_frame 실패: %s\n", errText(e));
      delay(1000);
      return;
    }
  } while (!ready && (millis() - t0) < FRAME_READY_TIMEOUT_MS);

  if (!ready) {
    Serial.printf("프레임 타임아웃 (%u ms)\n", (unsigned)FRAME_READY_TIMEOUT_MS);
    delay(500);
    return;
  }

  e = vl53l9_get_frame(&g_dev, g_frame, g_frame_size);
  if (e != VL53L9_ERROR_NONE) {
    Serial.printf("get_frame 실패: %s\n", errText(e));
    delay(500);
    return;
  }

  // --- 메타데이터가 말하는 실제 해상도를 쓴다 ---------------------------
  const uint8_t *sl = &g_frame[OFF_STATUS];
  const uint16_t mw = u16at(sl, SL_FRAME_W);
  const uint16_t mh = u16at(sl, SL_FRAME_H);
  if (mw > 0 && mh > 0 && mw * mh <= ZONES) {
    if ((mw != g_w || mh != g_h) && !g_warnedDims) {
      Serial.printf("  ** 프레임 메타데이터 해상도 %ux%u — 표의 8x8 과 다르다.\n", mw, mh);
      Serial.println(F("     메타데이터를 따른다 (crop 이 시리얼 경로에도 적용된 것)."));
      g_warnedDims = true;
    }
    g_w = mw; g_h = mh;
  }

  if (millis() - g_lastPrint < PRINT_PERIOD_MS) return;
  g_lastPrint = millis();

  const uint16_t *depth = (const uint16_t *)&g_frame[OFF_DEPTH];
  const uint16_t *amp   = (const uint16_t *)&g_frame[OFF_AMP];

  // 기준 채널 진폭 — 레이저가 실제로 나오는지의 지표
  const uint16_t refAmp = u16at(sl, SL_REF_SHORT_CH1);

  Serial.printf("--- frame #%lu  %ux%u  die=%uC ldd=%uC  err=0x%04X/0x%02X  ref_amp=%u%s\n",
                (unsigned long)u32at(sl, SL_FRAME_COUNTER), g_w, g_h,
                u16at(sl, SL_TEMPERATURE), u16at(sl, SL_LDD_TEMP),
                u16at(sl, SL_ERROR_CODE), sl[SL_ERROR_STATUS], refAmp,
                (refAmp == 0) ? "  <-- 레이저 미발광" : "");

  for (uint16_t y = 0; y < g_h; y++) {
    Serial.print(F("  "));
    for (uint16_t x = 0; x < g_w; x++) Serial.printf("%6u", depth[y * g_w + x]);
    Serial.print(F("   |"));
    for (uint16_t x = 0; x < g_w; x++) Serial.printf("%6u", amp[y * g_w + x]);
    Serial.println();
  }
  Serial.println();
}
