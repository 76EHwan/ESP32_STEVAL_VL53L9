// ---------------------------------------------------------------------------
// VL53L9CX 레이저 미점등 진단 — 읽기 전용 스케치
//
// 배경: 측거가 REF_ARRAY_ERROR 로 실패하고, 상태 라인을 보면 VCSEL 2채널의
// 기준 진폭이 모두 0 이다 (레이저가 안 켜진다). 같은 프레임에서 다이 온도는
// 정상이고, 전류/전압 관련 폴트 비트는 전부 0 이다. 즉 "구동을 시도했다가
// 실패" 가 아니라 "구동 명령 자체가 안 나간다" 에 가깝다.
//
// 이 스케치가 답하려는 것 세 가지:
//
//   1. CAL_TARGET_LD (0x049C) 의 정체
//        ST 헤더에 주석이 없고 드라이버는 이 레지스터를 읽지도 쓰지도 않는다.
//        LD 가 Laser Driver 인지 Long Distance 인지에 따라 진단이 갈린다.
//        - 드라이버는 레이저 드라이버를 일관되게 LDD 로 쓴다
//          (LDD_TEMPERATURE, LDD_STATUS, laser_driver[]). 여기만 LD 다.
//        - 이웃이 전부 측거 오프셋 캘리브레이션이다 (CAL_REF_OFFSET,
//          CAL_RTN_OFFSET, CAL_PROG_OFFSET_*). 그리고 그 블록은
//          _init_default_config 주석대로 "컨텍스트에 따라" 값이 갈린다.
//        => SHORT / LONG 을 바꿔가며 읽어 값이 따라 움직이면 거리
//           캘리브레이션이고 레이저와 무관하다. 안 움직이면 해석이 다시 열린다.
//
//   2. VCSEL 스텝 테이블이 실제로 장치에 들어갔는가
//        _init_default_config 가 쓰는 유일한 "어느 VCSEL 을 언제 켤지" 설정인데,
//        지금까지 되읽어 대조한 적이 없는 유일한 설정 그룹이다.
//        (STREAM_STEP_NUMBER, NB_SHOT_STEP, CONTEXT, SYNCHRO, FRAME_PERIOD,
//         OUTPUT_IF, POWER_MODE, STANDBY_BINNING 은 이미 대조 완료)
//
//   3. LDD_STATUS 5바이트의 유효한 값
//        원본 드라이버가 5회 모두 [0] 에 쓰던 인덱싱 버그를 고친 뒤의 값이
//        필요하다. 수정 전 값(00 00 14 00 00)은 4바이트가 스택 쓰레기였다.
//
// ===========================================================================
// 안전
// ===========================================================================
// 레이저 관련 레지스터에 쓰지 않는다. 이 스케치의 쓰기는 두 가지가 전부다.
//
//   * vl53l9_init()        펌웨어 패치 설치. 센서를 쓰려면 불가피하다.
//   * vl53l9_set_context() 공개 API. 컨텍스트 선택만 바꾼다.
//
// 측거를 시작하지 않는다 — vl53l9_start() / vl53l9_trigger_frame() 을
// 호출하지 않는다. 따라서 VCSEL 구동 경로는 전 구간 건드리지 않는다.
// docs/vl53l9cx-i2c-map.md 의 "안전" 방침을 그대로 지킨다.
// ---------------------------------------------------------------------------

#include <Arduino.h>

#include "board_config.h"
#include "vl53l9_esp32.h"

extern "C" {
#include "vl53l9.h"
#include "vl53l9_platform.h"
}

// ---------------------------------------------------------------------------
// 레지스터 주소
//
// vl53l9_reg.h 를 include 하지 않고 숫자를 직접 적는다. 검수할 때 주소가
// 눈에 보이는 편이 낫고, C 헤더의 BIT/GENMASK 의존을 끌고 오지 않아도 된다.
// 각 값의 출처는 lib/vl53l9/vl53l9_reg.h 해당 줄이다.
// ---------------------------------------------------------------------------

// STANDBY_SETTINGS (reg.h:66) = 0x0460
#define R_CAL_REF_OFFSET        0x0496u   // reg.h:93  (+0x36)
#define R_CAL_RTN_OFFSET        0x0497u   // reg.h:94  (+0x37)
#define R_CAL_PROG_OFFSET_1TO5  0x0498u   // reg.h:95  (+0x38)
#define R_CAL_PROG_OFFSET_6     0x049Au   // reg.h:96  (+0x3A)
#define R_CAL_TARGET_LD         0x049Cu   // reg.h:97  (+0x3C)   <-- 규명 대상
#define R_CROP_PARAMS           0x04A4u   // reg.h:98  (+0x44)

// CAL 블록을 여유 있게 감싸는 덤프 창. 0x049C 와 0x04A4 사이 8바이트 공백에
// 문서화되지 않은 형제(예: SHORT 용 짝)가 있는지 보는 게 목적이다.
#define CALDUMP_BASE            0x0490u
#define CALDUMP_LEN             32        // 0x0490 ~ 0x04AF

// STREAM_CONTEXT(C) (reg.h:113) = C * 0x80 + 0x04CC
#define R_STREAM_BASE(c)        (uint16_t)((c) * 0x80u + 0x04CCu)
#define R_STREAM_STEP_NUMBER(c) (uint16_t)(R_STREAM_BASE(c) + 0x0000u)          // reg.h:114
#define R_VCSEL_CH0(n, c)       (uint16_t)(R_STREAM_BASE(c) + (n) * 0x1u + 0x01u) // reg.h:115
#define R_VCSEL_CH1(n, c)       (uint16_t)(R_STREAM_BASE(c) + (n) * 0x1u + 0x09u) // reg.h:116
#define R_BLANKING(n, c)        (uint16_t)(R_STREAM_BASE(c) + (n) * 0x2u + 0x12u) // reg.h:117
#define R_DITHERING(n, c)       (uint16_t)(R_STREAM_BASE(c) + (n) * 0x1u + 0x22u) // reg.h:118
#define R_NB_SHOT(n, c)         (uint16_t)(R_STREAM_BASE(c) + (n) * 0x4u + 0x34u) // reg.h:125

// SENSOR_STATUS (reg.h:35) = 0x0028
#define R_SENSOR_STATUS         0x0028u
#define R_TEMPERATURE           0x002Cu   // reg.h:37 (+0x04)
#define R_LDD_TEMPERATURE       0x002Eu   // reg.h:38 (+0x06)
#define R_ERROR_CODE            0x0064u   // reg.h:39 (+0x3C)
#define R_ERROR_STATUS          0x0066u   // reg.h:40 (+0x3E)
#define R_LDD_STATUS(n)         (uint16_t)(R_SENSOR_STATUS + (n) * 0x1u + 0x3Fu) // reg.h:41

#define R_CONTEXT_SELECTION     0x047Au   // reg.h:76 (+0x1A)

// ---------------------------------------------------------------------------
// 기대값
//
// lib/vl53l9/vl53l9.c:134-142 의 static const 배열 복제본이다. 그쪽이
// static 이라 외부에서 참조할 수 없어 부득이하게 옮겨 적었다.
// 드라이버를 업데이트하면 여기도 같이 맞춰야 한다.
// ---------------------------------------------------------------------------
static const uint8_t  exp_tx0_short[7] = { 1, 3, 5, 11, 9, 9, 0 };
static const uint8_t  exp_tx1_short[7] = { 2, 4, 6, 12, 10, 10, 0 };
static const uint16_t exp_blank_short[7] = { 296, 116, 26, 20, 18, 18, 0 };
static const uint8_t  exp_dith_short[7] = { 31, 31, 31, 31, 31, 31, 0 };

static const uint8_t  exp_tx0_long[7] = { 1, 3, 13, 11, 0, 11, 0 };
static const uint8_t  exp_tx1_long[7] = { 2, 4, 14, 12, 0, 12, 0 };
static const uint16_t exp_blank_long[7] = { 296, 116, 54, 20, 0, 20, 0 };
static const uint8_t  exp_dith_long[7] = { 31, 31, 31, 31, 31, 31, 0 };

static vl53l9_esp32_dev_t g_dev;

// 두 컨텍스트에서 뜬 CAL 블록. 마지막에 비교한다.
static uint8_t g_cal_short[CALDUMP_LEN];
static uint8_t g_cal_long[CALDUMP_LEN];
static bool    g_cal_short_ok = false;
static bool    g_cal_long_ok = false;

// ---------------------------------------------------------------------------
// 보조
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

static void banner(const char *title) {
  Serial.println();
  Serial.println(F("==================================================="));
  Serial.print(F("  "));
  Serial.println(title);
  Serial.println(F("==================================================="));
}

static void enableSensor() {
  if (PIN_XSHUT < 0) return;
  pinMode(PIN_XSHUT, OUTPUT);
  digitalWrite(PIN_XSHUT, LOW);
  delay(10);
  digitalWrite(PIN_XSHUT, HIGH);
  delay(200);
}

// 16바이트 한 줄씩 주소와 함께 찍는다.
static void hexdump(uint16_t base, const uint8_t *b, uint16_t n) {
  for (uint16_t off = 0; off < n; off += 16) {
    Serial.printf("  %04X  ", (unsigned)(base + off));
    for (uint16_t k = 0; k < 16 && (off + k) < n; k++) Serial.printf("%02X ", b[off + k]);
    Serial.println();
  }
}

// 바이트열 비교. 다르면 첫 불일치 위치를 알려준다.
static bool cmpBytes(const char *what, uint16_t addr,
                     const uint8_t *got, const uint8_t *want, uint16_t n) {
  int32_t first = -1;
  uint16_t bad = 0;
  for (uint16_t i = 0; i < n; i++) {
    if (got[i] != want[i]) { bad++; if (first < 0) first = (int32_t)i; }
  }
  Serial.printf("  %-28s @%04X  ", what, (unsigned)addr);
  if (bad == 0) {
    Serial.print(F("일치   "));
    for (uint16_t i = 0; i < n; i++) Serial.printf("%02X ", got[i]);
    Serial.println();
    return true;
  }
  Serial.printf("!! 불일치 %u/%u (첫 위치 step %ld)\n", bad, n, (long)(first + 1));
  Serial.print(F("      기대  "));
  for (uint16_t i = 0; i < n; i++) Serial.printf("%02X ", want[i]);
  Serial.println();
  Serial.print(F("      실제  "));
  for (uint16_t i = 0; i < n; i++) Serial.printf("%02X ", got[i]);
  Serial.println();
  return false;
}

// uint16 배열을 리틀엔디안 바이트열로 펴서 비교한다.
// 드라이버가 (uint8_t*) 캐스팅해 그대로 써 넣으므로 장치에도 LE 로 들어간다.
static bool cmpU16(const char *what, uint16_t addr,
                   const uint8_t *got, const uint16_t *want, uint16_t count) {
  uint8_t flat[14];
  for (uint16_t i = 0; i < count; i++) {
    flat[i * 2]     = (uint8_t)(want[i] & 0xFF);
    flat[i * 2 + 1] = (uint8_t)(want[i] >> 8);
  }
  return cmpBytes(what, addr, got, flat, (uint16_t)(count * 2));
}

// ---------------------------------------------------------------------------
// PHASE 1/2 — CAL 블록을 컨텍스트별로 뜬다
// ---------------------------------------------------------------------------
static bool dumpCalBlock(vl53l9_context_t ctx, uint8_t *out) {
  const char *name = (ctx == VL53L9_CONTEXT_SHORT) ? "SHORT" : "LONG";

  const int se = vl53l9_set_context(&g_dev, ctx);
  Serial.printf("  set_context(%s) -> %s\n", name, errText(se));
  if (se != VL53L9_ERROR_NONE) return false;
  delay(20);

  // 장치가 실제로 그 컨텍스트인지 확인
  uint8_t sel = 0xFF;
  vl53l9_read8(&g_dev, R_CONTEXT_SELECTION, &sel);
  Serial.printf("  CONTEXT_SELECTION(0x047A) = 0x%02X\n", sel);

  const int re = vl53l9_read(&g_dev, CALDUMP_BASE, out, CALDUMP_LEN);
  if (re != VL53L9_ERROR_NONE) {
    Serial.printf("  !! CAL 블록 읽기 실패: %s\n", errText(re));
    return false;
  }
  hexdump(CALDUMP_BASE, out, CALDUMP_LEN);

  // 개별 필드도 이름을 붙여 찍는다
  const uint16_t o = CALDUMP_BASE;
  Serial.printf("    CAL_REF_OFFSET      (%04X) = 0x%02X\n",
                R_CAL_REF_OFFSET, out[R_CAL_REF_OFFSET - o]);
  Serial.printf("    CAL_RTN_OFFSET      (%04X) = 0x%02X\n",
                R_CAL_RTN_OFFSET, out[R_CAL_RTN_OFFSET - o]);
  Serial.printf("    CAL_PROG_OFFSET_1TO5(%04X) = 0x%04X\n", R_CAL_PROG_OFFSET_1TO5,
                (unsigned)(out[R_CAL_PROG_OFFSET_1TO5 - o] |
                           (out[R_CAL_PROG_OFFSET_1TO5 - o + 1] << 8)));
  Serial.printf("    CAL_PROG_OFFSET_6   (%04X) = 0x%04X\n", R_CAL_PROG_OFFSET_6,
                (unsigned)(out[R_CAL_PROG_OFFSET_6 - o] |
                           (out[R_CAL_PROG_OFFSET_6 - o + 1] << 8)));
  Serial.printf("  * CAL_TARGET_LD       (%04X) = 0x%04X   <-- 규명 대상\n", R_CAL_TARGET_LD,
                (unsigned)(out[R_CAL_TARGET_LD - o] |
                           (out[R_CAL_TARGET_LD - o + 1] << 8)));

  // 0x049C ~ 0x04A3: 문서화되지 않은 8바이트
  Serial.print(F("    0x049C~0x04A3 (미문서 8B): "));
  for (uint16_t i = 0; i < 8; i++) Serial.printf("%02X ", out[R_CAL_TARGET_LD - o + i]);
  Serial.println();
  return true;
}

// ---------------------------------------------------------------------------
// PHASE 3 — VCSEL 스텝 테이블 되읽기
// ---------------------------------------------------------------------------
static void verifyStreamTables(vl53l9_context_t ctx) {
  const char *name = (ctx == VL53L9_CONTEXT_SHORT) ? "SHORT" : "LONG";
  const bool  isShort = (ctx == VL53L9_CONTEXT_SHORT);
  const uint16_t c = (uint16_t)ctx;

  Serial.printf("\n  --- %s 컨텍스트 (STREAM_BASE=0x%04X) ---\n", name, R_STREAM_BASE(c));

  uint8_t steps = 0xFF;
  vl53l9_read8(&g_dev, R_STREAM_STEP_NUMBER(c), &steps);
  Serial.printf("  STREAM_STEP_NUMBER          @%04X  = %u %s\n",
                R_STREAM_STEP_NUMBER(c), steps, (steps == 7) ? "" : "  <-- 7 이 아니다");

  uint8_t buf[14];

  if (vl53l9_read(&g_dev, R_VCSEL_CH0(1, c), buf, 7) == VL53L9_ERROR_NONE)
    cmpBytes("VCSEL_CH0_STEP[1..7]", R_VCSEL_CH0(1, c), buf,
             isShort ? exp_tx0_short : exp_tx0_long, 7);
  else Serial.println(F("  VCSEL_CH0 읽기 실패"));

  if (vl53l9_read(&g_dev, R_VCSEL_CH1(1, c), buf, 7) == VL53L9_ERROR_NONE)
    cmpBytes("VCSEL_CH1_STEP[1..7]", R_VCSEL_CH1(1, c), buf,
             isShort ? exp_tx1_short : exp_tx1_long, 7);
  else Serial.println(F("  VCSEL_CH1 읽기 실패"));

  if (vl53l9_read(&g_dev, R_BLANKING(1, c), buf, 14) == VL53L9_ERROR_NONE)
    cmpU16("BLANKING_STEP[1..7]", R_BLANKING(1, c), buf,
           isShort ? exp_blank_short : exp_blank_long, 7);
  else Serial.println(F("  BLANKING 읽기 실패"));

  if (vl53l9_read(&g_dev, R_DITHERING(1, c), buf, 7) == VL53L9_ERROR_NONE)
    cmpBytes("MAX_DITHERING_STEP[1..7]", R_DITHERING(1, c), buf,
             isShort ? exp_dith_short : exp_dith_long, 7);
  else Serial.println(F("  DITHERING 읽기 실패"));

  // NB_SHOT 은 드라이버가 아니라 set_exposure 가 채운다. 참고로만 찍는다.
  Serial.print(F("  NB_SHOT_STEP[1..7]           = "));
  for (uint8_t n = 1; n <= 7; n++) {
    uint32_t v = 0;
    vl53l9_read32(&g_dev, R_NB_SHOT(n, c), &v);
    Serial.printf("%lu ", (unsigned long)v);
  }
  Serial.println();
}

// ---------------------------------------------------------------------------
// PHASE 4 — LDD / 상태
// ---------------------------------------------------------------------------
static void dumpLdd() {
  vl53l9_status_t st;
  const int e = vl53l9_get_status(&g_dev, &st);
  Serial.printf("  vl53l9_get_status -> %s\n", errText(e));
  if (e == VL53L9_ERROR_NONE) {
    const char *fsmName = (st.fsm == 1) ? "READY_TO_BOOT"
                        : (st.fsm == 2) ? "STANDBY"
                        : (st.fsm == 3) ? "STREAMING" : "?";
    Serial.printf("  fsm=0x%02X(%s)  command_err=0x%02X  firmware=0x%04X\n",
                  st.fsm, fsmName, st.command, st.firmware);
    Serial.printf("  error: vhv_ov=%u vhv_uv=%u spad_ovl=%u hvboost=%u "
                  "sof_blank=%u pll_lock=%u ref_array=%u internal_fw=%u\n",
                  st.error.vhv_overvoltage, st.error.vhv_undervoltage,
                  st.error.spad_supply_overload, st.error.hvboost_limit,
                  st.error.sof_outside_blanking, st.error.pll_lock,
                  st.error.ref_array, st.error.internal_fw);
    Serial.printf("  laser_driver[0..4] = %02X %02X %02X %02X %02X  (인덱싱 버그 수정 후)\n",
                  st.laser_driver[0], st.laser_driver[1], st.laser_driver[2],
                  st.laser_driver[3], st.laser_driver[4]);
  }

  // get_status 를 거치지 않고 레지스터에서 직접 한 번 더. 교차 확인용.
  Serial.print(F("  LDD_STATUS(0..4) 직접 읽기 = "));
  for (uint8_t n = 0; n < 5; n++) {
    uint8_t v = 0xFF;
    vl53l9_read8(&g_dev, R_LDD_STATUS(n), &v);
    Serial.printf("%02X ", v);
  }
  Serial.println();

  uint16_t die = 0, ldd = 0, ecode = 0;
  uint8_t  estat = 0;
  vl53l9_read16(&g_dev, R_TEMPERATURE, &die);
  vl53l9_read16(&g_dev, R_LDD_TEMPERATURE, &ldd);
  vl53l9_read16(&g_dev, R_ERROR_CODE, &ecode);
  vl53l9_read8(&g_dev, R_ERROR_STATUS, &estat);
  Serial.printf("  die_temp=%u  ldd_temp=%u  error_code=0x%04X  error_status=0x%02X\n",
                die, ldd, ecode, estat);
  if (die != 0 && ldd == 0) {
    Serial.println(F("  -> 다이 온도는 정상인데 LDD 온도만 0. LDD 블록이 살아난 적이 없다."));
  }
}

// ---------------------------------------------------------------------------

void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(500);

  banner("VL53L9CX 레이저 미점등 진단 (읽기 전용)");
  Serial.println(F("  측거를 시작하지 않는다. VCSEL 구동 경로는 건드리지 않는다."));

  g_dev.address = TOF_I2C_ADDR_7BIT;
  enableSensor();
  vl53l9_esp32_bus_begin();
  Serial.printf("  I2C %lu Hz, SDA=GPIO%d, SCL=GPIO%d, XSHUT=GPIO%d\n",
                (unsigned long)I2C_FREQ_HZ, PIN_SDA, PIN_SCL, PIN_XSHUT);

  banner("초기화 (펌웨어 패치 설치)");
  int e = vl53l9_init(&g_dev);
  Serial.printf("  vl53l9_init -> %s\n", errText(e));
  if (e != VL53L9_ERROR_NONE) {
    Serial.println(F("  중단. R25 제거 / R24 장착 상태부터 확인할 것."));
    return;
  }
  uint32_t id = 0;
  vl53l9_get_device_id(&g_dev, &id);
  Serial.printf("  device id = 0x%08lX  (기대 0x53334C39)\n", (unsigned long)id);

  banner("[1] CAL 블록 — SHORT 컨텍스트");
  g_cal_short_ok = dumpCalBlock(VL53L9_CONTEXT_SHORT, g_cal_short);

  banner("[2] CAL 블록 — LONG 컨텍스트");
  g_cal_long_ok = dumpCalBlock(VL53L9_CONTEXT_LONG, g_cal_long);

  banner("[2-판정] CAL_TARGET_LD 는 컨텍스트를 따라가는가");
  if (g_cal_short_ok && g_cal_long_ok) {
    uint16_t diff = 0;
    Serial.println(F("  SHORT vs LONG 차이 나는 바이트:"));
    for (uint16_t i = 0; i < CALDUMP_LEN; i++) {
      if (g_cal_short[i] != g_cal_long[i]) {
        Serial.printf("    %04X  SHORT=%02X  LONG=%02X\n",
                      (unsigned)(CALDUMP_BASE + i), g_cal_short[i], g_cal_long[i]);
        diff++;
      }
    }
    if (diff == 0) Serial.println(F("    (없음 — 블록 전체가 동일하다)"));

    const uint16_t o = R_CAL_TARGET_LD - CALDUMP_BASE;
    const uint16_t ts = (uint16_t)(g_cal_short[o] | (g_cal_short[o + 1] << 8));
    const uint16_t tl = (uint16_t)(g_cal_long[o]  | (g_cal_long[o + 1] << 8));
    Serial.printf("\n  CAL_TARGET_LD:  SHORT=0x%04X   LONG=0x%04X\n", ts, tl);

    if (ts != tl) {
      Serial.println(F("  => 컨텍스트에 따라 값이 바뀐다."));
      Serial.println(F("     LD = Long Distance (거리 캘리브레이션) 해석이 맞다."));
      Serial.println(F("     레이저 미점등과 무관하다. 용의선상에서 제외."));
    } else if (ts == 0) {
      Serial.println(F("  => 양쪽 다 0 이고 움직이지 않는다."));
      Serial.println(F("     해석이 갈리지 않는다. ST 문의가 필요하다."));
      Serial.println(F("     (st-community-question.md 질문 2번)"));
    } else {
      Serial.println(F("  => 0 이 아닌 고정값. 컨텍스트와 무관한 상수로 보인다."));
    }
  } else {
    Serial.println(F("  블록을 못 떠서 판정 불가."));
  }

  banner("[3] VCSEL 스텝 테이블 되읽기");
  Serial.println(F("  _init_default_config 가 쓴 값이 실제로 장치에 있는지 본다."));
  Serial.println(F("  기대값 출처: lib/vl53l9/vl53l9.c:134-142"));
  verifyStreamTables(VL53L9_CONTEXT_SHORT);
  verifyStreamTables(VL53L9_CONTEXT_LONG);

  // 원래 프로파일대로 SHORT 로 되돌려 놓는다
  vl53l9_set_context(&g_dev, VL53L9_CONTEXT_SHORT);

  banner("[4] LDD / 상태 레지스터");
  dumpLdd();

  banner("완료");
  Serial.println(F("  위 출력을 그대로 docs/st-community-question.md 에 붙일 수 있다."));
}

void loop() {
  delay(5000);
}
