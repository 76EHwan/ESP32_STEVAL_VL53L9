// ---------------------------------------------------------------------------
// I2C 하부 진단 루틴 구현 — i2c_diag.h 참고
// ---------------------------------------------------------------------------

#include <Arduino.h>
#include <Wire.h>

#include "board_config.h"
#include "i2c_diag.h"

// endTransmission() 반환 코드 해설.
// ESP32 Arduino 코어는 IDF 레거시 i2c 드라이버의 ESP_FAIL 을 2 로 접는다.
// ESP_FAIL 의 IDF 정의는 "slave hasn't ACK the transfer" 하나뿐이다.
static const char *txErrText(uint8_t rc) {
  switch (rc) {
    case 0: return "ACK";
    case 1: return "데이터가 버퍼보다 길다";
    case 2: return "주소 NACK (슬레이브 무응답)";
    case 3: return "데이터 NACK";
    case 4: return "기타 오류";
    case 5: return "타임아웃";
    default: return "알 수 없음";
  }
}

// ---------------------------------------------------------------------------
// 1. 라인 유휴 전압
//
// STEVAL-VL53L9 는 호스트 쪽 SDA/SCL 에 2.2k 풀업(R7/R8)을 달고 있고, 그
// 풀업은 보드의 3V3 에서 나온다. 그래서 ESP32 핀을 그냥 INPUT (내부 풀업
// 끔) 으로 두고 읽기만 해도 보드 전원 상태를 알 수 있다.
//
//   INPUT=1                 -> 외부 풀업 살아있음. 보드에 전원이 들어온다
//   INPUT=0, PULLUP=1       -> 외부 풀업 없음. 보드 전원 또는 배선 문제
//   INPUT=0, PULLUP=0       -> 라인이 GND 로 단락, 또는 슬레이브가 물고 있음
// ---------------------------------------------------------------------------
static void readPair(int pin, int *p_float, int *p_pullup) {
  pinMode(pin, INPUT);
  delayMicroseconds(200);
  *p_float = digitalRead(pin);
  pinMode(pin, INPUT_PULLUP);
  delayMicroseconds(200);
  *p_pullup = digitalRead(pin);
  pinMode(pin, INPUT);
}

void i2c_diag_line_state(void) {
  Wire.end();
  delay(2);

  int sda_f, sda_p, scl_f, scl_p;
  readPair(PIN_SDA, &sda_f, &sda_p);
  readPair(PIN_SCL, &scl_f, &scl_p);

  Serial.printf("  SDA(GPIO%d)  내부풀업끔=%d  내부풀업켬=%d\n", PIN_SDA, sda_f, sda_p);
  Serial.printf("  SCL(GPIO%d)  내부풀업끔=%d  내부풀업켬=%d\n", PIN_SCL, scl_f, scl_p);

  if (sda_f == 1 && scl_f == 1) {
    Serial.println(F("  -> 외부 풀업 정상. STEVAL 보드에 전원이 들어와 있다."));
  } else if (sda_p == 1 && scl_p == 1) {
    Serial.println(F("  -> !! 외부 풀업이 없다. 내부 풀업으로만 올라온다."));
    Serial.println(F("     STEVAL 보드 3V3 미인가 / J2 배선 빠짐 / GND 미연결 의심."));
  } else {
    Serial.println(F("  -> !! 라인이 LOW 에 붙어 있다. GND 단락이거나"));
    Serial.println(F("     슬레이브가 버스를 물고 있다. 버스 복구를 시도한다."));
  }
}

// ---------------------------------------------------------------------------
// 2. XSHUT 제어 확인
// ---------------------------------------------------------------------------
static void xshutSet(int level) {
  if (PIN_XSHUT < 0) return;
  pinMode(PIN_XSHUT, OUTPUT);
  digitalWrite(PIN_XSHUT, level);
}

static void xshutReport(void) {
  if (PIN_XSHUT < 0) {
    Serial.println(F("  XSHUT 미연결 설정(-1). 건너뛴다."));
    return;
  }
  pinMode(PIN_XSHUT, INPUT);
  delayMicroseconds(200);
  const int idle = digitalRead(PIN_XSHUT);
  pinMode(PIN_XSHUT, INPUT_PULLDOWN);
  delayMicroseconds(200);
  const int pd = digitalRead(PIN_XSHUT);
  Serial.printf("  XSHUT(GPIO%d) 개방시=%d  내부풀다운시=%d\n", PIN_XSHUT, idle, pd);
  if (pd == 1) {
    Serial.println(F("  -> 외부에서 HIGH 로 구동되고 있다. 배선 확인."));
  }
  xshutSet(HIGH);
}

// ---------------------------------------------------------------------------
// 3. 주소 스캔
// ---------------------------------------------------------------------------
static bool busBegin(uint32_t freq_hz) {
  Wire.end();
  delay(2);
  if (!Wire.begin(PIN_SDA, PIN_SCL, freq_hz)) {
    Serial.println(F("  !! Wire.begin 실패"));
    return false;
  }
  Wire.setTimeOut(50);
  return true;
}

int i2c_diag_scan(uint32_t freq_hz) {
  if (!busBegin(freq_hz)) return -1;

  int found = 0;
  Serial.printf("  %lu Hz: ", (unsigned long)freq_hz);
  for (uint8_t a = 1; a < 0x7F; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission(true) == 0) {
      Serial.printf("0x%02X ", a);
      found++;
    }
  }
  if (found == 0) Serial.print(F("(응답 없음)"));
  Serial.printf("  -> %d 개\n", found);
  return found;
}

bool i2c_diag_probe(uint8_t addr, uint32_t freq_hz, uint8_t *p_raw_code) {
  if (!busBegin(freq_hz)) return false;
  Wire.beginTransmission(addr);
  const uint8_t rc = Wire.endTransmission(true);
  if (p_raw_code) *p_raw_code = rc;
  return (rc == 0);
}

// ---------------------------------------------------------------------------
// 4. 버스 복구 — SCL 9발로 슬레이브의 비트 시프터를 비우고 STOP
// ---------------------------------------------------------------------------
void i2c_diag_bus_recover(void) {
  Wire.end();
  delay(2);

  pinMode(PIN_SDA, INPUT_PULLUP);
  pinMode(PIN_SCL, OUTPUT);
  for (int i = 0; i < 9; i++) {
    digitalWrite(PIN_SCL, HIGH);
    delayMicroseconds(5);
    digitalWrite(PIN_SCL, LOW);
    delayMicroseconds(5);
  }
  digitalWrite(PIN_SCL, HIGH);
  delayMicroseconds(5);

  // STOP 조건: SCL 이 HIGH 인 동안 SDA 를 LOW -> HIGH
  pinMode(PIN_SDA, OUTPUT);
  digitalWrite(PIN_SDA, LOW);
  delayMicroseconds(5);
  digitalWrite(PIN_SDA, HIGH);
  delayMicroseconds(5);

  pinMode(PIN_SDA, INPUT);
  pinMode(PIN_SCL, INPUT);
  delay(2);
}

// ---------------------------------------------------------------------------
// 전체 진단
// ---------------------------------------------------------------------------
bool i2c_diag_run(uint8_t target_addr) {
  static const uint32_t kFreqs[] = { 400000UL, 100000UL, 50000UL, 10000UL };

  Serial.println();
  Serial.println(F("==================================================="));
  Serial.println(F("  [진단 1] 버스 라인 상태 (Wire 사용 안 함)"));
  Serial.println(F("==================================================="));
  i2c_diag_line_state();

  Serial.println();
  Serial.println(F("==================================================="));
  Serial.println(F("  [진단 2] XSHUT 핀"));
  Serial.println(F("==================================================="));
  xshutReport();

  Serial.println();
  Serial.println(F("==================================================="));
  Serial.println(F("  [진단 3] XSHUT LOW / HIGH 주소 스캔"));
  Serial.println(F("==================================================="));
  xshutSet(LOW);
  delay(50);
  Serial.print(F("  XSHUT=LOW  "));
  const int found_low = i2c_diag_scan(100000UL);

  xshutSet(HIGH);
  delay(200);
  Serial.print(F("  XSHUT=HIGH "));
  const int found_high = i2c_diag_scan(100000UL);

  if (found_low > 0 && found_high == found_low) {
    Serial.println(F("  -> XSHUT 이 버스에 영향을 주지 않는다. 핀 연결 확인."));
  }

  Serial.println();
  Serial.println(F("==================================================="));
  Serial.println(F("  [진단 4] 속도별 전체 스캔 (XSHUT=HIGH)"));
  Serial.println(F("==================================================="));
  int total = 0;
  for (unsigned i = 0; i < sizeof(kFreqs) / sizeof(kFreqs[0]); i++) {
    const int n = i2c_diag_scan(kFreqs[i]);
    if (n > 0) total += n;
  }

  if (total == 0) {
    Serial.println();
    Serial.println(F("  응답이 전혀 없다. 버스 복구 후 재시도한다."));
    i2c_diag_bus_recover();
    total = i2c_diag_scan(100000UL);
  }

  Serial.println();
  Serial.println(F("==================================================="));
  Serial.printf("  [진단 5] 목표 주소 0x%02X 반복 프로브 20회\n", target_addr);
  Serial.println(F("==================================================="));
  int ack = 0;
  uint8_t last_rc = 0xFF;
  for (int i = 0; i < 20; i++) {
    uint8_t rc = 0xFF;
    if (i2c_diag_probe(target_addr, 100000UL, &rc)) ack++;
    last_rc = rc;
    delay(5);
  }
  Serial.printf("  ACK %d / 20,  마지막 코드 %u (%s)\n", ack, last_rc, txErrText(last_rc));

  // 참고: U4(M24C64, 0x55) 는 레벨시프터 안쪽 1.8V 버스에 있다.
  // 응답하면 시프터가 통과한다는 뜻. 단 이 보드는 U4 를 떼어냈다.
  uint8_t rc_eep = 0xFF;
  (void)i2c_diag_probe(EEPROM_I2C_ADDR_7BIT, 100000UL, &rc_eep);
  Serial.printf("  참고: EEPROM 0x%02X -> %s  (이 보드는 U4 제거됨, 무응답이 정상)\n",
                EEPROM_I2C_ADDR_7BIT, txErrText(rc_eep));

  Wire.end();

  Serial.println();
  if (ack > 0 && ack < 20) {
    Serial.println(F("  !! 간헐 응답. 접촉 불량 / 신호 무결성 문제."));
    Serial.println(F("     듀퐁선 교체, 배선 길이 단축, 속도 100k 로 낮춰서 재시도."));
  }
  return (ack > 0);
}
