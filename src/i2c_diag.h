// ---------------------------------------------------------------------------
// I2C 하부 진단 루틴
//
// VL53L9 드라이버가 "타임아웃"을 뱉기 전에, 버스 자체가 살아 있는지부터
// 가린다. 드라이버는 슬레이브 무응답과 프로토콜 오류를 구분해 주지 않으므로
// 이 계층에서 먼저 확인해야 한다.
//
// 검사 항목
//   1. 라인 유휴 전압   -> STEVAL 보드 전원 / 풀업(R7,R8) 생존 확인
//   2. XSHUT 제어       -> 핀이 실제로 센서를 켜고 끄는지
//   3. 주소 스캔        -> 0x01~0x7E, 여러 버스 속도에서
//   4. 목표 주소 반복   -> 접촉 불량(간헐 ACK) 검출
//   5. 버스 복구        -> 슬레이브가 SDA 를 물고 있을 때 클럭 9발
// ---------------------------------------------------------------------------

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// 전체 진단을 실행하고 결과를 시리얼로 출력한다.
// 반환값: target_addr 가 한 번이라도 ACK 하면 true.
// 종료 시 Wire 는 닫힌 상태로 남는다. 이후 vl53l9_esp32_bus_begin() 을 호출할 것.
bool i2c_diag_run(uint8_t target_addr);

// 개별 루틴 (필요하면 따로 호출)
void i2c_diag_line_state(void);          // Wire 를 닫고 라인 전압만 본다
int  i2c_diag_scan(uint32_t freq_hz);    // ACK 한 주소 개수 반환
bool i2c_diag_probe(uint8_t addr, uint32_t freq_hz, uint8_t *p_raw_code);
void i2c_diag_bus_recover(void);         // SCL 9발 + STOP

#ifdef __cplusplus
}
#endif
