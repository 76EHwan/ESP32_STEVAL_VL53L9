// ---------------------------------------------------------------------------
// ESP32 포팅용 디바이스 컨텍스트
//
// ST 드라이버는 p_dev 를 불투명 포인터로만 다룬다 (vl53l9.c 는 역참조하지
// 않는다). 따라서 컨텍스트 구조는 플랫폼 구현이 자유롭게 정의한다.
// ---------------------------------------------------------------------------

#pragma once

#include <stdint.h>

// 온보드 Y1 발진기 주파수. R25 를 제거해야 실제로 동작한다.
#define VL53L9_EXT_CLOCK_HZ   12000000UL

typedef struct {
  uint8_t address;      // 7-bit I2C 주소. 0 이면 board_config.h 기본값 사용
} vl53l9_esp32_dev_t;

#ifdef __cplusplus
extern "C" {
#endif

// Wire 초기화 + 버퍼/타임아웃 설정. 드라이버를 쓰기 전에 한 번 호출한다.
void vl53l9_esp32_bus_begin(void);

#ifdef __cplusplus
}
#endif
