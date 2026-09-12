// ---------------------------------------------------------------------------
// VL53L9CX 드라이버 플랫폼 계층 — ESP32 / Arduino Wire 구현
//
// ST 드라이버(drivers/vl53l9, BSD-3-Clause)는 vl53l9_platform.h 에 선언된
// 함수들을 통합자가 구현해 주기를 요구한다. 이 파일이 그 구현이다.
//
// 설계 메모:
//
// * p_dev 는 드라이버 입장에서 완전한 불투명 포인터다. vl53l9.c 는 이걸
//   역참조하지 않고 플랫폼 함수에 그대로 넘기기만 한다. 그래서 컨텍스트
//   구조체를 우리가 정의한다 (vl53l9_esp32_dev_t).
//
// * 엔디안 변환을 하지 않는다. ST 참조 구현도 4바이트를 uint32_t 메모리에
//   직접 읽어넣는다. STM32(ARM)도 ESP32(Xtensa)도 리틀엔디안이라 바이트
//   순서가 동일하다.
//
// * Wire 버퍼보다 큰 전송은 여기서 쪼갠다. 센서가 주소 자동증가를 하므로
//   (실측 확인) 청크마다 주소를 올려가며 나눠 보내면 된다. 펌웨어 패치
//   9865바이트, 프레임 버퍼 최대 46KB 가 이 경로를 탄다.
// ---------------------------------------------------------------------------

#include <Arduino.h>
#include <Wire.h>

#include "board_config.h"

extern "C" {
#include "vl53l9.h"
#include "vl53l9_platform.h"
}

#include "vl53l9_esp32.h"

// Wire 버퍼에 2바이트 주소 프리픽스가 같이 들어가므로 데이터 청크는 그보다 작게.
#define WIRE_BUF     256
#define CHUNK_BYTES  (WIRE_BUF - 8)

void vl53l9_esp32_bus_begin(void) {
  Wire.begin(PIN_SDA, PIN_SCL, I2C_FREQ_HZ);
  Wire.setBufferSize(WIRE_BUF);
  Wire.setTimeOut(100);
}

static inline uint8_t devAddr(void *const p_dev) {
  const vl53l9_esp32_dev_t *d = (const vl53l9_esp32_dev_t *)p_dev;
  return (d && d->address) ? d->address : TOF_I2C_ADDR_7BIT;
}

// ---------------------------------------------------------------------------
// 읽기
// ---------------------------------------------------------------------------
extern "C" int vl53l9_read(void *const p_dev, uint16_t address,
                           uint8_t *p_values, uint32_t size) {
  if (p_dev == NULL || p_values == NULL || size == 0) return VL53L9_ERROR_INVALID_PARAM;
  const uint8_t addr = devAddr(p_dev);

  uint32_t done = 0;
  while (done < size) {
    const uint32_t n = (size - done > CHUNK_BYTES) ? CHUNK_BYTES : (size - done);
    const uint16_t a = (uint16_t)(address + done);

    Wire.beginTransmission(addr);
    Wire.write((uint8_t)(a >> 8));
    Wire.write((uint8_t)(a & 0xFF));
    if (Wire.endTransmission(false) != 0) return VL53L9_ERROR_PLATFORM;

    if (Wire.requestFrom((int)addr, (int)n, (int)true) != (int)n) return VL53L9_ERROR_PLATFORM;
    for (uint32_t i = 0; i < n; i++) p_values[done + i] = Wire.read();

    done += n;
  }
  return VL53L9_ERROR_NONE;
}

extern "C" int vl53l9_read8(void *const p_dev, uint16_t address, uint8_t *p_value) {
  return vl53l9_read(p_dev, address, p_value, 1);
}

extern "C" int vl53l9_read16(void *const p_dev, uint16_t address, uint16_t *p_value) {
  return vl53l9_read(p_dev, address, (uint8_t *)p_value, 2);
}

extern "C" int vl53l9_read32(void *const p_dev, uint16_t address, uint32_t *p_value) {
  return vl53l9_read(p_dev, address, (uint8_t *)p_value, 4);
}

// 비동기 DMA 경로가 없으므로 동기 읽기로 대체한다. 드라이버는 이 함수가
// 완료된 것으로 간주하고 진행하므로 동작상 문제는 없고, 블로킹될 뿐이다.
extern "C" int vl53l9_read_async(void *const p_dev, uint16_t address,
                                 volatile uint8_t *p_values, uint32_t size) {
  return vl53l9_read(p_dev, address, (uint8_t *)p_values, size);
}

// ---------------------------------------------------------------------------
// 쓰기
// ---------------------------------------------------------------------------
extern "C" int vl53l9_write(void *const p_dev, uint16_t address,
                            uint8_t *p_values, uint32_t size) {
  if (p_dev == NULL || p_values == NULL || size == 0) return VL53L9_ERROR_INVALID_PARAM;
  const uint8_t addr = devAddr(p_dev);

  uint32_t done = 0;
  while (done < size) {
    const uint32_t n = (size - done > CHUNK_BYTES) ? CHUNK_BYTES : (size - done);
    const uint16_t a = (uint16_t)(address + done);

    Wire.beginTransmission(addr);
    Wire.write((uint8_t)(a >> 8));
    Wire.write((uint8_t)(a & 0xFF));
    if (Wire.write(p_values + done, n) != n) return VL53L9_ERROR_PLATFORM;
    if (Wire.endTransmission(true) != 0) return VL53L9_ERROR_PLATFORM;

    done += n;
  }
  return VL53L9_ERROR_NONE;
}

extern "C" int vl53l9_write8(void *const p_dev, uint16_t address, uint8_t value) {
  return vl53l9_write(p_dev, address, &value, 1);
}

extern "C" int vl53l9_write16(void *const p_dev, uint16_t address, uint16_t value) {
  return vl53l9_write(p_dev, address, (uint8_t *)&value, 2);
}

extern "C" int vl53l9_write32(void *const p_dev, uint16_t address, uint32_t value) {
  return vl53l9_write(p_dev, address, (uint8_t *)&value, 4);
}

// ---------------------------------------------------------------------------
// 기타
// ---------------------------------------------------------------------------
extern "C" int vl53l9_wait_ms(void *const p_dev, uint32_t delay_ms) {
  (void)p_dev;
  delay(delay_ms);
  return VL53L9_ERROR_NONE;
}

// 아래 셋은 보드 배선에 따라 값이 정해진다.
// STEVAL-VL53L9 스키매틱 NOTE: AVDD = 2.8V, IOVDD = 1.8V.
extern "C" int vl53l9_get_config_vdda(void *const p_dev, vl53l9_vdda_t *voltage) {
  (void)p_dev;
  if (voltage == NULL) return VL53L9_ERROR_INVALID_PARAM;
  *voltage = VDDA_2V8;
  return VL53L9_ERROR_NONE;
}

extern "C" int vl53l9_get_config_vddio(void *const p_dev, vl53l9_vddio_t *voltage) {
  (void)p_dev;
  if (voltage == NULL) return VL53L9_ERROR_INVALID_PARAM;
  *voltage = VDDIO_1V8;
  return VL53L9_ERROR_NONE;
}

// 온보드 Y1 = 12MHz. R25 를 제거해야 실제로 발진한다 (board_config.h 참고).
extern "C" int vl53l9_get_config_ext_clock(void *const p_dev, uint32_t *ext_clock) {
  (void)p_dev;
  if (ext_clock == NULL) return VL53L9_ERROR_INVALID_PARAM;
  *ext_clock = VL53L9_EXT_CLOCK_HZ;
  return VL53L9_ERROR_NONE;
}
