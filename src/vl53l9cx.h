// ---------------------------------------------------------------------------
// VL53L9CX 드라이버 — ESP32 Arduino / Wire, 순수 I2C
//
// VanBruce/vl53l9cx-python (BSD-3-Clause) 을 C++ 로 옮겼다. 그 드라이버는
// STEVAL-VL53L9 + Raspberry Pi 5 에서 I2C 만으로 54x42 프레임을 받는 것이
// 검증됐다. 레지스터 맵과 시퀀스의 원 출처는 ST 53L9A1 BSP (core 1.0.0,
// fw patch 0.17) 다.
//
// 사용 순서:
//   powerOn() -> loadFirmware() -> boot() -> configure() -> start()
//   -> [triggerFrame() -> waitFrame() -> readFrame()] -> stop()
//
// 전송 규칙 (어기면 센서가 죽은 것처럼 보인다):
//   * 인덱스 16bit 빅엔디안, 데이터 리틀엔디안, 주소 자동 증가
//   * 인덱스 쓰기와 데이터 읽기는 반드시 별개 트랜잭션 (사이에 STOP).
//     repeated START 로 읽으면 STOP 을 볼 때까지 모든 전송을 NAK 한다.
//   * 주소만 보내는 빈 트랜잭션(프로브) 금지. 같은 이유로 버스가 막힌다.
// ---------------------------------------------------------------------------

#pragma once

#include <Arduino.h>
#include <Wire.h>

namespace vl53l9cx {

constexpr uint8_t  kDefaultAddress   = 0x29;
constexpr uint32_t kApClockHz        = 12000000UL;   // STEVAL 온보드 Y1
constexpr size_t   kStatusLineBytes  = 100;

enum class Err : int8_t {
  Ok = 0,
  Nack,          // 슬레이브가 주소/데이터에 NAK
  Bus,           // 그 밖의 버스 오류, 짧은 읽기
  Timeout,       // 명령 완료 또는 FSM 전이 대기 초과
  State,         // FSM 이 기대한 상태가 아님
  ModelId,       // 모델 ID 가 0 또는 0xFFFFFFFF
  PatchVersion,  // 부팅 후 패치 리비전 불일치
  Param,
  NoFrame,       // FRAME_READY 가 서지 않음
};
const char *errName(Err e);

enum Fsm : uint8_t { FSM_NONE = 0, FSM_READY_TO_BOOT = 1, FSM_STANDBY = 2, FSM_STREAMING = 3 };
enum Sync : uint8_t { SYNC_SLAVE = 0, SYNC_MANUAL = 1, SYNC_AUTONOMOUS = 2 };
enum Power : uint8_t { POWER_REGULAR = 0, POWER_LOW = 1, POWER_ULTRA_LOW = 2 };
enum Context : uint8_t { CONTEXT_SHORT = 0, CONTEXT_LONG = 1 };
const char *fsmName(uint8_t fsm);

// cols x rows 가 실제 화면. 정사각 전송 모드는 tx_rows 줄을 보내고 기기가 잘라낸다.
struct Resolution {
  const char *name;
  uint8_t cols, rows, tx_rows, y_off, binning;
  bool square;                         // FORMAT: 1 = square, 0 = wide
  uint16_t zones() const { return (uint16_t)cols * tx_rows; }
  // depth + amplitude + ambient (u16) + DSS (4bit) + status line
  size_t frameBytes() const { return (size_t)zones() * 6 + zones() / 2 + kStatusLineBytes; }
};
const Resolution *findResolution(const char *name);   // "54x42" "24x20" "18x14" "12x10" "8x6" "4x4"

// status line (0x0028 부터 100B) + FW 상태 레지스터
struct Status {
  uint8_t  fsm;
  uint8_t  command_error;
  uint32_t frame_counter;
  uint16_t temperature;
  uint16_t ldd_temperature;
  uint16_t ref_long_amp[2],  ref_long_dist[2];
  uint16_t ref_short_amp[2], ref_short_dist[2];
  uint16_t error_code;
  uint8_t  error_status;               // bit7 FW, bit6 REF_ARRAY, bit5 PLL, bit4 SOF, bit3 I_LIMIT ...
  uint8_t  ldd_status[5];
};

// readFrame() 이 채운 원시 버퍼를 장면 방향(렌즈 반전 해제, 크롭 적용)으로 본다.
class FrameView {
 public:
  FrameView(const uint8_t *raw, const Resolution &res) : raw_(raw), res_(res) {}
  uint16_t depthMm(int row, int col) const { return u16(0, row, col) & 0x7FFF; }
  bool     valid(int row, int col) const { return (u16(0, row, col) >> 15) != 0; }
  uint16_t amplitude(int row, int col) const { return u16(1, row, col); }
  uint16_t ambient(int row, int col) const { return u16(2, row, col); }
  uint32_t frameCounter() const;
  const uint8_t *statusLine() const;

 private:
  uint16_t u16(int plane, int row, int col) const;
  const uint8_t *raw_;
  const Resolution &res_;
};

class Device {
 public:
  Device(TwoWire &wire, uint8_t address = kDefaultAddress, int xshut_pin = -1)
      : wire_(wire), addr_(address), xshut_(xshut_pin) {}

  // 수명 주기
  Err powerOn();                 // XSHUT 사이클 -> READY_TO_BOOT, 모델 ID 확인
  Err loadFirmware();            // 부트 설정 + 패치 업로드 (READY_TO_BOOT 에서만)
  Err boot();                    // BOOT -> STANDBY, 패치 리비전 확인, 기본 설정
  Err configure(const Resolution &res, Context ctx, Power power, Sync sync,
                uint16_t exposure_ms, uint32_t frame_period_us);
  Err start();
  Err stop();

  // 측거
  Err triggerFrame();
  Err frameReady(bool *ready);
  Err waitFrame(uint32_t timeout_ms);
  Err readFrame(uint8_t *buf, size_t len);   // len == res.frameBytes()
  Err readStatus(Status *st);

  uint32_t modelId() const { return model_id_; }
  uint8_t  patchMajor() const { return patch_major_; }
  uint8_t  patchMinor() const { return patch_minor_; }
  uint32_t firmwareLoadMs() const { return fw_load_ms_; }

  // 전송 (진단용으로 공개)
  Err read(uint16_t index, uint8_t *dst, size_t len);
  Err write(uint16_t index, const uint8_t *src, size_t len);
  Err read8(uint16_t index, uint8_t *v);
  Err read32(uint16_t index, uint32_t *v);
  Err write8(uint16_t index, uint8_t v);
  Err write16(uint16_t index, uint16_t v);
  Err write32(uint16_t index, uint32_t v);

 private:
  Err cmd(uint8_t command, uint32_t timeout_ms);
  Err waitState(uint8_t state, uint32_t timeout_ms, bool tolerate_nak);
  Err applyDefaultConfig();
  Err setContext(Context ctx);
  Err setBinning(const Resolution &res);
  Err setExposure(uint16_t exposure_ms);

  TwoWire &wire_;
  uint8_t addr_;
  int xshut_;
  Context context_ = CONTEXT_SHORT;
  const Resolution *res_ = nullptr;
  uint32_t model_id_ = 0;
  uint8_t patch_major_ = 0, patch_minor_ = 0;
  uint32_t fw_load_ms_ = 0;
};

}  // namespace vl53l9cx
