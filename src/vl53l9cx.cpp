// VL53L9CX 드라이버 구현. 설계와 출처는 vl53l9cx.h 머리말 참고.

#include "vl53l9cx.h"

#include <string.h>

#include "vl53l9cx_fw_patch.h"

namespace vl53l9cx {
namespace {

// ---- 레지스터 (ST vl53l9_reg.h) ----
constexpr uint16_t REG_MODEL_ID                = 0x0000;   // u32
constexpr uint16_t REG_PATCH_REVISION          = 0x000C;   // [minor, major]
constexpr uint16_t REG_SENSOR_STATUS           = 0x0028;   // status line 100B
constexpr uint16_t REG_SYSTEM_FSM              = 0x008C;
constexpr uint16_t REG_COMMAND_ERROR           = 0x008D;
constexpr uint16_t REG_FRAME_READY             = 0x008E;   // bit0
constexpr uint16_t REG_COMMAND                 = 0x0400;   // 완료되면 0 으로 돌아온다
constexpr uint16_t REG_EXT_CLOCK               = 0x042C;   // u32 Hz
constexpr uint16_t REG_INSTALL_PATCH           = 0x0430;
constexpr uint16_t REG_VDDA_CFG                = 0x0438;   // 0 = 2.8V, 1 = 3.3V
constexpr uint16_t REG_VDDIO_CFG               = 0x0439;   // 0 = 1.2V, 1 = 1.8V
constexpr uint16_t REG_CONTEXT_SELECTION       = 0x047A;
constexpr uint16_t REG_SYNCHRO                 = 0x047C;
constexpr uint16_t REG_FORMAT                  = 0x047D;   // 0 = square, 1 = wide
constexpr uint16_t REG_FRAME_PERIOD            = 0x0480;   // u32 us
constexpr uint16_t REG_OUTPUT_IF               = 0x0484;   // 0 = CSI2, 1 = 제어 인터페이스(I2C/I3C)
constexpr uint16_t REG_FRAME_SIGNALING_MODE    = 0x048A;   // 1 = 인터럽트 패드
constexpr uint16_t REG_POWER_MODE              = 0x048C;
constexpr uint16_t REG_CAL_PROG_OFFSET_1TO5    = 0x0498;
constexpr uint16_t REG_CAL_PROG_OFFSET_6       = 0x049A;
constexpr uint16_t REG_CROP_PARAMS             = 0x04A4;
constexpr uint16_t REG_DSS_SHORT_OR_TREE_LIMIT = 0x05D8;
constexpr uint16_t REG_DSS_SHORT_AMB_WEIGHT    = 0x05E4;
constexpr uint16_t REG_DSS_LONG_OR_TREE_LIMIT  = 0x0658;
constexpr uint16_t REG_DSS_LONG_AMB_WEIGHT     = 0x0664;
constexpr uint16_t REG_FW_WINDOW               = 0x1800;   // 패치 창 / 프레임 버퍼 창
constexpr uint16_t REG_CAB_ALGO_SCALE          = 0xD520;   // ambient attenuation [27:24]
constexpr uint16_t REG_CAB_DIST_SCALE          = 0xD524;   // short [31:16], long [15:0]

inline uint16_t regStandbyBinning(uint8_t ctx) { return 0x04C4 + ctx * 4; }
inline uint16_t regStandbyDssMode(uint8_t ctx) { return 0x04C5 + ctx * 4; }
inline uint16_t regStreamBase(uint8_t ctx)     { return 0x04CC + ctx * 0x80; }

// ---- 명령 ----
constexpr uint8_t CMD_BOOT               = 0x01;
constexpr uint8_t CMD_START_STREAM       = 0x02;
constexpr uint8_t CMD_STOP_STREAM        = 0x03;
constexpr uint8_t CMD_TRIGGER_NEXT_FRAME = 0x05;
constexpr uint8_t CMD_ACK_FRAME_READ     = 0x06;
constexpr uint8_t CMD_DSS_LUT_MAP        = 0x0A;
constexpr uint8_t CMD_DSS_LUT_UNMAP      = 0x0B;

// Wire 버퍼(256B) 안에 인덱스 2B 가 같이 들어간다. 쓰기는 ST 레퍼런스 플랫폼과 같은 64B.
constexpr size_t kReadChunk  = 240;
constexpr size_t kWriteChunk = 64;

// ---- 기본 설정 표 (ST vl53l9.c _init_default_config, 값 그대로) ----
const uint8_t  kTxCh0[2][7]     = {{1, 3, 5, 11, 9, 9, 0},       {1, 3, 13, 11, 0, 11, 0}};
const uint8_t  kTxCh1[2][7]     = {{2, 4, 6, 12, 10, 10, 0},     {2, 4, 14, 12, 0, 12, 0}};
const uint16_t kBlanking[2][7]  = {{296, 116, 26, 20, 18, 18, 0}, {296, 116, 54, 20, 0, 20, 0}};
const uint8_t  kDithering[7]    = {31, 31, 31, 31, 31, 31, 0};
const uint16_t kExpoRatio[2][7] = {{100, 200, 400, 615, 1231, 1231, 100},
                                   {100, 200, 308, 615, 0, 615, 100}};

// name, cols, rows, tx_rows, y_off, binning, square
const Resolution kResolutions[] = {
    {"54x42", 54, 42, 42, 0, 2,  false},
    {"24x20", 24, 20, 24, 2, 4,  true},
    {"18x14", 18, 14, 14, 0, 6,  false},
    {"12x10", 12, 10, 10, 0, 8,  false},
    {"8x6",    8,  6,  8, 1, 12, true},
    {"4x4",    4,  4,  4, 0, 24, true},
};

inline uint16_t le16(const uint8_t *p) { return (uint16_t)(p[0] | (p[1] << 8)); }
inline uint32_t le32(const uint8_t *p) {
  return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

#define TRY(expr)                            \
  do {                                       \
    const Err _e = (expr);                   \
    if (_e != Err::Ok) return _e;            \
  } while (0)

}  // namespace

const char *errName(Err e) {
  switch (e) {
    case Err::Ok:           return "성공";
    case Err::Nack:         return "NAK (슬레이브 무응답)";
    case Err::Bus:          return "버스 오류";
    case Err::Timeout:      return "타임아웃";
    case Err::State:        return "FSM 상태 불일치";
    case Err::ModelId:      return "모델 ID 이상";
    case Err::PatchVersion: return "패치 리비전 불일치";
    case Err::Param:        return "잘못된 인자";
    case Err::NoFrame:      return "프레임 없음";
  }
  return "?";
}

const char *fsmName(uint8_t fsm) {
  switch (fsm) {
    case FSM_NONE:          return "NONE";
    case FSM_READY_TO_BOOT: return "READY_TO_BOOT";
    case FSM_STANDBY:       return "STANDBY";
    case FSM_STREAMING:     return "STREAMING";
    default:                return "?";
  }
}

const Resolution *findResolution(const char *name) {
  for (const Resolution &r : kResolutions) {
    if (strcmp(r.name, name) == 0) return &r;
  }
  return nullptr;
}

// ---------------------------------------------------------------------------
// FrameView
// ---------------------------------------------------------------------------

// 렌즈가 장면을 상하좌우로 뒤집는다. 전송 배열을 뒤집은 뒤 y_off 부터 rows 줄을 쓴다.
uint16_t FrameView::u16(int plane, int row, int col) const {
  const int tr = res_.tx_rows - 1 - (res_.y_off + row);
  const int tc = res_.cols - 1 - col;
  const size_t i = (size_t)plane * res_.zones() + (size_t)tr * res_.cols + tc;
  return le16(raw_ + i * 2);
}

const uint8_t *FrameView::statusLine() const {
  return raw_ + (size_t)res_.zones() * 6 + res_.zones() / 2;
}

uint32_t FrameView::frameCounter() const { return le32(statusLine()); }

// ---------------------------------------------------------------------------
// 전송
// ---------------------------------------------------------------------------

Err Device::read(uint16_t index, uint8_t *dst, size_t len) {
  size_t done = 0;
  while (done < len) {
    const size_t n = (len - done > kReadChunk) ? kReadChunk : (len - done);
    const uint16_t a = (uint16_t)(index + done);

    wire_.beginTransmission(addr_);
    wire_.write((uint8_t)(a >> 8));
    wire_.write((uint8_t)(a & 0xFF));
    const uint8_t rc = wire_.endTransmission(true);          // STOP. repeated START 금지
    if (rc != 0) return (rc == 2 || rc == 3) ? Err::Nack : Err::Bus;

    const size_t got = wire_.requestFrom((uint16_t)addr_, n, true);
    if (got != n || (size_t)wire_.available() < n) {
      while (wire_.available()) wire_.read();
      return (got == 0) ? Err::Nack : Err::Bus;
    }
    for (size_t i = 0; i < n; i++) dst[done + i] = (uint8_t)wire_.read();
    done += n;
  }
  return Err::Ok;
}

Err Device::write(uint16_t index, const uint8_t *src, size_t len) {
  size_t done = 0;
  while (done < len) {
    const size_t n = (len - done > kWriteChunk) ? kWriteChunk : (len - done);
    const uint16_t a = (uint16_t)(index + done);

    wire_.beginTransmission(addr_);
    wire_.write((uint8_t)(a >> 8));
    wire_.write((uint8_t)(a & 0xFF));
    if (wire_.write(src + done, n) != n) {
      wire_.endTransmission(true);
      return Err::Bus;                                       // Wire 버퍼가 작다
    }
    const uint8_t rc = wire_.endTransmission(true);
    if (rc != 0) return (rc == 2 || rc == 3) ? Err::Nack : Err::Bus;
    done += n;
  }
  return Err::Ok;
}

Err Device::read8(uint16_t index, uint8_t *v) { return read(index, v, 1); }

Err Device::read32(uint16_t index, uint32_t *v) {
  uint8_t b[4];
  TRY(read(index, b, 4));
  *v = le32(b);
  return Err::Ok;
}

Err Device::write8(uint16_t index, uint8_t v) { return write(index, &v, 1); }

Err Device::write16(uint16_t index, uint16_t v) {
  const uint8_t b[2] = {(uint8_t)v, (uint8_t)(v >> 8)};
  return write(index, b, 2);
}

Err Device::write32(uint16_t index, uint32_t v) {
  const uint8_t b[4] = {(uint8_t)v, (uint8_t)(v >> 8), (uint8_t)(v >> 16), (uint8_t)(v >> 24)};
  return write(index, b, 4);
}

// ---------------------------------------------------------------------------
// 명령 / 상태
// ---------------------------------------------------------------------------

Err Device::cmd(uint8_t command, uint32_t timeout_ms) {
  TRY(write8(REG_COMMAND, command));
  const uint32_t t0 = millis();
  for (;;) {
    uint8_t v = 0xFF;
    TRY(read8(REG_COMMAND, &v));
    if (v == 0) return Err::Ok;
    if (millis() - t0 >= timeout_ms) return Err::Timeout;
    delay(1);
  }
}

// tolerate_nak: XSHUT 상승 직후 ROM 이 부팅하는 몇 ms 동안은 NAK 가 정상이다.
Err Device::waitState(uint8_t state, uint32_t timeout_ms, bool tolerate_nak) {
  const uint32_t t0 = millis();
  for (;;) {
    uint8_t s = 0;
    const Err e = read8(REG_SYSTEM_FSM, &s);
    if (e == Err::Ok) {
      if (s == state) return Err::Ok;
    } else if (!(tolerate_nak && e == Err::Nack)) {
      return e;
    }
    if (millis() - t0 >= timeout_ms) return (e == Err::Ok) ? Err::State : Err::Timeout;
    delay(2);
  }
}

Err Device::readStatus(Status *st) {
  memset(st, 0, sizeof(*st));
  TRY(read8(REG_SYSTEM_FSM, &st->fsm));
  TRY(read8(REG_COMMAND_ERROR, &st->command_error));
  uint8_t s[kStatusLineBytes];
  TRY(read(REG_SENSOR_STATUS, s, sizeof(s)));
  st->frame_counter   = le32(s + 0);
  st->temperature     = le16(s + 4);
  st->ldd_temperature = le16(s + 6);
  for (int i = 0; i < 7; i++) {
    st->ldd_power_ch0[i] = le16(s + 8 + i * 2);    // 0x0030
    st->ldd_power_ch1[i] = le16(s + 22 + i * 2);   // 0x003E
  }
  memcpy(st->raw, s, sizeof(s));
  for (int i = 0; i < 2; i++) {
    st->ref_long_amp[i]   = le16(s + 36 + i * 4);
    st->ref_long_dist[i]  = le16(s + 38 + i * 4);
    st->ref_short_amp[i]  = le16(s + 44 + i * 4);
    st->ref_short_dist[i] = le16(s + 46 + i * 4);
  }
  st->error_code   = le16(s + 60);
  st->error_status = s[62];
  memcpy(st->ldd_status, s + 63, 5);
  return Err::Ok;
}

// ---------------------------------------------------------------------------
// 수명 주기
// ---------------------------------------------------------------------------

Err Device::powerOn() {
  if (xshut_ >= 0) {
    // ST 레퍼런스 타이밍: LOW 50ms, 해제 후 50ms. 그 뒤 NAK 를 견디며 폴링.
    pinMode(xshut_, OUTPUT);
    digitalWrite(xshut_, LOW);
    delay(50);
    digitalWrite(xshut_, HIGH);
    delay(50);
    TRY(waitState(FSM_READY_TO_BOOT, 500, true));
  } else {
    // XSHUT 제어 없이는 이전 실행이 남긴 STANDBY/STREAMING 을 READY_TO_BOOT 로 못 돌린다.
    uint8_t s = 0;
    TRY(read8(REG_SYSTEM_FSM, &s));
    if (s != FSM_READY_TO_BOOT) return Err::State;
  }
  TRY(read32(REG_MODEL_ID, &model_id_));
  if (model_id_ == 0 || model_id_ == 0xFFFFFFFFUL) return Err::ModelId;
  return Err::Ok;
}

Err Device::loadFirmware() {
  uint8_t s = 0;
  TRY(read8(REG_SYSTEM_FSM, &s));
  if (s != FSM_READY_TO_BOOT) return Err::State;

  TRY(write32(REG_EXT_CLOCK, kApClockHz));
  TRY(write8(REG_VDDIO_CFG, 1));   // IOVDD 1.8V (STEVAL)
  TRY(write8(REG_VDDA_CFG, 0));    // AVDD 2.8V (STEVAL)
  const uint32_t t0 = millis();
  TRY(write(REG_FW_WINDOW, VL53L9CX_FW_PATCH, VL53L9CX_FW_PATCH_SIZE));
  fw_load_ms_ = millis() - t0;
  return write8(REG_INSTALL_PATCH, 1);
}

Err Device::boot() {
  TRY(cmd(CMD_BOOT, 500));
  TRY(waitState(FSM_STANDBY, 100, false));
  uint8_t rev[2];
  TRY(read(REG_PATCH_REVISION, rev, 2));
  patch_minor_ = rev[0];
  patch_major_ = rev[1];
  if (patch_major_ != VL53L9CX_FW_PATCH_MAJOR || patch_minor_ != VL53L9CX_FW_PATCH_MINOR) {
    return Err::PatchVersion;
  }
  return applyDefaultConfig();
}

Err Device::applyDefaultConfig() {
  for (uint8_t ctx = CONTEXT_SHORT; ctx <= CONTEXT_LONG; ctx++) {
    const uint16_t base = regStreamBase(ctx);
    uint8_t blank[14];
    for (int i = 0; i < 7; i++) {
      blank[i * 2]     = (uint8_t)kBlanking[ctx][i];
      blank[i * 2 + 1] = (uint8_t)(kBlanking[ctx][i] >> 8);
    }
    TRY(write(base + 0x02, kTxCh0[ctx], 7));     // VCSEL_CH0_STEP(1..7)
    TRY(write(base + 0x0A, kTxCh1[ctx], 7));     // VCSEL_CH1_STEP(1..7)
    TRY(write(base + 0x14, blank, sizeof(blank)));// BLANKING_STEP(1..7)
    TRY(write(base + 0x23, kDithering, 7));      // MAX_DITHERING_STEP(1..7)
  }

  TRY(write8(REG_OUTPUT_IF, 1));                 // 프레임을 제어 인터페이스로 내보낸다
  TRY(write8(REG_FRAME_SIGNALING_MODE, 1));      // 인터럽트 패드

  TRY(write32(REG_DSS_SHORT_OR_TREE_LIMIT, 656));
  TRY(write32(REG_DSS_LONG_OR_TREE_LIMIT, 656));
  TRY(write8(REG_DSS_SHORT_AMB_WEIGHT, 12));
  TRY(write8(REG_DSS_LONG_AMB_WEIGHT, 12));

  TRY(write16(REG_CAL_PROG_OFFSET_1TO5, 1));     // short 컨텍스트 기본값
  TRY(write16(REG_CAL_PROG_OFFSET_6, 5));

  uint32_t cab = 0;
  TRY(read32(REG_CAB_ALGO_SCALE, &cab));
  cab = (cab & ~0x0F000000UL) | (4UL << 24);
  TRY(write32(REG_CAB_ALGO_SCALE, cab));
  return write32(REG_CAB_DIST_SCALE, (256UL << 16) | 2048UL);
}

Err Device::setContext(Context ctx) {
  const bool s = (ctx == CONTEXT_SHORT);
  TRY(write16(REG_CAL_PROG_OFFSET_1TO5, s ? 1 : 0xFFF8));   // long: -8
  TRY(write16(REG_CAL_PROG_OFFSET_6, s ? 5 : 0xFFFF));      // long: -1
  TRY(write32(REG_CAB_DIST_SCALE, ((uint32_t)(s ? 256 : 683) << 16) | 2048UL));
  TRY(write8(REG_CONTEXT_SELECTION, ctx));
  context_ = ctx;
  return Err::Ok;
}

Err Device::setBinning(const Resolution &res) {
  TRY(write8(regStandbyBinning(context_), res.binning));
  TRY(write8(regStandbyDssMode(context_), context_ == CONTEXT_SHORT ? 2 : 1));
  TRY(write8(REG_FORMAT, res.square ? 0 : 1));
  uint32_t crop = 0;
  if (res.tx_rows != res.rows) {                 // 정사각 전송 + 기기 쪽 크롭
    crop = (1UL << 24) | ((uint32_t)res.y_off << 18) | ((uint32_t)res.rows << 6) | res.cols;
  }
  TRY(write32(REG_CROP_PARAMS, crop));
  res_ = &res;
  return Err::Ok;
}

Err Device::setExposure(uint16_t exposure_ms) {
  if (exposure_ms < 1 || exposure_ms > 30) return Err::Param;
  uint32_t blank_sum = 0;
  for (int i = 0; i < 7; i++) {
    blank_sum += (64UL + kBlanking[context_][i]) * kExpoRatio[context_][i];
  }
  const uint32_t base = (500000UL * exposure_ms) / blank_sum;
  uint8_t shots[28];
  for (int i = 0; i < 7; i++) {
    const uint32_t v = (base * kExpoRatio[context_][i]) & 0xFFFFFFUL;
    shots[i * 4]     = (uint8_t)v;
    shots[i * 4 + 1] = (uint8_t)(v >> 8);
    shots[i * 4 + 2] = (uint8_t)(v >> 16);
    shots[i * 4 + 3] = 0;
  }
  return write(regStreamBase(context_) + 0x38, shots, sizeof(shots));   // NB_SHOT_STEP(1..7)
}

Err Device::configure(const Resolution &res, Context ctx, Power power, Sync sync,
                      uint16_t exposure_ms, uint32_t frame_period_us) {
  if (frame_period_us < 10000UL || frame_period_us > 1000000UL) return Err::Param;
  TRY(waitState(FSM_STANDBY, 100, false));
  TRY(write8(REG_SYNCHRO, sync));
  TRY(write8(REG_POWER_MODE, power));
  TRY(write32(REG_FRAME_PERIOD, frame_period_us));
  TRY(setContext(ctx));
  TRY(setBinning(res));
  return setExposure(exposure_ms);
}

Err Device::start() {
  TRY(waitState(FSM_STANDBY, 100, false));
  const Err e = cmd(CMD_START_STREAM, 200);
  if (e != Err::Ok && e != Err::Timeout) return e;
  // 저전력 모드는 명령 완료가 늦을 수 있다. STREAMING 도달로 판정한다.
  return waitState(FSM_STREAMING, (e == Err::Timeout) ? 500 : 100, false);
}

Err Device::stop() {
  TRY(cmd(CMD_STOP_STREAM, 200));
  return waitState(FSM_STANDBY, 100, false);
}

// ---------------------------------------------------------------------------
// 측거
// ---------------------------------------------------------------------------

Err Device::triggerFrame() { return cmd(CMD_TRIGGER_NEXT_FRAME, 100); }

Err Device::frameReady(bool *ready) {
  uint8_t v = 0;
  TRY(read8(REG_FRAME_READY, &v));
  *ready = (v & 1) != 0;
  return Err::Ok;
}

Err Device::waitFrame(uint32_t timeout_ms) {
  const uint32_t t0 = millis();
  for (;;) {
    bool r = false;
    TRY(frameReady(&r));
    if (r) return Err::Ok;
    if (millis() - t0 >= timeout_ms) return Err::NoFrame;
    delay(2);
  }
}

// vl53l9_get_frame 절차: 맵 3장 -> DSS LUT 매핑 후 DSS -> 언매핑 -> status line -> ACK
Err Device::readFrame(uint8_t *buf, size_t len) {
  if (res_ == nullptr || len != res_->frameBytes()) return Err::Param;
  bool r = false;
  TRY(frameReady(&r));
  if (!r) return Err::NoFrame;

  const size_t zones = res_->zones();
  size_t off = 0;
  TRY(read(REG_FW_WINDOW, buf + off, zones * 6));
  off += zones * 6;
  TRY(cmd(CMD_DSS_LUT_MAP, 100));
  TRY(read(REG_FW_WINDOW, buf + off, zones / 2));
  off += zones / 2;
  TRY(cmd(CMD_DSS_LUT_UNMAP, 100));
  TRY(read(REG_SENSOR_STATUS, buf + off, kStatusLineBytes));
  return cmd(CMD_ACK_FRAME_READ, 100);   // INTR 해제
}

}  // namespace vl53l9cx
