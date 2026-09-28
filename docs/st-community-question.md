# ST 커뮤니티 문의 초안

https://community.st.com/ → Imaging (sensors) 게시판에 그대로 붙여넣으면 된다.
제목과 본문은 영어로 준비했다.

---

**Title:** VL53L9CX: ref_amplitude = 0 on all channels, ERROR_CODE 0x0903 / 0x0F00 on first frame, reproduced on two parts — VCSELs appear not to emit (STEVAL-VL53L9, I2C host)

---

I am running the VL53L9CX on a STEVAL-VL53L9 board with a non-ST host (ESP32,
plain I2C at 400 kHz, 7-bit address 0x29). I ported the `drivers/vl53l9`
sources from STSW-IMG053 and implemented the `vl53l9_platform.h` functions.

Everything works up to and including `vl53l9_start()`. The device reaches
STREAMING with every error bit clear. But the moment ranging starts, the
firmware faults and the device drops back to STANDBY, and the reference SPAD
array reports **zero received amplitude on every channel**.

**I have now replaced the VL53L9CX with a second, previously unused part and
the symptom is identical.** Details in "Two parts, same failure" below.

## What works

```
vl53l9_init()            OK   (FW patch 9865 B installed, version check passes)
vl53l9_get_device_id()   OK   0x53334C39
vl53l9_get_calib_data()  OK   2332 B (content differs per part, see below)
vl53l9_set_sync_mode / set_power_mode / set_frame_period / set_context /
  set_binning / set_exposure                          all OK
vl53l9_start()           OK   fsm = 0x03 (STREAMING), all error bits clear
```

`vl53l9_get_calib_data()` succeeds, which means `COMMAND_SWITCH_TO_FAST_CLOCK`
works and the PLL comes up.

I also verified my platform layer: writing 4096 bytes across several chunk
boundaries to 0x1800 and reading them back gives an exact match, so the
firmware patch is not being corrupted by my chunked I2C transfers.

## The failure

Current configuration is binning 12 (8x8, 64 zones), SHORT context, 10 ms
exposure, MANUAL sync, 30 fps frame period.

```
trigger_frame            OK
FRAME_COUNTER 0 -> 1                      <- a frame IS acquired
fsm = 0x02 (STANDBY)     ERROR_CODE = 0x0903

error bits:  vhv_ov=0  vhv_uv=0  spad_ovl=0  hvboost(I_LIMIT)=0
             sof_blank=0  pll_lock=0  ref_array=0/1  internal_fw=1

LDD_STATUS[0..4]   = 00 00 00 00 00      (sometimes 00 00 14 00 00)
frame_counter = 1  temperature = 32      ldd_temperature = 0
ref LONG   ch1 amp=0 dist=0     ch2 amp=0 dist=0
ref SHORT  ch1 amp=0 dist=149   ch2 amp=0 dist=149
frame 8x8          ERROR_STATUS = 0x80   (sometimes 0xC0)
```

Every subsequent `vl53l9_trigger_frame()` then returns
`VL53L9_ERROR_INVALID_STATE` until the device is stopped and restarted.

`ERROR_STATUS` is either 0x80 (`FW_ERROR` alone) or 0xC0
(`FW_ERROR | REF_ARRAY_ERROR`). **Bits 0-5 are always clear** — no
`I_LIMIT`, no `VHV_UNDERVOLTAGE`, no `VHV_OVERVOLTAGE`, no
`SPAD_SUPPLY_OVERLOAD`, no `PLL_LOCK`, no `SOF_OUTSIDE_BLANKING`.

In I3C (serial) output mode the code is `0x0F00` instead of `0x0903`, and the
frame counter does not advance.

**`ref_amplitude` is 0 on both channels in both contexts.** The reference SPAD
array receives no light at all, while `ldd_temperature` reads 0 and the die
temperature reads a sane value in the same frame.

## Two parts, same failure

I removed the original part and soldered a second, previously unused
VL53L9CX onto the same board. The calibration data read by
`vl53l9_get_calib_data()` differs between the two, so these are genuinely
different dies:

| | Part A (original) | Part B (replacement) |
|---|---|---|
| calib data, non-zero / 0xFF bytes | 1839 / 117 | 1806 / 122 |
| calib data FNV-1a hash | not recorded | `08B04953` |
| CSI2 output mode, binning 12 / MANUAL | `0x0903`, ERROR_STATUS 0x80 **or** 0xC0, varies run to run | `0x0903`, ERROR_STATUS **always 0x80** |
| CSI2 output mode, binning 2 / AUTONOMOUS | — | `0x0F00`, ERROR_STATUS **always 0xC0** (REF_ARRAY) |
| CSI2 LDD_STATUS[0..4] | `00 00 00 00 00` or `00 00 14 00 00` | always `00 00 00 00 00` |
| I3C output mode | `0x0F00`, frame counter stays 0 | `0x0F00`, frame counter stays 0, LDD_STATUS `00 00 14 00 00` |
| ref_amplitude, all channels | 0 | 0 |
| die temperature during test | 30-80 °C | 30-47 °C, no effect |

Part B is fully deterministic: over several boots and 100+ trigger attempts,
every frame ends with the same values, bit for bit. Part A's
0x80 / 0xC0 variation is gone.

Two different dies failing the same way points away from the sensor itself and
towards something common to both: the board, the host-side configuration, or
the way I assemble the part.

## Configuration read back from the device before `vl53l9_start()`

```
0x04CC STREAM_STEP_NUMBER(SHORT)      = 7
0x04CD VCSEL_CH0_STEP(0..7, SHORT)    = 20 01 03 05 0B 09 09 00
0x04D5 VCSEL_CH1_STEP(0..7, SHORT)    = 20 02 04 06 0C 0A 0A 00
0x0504 NB_SHOT_STEP(1, SHORT)         = 1300
0x047A CONTEXT_SELECTION              = 0x00 (SHORT)
0x047C SYNCHRO                        = 0x01 (MANUAL)
0x0480 FRAME_PERIOD                   = 33333
0x0484 OUTPUT_IF                      = 0x01 (I3C / serial)
0x048C POWER_MODE                     = 0x00 (REGULAR)
0x04C4 STANDBY_BINNING                = 0x0C (12)
```

The per-step VCSEL registers are populated with non-zero values, so the laser
drive configuration does appear to be loaded.

One value I cannot interpret: `CAL_TARGET_LD` (0x049C) reads **0x0000**, while
the adjacent `CAL_RTN_OFFSET` (0x0497) is 0x20. The driver never writes
`CAL_TARGET_LD`, so I assume it is loaded from OTP at boot.

## Hardware checked

- `VBAT_LDD` / `VBAT_RX` rail (P3V3) measured at C6/C7: **3.3 V, good**
- AVDD 2.8 V, DVDD 1.2 V, IOVDD 1.8 V: all good
- On-board 12 MHz oscillator in use (R25 removed, R24 fitted, R23 removed);
  `ext_clock` reported as 12000000
- `pll_lock` error bit is clear, so the device itself confirms the clock and
  PLL are healthy
- No brownout or reset of either the host or the sensor at any point

## Things I tested that did NOT change the symptom

| Test | Result |
|---|---|
| context SHORT / LONG | no change |
| binning 2 / 8 / 12 | no change |
| exposure 1 ms / 10 ms | no change |
| power_mode REGULAR / ULTRA_LOW | no change |
| sync MANUAL / AUTONOMOUS | fault moves to `start()`, consistent |
| `vl53l9_set_hw_config()` called / not called | no change |
| output_interface I3C / CSI2 | different error code, same root symptom |
| `set_sync_mode` first vs last in the profile sequence | no change |
| Mechanical pressure on the package at room temperature | no change |
| Supply current, STANDBY vs. repeated frame attempts | no measurable delta |
| **Replace the VL53L9CX with a second, unused part** | **no change** |

On the current measurement: I compared the board's supply current while idle in
STANDBY against 10 s of continuously retried frames. I could not resolve any
difference. I am aware this test is weak — since the firmware aborts early, a
healthy part may also draw very little here — so I do not read much into it.

I also heated the part and saw `ref_amplitude` rise to small non-zero values
(2-24) at 90-130 °C. I no longer believe this was real signal: `LDD_STATUS[2]`
was 0x00 during those runs and the reported distances were uncorrelated noise,
so I attribute it to SPAD dark count rising with temperature.

## Comparison against a known-working implementation

I compared my port against a working third-party VL53L9CX project
(kamibukuro5656/VL53L9CX_ESP32-P4_USB_ROS2, ESP32-P4 over MIPI CSI-2). The
platform callbacks return identical values to mine — `VDDA_2V8`, `VDDIO_1V8`,
`ext_clock = 12000000` — and the initialization call order matches. That
project also uses the on-board oscillator with R23/R25 removed. So I do not
believe this is a configuration mistake on my side.

I then went further and made my host reproduce that project exactly:

- same FW patch (v0.17, byte-identical `vl53l9_patch.h`) and same driver core
- I2C reads as two transactions with a STOP between the index write and the
  data read (no repeated START), as that project and ST's reference platform do
- no address-only probe transactions on the bus
- same call order: `init` → `get_calib_data` → profile → `set_hw_config` → `start`
- same profile: CSI-2, AUTONOMOUS, 10 ms frame period, 4 ms exposure,
  binning 2, DSS disabled, SHORT context, REGULAR power
- same CSI-2 settings: 1000 Mbps, VC 0, data type 0x2A for frame and status
  line, 100-byte lines, height 148

Result, 100 ms after `vl53l9_start()`:

```
fsm = 0x02 (STANDBY)   ERROR_CODE = 0x0F00   ERROR_STATUS = 0xC0
error bits: ref_array=1 internal_fw=1, all others 0
LDD_STATUS[0..4] = 00 00 14 00 00
frame_counter = 1   temperature = 31
ref_amplitude = 0 on all channels
```

So with a configuration that works on another STEVAL-VL53L9, both of my parts
fail the reference-array check.

## Questions

1. **What does `ERROR_CODE` 0x0903 mean?** (and 0x0F00 in I3C output mode) Is
   there a published list of firmware error codes? This is the single piece of
   information I am missing.
2. **Is `CAL_TARGET_LD` = 0 expected?** If this is the laser drive target and it
   failed to load from OTP, that would explain everything.
3. **Is there an initialization step that enables the laser driver** that the
   public driver API does not expose, and that I might be missing?
4. `ref_amplitude` = 0 on all channels with every supply rail good and every
   supply-related error bit clear — **does this indicate a failed part?** Given
   that two different parts behave identically, I now suspect a common cause
   rather than two bad parts.
5. **What would an open `VBAT_LDD` (B1) connection look like?** Both parts were
   hand-soldered onto the same board and I cannot inspect the LGA joints
   (no X-ray). If B1 were not connected, would the firmware report exactly this
   (`FW_ERROR` alone, no `I_LIMIT` / VHV bits, `ref_amplitude` = 0), or is there
   a specific error bit or status register that would flag a missing laser
   driver supply?
6. **Is plain I2C (I3C legacy mode) an officially supported host interface for
   the VL53L9CX?** The product material I can find lists only MIPI I3C and MIPI
   CSI-2. My host enumerates both 0x29 and 0x7E on the bus and all register
   access works reliably, but I would like to know whether there are documented
   restrictions when the device is driven this way.

## Board notes

- I assembled the VL53L9CX onto this STEVAL-VL53L9 myself. Part A was reworked
  several times with hot air, so thermal damage to it cannot be excluded.
  Part B was soldered once, by the same hand-soldering method, on the same
  board.
- The EEPROM U4 has been physically removed from this board. As far as I can
  tell the driver never writes calibration data back to the sensor
  (`vl53l9_get_calib_data()` has no setter), so I do not think this matters.

## Possible driver issues I found while porting

These are separate from the question above, but may be worth checking.

**1. `_init_default_config()` — `read32` where `write32` is intended**

```c
// set cab_dist_scale according to default context selection (short)
data = 0x01000800; // short 256 - long 2048
return vl53l9_read32(p_dev, VL53L9_REGADDR_CAB_DIST_SCALE, &data);
```

`data` is overwritten immediately, so CAB_DIST_SCALE is never written. I read
back 0x00000000 from this register on my device before fixing it.

**2. `vl53l9_get_status()` — LDD status always written to index 0**

```c
for (uint16_t i = 0U; i < 5U; i++) {
    ret = vl53l9_read8(p_dev, VL53L9_REGADDR_LDD_STATUS(i), (uint8_t *)status->laser_driver);
}
```

Should be `&status->laser_driver[i]`. As written, indices 1-4 are left as
uninitialized stack.

**3. Uninitialized enum locals read via `vl53l9_read8()`**

Seven places, e.g.

```c
static _fsm_state_t _get_fsm_state(void *const p_dev) {
    _fsm_state_t state;
    (void)vl53l9_read8(p_dev, VL53L9_REGADDR_SYSTEM_FSM, (uint8_t *)&state);
    return state;
}
```

This works on ARM because the EABI defaults to `-fshort-enums` (1-byte enums),
but on a toolchain with 4-byte enums only the low byte is written and the
remaining three bytes are stack garbage, so the state comparison never matches.
Zero-initializing the locals fixes it.
