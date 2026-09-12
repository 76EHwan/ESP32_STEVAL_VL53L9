# ST 커뮤니티 문의 초안

https://community.st.com/ → Imaging (sensors) 게시판에 그대로 붙여넣으면 된다.
제목과 본문은 영어로 준비했다.

---

**Title:** VL53L9CX: FW_ERROR (ERROR_CODE 0x0F00) immediately on first frame trigger, STEVAL-VL53L9 over I2C

---

I am running the VL53L9CX on a STEVAL-VL53L9 board with a non-ST host (ESP32,
plain I2C at 400 kHz, 7-bit address 0x29). I ported the `drivers/vl53l9`
sources from STSW-IMG053 and implemented the `vl53l9_platform.h` functions.

Everything works up to and including `vl53l9_start()`. The device reaches
STREAMING with no error bits. But the moment ranging actually starts, the
firmware faults and the device drops back to STANDBY.

## What works

```
vl53l9_init()            OK   (FW patch 9865 B installed, version check passes)
vl53l9_get_device_id()   OK   0x53334C39
vl53l9_get_calib_data()  OK   2332 B, 1839 non-zero
vl53l9_set_power_mode / set_frame_period / set_context /
  set_binning / set_exposure / set_sync_mode        all OK
vl53l9_start()           OK   fsm = 0x03 (STREAMING), all error bits clear
```

Note `vl53l9_get_calib_data()` succeeds, which means
`COMMAND_SWITCH_TO_FAST_CLOCK` works and the PLL comes up fine.

## The failure

Polling `vl53l9_get_status()` every 1 ms right after `vl53l9_trigger_frame()`:

```
trigger + 0 ms   fsm = 0x03 (STREAMING)   ERROR_STATUS = 0x80   ERROR_CODE = 0x0F00
                 LDD_STATUS[0..4] = 00 00 14 00 00
```

Then the device transitions to STANDBY (fsm = 0x02) and every subsequent
`vl53l9_trigger_frame()` returns `VL53L9_ERROR_INVALID_STATE`.

`ERROR_STATUS = 0x80` is **only** `VL53L9_REGFIELD_FW_ERROR` (BIT 7).
All of `I_LIMIT`, `VHV_UNDERVOLTAGE`, `VHV_OVERVOLTAGE`,
`SPAD_SUPPLY_OVERLOAD`, `PLL_LOCK`, `REF_ARRAY` and
`SOF_OUTSIDE_BLANKING` are clear. The fault occurs at +0 ms, before any
meaningful VCSEL load, so this does not look like a supply/current issue.

**My question: what does ERROR_CODE (0x0064) = 0x0F00 mean?** Is there a
published list of firmware error codes? That is the only piece of information
I am missing.

## Configuration actually present in the device, read back before `vl53l9_start()`

```
0x04CC STREAM_STEP_NUMBER(SHORT) = 7
0x0504 NB_SHOT_STEP(1..7, SHORT) = 100 / 200 / 400 / 615 / 1231 / 1231 / 100
0x047A CONTEXT_SELECTION = 0x00 (SHORT)
0x047C SYNCHRO           = 0x01 (MANUAL)
0x0480 FRAME_PERIOD      = 33333
0x0484 OUTPUT_IF         = 0x01 (I3C / serial)
0x048C POWER_MODE        = 0x00 (REGULAR)
0x04C4 STANDBY_BINNING   = 0x08
```

## Root cause narrowed: the VCSELs do not emit

Switching `output_interface` to CSI2 (purely as a bisect - the host has no CSI
receiver, I only read `FRAME_COUNTER` and the error bits) gets much further and
exposes a different error:

```
trigger_frame -> OK
FRAME_COUNTER 0 -> 1                     <- a frame IS acquired
fsm = 0x02 (STANDBY)   ERROR_CODE = 0x0903
ERROR_STATUS = 0xC0  ->  FW_ERROR | REF_ARRAY_ERROR
```

Reading the 100-byte status line at `VL53L9_REGBASE_SENSOR_STATUS` (0x0028),
which the frame metadata struct overlays directly:

```
frame_counter = 1     temperature = 32     ldd_temperature = 0
ref LONG   ch1 amp=0 dist=0     ch2 amp=0 dist=0
ref SHORT  ch1 amp=0 dist=149   ch2 amp=0 dist=149
frame 12x10   (matches the configured binning 8)
LDD_STATUS[0..4] = 00 00 14 00 00
```

**`ref_amplitude` is 0 on both VCSEL channels in both contexts.** The reference
SPAD array receives no light at all. `ldd_temperature` reads 0 while the die
temperature reads a sane 32 C in the same frame.

So the digital core, the configuration path and the ranging pipeline all work -
the laser simply does not fire.

## Hardware checked

- `VBAT_LDD` / `VBAT_RX` rail (P3V3) measured at C6/C7: **3.3 V, good**
- AVDD 2.8 V, DVDD 1.2 V, IOVDD 1.8 V: all good
- PLL: `vl53l9_get_calib_data()` succeeds, so `COMMAND_SWITCH_TO_FAST_CLOCK`
  works
- On-chip calibration: 2332 bytes read, 1839 non-zero
- On-board 12 MHz oscillator in use (R25 removed, R24 fitted)

One observation I cannot interpret: `CAL_TARGET_LD` (0x049C) reads
**0x0000**, while the adjacent `CAL_RTN_OFFSET` (0x0497) is 0x20. The driver
never writes `CAL_TARGET_LD`, so I assume it is loaded from OTP at boot. Is 0 a
valid value here, or does it indicate the laser drive target failed to load?

**Questions:**

1. What does `ERROR_CODE` 0x0903 (and 0x0F00 in I3C output mode) mean?
2. Is `CAL_TARGET_LD` = 0 expected?
3. `ref_amplitude` = 0 on both channels with all supplies good - does this
   indicate a failed part, or is there a configuration step that enables the
   laser driver that I am missing?

Note: this board has had its EEPROM (U4) desoldered, so heat damage to the
module cannot be excluded.

## What I already ruled out

Changing any of these does not change the symptom:

| Parameter | Values tried |
|---|---|
| context | SHORT, LONG |
| binning | 2, 8 |
| exposure | 1 ms, 10 ms |
| power_mode | REGULAR, ULTRA_LOW |
| sync mode | MANUAL, AUTONOMOUS |
| poll interval | 0 ms, 2 ms |
| `vl53l9_set_hw_config()` | called, not called |

With `SYNC_AUTONOMOUS` the fault happens immediately after `vl53l9_start()`
instead of on trigger, which is consistent: autonomous mode starts ranging
right away. So this is not specific to the trigger path.

I also verified my platform layer: writing 4096 bytes across several chunk
boundaries to 0x1800 and reading them back gives an exact match, so the
firmware patch is not being corrupted by my chunked I2C transfers.

## Board notes

- On-board 12 MHz oscillator Y1 in use (R25 removed, R24 fitted),
  `ext_clock` reported as 12000000
- VDDA = 2V8, VDDIO = 1V8 per the schematic note
- The EEPROM U4 has been physically removed from this board

## Possible driver issues I found while porting

These are separate from the question above, but may be worth checking:

**1. `_init_default_config()` — `read32` where `write32` is intended**

```c
// set cab_dist_scale according to default context selection (short)
data = 0x01000800; // short 256 - long 2048
return vl53l9_read32(p_dev, VL53L9_REGADDR_CAB_DIST_SCALE, &data);
```

`data` is overwritten immediately, so CAB_DIST_SCALE is never written.
I read back 0x00000000 from 0xD524 on my device.

**2. `vl53l9_get_status()` — LDD status always written to index 0**

```c
for (uint16_t i = 0U; i < 5U; i++) {
    ret = vl53l9_read8(p_dev, VL53L9_REGADDR_LDD_STATUS(i), (uint8_t *)status->laser_driver);
}
```

Should be `&status->laser_driver[i]`.

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
remaining three bytes are stack garbage, so the state comparison never
matches. Zero-initializing the locals fixes it.
