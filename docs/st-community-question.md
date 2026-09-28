# ST 커뮤니티 문의 초안

https://community.st.com/ → **Imaging (sensors)** 게시판. 아래 `---` 사이의 제목과
본문(영어)을 그대로 붙여넣으면 된다. 코드 블록은 게시판 편집기의 "Insert code"
로 넣으면 정렬이 유지된다.

---

**Title:** VL53L9CX on STEVAL-VL53L9: firmware faults on the first frame (ERROR_CODE 0x0F00, REF_ARRAY_ERROR, ref_amplitude = 0) — reproduced with 2 sensors and 3 independent host implementations

---

## Summary

On my STEVAL-VL53L9, the VL53L9CX boots, installs FW patch 0.17, reaches
STREAMING with all error bits clear — and then faults on the very first frame.
The reference SPAD array reports **zero amplitude on every channel**, as if the
VCSELs never fire.

I have reproduced this with **two different VL53L9CX parts** and **three
independent host implementations**, each of which is known to work on another
STEVAL-VL53L9. I have run out of things I can change from software, and I would
like to understand what the firmware error codes mean.

## Setup

- Board: STEVAL-VL53L9, VL53L9CX hand-soldered by me
- Host: ESP32 (DevKit v1) through the J2 header, plain I2C at 400 kHz, address 0x29
- Clock: on-board 12 MHz Y1 (R25 removed, R24 fitted, R23 removed); `EXT_CLOCK` = 12000000
- J3 (Host IO) = 3.3 V; `VDDA_CFG` = 2.8 V, `VDDIO_CFG` = 1.8 V
- XSHUT driven by the host, SYNC_IN tied high, INTR not used (FRAME_READY polled)
- FW patch 0.17 (9,865 bytes, byte-identical to `vl53l9_patch.h` in STSW-IMG053 / 53L9A1 BSP)

## Symptom

Default profile (54x42, SHORT context, ULTRA_LOW power, MANUAL sync, 10 ms exposure):

```
power on / load patch / BOOT / version check 0.17    OK
configure                                             OK
START_STREAM          fsm = 0x03 STREAMING, ERROR_STATUS = 0x00

TRIGGER_NEXT_FRAME    -> FRAME_READY never set
                      fsm = 0x02 STANDBY
                      ERROR_CODE   = 0x0F00
                      ERROR_STATUS = 0x80  (FW_ERROR only)
                      LDD_STATUS[0..4] = 00 00 14 00 00
                      frame_counter = 0, temperature = 35, ldd_temperature = 0
                      ref LONG/SHORT amplitude = 0 on both channels

START_STREAM again (no XSHUT cycle) -> accepted, next frame faults with
                      ERROR_CODE = 0x0008, ERROR_STATUS = 0x80, LDD_STATUS all 0
```

With CSI-2 output and the ESP32-P4 project's profile (binning 2, AUTONOMOUS,
10 ms period, 4 ms exposure, DSS off), the fault happens within 100 ms of
`START_STREAM`, and this time the reference-array check is reported explicitly:

```
fsm = 0x02 STANDBY   ERROR_CODE = 0x0F00   ERROR_STATUS = 0xC0 (FW_ERROR | REF_ARRAY_ERROR)
LDD_STATUS[0..4] = 00 00 14 00 00   frame_counter = 1
ref_amplitude = 0 on all channels
```

`I_LIMIT`, `VHV_UNDERVOLTAGE`, `VHV_OVERVOLTAGE`, `SPAD_SUPPLY_OVERLOAD`,
`PLL_LOCK` and `SOF_OUTSIDE_BLANKING` are **never** set.

## What I have ruled out

**Host software — three independent implementations, same result:**

1. A port of the ST core driver (`vl53l9.c`, core 1.0.0) with my own ESP32
   platform layer.
2. The same port configured to reproduce
   [kamibukuro5656/VL53L9CX_ESP32-P4_USB_ROS2](https://github.com/kamibukuro5656/VL53L9CX_ESP32-P4_USB_ROS2)
   exactly (works on a STEVAL-VL53L9 over MIPI CSI-2): same call order, same
   profile, same CSI-2 settings.
3. A from-scratch C++ port of
   [VanBruce/vl53l9cx-python](https://github.com/VanBruce/vl53l9cx-python)
   (works on a STEVAL-VL53L9 + Raspberry Pi 5 over plain I2C): same boot
   timing, same default profile, same frame-read procedure.

All three follow the I2C rules I know of: index write and data read as separate
transactions with a STOP in between (no repeated START), no address-only probe
transactions. A 4 KB write/read-back to 0x1800 matches exactly, the model ID
reads 0x53334C39, and the patch version check passes.

**The sensor:** I replaced the VL53L9CX with a second, previously unused part.
The calibration data returned by `vl53l9_get_calib_data()` differs between the
two (1839 vs 1806 non-zero bytes), so they are genuinely different dies. Both
fail the same way.

**The board:**

- R23 / R24 / R25 visually checked: removed / fitted / removed
- J3 on 3.3 V, SYNC_IN measured at 3.3 V
- P3V3 (VBAT_LDD / VBAT_RX supply) measured at C6/C7: 3.3 V, and it stays at
  3.3 V at the moment the frame is triggered
- AVDD 2.8 V, IOVDD 1.8 V, DVDD 1.2 V all good
- Clock: `COMMAND_SWITCH_TO_FAST_CLOCK` succeeds and a 2332-byte burst read
  works on the PLL clock; I also re-soldered the oscillator — no change
- No brownout or reset on either side at any point

**Configuration variations that did not change the outcome:** SHORT / LONG
context; binning 2 / 8 / 12; exposure 1 / 4 / 10 ms; REGULAR / ULTRA_LOW power;
MANUAL / AUTONOMOUS sync; I3C / CSI-2 output; DSS on / off.

## Questions

1. **What do `ERROR_CODE` 0x0F00, 0x0903 and 0x0008 mean?** (0x0903 appears
   with CSI-2 output and binning 12; 0x0008 after restarting the stream without
   an XSHUT cycle.) This is the one piece of information I cannot get anywhere
   else.
2. **What does `LDD_STATUS[2]` = 0x14 indicate?** It is set whenever the
   firmware gets as far as configuring the laser driver.
3. **What would an open `VBAT_LDD` (B1) joint look like?** I hand-soldered both
   parts and cannot inspect the LGA joints. Would an open B1 produce exactly
   this signature (FW_ERROR / REF_ARRAY_ERROR, no I_LIMIT or VHV bits,
   `ref_amplitude` = 0, `ldd_temperature` = 0), or would the laser driver
   report it differently?
4. **Is `CAL_TARGET_LD` (0x049C) = 0x0000 expected** after boot? The driver
   never writes it; I assume it comes from OTP.
5. Is there any initialization step that enables the laser driver and is not
   exposed through the public driver API?

## Appendix: possible issues in the ST core driver (`vl53l9.c`, core 1.0.0)

Found while porting. Separate from the question above, but may be worth a look.

1. `_init_default_config()` ends with `vl53l9_read32(..., CAB_DIST_SCALE, &data)`
   right after setting `data = 0x01000800`; it should be `vl53l9_write32`. On my
   device the register read back 0x00000000 before I changed it.
2. `vl53l9_get_status()` reads the five LDD status bytes into
   `status->laser_driver` (index 0) every time; it should be
   `&status->laser_driver[i]`.
3. Several enum locals (FSM state, sync mode, context, DSS mode) are filled with
   `vl53l9_read8()`. This works with 1-byte enums (`-fshort-enums`, the ARM
   EABI default) but leaves three bytes of stack garbage with 4-byte enums.
4. `vl53l9_set_hw_config()` masks the status-line data type with `0x2F`; the
   field is `GENMASK(5, 0)`, so bit 4 is dropped.
5. `_wait_for_state()` and `_write_cmd()` report a timeout when the expected
   state or command completion arrives on the last polling iteration.

Thanks in advance for any pointers.
