# VL53L9CX I2C 레지스터 공간 — 실측 관찰 기록

ST 문서를 구하지 못해 **실측으로 알아낸 내용**이다. 확정된 사실과 추정을
구분해서 적는다. 추정은 그대로 믿지 말 것.

측정 조건: ESP32 DevKit v1, I2C 100 kHz, 7-bit 주소 `0x29`, XSHUT HIGH,
R25 제거(Y1 12 MHz 동작). 64 KB 전 구간을 32바이트 청크로 읽었고
**읽기 실패 0 / 2048 청크**.

---

## 확정된 사실

### 주소 공간

16비트 레지스터 주소, `0x0000`~`0xFFFF` 전 구간이 응답한다. 다중 바이트
읽기가 **주소 자동증가**를 한다 (4바이트 블록 읽기 = 1바이트 4회 읽기와 일치).

### 디바이스 ID — `0x0000`

```
0x0000: 39 4C 33 53    "9L3S"   (역순 "S3L9")
```

전부 출력 가능한 ASCII이고 매 읽기마다 동일하다. `VL53L9` 에 들어 있는
`L9` 와 `53` 이 보인다. 바이트 순서 규칙은 확정하지 못했다.

**주의:** `0x010F` 은 ID 레지스터가 **아니다**. `0x00` 이 나온다. 그 주소는
VL53L5CX/L8CX 관례이고 L9 에는 해당하지 않는다.

### 전체 맵

| 범위 | 크기 | 엔트로피 | 내용 |
|---|---|---|---|
| `0x0000`-`0x000F` | 16 B | - | ID + 설정 바이트 |
| `0x0010`-`0x001F` | 16 B | 높음 | 고유값 (UID/시리얼 추정) |
| `0x0020`-`0x17FF` | ~6 KB | 0.09 | 설정 레지스터. 99% 가 0, 값은 드문드문 |
| **`0x1800`-`0x97FF`** | **32 KB** | **7.81** | **고엔트로피 블롭** |
| `0x9800`-`0x9FFF` | 2 KB | 0.00 | 전부 0 |
| `0xA000`-`0xA3FF` | 1 KB | 2.11 | 8바이트 단위 반복 테이블 |
| `0xA400`-`0xCFFF` | 11 KB | 0.00 | 전부 0 |
| `0xD000`-`0xD3FF` | 1 KB | 2.11 | 설정/상태 블록 |
| `0xD400`-`0xDBFF` | 2 KB | 0.68 | 대부분 `0xFF` |
| `0xDC00`-`0xDFFF` | 1 KB | 0.32 | 매직값 + `0xFF` 패딩 |
| `0xE000`-`0xEFFF` | 4 KB | 0.01 | 거의 전부 0 |
| `0xF000`-`0xFFFF` | 4 KB | ~0.7 | 대부분 `0xFF`, 설정 블록 두 개 |

### 32 KB 고엔트로피 블롭 — `0x1800`~`0x97FF`

엔트로피 7.81 / 8.0. 사실상 압축 불가능한 데이터다. 크기가 정확히
`0x8000` = 32768 바이트로 딱 떨어진다.

문자열 탐색에서 4자 이상 ASCII 가 394개 잡히지만 전부 무작위 일치다
(`aX%Y`, `Yc;#V` 같은 것들). **실제 텍스트는 없다.**

### 재읽기 안정성 — 65536 바이트 중 1바이트만 변한다

두 번 읽어 비교한 결과 **`0xD248` 한 바이트만 `00` → `01`** 로 변했다.
나머지 65535 바이트는 완전히 동일하다.

이건 중요하다. `0x1800`~`0x97FF` 가 미매핑 주소의 버스 노이즈였다면 읽을
때마다 값이 달라졌을 것이다. **안정적 + 고엔트로피 = 실제 저장된 데이터**다.

> **정정(2026-09-12):** 이 결론은 틀렸다. `0x1800` 은 펌웨어 패치가 올라갈
> 자리이고 패치 설치 전에는 **초기화되지 않은 SRAM** 이다. 전원 인가 후 값이
> 고정되므로 안정적으로 보였을 뿐이다. 아래 "후기" 절 참고.

### 매직 넘버 — `0xDC10`

```
0xDC10: 5E D0 FE CA  AA AA AA AA  FF FF ...
```

리틀엔디안 32비트로 읽으면 **`0xCAFED05E`**. 의도적으로 심은 매직값이고,
뒤에 `0xAA` 패딩이 붙는다.

### 자기 I2C 주소가 저장된 위치

블롭 구간을 제외하면 값 `0x29` 는 딱 세 곳에 있다:
`0xA09C`, `0xD10F`, `0xD20B`

---

## 추정 (확인 안 됨)

### `0xD248` = 32비트 카운터의 하위 바이트

```
0xD240: FF FF FF FF FF FF FF FF | 00 00 00 00 | FF FF FF FF
0xD250: FF FF FF FF | 00 00 00 00 | FF FF FF FF FF FF FF FF
```

`0xFF` 바다 가운데 32비트 0 필드가 두 개(`0xD248`, `0xD254`) 있고, 그중
앞의 것의 LSB 가 증가했다. **프레임/틱/에러 카운터로 보인다.**

지금까지 확인된 것 중 **센서 코어가 실제로 돌고 있음을 보여주는 유일한
레지스터**다. 정적 메모리만 읽고 있는 게 아니라는 증거.

### `0xD20B` = I2C 주소 레지스터

```
0xD200: 01 00 00 00 00 00 00 00 00 01 00 29 00 00 00 00
                                          ^^ 0x29
0xD210: 08 02 00 00 00 00 02 01 ...
0xD220: 96 00 00 00 ...
```

`0xD200` 블록이 통신 설정으로 보이고 그 안에 자기 주소가 들어 있다.
VL53L5CX 계열이 주소 변경 레지스터를 두는 것과 같은 패턴이다.

**검증하지 않았다.** 쓰기를 하지 않았기 때문이다. 아래 "안전" 참고.

### `0x05C0`~`0x06DF` = 스트림 설정 2세트

32비트 리틀엔디안으로 읽으면:

```
0x05D4: 1024   0x05D8: 656   0x05DC: 1640
0x05E0: 1640   0x05E4: 2064
```

그리고 **`0x0640` 블록이 `0x05C0` 블록과 완전히 동일하다** (중복 블록 탐지에서
`6806000010080000` 이 `0x05E0` 과 `0x0660` 두 곳에 나옴).

동일한 설정 2세트가 나란히 있다. VCSEL 2개, MIPI 가상 채널 2개, 또는 동작
모드 2개 중 하나로 보인다. 656 / 1640 / 2064 같은 값은 프레임 치수나 타이밍
파라미터일 가능성이 있으나 확인하지 못했다.

### `0xA000`~`0xA3FF` = (값, 해시) 쌍 테이블

8바이트 단위로 "4바이트 리틀엔디안 정수 + 4바이트 고엔트로피" 가 반복된다.

```
0xA000: 2C 2C 03 00 | 30 07 E4 1C     -> 207916 + ????
0xA008: 14 E4 03 00 | 00 EF B2 65     -> 254996 + ????
0xA010: 84 AB 00 00 | 00 FE E9 BA     ->  43908 + ????
```

정수값은 대략 43,000 ~ 255,000 범위에 분포한다. 캘리브레이션 엔트리로
보이지만 단위를 모른다.

---

## 안전

이 보드에는 **VCSEL 2개와 레이저 드라이버**가 있다. 맵을 모르는 상태에서
임의 레지스터에 쓰면 레이저 구동 설정을 건드릴 수 있다.

**위 관찰은 전부 읽기 전용으로 얻었다.** `tofWrite()` 는 API 로만 제공하고
호출하지 않는다. 맵이 확정되기 전까지 이 방침을 유지할 것.

---

## 여기서 더 나아가려면

레지스터 의미(무엇을 써야 측거가 시작되는지)는 실측만으로는 알아낼 수 없다.
현실적인 경로는 둘이다.

1. **ST 드라이버 확보** — `STSW-IMG053` / `STSW-IMG054` 에서 초기화 시퀀스를
   추출한다. 위 맵이 있으면 대조가 빠르다
2. **호스트 교체** — 측거 데이터는 MIPI CSI-2 로 나온다 (스키매틱의
   `DATA_P/N`, `CLK_P/N`). ESP32 에는 CSI 수신기가 없으므로, 실제 측거까지
   가려면 MIPI 입력이 있는 호스트가 필요하다. ST 가 명시한 지원 플랫폼은
   NUCLEO-N657X0-Q, STM32N6570-DK, Raspberry Pi, Rockchip

---

## 후기: ST 드라이버 확보 후 (2026-09-12)

`STSW-IMG053` / `STSW-IMG054` 를 입수해 대조한 결과, 위 실측 내용이 ST 정본
레지스터 맵과 일치했다.

| 실측 | ST 정의 |
|---|---|
| `0x0000` 이 ID | `VL53L9_REGADDR_MODEL_ID = 0x0000` |
| `0x1800` 에 32 KB 고엔트로피 | `VL53L9_REGADDR_FWPATCH = 0x1800` (32 kB) |
| `0xD20B` 에 `0x29` | `VL53L9_REGBASE_SOC_I2C_DEVICEID = 0xD208` |

`vl53l9_get_device_id()` 가 돌려준 값은 `0x53334C39` 로, 실측 바이트열
`39 4C 33 53` 을 리틀엔디안 32비트로 읽은 것과 정확히 같다.

### 정정: 고엔트로피 블롭은 저장된 데이터가 아니었다

위에서 "안정적 + 고엔트로피 = 실제 저장된 데이터" 라고 썼는데 **틀렸다.**

`0x1800` 은 펌웨어 패치가 올라갈 자리이고, 패치를 설치하기 전에는 **초기화되지
않은 SRAM** 이다. 전원을 넣으면 값이 고정되므로 재읽기에서도 변하지 않는다.
`vl53l9_patch.h` 의 실제 패치(9865 바이트)와 대조하니 일치율 0.4% 였다.

같은 `0x1800` 창이 상태에 따라 역할이 바뀐다:

```
READY_TO_BOOT -> FWPATCH      (32 kB)
STANDBY       -> FRAME_BUFFER (46 kB)
STREAMING     -> FB_DEPTH     (46 kB)
```

### 측거 데이터는 I2C 로 받을 수 있다

이전에 "데이터는 MIPI 로만 나오므로 ESP32 로는 불가능" 이라고 적었는데 이것도
정정한다. `vl53l9_hw_config_t.output_interface` 가 `false = CSI2`,
`true = I3C(시리얼)` 이고 **기본값이 I3C** 다. 프레임이 레지스터 공간으로
나오므로 `0x1800` 에서 읽으면 된다.

프레임 레이아웃 (`vl53l9_get_frame` 기준):

```
[0        .. res*2)    depth      zone 당 uint16, 리틀엔디안
[res*2    .. res*4)    amplitude
[res*4    .. res*6)    ambient
[res*6    .. +res/2)   DSS LUT 인덱스
[...      .. +100)     status line
```

| binning | 해상도 | 존 수 | 프레임 크기 |
|---|---|---|---|
| 2 | 54x42 | 2268 | 14842 B |
| 4 | 24x24 | 576 | 3844 B |
| 6 | 18x14 | 252 | 1738 B |
| 8 | 12x10 | 120 | 880 B |
| 12 | 8x8 | 64 | 516 B |
| 24 | 4x4 | 16 | 204 B |

### 드라이버 이식성 버그 (ESP32 포팅 중 발견)

`vl53l9.c` 가 `vl53l9_read8()` 로 **enum 지역변수**에 값을 받는 곳이 7군데
있는데, 그 변수들이 초기화되어 있지 않다.

```c
static _fsm_state_t _get_fsm_state(void *const p_dev) {
    _fsm_state_t state;                                 // 미초기화
    (void)vl53l9_read8(p_dev, ..., (uint8_t *)&state);  // 1바이트만 씀
    return state;
}
```

ARM EABI 는 `-fshort-enums` 가 기본이라 enum 이 1바이트여서 문제가 없다.
**Xtensa GCC 는 enum 이 4바이트**라 나머지 3바이트가 스택 쓰레기로 남고,
상태 비교가 영원히 실패한다. `vl53l9_init()` 이 첫 줄
`_wait_for_state(READY_TO_BOOT)` 에서 타임아웃 나는 원인이었다.

프로젝트 전체에 `-fshort-enums` 를 주는 방법은 ESP-IDF 사전컴파일
라이브러리와 ABI 가 어긋나므로 쓰지 않았다. 해당 7곳을 0 으로 초기화하는
최소 수정으로 해결했다 (`lib/vl53l9/vl53l9.c`, `// ESP32 포팅:` 주석).

### ST 드라이버에서 발견한 버그 3종

포팅 중 `lib/vl53l9/vl53l9.c` 에서 세 가지를 고쳤다. 모두 `// ESP32 포팅` 주석을 달아뒀다.

**1. 미초기화 enum (7곳) — 이식성 버그**

위 "드라이버 이식성 버그" 절 참고. ARM 은 enum 1바이트, Xtensa 는 4바이트.
`vl53l9_init()` 이 첫 줄에서 타임아웃 나던 원인이었다.

**2. `_init_default_config` 의 `read32` / `write32` 뒤바뀜**

```c
data = 0x01000800;   // short 256 - long 2048
return vl53l9_read32(p_dev, VL53L9_REGADDR_CAB_DIST_SCALE, &data);  // <- write32 여야 한다
```

값을 넣어놓고 읽기를 호출해 `data` 가 즉시 덮어써진다. 주석은 "set" 이라고
되어 있다. 실측으로 `CAB_DIST_SCALE(0xD524)` 가 `0x00000000` 인 것을 확인했다.
(이 수정만으로 아래 폴트가 해결되지는 않았지만, 명백한 버그라 유지한다.)

**3. `vl53l9_get_status` 의 LDD 인덱싱**

```c
for (uint16_t i = 0U; i < 5U; i++)
    vl53l9_read8(p_dev, VL53L9_REGADDR_LDD_STATUS(i), (uint8_t *)status->laser_driver);
```

항상 `[0]` 에 쓴다. `&status->laser_driver[i]` 여야 한다. 레이저 드라이버
상태 5바이트 중 4바이트가 쓰레기로 남는다. 진단에만 영향.

### 미해결: 측거 시작 시 펌웨어 폴트

여기까지는 동작한다.

```
vl53l9_init (패치 9865B 설치)   성공
vl53l9_get_device_id            성공   0x53334C39
설정 6종                        전부 성공
vl53l9_start                    성공   fsm=0x03(STREAMING), 에러 비트 전부 0
```

그런데 trigger 직후 **+0ms 에 즉시** 폴트가 난다.

```
폴트 포착: trigger +0ms  fsm=0x03(STREAMING)  err=0x80  code=0x0F00
          laser_driver[0..4] = 00 00 14 00 00
```

`err=0x80` 은 **`FW_ERROR`(BIT 7) 하나뿐**이다. `I_LIMIT`,
`VHV_UNDERVOLTAGE`, `VHV_OVERVOLTAGE`, `SPAD_SUPPLY_OVERLOAD`,
`PLL_LOCK`, `REF_ARRAY`, `SOF_OUTSIDE_BLANKING` 은 전부 0.

**전원·전류 문제가 아니다.** VCSEL 부하가 걸릴 시간조차 없이(+0ms) 나고,
전류·전압 관련 에러 비트가 하나도 서지 않는다. FW 가 설정 자체를 거부하는
것으로 보인다.

#### 검증해서 정상으로 확인한 것

**플랫폼 계층 청크 분할 쓰기** — 패치가 Wire 버퍼보다 커서 쪼개 보내는데,
4096바이트를 쓰고 되읽어 **완전 일치**했다. 패치는 손상되지 않는다.

**start 직전 설정 레지스터 실측** —

```
STREAM_STEP_NUMBER(SHORT, 0x04CC) = 7
NB_SHOT_STEP(1..7, 0x0504~) = 100 / 200 / 400 / 615 / 1231 / 1231 / 100
0x047A CONTEXT_SELECTION = 00 (SHORT)      0x047C SYNCHRO = 01 (MANUAL)
0x0480 FRAME_PERIOD = 33333                0x0484 OUTPUT_IF = 01 (I3C)
0x048C POWER_MODE = 00 (REGULAR)           0x04C4 STANDBY_BINNING = 08
```

전부 의도한 값이고 비어 있는 항목이 없다.

#### 바꿔도 증상이 같은 것

| 항목 | 시도한 값 |
|---|---|
| context | SHORT / LONG |
| binning | 2 / 8 |
| exposure | 1 ms / 10 ms |
| power_mode | REGULAR / ULTRA_LOW |
| sync | MANUAL / AUTONOMOUS |
| 폴링 간격 | 0 ms / 2 ms |
| `set_hw_config` | 호출 / 미호출 |

`SYNC_AUTONOMOUS` 는 자율 모드라 측거를 즉시 시작하므로 `start` 직후 바로
폴트가 난다. 즉 트리거 경로의 문제가 아니라 **측거 파이프라인이 도는 순간**의
문제다.

#### ST 앱과의 전체 대조 (빠뜨린 단계 없음)

`simple_ranging_i3c/vl53l9_app.c` 를 `platform_power_reset()` 부터 한 줄씩
대조했다.

| ST | 이 포팅 | |
|---|---|---|
| `platform_power_reset` (XSHUT LOW 50ms -> HIGH 50ms) | `enableSensor` (LOW 10ms -> HIGH 200ms) | 동등 |
| `platform_assign_dynamic_address` (I3C 일 때만) | 스킵 (I2C 라 정당) | OK |
| `vl53l9_init` | 동일 | OK |
| `vl53l9_utils_set_profile` 의 6개 호출 | 동일 6개 재현 | OK |
| `vl53l9_set_sync_mode(MANUAL)` | 동일 | OK |
| `vl53l9_start` -> `vl53l9_trigger_frame` | 동일 | OK |
| INTR GPIO 인터럽트 대기 | 폴링 (및 무통신 대기) | 증상 무관 확인 |

레지스터 실측값도 드라이버 의도와 전부 일치한다 (`FORMAT=1(WIDE)`,
`DSS_MODE=2(SHORT)`, `CONTEXT=0`, `SYNCHRO=1`, `BINNING=8`, shot 7단계).
**빠진 쓰기가 하나도 없다.**

#### 중요: ST 는 이 조합을 검증한 적이 없다

`interface/vl53l9/vl53l9_device.c` 의 디바이스 기술자:

```c
#ifdef CONFIG_HW_STEVAL_MIPI          // STEVAL-VL53L9 보드
      .bus_type = PLATFORM_BUS_I3C | PLATFORM_BUS_CSI,
      .ext_clock = 12.5e6,            // 호스트가 생성
#ifdef CONFIG_HW_X_NUCLEO             // X-NUCLEO 쉴드
      .ext_clock = 12.0e6,            // SW1 = INT (온보드 발진기)
```

**`simple_ranging_i3c` 예제는 X-NUCLEO 용이다.** STEVAL 항목에는
`PLATFORM_BUS_CSI` 가 붙어 있고 ST 가 STEVAL 용으로 제공하는 예제는 `_csi`
쪽뿐이다. 즉 **"STEVAL 보드 + 시리얼(I3C) 프레임 출력"은 ST 가 검증한 적
없는 조합이다.**

방증: 이 보드의 기본 CSI 프레임 크기가 `108 x 126 = 13608` 인데 binning 2 가
요구하는 것은 `2268 * 6 + 1134 = 14742` 다. ST 기본값으로는 최대 해상도
프레임도 담지 못한다.

(참고로 `ext_clock = 12.0e6` 자체는 정당하다. X-NUCLEO 의 온보드 발진기
설정이 12.0 MHz 이고 그게 shipped default 다.)

#### 결정적: CSI2 로 바꾸면 REF_ARRAY 에러가 드러난다

`output_interface` 를 CSI2 로 바꾸고 측거만 시험했다 (ESP32 에 CSI 수신기가
없으므로 프레임은 못 받는다. `FRAME_COUNTER(0x0028)` 와 에러 비트만 본다).

```
trigger_frame -> 성공
FRAME_COUNTER 0 -> 1                    <- 프레임 한 장 실제 획득
fsm=0x02(STANDBY)  ERROR_CODE=0x0903
error: ... ref_array=1  internal_fw=1   <- 처음 보는 비트
laser_driver[0..4] = 00 00 14 00 00
temperature = 30
```

시리얼(I3C) 모드와 결정적으로 다르다.

| | 시리얼(I3C) | CSI2 |
|---|---|---|
| FRAME_COUNTER | 증가 없음 | **0 -> 1** |
| ERROR_CODE | `0x0F00` | **`0x0903`** |
| ref_array | 0 | **1** |
| internal_fw | 1 | 1 |

**측거 파이프라인은 실제로 돈다.** 프레임 카운터가 올라가고 온도계가 30 도를
정상 보고한다. 아날로그 프런트엔드는 살아 있다.

실패 지점은 **REF_ARRAY** — VCSEL 발광을 내부 경로로 받아 기준으로 삼는
기준 SPAD 배열이다. 설정 오류가 아니라 **광학/레이저 경로의 물리 문제**를
가리킨다.

시리얼 모드는 이 단계에 도달하기 전에 죽기 때문에 그동안 이 에러가 가려져
있었다. 소프트웨어로 좁힐 수 있는 영역은 여기까지다.

물리적으로 확인할 것:

1. **센서 광학창** — 보호 필름이 남아 있는지, 먼지/지문/리워크 시 튄 플럭스나
   솔더 잔여물이 덮고 있는지
2. **리워크 열손상** — U4 를 떼면서 같은 보드의 센서 모듈이 열을 받았는지
3. 정상 보드와 대조

#### 근본 원인: VCSEL 이 발광하지 않는다

프레임 메타데이터 구조체(`vl53l9_utils.h`)는 `SENSOR_STATUS(0x0028)` 영역의
직접 오버레이다. 레지스터 맵으로 검증된다:

```
frame_counter  offset 0   = REGADDR_FRAME_COUNTER (base + 0x00)
temperature    offset 4   = REGADDR_TEMPERATURE   (base + 0x04)
ldd_temperature offset 6  = REGADDR_LDD_TEMPERATURE (base + 0x06)
error_code     offset 60  = REGADDR_ERROR_CODE    (base + 0x3C)
```

따라서 `0x0028` 에서 100바이트를 읽으면 기준 채널 진폭을 직접 볼 수 있다.
`ref_amplitude` 는 VCSEL 이 쏜 빛을 내부 기준 경로로 받은 세기다.

CSI2 모드에서 프레임 획득 직후 읽은 값:

```
ref LONG   ch1 amp=0 dist=0   | ch2 amp=0 dist=0
ref SHORT  ch1 amp=0 dist=149 | ch2 amp=0 dist=149
temperature=32   ldd_temp=0
laser_driver[0..4] = 00 00 14 00 00
frame 12x10   error_code=0x0903  error_status=0xC0
```

**VCSEL 2채널 모두 기준 진폭이 0이다.** 기준 SPAD 배열이 레이저 빛을 전혀
받지 못했다. `ref SHORT` 의 `dist=149` 는 진폭이 0 이므로 의미 없는 잡음이다.

그리고 **`ldd_temp = 0`** 인데 같은 프레임에서 센서 다이 온도는 32 도로
정상 보고된다. 동작 중인 레이저 드라이버가 0 을 보고할 수는 없다.

**결론: 레이저가 켜지지 않는다.** 이것이 `REF_ARRAY_ERROR` 의 원인이고,
측거가 실패하는 근본 이유다.

반대로 아래는 모두 정상임이 같은 프레임에서 확인된다.

- 디지털/아날로그 코어: 다이 온도 32 도 정상 보고
- 설정 도달: 메타데이터의 `frame 12x10` 이 우리가 설정한 binning 8 과 일치
- 파이프라인: 프레임 카운터 증가

#### 테스트 포인트 맵 (레벨시프터 시트)

TP 는 1~6 이 전부이고 **모두 신호용이다. 전원 TP 는 없다.**
전부 레벨시프터 B측(센서측, 1.8V)에 붙어 있다.

| TP | 신호 | 위치 |
|---|---|---|
| TP1 | SENSOR_SDA | U1 B1 (pin 13) |
| TP2 | SENSOR_SCL | U1 B2 (pin 12) |
| TP3 | SENSOR_SYNC_IN | U1 B3 (pin 11). 이 프로젝트는 미사용 |
| TP4 | SENSOR_XSHUT | U1 B4 (pin 10) |
| TP5 | **SENSOR_AP_CLK** | U2 B1 (pin 13) |
| TP6 | SENSOR_INTR | U2 B2 (pin 12) |

레벨시프터는 U1(SDA/SCL/SYNC_IN/XSHUT)과 U2(AP_CLK/INTR) 두 개이고, 둘 다
PI4ULS3V204 다. VCCA = HOST_IOVDD, VCCB = P1V8.

`LS_ENABLE`(EN, pin 8)은 R9 47k 로 HOST_IOVDD 에 풀업되어 있어 기본 활성이다.
호스트측 풀업 R7/R8 = 2.2k 도 이 시트에 있다.

쓸모 있는 지점 두 개:

- **TP5** — 센서 코앞의 AP_CLK. 12MHz 가 실제로 들어가는지 스코프로 직접 확인
- **TP6** — SENSOR_INTR. `FRAME_SIGNALING_MODE` 가 인터럽트 패드 모드이므로
  프레임 완료 시 펄스가 나와야 한다. CSI 테스트에서 프레임 카운터가 0->1 로
  올라갔으니 그때 여기에 펄스가 있었는지 확인할 수 있다

**전원 TP 가 없으므로 VBAT_LDD 확인은 C6/C7 에서 해야 한다.**

#### 확인할 하드웨어 — VBAT_LDD

스키매틱 NOTE 기준 센서 전원은 다음과 같다.

| 핀 | 이름 | 전압 | 용도 |
|---|---|---|---|
| B1 | **VBAT_LDD** | **3.3 V** | **레이저 드라이버 전원** |
| D1 | VBAT_RX | 3.3 V | 수신부 |
| E6/E7 | AVDD | 2.8 V | SPAD |
| C12 | DVDD | 1.2 V | 디지털 |
| E8 | IOVDD | 1.8 V | I/O |

**VBAT_LDD 만 레이저 드라이버 전용이다.** 이 레일이 센서 핀에서 빠져 있으면
나머지가 전부 정상이어도 레이저만 안 켜진다 — 지금 증상과 정확히 일치한다.

앞서 "3V3/2V8/1V8/1V2 LDO 출력 정상" 을 확인했지만, 그것은 **LDO 출력단**
측정이었다. LDO 출력과 센서 핀 사이가 끊겨 있을 수 있다.

측정할 곳:

1. **C6 / C7 (P3V3 의 10uF 디커플링 캡)** 양단 전압
2. 가능하면 센서 **B1(VBAT_LDD), D1(VBAT_RX)** 핀 직접
3. LDO 출력 -> C6/C7 -> 센서 핀 도통

#### 남은 미지수

**`ERROR_CODE(0x0064) = 0x0F00` 의 의미 하나다.** ST 가 FW 에러 코드표를
공개하지 않아 소프트웨어만으로는 여기서 더 좁힐 수 없다.

재현 조건이 명확하므로 ST 커뮤니티에 문의하는 것이 가장 빠르다. 위 데이터를
그대로 쓰면 된다.

하드웨어 쪽에서 아직 배제하지 못한 것:

1. **AP_CLK 품질** — Y1 이 SoC 부팅에는 충분해도 측거 파이프라인의 PLL
   요구를 못 맞출 가능성. `PLL_LOCK` 비트는 0 이지만 FW 가 먼저 죽으면
   안 세워질 수 있다. Y1 출력에 스코프를 대서 12 MHz 파형을 확인할 것
2. **U4 리워크** — 이 보드는 EEPROM 을 냉땜으로 떼어낸 이력이 있다.
   U4 는 모듈 캘리브레이션 데이터를 담고 있었을 수 있다
