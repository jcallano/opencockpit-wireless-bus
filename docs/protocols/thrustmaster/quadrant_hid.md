# TCA Airbus Quadrant + Captain Pack Add-Ons (VID 0x044F, PID 0x0407)
HID Input Report Reverse Engineering (ESP32-S3 USB Host)

This document describes the observed HID Interrupt IN input report for the Thrustmaster TCA Airbus Quadrant when used with the Captain Pack / Add-Ons.

Source of truth: real dumps captured with an ESP32-S3 (WeAct ESP32-S3 Dev Board Rev A, Arduino Core 3.3.5, ESP-IDF USB Host stack via `usb/usb_host.h`).

IMPORTANT:
- All mappings labeled **Confirmed** were observed with clean before/after deltas and repeatable behavior.
- Items labeled **Observed** are consistent in dumps but still benefit from more boundary tests.
- Items labeled **Unknown** never changed in provided captures (cannot be honestly mapped yet).
- Multi-byte fields are little-endian.

---

## Report Layout (validated)

Offset | Size | Field        | Type        | Notes
------ | ---- | ------------ | ----------- | -----
0      | 1    | reportId     | uint8       | Report ID (observed constant during tests)
1      | 1    | B01          | uint8 bits  | Engine masters, A/THR disconnect, ENG mode, ENG fire
2      | 1    | B02          | uint8 bits  | Throttle detents (virtual detents) + reverse zone flags
3      | 2    | throttle_1   | uint16 LE   | T1 axis
5      | 2    | throttle_2   | uint16 LE   | T2 axis
7      | 2    | flaps        | uint16 LE   | Flaps axis
9      | 1    | B09          | uint8 bits  | Parking brake + rudder trim buttons
10     | 2    | speedbrake   | uint16 LE   | Speedbrake/spoilers axis
12     | 1    | B12          | uint8 bits  | Gear, autobrake selector, speedbrake latch/flags
13     | 1    | B13          | uint8 bits  | Speedbrake detent markers
14     | 1    | B14          | uint8       | Observed constant 0xFF in all captures (unknown/reserved)

Total (struct): 15 bytes (as decoded by the firmware)

---

## Byte/Bit Mappings

### B01 (offset 1) — Engine controls / A/THR disconnect / ENG mode / ENG fire
Bit | Meaning | Type | Active meaning | Status
--- | ------- | ---- | ------------- | ------
0 | A/THR Disconnect (Left) | momentary | 1 = pressed | Confirmed
1 | A/THR Disconnect (Right) | momentary | 1 = pressed | Confirmed
2 | Engine Master L (ENG 1) | latching | 1 = ON | Confirmed
3 | Engine Master R (ENG 2) | latching | 1 = ON | Confirmed
4 | ENG FIRE Left | momentary | 1 = pressed | Confirmed
5 | ENG FIRE Right | momentary | 1 = pressed | Confirmed
6 | ENG MODE selector = CRANK | selector | 1 = CRANK | Confirmed
7 | ENG MODE selector = IGN | selector | 1 = IGN | Confirmed

ENG MODE selector = NORM is represented by bit6=0 and bit7=0 (Confirmed).

---

### B02 (offset 2) — Throttle virtual detents + reverse zone flags (mirrored groups)
This byte contains two mirrored one-hot detent groups (T1 and T2) plus reverse zone flags.

Bit | Meaning | Notes | Status
--- | ------- | ----- | ------
0 | T1 detent = TOGA | forward range | Confirmed
1 | T1 detent = FLX/MCT | forward range | Confirmed
2 | T1 detent = CL | forward range | Confirmed
3 | T1 reverse zone active | 1 = in reverse range | Confirmed
4 | T2 detent = TOGA | forward range | Confirmed
5 | T2 detent = FLX/MCT | forward range | Confirmed
6 | T2 detent = CL | forward range | Confirmed
7 | T2 reverse zone active | 1 = in reverse range | Confirmed

IDLE: B02 = 0 (no detent bits asserted) (Confirmed).

---

### B09 (offset 9) — Parking brake + rudder trim
Bit | Meaning | Type | Active meaning | Status
--- | ------- | ---- | ------------- | ------
0 | Rudder Trim RESET | momentary | 1 = pressed | Confirmed
1 | Rudder Trim LEFT  | momentary | 1 = pressed | Confirmed
2 | Rudder Trim RIGHT | momentary | 1 = pressed | Confirmed
3 | Parking Brake     | latching  | 1 = ON | Confirmed
4 | (unused/unknown)  | — | — | Unknown
5 | (unused/unknown)  | — | — | Unknown
6 | (unused/unknown)  | — | — | Unknown
7 | (unused/unknown)  | — | — | Unknown

---

### B12 (offset 12) — Gear + autobrake selector + speedbrake latch/flags
Bit | Meaning | Type | Active meaning | Status
--- | ------- | ---- | ------------- | ------
0 | Landing Gear lever position | latching | 1 = UP, 0 = DOWN | Confirmed
1 | (unknown) | — | — | Unknown
2 | (unknown) | — | — | Unknown
3 | Autobrake LO | selector | 1 = LO (one-hot) | Confirmed
4 | Autobrake MED | selector | 1 = MED (one-hot) | Confirmed
5 | Autobrake HI | selector | 1 = HI (one-hot) | Confirmed
6 | Speedbrake retracted detent latch | state | 1 = seated in retracted detent (includes small near-zero tolerance), 0 = not seated | Confirmed
7 | Speedbrake mid-band flag | state | observed ON around SPD ~14952–15389 | Observed (needs boundary confirmation)

Autobrake DISARM/OFF is represented by bits3=0, bit4=0, bit5=0 (Confirmed).

Speedbrake mid-band flag (bit7) observations:
- ON: SPD ≈ 14952–15389
- OFF: SPD ≈ 10020, 19967, 27021, detents >= 1/2, and retracted
This appears to be a narrow threshold band; exact bounds may vary by unit.

---

### B13 (offset 13) — Speedbrake detent markers (one-hot)
Bit | Meaning | Notes | Status
--- | ------- | ----- | ------
0 | Speedbrake at 1/2 detent | axis ≈ 31812 | Confirmed
1 | Speedbrake at 3/4 detent | axis ≈ 47992 | Confirmed
2 | Speedbrake at FULL detent | axis = 65535 | Confirmed
3 | (unknown) | — | Unknown
4 | (unknown) | — | Unknown
5 | (unknown) | — | Unknown
6 | (unknown) | — | Unknown
7 | (unknown) | — | Unknown

B13 = 0 for retracted and for intermediate positions that are not these detents (Confirmed).

---

### B14 (offset 14)
- Observed constant 0xFF in all provided dumps.
- No mapped controls yet.

Status: Unknown

---

## Axis Tables (Observed Values)

### Throttle axes (T1/T2)
Direction (observed):
- IDLE is a high value (~48k)
- Advancing thrust decreases toward 0
- Reverse range increases toward 65535

#### Throttle 1 (T1)
Detent / zone | Axis value | B02 flags
---|---:|---
IDLE | ~48267–48359 | B02=0
CL | 33266 | B02.bit2
FLX/MCT | 17603 | B02.bit1
TOGA | 0 | B02.bit0
Reverse zone (partial) | ~60116 | B02.bit3
Reverse zone (full) | 65535 | B02.bit3

#### Throttle 2 (T2)
Detent / zone | Axis value | B02 flags
---|---:|---
IDLE | 48221 | B02=0
CL | 32231 | B02.bit6
FLX/MCT | 17105 | B02.bit5
TOGA | 0 | B02.bit4
Reverse zone (partial) | ~60522–60535 | B02.bit7
Reverse zone (full) | 65535 | B02.bit7

---

### Flaps axis (FLP)
Detent | Axis value | Notes
---|---:|---
0 | 0 | —
1 | 14361 | —
2 | 30165 | —
3 | 46093 | —
FULL | 65535 | —

No separate detent marker bits were observed for flaps (pure axis in provided captures).

---

### Speedbrake axis (SPD)
Key behaviors:
- Fully retracted detent latch is indicated by B12.bit6=1 (even if axis is slightly above 0).
- Specific detents (1/2, 3/4, FULL) are indicated by B13 one-hot bits.

Position / detent | SPD axis value | B12.bit6 | B12.bit7 | B13
---|---:|---:|---:|---
Retracted | 0 | 1 | 0 | 0
Near-zero tolerance (still “retracted detent”) | 445 | 1 | 0 | 0
Intermediate | 10020 | 0 | 0 | 0
Intermediate (mid-band flag ON) | 14952–15389 | 0 | 1 | 0
Intermediate | 19967 | 0 | 0 | 0
Intermediate | 27021 | 0 | 0 | 0
1/2 detent | 31812 | 0 | 0 | bit0
3/4 detent | 47992 | 0 | 0 | bit1
FULL detent | 65535 | 0 | 0 | bit2

Note: B13=0 for intermediate positions that are not 1/2, 3/4, FULL.

---

## Captain Pack + Add-Ons Coverage: What We Know vs What’s Missing

### Confirmed controls (explicitly tested)
- Engine Masters L/R
- A/THR Disconnect L/R
- Engine MODE selector (NORM / IGN / CRANK)
- Engine FIRE pushbuttons L/R
- Parking Brake
- Rudder Trim LEFT / RIGHT / RESET
- Landing Gear lever UP/DOWN
- Autobrake selector (DISARM/LO/MED/HI)
- Throttle detents (IDLE/CL/FLX/TOGA) for T1 and T2
- Reverse zone for T1 and T2
- Flaps detents (0/1/2/3/FULL)
- Speedbrake detents (1/2, 3/4, FULL) + retracted detent latch behavior

### Missing / Not yet verified (no evidence in provided dumps)
These may exist on some Captain Pack revisions or may be encoded in other report IDs:
- Any additional add-on switches/buttons not toggled in captures
- Gear lever center/neutral position (if present on your unit)
- Remaining unmapped bits in:
  - B09 bits 4–7
  - B12 bits 1–2
  - B13 bits 3–7
  - B14 (all bits; always 0xFF in captures)
- Alternate report IDs or alternate report formats (not observed)

---

## How to contribute missing mappings
For each unknown control:
1. Record a baseline line.
2. Toggle ONLY that control.
3. Record the changed line.
4. Return to baseline (especially for momentary buttons).
5. Include the three lines and label the physical control name.

If any previously constant byte (especially B14) changes, include that capture immediately (it is high-value).
