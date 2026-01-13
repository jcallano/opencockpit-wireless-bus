# USB Host Power Note (WeAct ESP32-S3 Dev Board Rev A)
## VBUS diode drop and “diode bypass” hardware mod for powering USB devices

This project uses the ESP32-S3 in **USB Host** mode. In Host mode, the board must provide **5V on VBUS** to power the attached USB device (e.g., Thrustmaster controls). On the **WeAct ESP32-S3 Dev Board Rev A**, the native USB VBUS path typically includes a **series diode** (and/or power-path element) that block current flow to devices connected to usb-c native port. Some USB devices are sensitive to undervoltage and may:
- fail enumeration,
- reset intermittently,
- behave erratically under load,
- disconnect when LEDs/logic draw increases.

This document explains how to **bypass the VBUS series diode**so the board can supply a solid **5V** to the USB port, and also lists safer alternatives.

> IMPORTANT: This is a hardware modification. Do it only if you understand the risks. You can permanently damage your board, your USB device, or a connected computer if you backfeed 5V into the wrong place.

---

## Summary

- Problem: **VBUS is not a stable ~5.0V** at the USB port in Host mode due to series diode drop / current limit.
- Solution: **Bypass the VBUS diode** so VBUS is fed directly from the 5V supply path (or from an external 5V source).
- Safer alternative: Use a **powered USB hub** between the board and the device.

---

## Symptoms that indicate VBUS power issues

- Device enumerates sometimes, then fails randomly
- Continuous connect/disconnect events
- Device works until buttons/LEDs activate, then resets
- USB host logs show transfers failing after initial success
- Measured VBUS at the USB port is significantly below 5V (e.g., ~4.2–4.6V)

---

## How to confirm (recommended measurements)

Using a multimeter:

1. Put the board in USB Host mode (the same way you use it for this project).
2. Plug the USB device into the host port.
3. Measure voltage:
   - **VBUS at the USB connector** (5V pin to GND)
   - **5V rail/source point** (upstream of the diode)
4. If you see a consistent drop between source and VBUS (often 0.2–0.6V), the series element is likely the cause.

---

## The VBUS diode bypass concept

Many dev boards implement VBUS like:

5V source  →  **Series diode**  →  USB VBUS pin

The diode prevents backfeeding and provides simple power-path protection, but it also causes a voltage drop. Bypassing means electrically connecting the input and output of that diode so VBUS receives closer to the full 5V.

### Typical board markings
Depending on revision, the component may be labeled as one of:
- D? (diode reference designator)
- “SS14”, “SS34”, or similar Schottky marking
- A power-path diode near the USB connector area

Some WeAct boards also provide **solder jumpers or pads** intended for VBUS routing. If your board has a labeled jumper (e.g., “VBUS”, “5V”, “OTG”, “HOST”), prefer using that jumper instead of directly bridging a diode.

---

## Recommended approach (safer first)

### Option A — Powered USB hub (no soldering)
1. Plug a **powered USB hub** into the ESP32-S3 host port.
2. Plug the Thrustmaster device into the hub.
3. The hub supplies 5V VBUS to the device; the ESP32-S3 only handles data.

Pros:
- No hardware mods
- Best protection against overcurrent
- Less risk of backfeeding

Cons:
- Adds another device in the chain

### Option B — External 5V injection to VBUS (with protection)
If your board has a VBUS power switch and test pads, you can feed VBUS from a known stable 5V regulator/buck converter rated for your device.

Pros:
- Stable voltage
- Can support higher current

Cons:
- Requires careful wiring and grounding
- Must avoid feeding 5V into your PC’s USB port

### Option C — Diode bypass mod (what this doc is about)
You bridge across the series diode (or use the board’s solder jumper) so VBUS is effectively “direct 5V”.

Pros:
- Simple, compact, no extra hardware
- Restores near-5V at VBUS

Cons:
- Higher risk
- Removes/weakens protection against backfeeding
- Can damage host/PC if misused

---

## Diode bypass mod (generic procedure)

> Because board revisions vary, **do not assume the exact diode reference** without verifying on your PCB.

### Tools
- Fine tip soldering iron
- Flux
- Thin solder
- Tweezers
- Magnification (recommended)
- Multimeter (mandatory)

### Steps
1. **Power off** the board completely.
2. Locate the **VBUS series diode**:
   - Usually near the USB connector / VBUS routing trace.
3. Identify the diode pads:
   - One side is upstream 5V source
   - The other side goes to USB VBUS pin
4. **Confirm with continuity test**:
   - Pad A → 5V source rail
   - Pad B → USB VBUS pin
5. Apply flux and **bridge the diode pads**:
   - Either solder a small blob across the pads, or
   - Add a tiny wire link between pad A and pad B
6. Inspect for shorts to ground.
7. Power on and measure VBUS again:
   - Target: close to **5.0V under load** (device plugged in)

---

## Critical warnings (read this)

### Avoid backfeeding a computer USB port
If you connect the board’s USB port to a PC for programming/debug AND you also force 5V onto VBUS (especially after a diode bypass), you may backfeed 5V into the PC’s USB port.

Best practices:
- Use a **data-only cable** (VBUS disconnected) when flashing/debugging, OR
- Use separate ports: one for programming (native USB/device mode) and one dedicated for host power, OR
- Add a protection element (ideal diode / load switch) if you need both.

### Current draw
Some flight sim peripherals can draw significant current, especially if they have LEDs, internal hubs, or high-power logic. Ensure your 5V source can supply enough current (and your wiring/connector can handle it).

### Overcurrent protection
Bypassing protection can increase risk of damage if the device or cable shorts VBUS. Consider adding:
- A resettable fuse (PTC)
- A proper USB power switch IC
- A powered hub

---

## Verification checklist (after mod)

- [ ] VBUS reads ~5.0V with the device connected
- [ ] No unexpected heating on the board near the mod area
- [ ] Device enumerates reliably on every plug-in
- [ ] No random disconnects when actuating controls
- [ ] No backfeeding scenarios in your setup (PC connection separated/isolated)

---

## Notes for this project

For reverse-engineering and HID capture, stable VBUS power is essential. If your Thrustmaster device shows intermittent behavior, prioritize VBUS stability before debugging USB host software.

If you want to contribute:
- Add photos of your board revision and the exact component/jumper used
- Provide before/after VBUS measurements under load
- Note whether you used a powered hub or diode bypass
