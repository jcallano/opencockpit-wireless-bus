# USB HID Performance Analysis

**Date:** 2026-01-14
**Tools:** `direct_usb_tool.py` (hidapi)

## 1. Thrustmaster TCA Sidestick
*   **Idle Rate:** ~10-16 Hz
*   **Active Rate:** ~130-150 Hz
*   **Jitter:** High when idle (>100ms), Low when active (~2-4ms).
*   **Payload:** 64 bytes (mostly 0s, axes in bytes 0-8).
*   **Optimization Strategy:**
    *   **Change-Based Transmission:** Essential. Only send when axes change > delta.
    *   **Deadband:** Required for null zones (center stick) to prevent noise-triggered transmission.
    *   **Rate Limiting:** Cap at 50Hz or 60Hz might be sufficient for civilian flight sims, saving 60% bandwidth vs 150Hz.

## 2. Thrustmaster TCA Quadrant
*   **Idle Rate:** ~19-20 Hz
*   **Active Rate:** **~90-105 Hz** (Bursts up to 100Hz when levers are moved rapidly).
*   **Jitter:** High (~34ms) at idle, stabilizes to ~3-8ms when active.
*   **Optimization Strategy:**
    *   **Change-Based:** Highly effective. The device seemingly implements this internally (dropping to 20Hz). We can be even more aggressive (0Hz if no change).
    *   **Quantization:** Throttle axes often have "jitter" of +/- 1 value. 10-bit or 12-bit quantization can strip this noise.

## Proposed Firmware Policies
To optimize the limited ESP-NOW bandwidth (~250 bytes max payload, shared medium):

1.  **Dynamic Rate Limiting:**
    *   If `delta < threshold`: Send max 1 packet/sec (Heartbeat).
    *   If `delta > threshold`: Send immediately, but max `N` times/sec (e.g., 60Hz).
2.  **Packet Aggregation:**
    *   Combine multiple inputs if they occur simultaneously (less relevant for separate ESP32 nodes).
