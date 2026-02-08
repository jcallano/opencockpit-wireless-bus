"""
Validation Script for ENG LEDs - URSA MINOR 32

Tests all 4 ENG LEDs (FIRE/FAULT) with ON/OFF states.
"""

import sys
import os
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

try:
    import hid
except ImportError:
    print("Error: hidapi not installed")
    print("Install with: pip install hidapi")
    sys.exit(1)

VID = 0x4098
PID = 0xB920

# LED mappings
ENG_LEDS = [
    {'id': 0x03, 'name': 'ENG 1 FAULT'},
    {'id': 0x04, 'name': 'ENG 1 FIRE'},
    {'id': 0x05, 'name': 'ENG 2 FAULT'},
    {'id': 0x06, 'name': 'ENG 2 FIRE'},
]


def send_led_command(device, led_id, state):
    """Send LED control command"""
    value = 255 if state else 0
    
    buffer = [0] * 64
    buffer[0] = 0x02
    buffer[1] = 0x10
    buffer[2] = 0xb9
    buffer[3] = 0x00
    buffer[4] = 0x00
    buffer[5] = 0x03
    buffer[6] = 0x49
    buffer[7] = led_id
    buffer[8] = value
    
    try:
        device.write(buffer)
    except Exception as e:
        print(f"Error: {e}")


def test_led(device, led):
    """Test a single LED ON/OFF"""
    print(f"\n{'=' * 70}")
    print(f"Testing: {led['name']} (ID 0x{led['id']:02X})")
    print('=' * 70)
    
    # Blink test: ON -> OFF -> ON -> OFF
    for cycle in range(2):
        print(f"\nCycle {cycle + 1}/2:")
        
        # ON
        print(f"  → {led['name']} ON", end="", flush=True)
        send_led_command(device, led['id'], True)
        time.sleep(1.0)
        
        # OFF
        print(f"  → OFF")
        send_led_command(device, led['id'], False)
        time.sleep(0.5)
    
    print(f"\n✓ {led['name']} test complete")
    
    # Verify with user
    print(f"\nDid {led['name']} blink ON/OFF correctly?")
    print("  y. Yes - LED working")
    print("  n. No - LED not working")
    print("  s. Skip")
    print()
    
    response = input("Your answer: ").strip().lower()
    
    if response == 'y':
        return True
    elif response == 'n':
        return False
    else:
        return None


def main():
    """Main validation"""
    
    print("=" * 70)
    print("URSA MINOR 32 - ENG LEDs Validation")
    print("=" * 70)
    print()
    print("Testing all 4 ENG LEDs (FIRE/FAULT)")
    print("Each LED will blink ON/OFF twice")
    print()
    
    # Open device
    print("Opening device...")
    try:
        device = hid.device()
        device.open(VID, PID)
        print("✓ Device opened")
        print()
    except Exception as e:
        print(f"✗ Device not found: {e}")
        return 1
    
    try:
        input("Press Enter to start testing...")
        
        # Test each LED
        results = {}
        for led in ENG_LEDS:
            result = test_led(device, led)
            results[led['name']] = result
        
        # Summary
        print("\n" + "=" * 70)
        print("VALIDATION SUMMARY")
        print("=" * 70)
        print()
        
        verified = sum(1 for r in results.values() if r is True)
        failed = sum(1 for r in results.values() if r is False)
        skipped = sum(1 for r in results.values() if r is None)
        
        print(f"Verified: {verified}/4")
        print(f"Failed:   {failed}/4")
        print(f"Skipped:  {skipped}/4")
        print()
        
        print("Detailed results:")
        for name, result in results.items():
            if result is True:
                status = "✓ WORKING"
            elif result is False:
                status = "✗ FAILED"
            else:
                status = "⊘ SKIPPED"
            print(f"  {status} - {name}")
        
        print()
        
        if verified == 4:
            print("🎉 ALL ENG LEDs VERIFIED!")
        elif failed > 0:
            print("⚠️  Some LEDs did not work")
        
    finally:
        # Turn off all LEDs
        for led in ENG_LEDS:
            send_led_command(device, led['id'], False)
        
        device.close()
        print("\nDevice closed")
    
    return 0


if __name__ == "__main__":
    sys.exit(main())
