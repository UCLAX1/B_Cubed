"""
Diagnostic script to find correct quadrature encoder pins for the 5.5kg servo.
Tests various GPIO pairs while the servo spins to detect which ones have signals.
"""

import time
from gpiozero import RotaryEncoder, Servo

# Servo pin (known to be correct)
SERVO_PIN = 18
SPIN_COMMAND = 0.4

# Candidate BCM pins to test (based on 40-pin header)
# Common pattern: consecutive or nearby pins
CANDIDATES = [
    (23, 24),  # physical 16, 18
    (25, 8),   # physical 22, 24
    (7, 11),   # physical 26, 23
    (2, 3),    # physical 3, 5
    (9, 10),   # physical 21, 19
    (14, 15),  # physical 8, 10
    (27, 22),  # current (known wrong)
    (26, 6),   # physical 37, 31 (from 150kg servo)
    (5, 13),   # physical 29, 33
    (19, 20),  # physical 35, 38
    (4, 17),   # physical 7, 11 (17 is abs encoder pin)
    (21, 12),  # physical 40, 32
]

def test_encoder_pair(a_pin, b_pin):
    """Test if (a_pin, b_pin) are the real encoder pins."""
    try:
        print(f"\nTesting A={a_pin}, B={b_pin}...", end=" ", flush=True)
        encoder = RotaryEncoder(a=a_pin, b=b_pin, max_steps=1000000)
        
        initial_steps = encoder.steps
        time.sleep(1.0)
        final_steps = encoder.steps
        step_delta = final_steps - initial_steps
        
        # Clean up
        encoder.close()
        
        # If steps changed, this is likely the correct pair
        if step_delta != 0:
            print(f"✓ FOUND! Delta={step_delta:+6d} steps")
            return True, step_delta
        else:
            print(f"  no signal")
            return False, 0
            
    except Exception as e:
        print(f"  error: {e}")
        return False, 0

def main():
    print("Initializing servo and spinning...")
    servo = Servo(SERVO_PIN)
    servo.value = SPIN_COMMAND
    time.sleep(0.5)
    
    print("\nTesting encoder pin pairs (servo spinning at 0.4):")
    print("=" * 60)
    
    found = []
    for a_pin, b_pin in CANDIDATES:
        is_correct, delta = test_encoder_pair(a_pin, b_pin)
        if is_correct:
            found.append((a_pin, b_pin, delta))
    
    print("\n" + "=" * 60)
    servo.value = 0.0
    servo.close()
    
    if found:
        print(f"\n✓ Found {len(found)} candidate pin pair(s):")
        for a, b, delta in found:
            print(f"  A={a}, B={b} (delta={delta:+d} steps)")
            print(f"    Physical pins: A={find_physical_pin(a)}, B={find_physical_pin(b)}")
    else:
        print("\n✗ No encoder signals detected. Check wiring!")

def find_physical_pin(bcm):
    """Convert BCM to physical pin number on 40-pin header."""
    bcm_to_physical = {
        2: 3, 3: 5, 4: 7, 17: 11, 27: 13, 22: 15, 10: 19, 9: 21,
        11: 23, 5: 29, 6: 31, 13: 33, 19: 35, 26: 37, 14: 8, 15: 10,
        18: 12, 23: 16, 24: 18, 25: 22, 8: 24, 7: 26, 12: 32, 16: 36,
        20: 38, 21: 40, 1: 2, 0: 1,  # I2C/SPI
    }
    return bcm_to_physical.get(bcm, "?")

if __name__ == "__main__":
    main()
