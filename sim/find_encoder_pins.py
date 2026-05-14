"""
Diagnostic script to find correct quadrature encoder pins for the 5.5kg servo.
Tests various GPIO pairs while the servo spins to detect which ones have signals.
"""

import time
from gpiozero import RotaryEncoder, Servo

# Servo pin (known to be correct)
SERVO_PIN = 18
SPIN_COMMAND = 0.4

# Candidate BCM pins to test (all 28 GPIO available on 40-pin header, excluding used ones)
# Skip: 12 (servo), 13 (servo), 16 (MOSFET), 17 (abs encoder), 18 (servo)
# Try all other combinations
CANDIDATES = [
    (2, 3),    # physical 3, 5
    (2, 4),    # physical 3, 7
    (2, 5),    # physical 3, 29
    (2, 6),    # physical 3, 31
    (2, 7),    # physical 3, 26
    (2, 8),    # physical 3, 24
    (2, 9),    # physical 3, 21
    (2, 10),   # physical 3, 19
    (2, 11),   # physical 3, 23
    (2, 14),   # physical 3, 8
    (2, 15),   # physical 3, 10
    (3, 4),    # physical 5, 7
    (3, 5),    # physical 5, 29
    (4, 5),    # physical 7, 29
    (4, 6),    # physical 7, 31
    (5, 6),    # physical 29, 31
    (5, 10),   # physical 29, 19
    (5, 11),   # physical 29, 23
    (6, 10),   # physical 31, 19
    (8, 9),    # physical 24, 21
    (9, 10),   # physical 21, 19
    (10, 11),  # physical 19, 23
    (14, 15),  # physical 8, 10
    (19, 20),  # physical 35, 38
    (19, 21),  # physical 35, 40
    (20, 21),  # physical 38, 40
    (23, 24),  # physical 16, 18
    (23, 25),  # physical 16, 22
    (24, 25),  # physical 18, 22
    (26, 27),  # physical 37, 13
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
    print("Initializing servo...")
    servo = Servo(SERVO_PIN)
    print(f"Servo initialized on GPIO {SERVO_PIN}")
    
    print("\nSpinning servo at 0.4 for 2 seconds to warm up encoder...")
    servo.value = SPIN_COMMAND
    time.sleep(2.0)
    
    print("\nStopping servo and closing GPIO to release pins...")
    servo.value = 0.0
    servo.close()
    time.sleep(0.5)
    
    print("\nTesting encoder pin pairs:")
    print("=" * 60)
    
    found = []
    for a_pin, b_pin in CANDIDATES:
        is_correct, delta = test_encoder_pair(a_pin, b_pin)
        if is_correct:
            found.append((a_pin, b_pin, delta))
    
    print("\n" + "=" * 60)
    
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
