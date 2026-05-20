"""
Test 5.5kg encoder with A/B channels in both orders to detect if they're swapped.
"""

import time
from ServoEx import ServoEx
from gpiozero import Servo

def test_channel_order(servo_pin, a_pin, b_pin, abs_pin, label):
    """Test encoder with given channel order."""
    print(f"\n{'='*60}")
    print(f"Testing: {label}")
    print(f"  Servo: GPIO {servo_pin}")
    print(f"  Encoder A={a_pin}, B={b_pin}, ABS={abs_pin}")
    print(f"{'='*60}")
    
    try:
        servo = ServoEx(
            servo_pin=servo_pin,
            encoder_pin_a=a_pin,
            encoder_pin_b=b_pin,
            absolute_encoder_pin=abs_pin,
        )
        
        print(f"Spinning at +0.40 for 5 seconds...")
        servo.value = 0.4
        start_time = time.time()
        
        while time.time() - start_time < 5.0:
            servo.update()
            t = time.time() - start_time
            rel = servo.get_position()
            steps = servo.encoder.steps
            
            if t % 1.0 < 0.1:  # Print every ~1 second
                print(f"  t={t:.1f}s | steps={steps:8d} | rel={rel:8.4f} rot")
            
            time.sleep(0.1)
        
        print(f"Final: steps={servo.encoder.steps} | rel={servo.get_position():.4f}")
        servo.value = 0.0
        servo.save_encoder_position()
        servo.close()
        
        # Return True if we got any encoder movement
        return servo.encoder.steps != 0
        
    except Exception as e:
        print(f"  ERROR: {e}")
        return False

def main():
    servo_pin = 18
    abs_pin = 17
    
    # Test original order (A on physical 7 / BCM 4, physical 13 is dead)
    result1 = test_channel_order(
        servo_pin=servo_pin,
        a_pin=4, b_pin=22, abs_pin=abs_pin,
        label="A=4 (phys 7), B=22"
    )

    time.sleep(1.0)

    # Test swapped order
    result2 = test_channel_order(
        servo_pin=servo_pin,
        a_pin=22, b_pin=4, abs_pin=abs_pin,
        label="Swapped: A=22, B=4"
    )
    
    print(f"\n{'='*60}")
    print("RESULTS:")
    print(f"  Original (A=27, B=22): {'✓ WORKS' if result1 else '✗ NO SIGNAL'}")
    print(f"  Swapped (A=22, B=27):  {'✓ WORKS' if result2 else '✗ NO SIGNAL'}")
    
    if result1:
        print("\n→ Use: A=27, B=22")
    elif result2:
        print("\n→ Use: A=22, B=27 (CHANNELS SWAPPED)")
    else:
        print("\n→ Neither works - encoder may be disconnected/broken")

if __name__ == "__main__":
    main()
