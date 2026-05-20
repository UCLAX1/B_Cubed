"""
Encoder 0 test for the 70kg arm servo (standard servo, BCM 13, physical 33).
Sweeps slowly back and forth within the 90-degree limit and prints encoder readings.

Enc 0 pins (BCM): A=26 (physical 37), B=6 (physical 31), ABS=5 (physical 29)
"""

import time
from gpiozero import DigitalInputDevice, DigitalOutputDevice, Servo

SERVO_PIN = 12       # BCM 12, physical 32 (70kg continuous servo)
ENC_A_PIN = 26       # BCM 26, physical 37
ENC_B_PIN = 6        # BCM 6,  physical 31
ABS_PIN   = 5        # BCM 5,  physical 29
MOSFET_PIN = 16

SWEEP_SPEED = 0.005  # servo units per step (smaller = slower)
STEP_DELAY  = 0.05   # seconds between steps
LIMIT       = 0.9    # servo value limit (~90 deg), stay just under 1.0

mosfet = DigitalOutputDevice(MOSFET_PIN)
enc_a  = DigitalInputDevice(ENC_A_PIN, pull_up=True)
enc_b  = DigitalInputDevice(ENC_B_PIN, pull_up=True)
abs_enc = DigitalInputDevice(ABS_PIN,  pull_up=False)

print("Turning MOSFET on...")
mosfet.on()
time.sleep(0.5)

servo = Servo(SERVO_PIN)

# simple step counter using channel A edges
step_count = 0
last_a = enc_a.value

def read_abs_pulse():
    """Measure absolute encoder pulse width in ms (rough blocking read)."""
    timeout = 0.1
    start = time.time()
    while abs_enc.value == 1:
        if time.time() - start > timeout:
            return None
    while abs_enc.value == 0:
        if time.time() - start > timeout:
            return None
    t0 = time.time()
    while abs_enc.value == 1:
        if time.time() - start > timeout:
            return None
    return (time.time() - t0) * 1000  # ms

print("Starting sweep. Ctrl+C to stop.\n")
servo.value = 0.0
time.sleep(1.0)

direction = 1
current_val = 0.0

try:
    while True:
        current_val += SWEEP_SPEED * direction
        if current_val >= LIMIT:
            current_val = LIMIT
            direction = -1
        elif current_val <= -LIMIT:
            current_val = -LIMIT
            direction = 1

        servo.value = current_val

        # count A channel edges
        a = enc_a.value
        if a != last_a:
            step_count += direction
            last_a = a

        abs_ms = read_abs_pulse()
        abs_pos = f"{abs_ms:.2f}ms" if abs_ms else "---"

        print(
            f"cmd={current_val:+.3f} | steps={step_count:6d} | "
            f"A={enc_a.value} B={enc_b.value} | abs={abs_pos}",
            flush=True
        )

        time.sleep(STEP_DELAY)

except KeyboardInterrupt:
    print("\nStopping.")
finally:
    servo.value = 0.0
    time.sleep(0.5)
    mosfet.off()
    print("Done.")
