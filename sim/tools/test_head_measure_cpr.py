"""
Measure counts per revolution for the 5.5kg head servo (BCM 18).

Enc 1 pins (BCM): A=4 (physical 7), B=22 (physical 15), ABS=17 (physical 11)

Spin one full revolution manually or under power, press Ctrl+C when back at the mark.
"""

import time
from gpiozero import DigitalOutputDevice
from ServoEx import ServoEx

SERVO_PIN     = 18
MOSFET_PIN    = 16
ENCODER_PIN_A = 4
ENCODER_PIN_B = 22
ABS_PIN       = 17
SPIN_SPEED    = 0.2

mosfet = DigitalOutputDevice(MOSFET_PIN)
mosfet.on()
time.sleep(0.5)

print("Initializing head encoder...")
enc = ServoEx(
    servo_pin=SERVO_PIN,
    encoder_pin_a=ENCODER_PIN_A,
    encoder_pin_b=ENCODER_PIN_B,
    absolute_encoder_pin=ABS_PIN,
    initial_value=None,
)
enc.reset_encoder_position()

# ── detect direction ──────────────────────────────────────────
print("Detecting motor direction (spinning briefly)...")
enc.value = SPIN_SPEED
time.sleep(0.5)
enc.value = 0.0
time.sleep(0.3)
test_steps = enc.encoder.steps
direction_sign = 1 if test_steps >= 0 else -1
enc.encoder.steps = 0
print(f"Direction sign: {direction_sign} (from {test_steps} steps)")

# ── measure CPR ───────────────────────────────────────────────
input("\nPosition the head at a clear reference mark, then press ENTER to start spinning...")
enc.encoder.steps = 0
print("Spinning — stop it manually when it returns to the mark, then press Ctrl+C.")
enc.value = direction_sign * SPIN_SPEED
try:
    while True:
        print(f"  steps={enc.encoder.steps}", end='\r', flush=True)
        time.sleep(0.05)
except KeyboardInterrupt:
    pass

enc.value = 0.0
time.sleep(1.5)
measured_cpr = abs(enc.encoder.steps)
print(f"\nMeasured CPR: {measured_cpr} steps = 360 degrees")
print(f"1 degree = {measured_cpr / 360.0:.2f} steps")
print(f"\nUpdate HEAD_CPR = {measured_cpr} in head_balance_servo_control.py")

enc.value = None
time.sleep(0.2)
mosfet.off()
print("Done.")
