"""
Probe REV-11-1271 through-bore encoder signals during guarded servo nudges.

The encoder must be switched to A mode. This script reports raw edge counts on
BCM GPIO 0..27 and decodes the REV absolute duty-cycle output when present.
"""

import time

import pigpio


MOSFET_PIN = 16
LAZY_SERVO = 12
HEAD_SERVO = 20

LAZY_A, LAZY_B, LAZY_ABS = 26, 6, 5
HEAD_A, HEAD_B, HEAD_ABS = 4, 22, 17
ENCODER_PINS = [LAZY_A, LAZY_B, LAZY_ABS, HEAD_A, HEAD_B, HEAD_ABS]

SIGNAL_LABELS = {
    LAZY_A: "lazy A/blue",
    LAZY_B: "lazy B/yellow",
    LAZY_ABS: "lazy ABS/white",
    HEAD_A: "head A/blue",
    HEAD_B: "head B/yellow",
    HEAD_ABS: "head ABS/white",
}

NUDGE_TIME_S = 0.6
SETTLE_TIME_S = 0.4


class EdgeMonitor:
    def __init__(self, pi, pins):
        self.pi = pi
        self.edge_counts = {pin: 0 for pin in pins}
        self.last_rising_tick = {}
        self.absolute_pulse_us = {}
        self.callbacks = [
            pi.callback(pin, pigpio.EITHER_EDGE, self._edge) for pin in pins
        ]

    def _edge(self, pin, level, tick):
        if level not in (0, 1):
            return
        self.edge_counts[pin] += 1
        if level == 1:
            self.last_rising_tick[pin] = tick
        elif pin in self.last_rising_tick:
            self.absolute_pulse_us[pin] = pigpio.tickDiff(
                self.last_rising_tick[pin], tick
            )

    def snapshot(self):
        return dict(self.edge_counts)

    def changes_since(self, before):
        return {
            f"GPIO {pin} ({SIGNAL_LABELS.get(pin, 'unmapped')})": count - before[pin]
            for pin, count in self.edge_counts.items()
            if count != before[pin]
        }

    def describe_absolute(self, pin):
        pulse_us = self.absolute_pulse_us.get(pin)
        if pulse_us is None:
            return "no pulse"
        if not 1 <= pulse_us <= 1024:
            return f"invalid pulse {pulse_us}us"
        position = max(0.0, min((pulse_us - 1.0) / 1023.0, 1.0))
        return f"{pulse_us}us ({position * 360.0:.1f} deg)"

    def close(self):
        for callback in self.callbacks:
            callback.cancel()


def nudge(pi, monitor, name, servo_pin, pulse_width_us):
    before = monitor.snapshot()
    pi.set_servo_pulsewidth(servo_pin, pulse_width_us)
    time.sleep(NUDGE_TIME_S)
    pi.set_servo_pulsewidth(servo_pin, 1500)
    time.sleep(SETTLE_TIME_S)
    print(f"{name}: edges={monitor.changes_since(before)}")
    print(
        f"  lazy abs GPIO {LAZY_ABS}: {monitor.describe_absolute(LAZY_ABS)}; "
        f"head abs GPIO {HEAD_ABS}: {monitor.describe_absolute(HEAD_ABS)}"
    )


def check_idle_absolute_outputs(monitor):
    before = monitor.snapshot()
    time.sleep(0.1)
    after = monitor.snapshot()
    print("Idle absolute pulse check:")
    for name, pin in (("lazy", LAZY_ABS), ("head", HEAD_ABS)):
        edges = after[pin] - before[pin]
        state = "OK" if edges >= 10 else "MISSING"
        print(
            f"  {name}: GPIO {pin} edges={edges}, "
            f"{monitor.describe_absolute(pin)} [{state}]"
        )
    print("  Expected roughly 195 edges per absolute pin in 0.1 seconds.")


pi = pigpio.pi()
if not pi.connected:
    raise RuntimeError("pigpiod is not available")

for pin in ENCODER_PINS:
    pi.set_mode(pin, pigpio.INPUT)
    pi.set_pull_up_down(pin, pigpio.PUD_UP)

monitor = EdgeMonitor(pi, range(28))

try:
    pi.set_mode(MOSFET_PIN, pigpio.OUTPUT)
    pi.write(MOSFET_PIN, 1)
    for servo_pin in (LAZY_SERVO, HEAD_SERVO):
        pi.set_mode(servo_pin, pigpio.OUTPUT)
        pi.set_servo_pulsewidth(servo_pin, 1500)

    time.sleep(0.5)
    print("REV-11-1271 probe. Confirm both encoder switches are set to A.")
    check_idle_absolute_outputs(monitor)
    nudge(pi, monitor, "lazy +", LAZY_SERVO, 1600)
    nudge(pi, monitor, "lazy -", LAZY_SERVO, 1400)
    nudge(pi, monitor, "head +", HEAD_SERVO, 1675)
    nudge(pi, monitor, "head -", HEAD_SERVO, 1325)
finally:
    for servo_pin in (LAZY_SERVO, HEAD_SERVO):
        pi.set_servo_pulsewidth(servo_pin, 0)
    pi.write(MOSFET_PIN, 0)
    monitor.close()
    pi.stop()
