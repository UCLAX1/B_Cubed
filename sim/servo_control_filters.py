class PIDCommandDamper:
    """PID-style target shaper for open-loop positional servos.

    This is not true closed-loop servo position PID because the caller does not
    provide measured arm angle feedback. It damps the commanded angle instead,
    which is useful for reducing jitter from tiny target changes near setpoint.
    """

    def __init__(
        self,
        initial_angle=0.0,
        kp=8.0,
        ki=0.0,
        kd=0.35,
        max_velocity_deg_s=180.0,
        deadband_deg=1.0,
        target_noise_deg=0.35,
        target_noise_rate_deg_s=12.0,
        integral_limit_deg_s=45.0,
    ):
        self.angle = initial_angle
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_velocity_deg_s = max_velocity_deg_s
        self.deadband_deg = deadband_deg
        self.target_noise_deg = target_noise_deg
        self.target_noise_rate_deg_s = target_noise_rate_deg_s
        self.integral_limit_deg_s = integral_limit_deg_s
        self._integral = 0.0
        self._previous_error = 0.0
        self._previous_target = initial_angle

    def reset(self, angle=0.0):
        self.angle = angle
        self._integral = 0.0
        self._previous_error = 0.0
        self._previous_target = angle

    @property
    def integral(self):
        return self._integral

    def update(self, target_angle, dt):
        if dt <= 0:
            return self.angle

        target_delta = target_angle - self._previous_target
        target_rate = target_delta / dt
        error = target_angle - self.angle

        # Once the arm is close, ignore small IMU/geometry target chatter.
        if (
            abs(error) <= self.deadband_deg
            and abs(target_delta) <= self.target_noise_deg
            and abs(target_rate) <= self.target_noise_rate_deg_s
        ):
            self._integral = 0.0
            self._previous_target = target_angle
            self._previous_error = error
            return self.angle

        self._integral = clamp(
            self._integral + error * dt,
            -self.integral_limit_deg_s,
            self.integral_limit_deg_s,
        )
        derivative = (error - self._previous_error) / dt
        velocity = self.kp * error + self.ki * self._integral + self.kd * derivative
        velocity = clamp(
            velocity,
            -self.max_velocity_deg_s,
            self.max_velocity_deg_s,
        )

        max_step = self.max_velocity_deg_s * dt
        self.angle += clamp(velocity * dt, -max_step, max_step)

        # Avoid dithering around the exact setpoint after a slow approach.
        if abs(target_angle - self.angle) <= self.deadband_deg * 0.5:
            self.angle = target_angle

        self._previous_target = target_angle
        self._previous_error = target_angle - self.angle
        return self.angle


class PDCommandDamper(PIDCommandDamper):
    """Backward-compatible PD wrapper around the PID command damper."""

    def __init__(self, *args, **kwargs):
        kwargs.setdefault("ki", 0.0)
        super().__init__(*args, **kwargs)


def angle_to_servo_value(angle_deg, servo_range_deg=90.0):
    """Convert a target angle in degrees to a servo value in [-1, 1]."""
    return clamp(angle_deg / servo_range_deg, -1.0, 1.0)


def clamp(value, minimum, maximum):
    return max(min(value, maximum), minimum)


def slew_limit(current, target, max_delta):
    """Move current toward target by at most max_delta."""
    diff = target - current
    if diff > max_delta:
        return current + max_delta
    if diff < -max_delta:
        return current - max_delta
    return target
