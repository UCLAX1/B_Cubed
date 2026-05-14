from servo_control_filters import (
    PDCommandDamper,
    PIDCommandDamper,
    angle_to_servo_value,
    slew_limit,
)


def assert_close(actual, expected, tolerance=1e-6):
    if abs(actual - expected) > tolerance:
        raise AssertionError(f"expected {expected}, got {actual}")


def test_angle_to_servo_value():
    assert_close(angle_to_servo_value(0.0), 0.0)
    assert_close(angle_to_servo_value(90.0), 1.0)
    assert_close(angle_to_servo_value(-90.0), -1.0)
    assert_close(angle_to_servo_value(180.0), 1.0)


def test_slew_limit():
    assert_close(slew_limit(0.0, 1.0, 0.2), 0.2)
    assert_close(slew_limit(0.0, -1.0, 0.2), -0.2)
    assert_close(slew_limit(0.0, 0.1, 0.2), 0.1)


def test_pd_damper_holds_small_target_noise_near_setpoint():
    damper = PDCommandDamper(
        initial_angle=10.0,
        deadband_deg=1.0,
        target_noise_deg=0.35,
        target_noise_rate_deg_s=12.0,
    )

    assert_close(damper.update(10.2, 0.05), 10.0)
    assert_close(damper.update(10.1, 0.05), 10.0)
    assert_close(damper.update(10.25, 0.05), 10.0)


def test_pd_damper_moves_for_real_target_change():
    damper = PDCommandDamper(initial_angle=0.0, max_velocity_deg_s=160.0)
    next_angle = damper.update(20.0, 0.05)

    if next_angle <= 0.0:
        raise AssertionError(f"expected arm command to move positive, got {next_angle}")
    if next_angle > 8.0:
        raise AssertionError(f"expected velocity limit to cap first step, got {next_angle}")


def test_pid_damper_integral_stays_bounded():
    damper = PIDCommandDamper(
        initial_angle=0.0,
        kp=0.0,
        ki=1.0,
        kd=0.0,
        integral_limit_deg_s=1.0,
        max_velocity_deg_s=160.0,
    )

    for _ in range(20):
        damper.update(30.0, 0.1)

    if damper.integral > 1.0:
        raise AssertionError(f"expected integral clamp, got {damper.integral}")


if __name__ == "__main__":
    test_angle_to_servo_value()
    test_slew_limit()
    test_pd_damper_holds_small_target_noise_near_setpoint()
    test_pd_damper_moves_for_real_target_change()
    test_pid_damper_integral_stays_bounded()
    print("servo_control_filters tests passed")
