"""Pi-facing entry point for the low-level motor control node."""

from low_level_runner.kiwi_drive_controller import KiwiDriveController, run_controller

__all__ = ["KiwiDriveController", "main"]


def main(args=None) -> None:
    """Run the motor controller with the Pi deployment node name."""
    run_controller(args=args, node_name="low_level_motor_control")
