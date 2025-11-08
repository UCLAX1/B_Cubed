# TODO: insert robot code here
import wpilib
import wpilib.drive
import rev
import time

# https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-4/creating-test-drivetrain-program-cpp-java-python.html
# https://docs.wpilib.org/en/stable/docs/software/python/subcommands/deploy.html


motor = rev.CANSparkMax(1, rev.CANSparkMax.MotorType.kBrushless)
motor.set(0.5)

time.sleep(2)


exit(0)

