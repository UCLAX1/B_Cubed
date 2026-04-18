class PIDController:
    def __init__(self, kP, kI, kD):
        self.kP = kP
        self.kI = kI
        self.kD = kD

        self.previous_error = 0
        self.integral = 0
        self.derivative = 0
        self.setpoint = 0

    def set_setpoint(self, setpoint):
        self.setpoint = setpoint

    def update(self, current_value, dt):
        error = self.setpoint - current_value
        self.integral += dt
        self.derivative += (error - self.previous_error) * dt

        control = self.kP * error + self.kI * self.integral + self.kD * self.derivative
        return control
