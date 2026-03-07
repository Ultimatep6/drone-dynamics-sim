from quad_sim.control.config.setpoints import (
    AttitudeSetpoint,
    PositionSetpoint,
    VelocitySetpoint,
)


class PID:
    def __init__(self, config, debug=False):
        if config.__class__.__name__ != "PidGains":
            raise TypeError("Config for PID must be a PidGains object")
        self.kp = config.kp
        self.ki = config.ki
        self.kd = config.kd

        self.i_limit = config.i_limit
        self.out_limit = config.out_limit
        self.alpha = config.alpha

        self.dt = config.dt

        self.integral = 0.0
        self.last_measurement = 0.0
        self.last_derivative = 0.0

        if debug:
            print(self)

    def compute(
        self,
        setpoint: AttitudeSetpoint | PositionSetpoint | VelocitySetpoint,
        measurement,
    ):
        error = setpoint - measurement

        # P
        p = self.kp * error

        # I (only integrate if not saturated)
        self.integral += error * self.ki * self.dt
        self.integral = max(min(self.integral, self.i_limit), -self.i_limit)

        # D on measurement
        derivative = -(measurement - self.last_measurement) / self.dt
        d = self.kd * (
            self.alpha * derivative + (1 - self.alpha) * self.last_derivative
        )
        # Combine
        output = p + self.integral + d

        # Output limit
        output = max(min(output, self.out_limit), -self.out_limit)

        # Save state
        self.last_measurement = measurement
        self.last_derivative = derivative

        return output

    def __str__(self) -> str:
        return f"KP:{self.kp}\nKD:{self.kd}\nKI:{self.ki}"
