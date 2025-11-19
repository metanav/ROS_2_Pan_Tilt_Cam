import rclpy

class PID:
    def __init__(self, kp, ki, kd, setpoint=0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = None

    def compute(self, current_value, current_time=None):
        if current_time is None:
            current_time = rclpy.clock.Clock().now().nanoseconds / 1e9

        if self.prev_time is None:
            self.prev_time = current_time
            self.prev_error = self.setpoint - current_value
            return 0.0

        dt = current_time - self.prev_time
        if dt <= 0:
            return 0.0

        error = self.setpoint - current_value
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt

        output = self.kp * error + self.ki * self.integral + self.kd * derivative

        self.prev_error = error
        self.prev_time = current_time
        return output

    def reset(self):
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = None

