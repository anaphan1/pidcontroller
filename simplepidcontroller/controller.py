class PID:
    def __init__(self, set_point, u_min=float('-inf'), u_max=float('inf'), tf=0.01):
        self.set_point = set_point

        self.U_MIN = u_min
        self.U_MAX = u_max

        self.T_F = tf

        self.prev_error = 0
        self.integral = 0
        self.prev_derivative = 0

        self.history = {
            "errors": [],
            "integrals": [],
            "derivatives": [],
            "outputs": [],
            "time": []
        }

        self.time = 0

    def compute(self, value, dt, filter_derivative=False, anti_windup=False):
        if dt <= 0:
            raise ValueError("Time interval (dt) must be greater than zero.")

        error = self.set_point - value

        if self.T_I:
            self.integral += self.K_P / self.T_I * error * dt
        derivative = self.T_D * (error - self.prev_error) / dt

        if filter_derivative:
            alpha = self.T_F / (self.T_F + dt)
            derivative = alpha * self.prev_derivative + (1 - alpha) * derivative

        derivative *= self.T_D * self.K_P
        u = self.K_P * error + self.integral + derivative

        if anti_windup:
            u_actual = max(self.U_MIN, min(u, self.U_MAX))
            if u != u_actual:
                self.integral -= (u - u_actual) / (self.K_P / self.T_I)
            u = u_actual

        self.prev_error = error
        self.prev_derivative = derivative
        self.time += dt

        self.history["errors"].append(error)
        self.history["integrals"].append(self.integral)
        self.history["derivatives"].append(derivative)
        self.history["outputs"].append(u)
        self.history["time"].append(self.time)

        return u

    @property
    def parallel_constants(self):
        ki = self.K_P / self.T_I
        kd = self.K_P * self.T_D
        return self.K_P, ki, kd

    @parallel_constants.setter
    def parallel_constants(self, constants):
        if len(constants) != 3:
            raise ValueError("Need 3 constants")
        kp, ki, kd = constants
        self.K_P = kp
        self.T_I = kp / ki
        self.T_D = kd / kp

    @property
    def ideal_constants(self):
        return self.K_P, self.T_I, self.T_D

    @ideal_constants.setter
    def ideal_constants(self, constants):
        if len(constants) != 3:
            raise ValueError("Need 3 constants")
        self.K_P, self.T_I, self.T_D = constants