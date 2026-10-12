class SinglePID:
    """Yahboom's per-sample PID, with an output clamp."""

    def __init__(self, P=0.1, I=0.0, D=0.1, limit=float('inf')):
        self.Kp, self.Ki, self.Kd, self.limit = P, I, D, limit
        self.pid_reset()

    def Set_pid(self, P, I, D):
        self.Kp, self.Ki, self.Kd = P, I, D
        self.pid_reset()

    def pid_compute(self, target, current):
        self.error = target - current
        self.intergral += self.error
        self.derivative = self.error - self.prevError
        self.prevError = self.error
        out = self.Kp * self.error + self.Ki * self.intergral + self.Kd * self.derivative
        return max(-self.limit, min(self.limit, out))

    def pid_reset(self):
        self.error = self.intergral = self.derivative = self.prevError = 0.0
