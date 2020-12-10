class PID:
    def __init__(self, target, dt, k_p=0.5, k_i=0.0, k_d=0, min_value=None, max_value=None):
        self.target = target
        self.dt = dt
        self.k_p = k_p
        self.k_i = k_i
        self.k_d = k_d
        self.last_value = None
        self.err_acc = 0.0
        self.min_value = min_value
        self.max_value = max_value

    def clamp_min(self, value):
        if self.min_value:
            return max(value, self.min_value)
        else:
            return value

    def clamp_max(self, value):
        if self.max_value:
            return min(value, self.max_value)
        else:
            return value

    def get_control(self, value):
        e = self.target - value
        de = - ((value - self.last_value) / self.dt) if self.last_value else 0.0
        self.err_acc += self.k_i * e * self.dt
        self.last_value = e
        u = self.k_p * e + self.err_acc + self.k_d * de
        return self.clamp_max(self.clamp_min(u))

    def set_target(self, new_target):
        self.target = new_target
        self.err_acc = 0
