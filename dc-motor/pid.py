class PID:
    def __init__(self, target, dt, k_p=0.5, k_i=0.0, k_d=0):
        self.target = target
        self.dt = dt
        self.k_p = k_p
        self.k_i = k_i
        self.k_d = k_d
        self.last_value = None
        self.err_acc = 0.0

    def get_control(self, value):
        e = self.target - value
        de = - ((value - self.last_value) / self.dt) if self.last_value else 0.0
        self.err_acc += self.k_i * e * self.dt
        self.last_value = e
        return self.k_p * e + self.err_acc + self.k_d * de

    def set_target(self, new_target):
        self.target = new_target
        self.err_acc = 0
