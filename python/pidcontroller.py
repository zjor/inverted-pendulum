class PIDController:
    def __init__(self, Kp, Kd, Ki, target, initValue=0.0):
        self.Kp = Kp
        self.Kd = Kd
        self.Ki = Ki
        self.target = target
        self.lastValue = initValue
        self.integralError = 0.0
    
    def getControl(self, value, dt):
        '''
            Returns PID control.
            Derivative spike is mitigated by dError/dt = -dInput/dt
            see: http://brettbeauregard.com/blog/2011/04/improving-the-beginner%e2%80%99s-pid-derivative-kick/
        '''
        error = self.target - value

        derivative = -(value - self.lastValue) / dt

        self.lastValue = value
        self.integralError += error * dt
        return self.Kp * error + self.Kd * derivative + self.Ki * self.integralError

    def setTarget(self, target):
        self.target = target

if __name__ == "__main__":
    import numpy as np

    import matplotlib
    matplotlib.use('TKAgg')

    import matplotlib.pyplot as pp

    dt = 0.01
    x0 = 1.0
    v0 = 0.0
    ts = np.arange(0, 10, dt)
    xs = [x0]
    vs = [v0]
    us = [0.0]
    pid = PIDController(8.0, 4.0, 1.0, 0.0, initValue=x0)
    for i in range(0, len(ts) - 1):
        x = xs[-1]
        v = vs[-1]
        u = pid.getControl(x, dt)
        new_v = v + u * dt
        new_x = x + new_v * dt
        xs.append(new_x)
        vs.append(new_v)
        us.append(u)

    # print(us[:10])
    
    x_line, = pp.plot(ts, xs, label="X")
    v_line, = pp.plot(ts, vs, label="V")
    u_line, = pp.plot(ts, us, label="U")
    pp.legend([x_line, v_line, u_line])
    pp.grid(True)
    pp.show()





