import time
import platform

from math import pi

def is_raspberry():
    return platform.system() != 'Darwin'

def normalize_angle(value):
    return ((value - 10740.0) / 12000.0) * pi

class BaseEncoder:
    
    def read():
        raise NotImplementedError


    def get_state(self, dt):
        current = self.read()            
        if dt == 0 or not self.last_read:
            state = (current, .0)
        else:
            state = (current, (current - self.last_read) / dt)
        self.last_read = current
        return state


if is_raspberry():
    import Adafruit_ADS1x15    
    class Encoder(BaseEncoder):

        def __init__(self):
            self.adc = Adafruit_ADS1x15.ADS1115()
            self.last_read = None


        def read(self):
            return normalize_angle(self.adc.read_adc(0, gain=2/3))

else:
    import random
    class Encoder(BaseEncoder):

        def __init__(self):            
            self.last_read = None


        def read(self):
            return random.uniform(-pi, pi)        


if __name__ == "__main__":    

    print("Press CTRL+C to exit")
    try:
        enc = Encoder()
        last_update = time.time()

        while 1:
            now = time.time()
            dt = now - last_update
            if dt >= 0.001:
                state = enc.get_state(dt)
                print("%f\t%f\t%f" % (now, state[0], state[1]))
                last_update = now
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass
