import time
import platform

def is_raspberry():
    return platform.system() != 'Darwin'

if is_raspberry():
    import Adafruit_ADS1x15    
    class Encoder:

        def __init__(self):
            self.adc = Adafruit_ADS1x15.ADS1115()
            self.last_read = None


        def read(self):
            return adc.read_adc(0, gain=2/3)


        def get_state(self, dt):
            current = self.read()            
            if dt == 0 or not self.last_read:
                state = (current, None)
            else:
                state = (current, (current - self.last_read) / dt)
            self.last_read = current
            return state

else:
    from math import pi
    import random
    class Encoder:

        def __init__(self):            
            self.last_read = 0


        def read(self):
            return random.uniform(-pi, pi)

        def get_state(self, dt):
            current = self.read()   
            if dt == 0:
                state = (current, None)
            else:
                state = (current, (current - self.last_read) / dt)
            self.last_read = current
            return state            


if __name__ == "__main__":    

    print("Press CTRL+C to exit")
    try:
        enc = Encoder()
        last_update = time.time()

        while 1:
            now = time.time()
            dt = now - last_update
            if dt > 0.001:
                state = enc.get_state(dt)
                print("%f\t%f\t%f" % (now, state[0], state[1]))
                last_update = now
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
