import Adafruit_BBIO.PWM as PWM
from scipy.interpolate import interp1d
import logging

class motor:
    MAX_POWER = 100
    MIN_POWER = 0

    def __init__(self, pin):
        self.pin = pin
        self.power = MIN_POWER
        # start motor with power = 0 (5 on the scale of [5, 10])
        PWM.start(self.pin, 5, 50)

    def setPower(self, power):
        if power > MAX_POWER:
            self.power = MAX_POWER
        elif power < MIN_POWER:
            self.power = MIN_POWER
        else:
            self.power = power

        # map values from [0,100] to [5,10].
        # A frequency of 50Hz translates to a period of 20ms
        # pulse of 1ms is no power, pulse of 2ms is full power.
        power = interp1d([MIN_POWER, MAX_POWER], [5.0, 10.0])(power)
        PWM.set_duty_cycle(self.pin, power)

    def getPower(self):
        return self.power
