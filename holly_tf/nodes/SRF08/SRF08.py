#!/usr/bin/env python

import Adafruit_GPIO.I2C as I2C
import time


class SRF08:
    delay_time = 0.1

    def __init__(self, address=0xE0, busnum=I2C.get_default_bus()):
        # Create I2C device.
        self._device = I2C.Device(address, busnum)

    def distance(self):
        self._device.write8(0x00, 0x51)
        time.sleep(self.delay_time)
        range_byte_1 = self._device.readU8(2)
        range_byte_2 = self._device.readU8(3)

        distance = (range_byte_1 << 8) + range_byte_2
        return distance
