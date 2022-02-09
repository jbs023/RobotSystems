import sys
import time
import logging

sys.path.append(r'../lib')
from adc import ADC
from pin import Pin
from utils import reset_mcu

reset_mcu()

logging_format = "%(asctime)s: %(message)s"
logging.basicConfig(format=logging_format, level=logging.ERROR , datefmt="%H:%M:%S ")

class GrayscaleSensor():
    '''Connector for the gray scale sensor'''
    def __init__(self, ref=1000):
        self.chn_0 = ADC("A0")
        self.chn_1 = ADC("A1")
        self.chn_2 = ADC("A2")
        self.ref = ref

    def read(self):
        adc_value_list = []
        adc_value_list.append(self.chn_0.read())
        adc_value_list.append(self.chn_1.read())
        adc_value_list.append(self.chn_2.read())

        logging.debug("ADC Values: {}".format(adc_value_list))
        return adc_value_list


class UltraSonicSensor():
    '''Connector class for the ultra sonic sensor'''
    def __init__(self, timeout=0.02):
        self.trig = Pin('D2')
        self.echo = Pin('D3')
        self.timeout = timeout

    def _read(self):
        self.trig.low()
        time.sleep(0.01)
        self.trig.high()
        time.sleep(0.00001)
        self.trig.low()
        pulse_end = 0
        pulse_start = 0
        timeout_start = time.time()
        while self.echo.value()==0:
            pulse_start = time.time()
            if pulse_start - timeout_start > self.timeout:
                return -1
        while self.echo.value()==1:
            pulse_end = time.time()
            if pulse_end - timeout_start > self.timeout:
                return -1
        during = pulse_end - pulse_start
        cm = round(during * 340 / 2 * 100, 2)
        return cm

    def read(self, times=10):
        for i in range(times):
            a = self._read()
            if a != -1 or a <= 300:
                logging.debug("Ultrasonic: {}".format(a))
                return a
        return -1