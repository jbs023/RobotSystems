timer = [
    {
        "arr": 0
    }
] * 4

class Servo():
    MAX_PW = 2500
    MIN_PW = 500
    _freq = 50

    def __init__(self, pwm):
        super().__init__()
        self.pwm = pwm
        self.pwm.period(4095)
        prescaler = int(float(self.pwm.CLOCK) /
                        self.pwm._freq/self.pwm.period())
        self.pwm.prescaler(prescaler)
        # self.angle(90)

    def map(self, x, in_min, in_max, out_min, out_max):
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    # angle ranges -90 to 90 degrees
    def angle(self, angle):
        pass


class PWM():
    REG_CHN = 0x20
    REG_FRE = 0x30
    REG_PSC = 0x40
    REG_ARR = 0x44

    ADDR = 0x14

    CLOCK = 72000000

    def __init__(self, channel, debug="critical"):
        if isinstance(channel, str):
            if channel.startswith("P"):
                channel = int(channel[1:])
            else:
                raise ValueError("PWM channel should be between [P1, P14], not {0}".format(channel))

        self.debug = debug
        self.channel = channel
        self.timer = int(channel/4)
        self.bus = None
        self._pulse_width = 0
        self._freq = 50
        self.freq(50)

    def i2c_write(self, reg, value):
        pass

    def freq(self, *freq):
        return 1

    def prescaler(self, *prescaler):
        return 1

    def period(self, *arr):
        return 1

    def pulse_width(self, *pulse_width):
        return 1

    def pulse_width_percent(self, *pulse_width_percent):
        return 1


class Pin():
    OUT = 0
    IN = 0
    IRQ_FALLING = 0
    IRQ_RISING = 0
    IRQ_RISING_FALLING = 0
    PULL_UP = 0
    PULL_DOWN = 0
    PULL_NONE = 0

    _dict = {
        "D0":  17,
        "D1":  18,
        "D2":  27,
        "D3":  22,
        "D4":  23,
        "D5":  24,
        "D6":  25,
        "D7":  4,
        "D8":  5,
        "D9":  6,
        "D10": 12,
        "D11": 13,
        "D12": 19,
        "D13": 16,
        "D14": 26,
        "D15": 20,
        "D16": 21,
        "SW":  19,
        "LED": 26,
        "BOARD_TYPE": 12,
        "RST": 16,
        "BLEINT": 13,
        "BLERST": 20,
        "MCURST": 21,
    }

    def __init__(self, *value):
        super().__init__()
        self.check_board_type()

        if len(value) > 0:
            pin = value[0]
        if len(value) > 1:
            mode = value[1]
        else:
            mode = None
        if len(value) > 2:
            setup = value[2]
        else:
            setup = None

        if isinstance(pin, str):
            try:
                self._board_name = pin
                self._pin = self.dict()[pin]
            except Exception as e:
                print(e)
                print('Pin should be in %s, not %s' % (self._dict.keys(), pin))
        elif isinstance(pin, int):
            self._pin = pin
        else:
            print('Pin should be in %s, not %s' % (self._dict.keys(), pin))
        self._value = 0
        self.init(mode, pull=setup)
        # self._info("Pin init finished.")

    def check_board_type(self):
        pass

    def init(self, mode, pull=PULL_NONE):
        pass

    def dict(self, *_dict):
        if len(_dict) == 0:
            return self._dict
        else:
            if isinstance(_dict, dict):
                self._dict = _dict
            else:
                self._error(
                    'argument should be a pin dictionary like {"my pin": ezblock.Pin.cpu.GPIO17}, not %s' % _dict)

    def __call__(self, value):
        return self.value(value)

    def value(self, *value):
        return value

    def on(self):
        return self.value(1)

    def off(self):
        return self.value(0)

    def high(self):
        return self.on()

    def low(self):
        return self.off()

    def mode(self, *value):
        if len(value) == 0:
            return self._mode
        else:
            mode = value[0]
            self._mode = mode

    def pull(self, *value):
        return self._pull

    def irq(self, handler=None, trigger=None, bouncetime=200):
        self.mode(self.IN)

    def name(self):
        return "GPIO%s" % self._pin

    def names(self):
        return [self.name, self._board_name]

    class cpu(object):
        GPIO17 = 17
        GPIO18 = 18
        GPIO27 = 27
        GPIO22 = 22
        GPIO23 = 23
        GPIO24 = 24
        GPIO25 = 25
        GPIO26 = 26
        GPIO4 = 4
        GPIO5 = 5
        GPIO6 = 6
        GPIO12 = 12
        GPIO13 = 13
        GPIO19 = 19
        GPIO16 = 16
        GPIO26 = 26
        GPIO20 = 20
        GPIO21 = 21

        def __init__(self):
            pass


class ADC():
    ADDR = 0x14

    def __init__(self, chn):
        if isinstance(chn, str):
            if chn.startswith("A"):
                chn = int(chn[1:])
            else:
                raise ValueError(
                    "ADC channel should be between [A0, A7], not {0}".format(chn))
        if chn < 0 or chn > 7:
            self._error('Incorrect channel range')
        chn = 7 - chn
        self.chn = chn | 0x10
        self.reg = 0x40 + self.chn

    def read(self):
        return 0

    def read_voltage(self):
        return self.read*3.3/4095

class fileDB():
    def __init__(self, db=None):
        if db != None:
            self.db = db
        else:
            self.db = "config"

    def get(self, name, default_value=None):
        if default_value:
            return default_value
        else:
            return 0

    def set(self, name, value):
        pass
