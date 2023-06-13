#https://docs.m5stack.com/en/core/core2
#https://docs.m5stack.com/en/api/core2/axp192_core2
#https://github.com/m5stack/M5Core2/blob/master/src/AXP192.cpp
#https://m5stack.oss-cn-shenzhen.aliyuncs.com/resource/docs/schematic/Core/CORE2_V1.0_SCH.pdf
import board
import busio
import time
import adafruit_focaltouch
import adafruit_pcf8563
from adafruit_display_text.label import Label
import adafruit_touchscreen
import adafruit_button
import adafruit_ntp
import terminalio
import displayio
import socketpool
import wifi
import math
import alarm

i2c = busio.I2C(board.SCL, board.SDA)
radio = socketpool.SocketPool(wifi.radio)
SCREEN_WIDTH = 320
SCREEN_HEIGHT = 240


class PowerController:
    register = {
        'power': 0x00,
        'status': 0x01,
        'otgStatus': 0x04,
        'vc2control': 0x10,
        'vc13control': 0x12,
        'chipVcc': 0x26,
        'display': 0x27,
        'power': 0x30,
        'voffVoltage': 0x31,
        'control': 0x32,
        'chargeControl': 0x33,
        'dcSettings': 0x80,
        'adcEnable1': 0x82,
        'adcEnable2': 0x83,
        'gpio1': 0x92,
        'gpio2': 0x93,
        'signalStatus012': 0x94,
        'functionCtrl34': 0x95,
        'signalStatus34': 0x96,
        'pullDown012': 0x97,
    }
    i2cAddress = 0x34
    converterMin = 0.7
    converterStep = 0.025
    minDisplayVoltage = 2.7
    maxDisplayVoltage = 3.4
    buffer = bytearray(4)
    
    def setDisplayVoltage(self, voltage):
        if voltage < self.minDisplayVoltage or voltage > self.maxDisplayVoltage:
            raise ValueError("Voltage must be between 2.7 and 3.4")
        self.setVoltage(self.register['display'], voltage)
        
    def setMcuVoltage(self, voltage):
        if voltage < 3.0 or voltage > 3.4:
            raise ValueError("Voltage must be between 3.0 and 3.4")
        self.setVoltage(self.register['chipVcc'], voltage)

    def setVoltage(self, reg, voltage):
        if voltage < self.converterMin:
            raise ValueError("Voltage must be greater than 0.7")
        print(f'Voltage: {voltage} - {self.converterMin} / {self.converterStep}')
        value = int((voltage - self.converterMin) / self.converterStep)
        self.write(reg, value)

    def isCharging(self):
        return self.read(self.register['power'], 8) & 0x80 == 0x80
    
    def powerOff(self):
        control = self.read(self.register['control'], 8)
        control |= 0x80
        self.write(self.register['control'], control)
        
    def enableAdc(self, enable):
        if enable:
            self.write(self.register['adcEnable1'], 0xFF)
        else:
            self.write(self.register['adcEnable1'], 0x00)

    def setRegBit(self, reg, bit, value):
        regValue = self.read(reg, 8)
        if value:
            regValue |= bit
        else:
            regValue &= ~bit
        self.write(reg, regValue)

    def enableLed(self, enable):
        self.setRegBit(self.register['signalStatus012'], 0x04, not enable)
            
    def enableSpeaker(self, enable):
        self.setRegBit(self.register['signalStatus012'], 0x02, enable)
        
    def enablePeripheral(self, enable):
        self.setRegBit(self.register['vc2control'], 0x04, enable)
        
    def enableLcd(self, enable):
        self.setRegBit(self.register['vc13control'], 0x02, enable)

    def setChargeCurrent(self, milliamps):
        # There are 8 options for charge current.
        # Round to the nearest option.
        options = [100, 190, 280, 360, 450, 550, 630, 700]
        setting = min(options, key=lambda x:abs(x - milliamps))
        control = self.read(self.register['chargeControl'], 8)
        control &= 0xF0
        control |= options.index(setting)
        self.write(self.register['chargeControl'], control)

            
    def vbusAvailable(self):
        return self.read(self.register['power'], 8) & 0x08 == 0x08
    
    def batteryConnected(self):
        return self.read(self.register['status'], 8) & 0x20 == 0x20
    
    def batteryCharging(self):
        return self.read(self.register['status'], 8) & 0x40 == 0x40
    
    def overheated(self):
        return self.read(self.register['status'], 8) & 0x80 == 0x80

    def preSleep(self):
        self.enableAdc(False)
        self.enableLed(False)
        self.enableLcd(False)

        
    def postSleep(self):
        self.enableLcd(True)
        self.enableLed(True)
        self.enableAdc(True)
        



    def write(self, reg, data):
        print('Ouput:', reg, data)
        i2c.try_lock()
        i2c.writeto(self.i2cAddress, bytearray([reg, data]))
        i2c.unlock()
        
    def read(self, reg, bits):
        if bits not in [8, 16, 32, 13, 12]:
            raise ValueError("Invalid number of bits")
        i2c.try_lock()
        i2c.writeto(self.i2cAddress, bytearray([reg]))
        i2c.readfrom_into(self.i2cAddress, self.buffer, start=0, end=math.ceil(bits / 8))
        i2c.unlock()
        if bits == 8:
            return self.buffer[0]
        elif bits == 16:
            return self.buffer[0] << 8 | self.buffer[1]
        elif bits == 32:
            return self.buffer[0] << 24 | self.buffer[1] << 16 | self.buffer[2] << 8 | self.buffer[3]
        elif bits == 13:
            return self.buffer[0] << 5 | self.buffer[1]
        elif bits == 12:
            return self.buffer[0] << 4 | self.buffer[1]
    
    
class TouchScreen:
    buffer = bytearray(4)

    def __init__(self, powerConverter):
        self.ts = adafruit_focaltouch.Adafruit_FocalTouch(i2c, debug=False)
        board.DISPLAY.root_group = displayio.Group()
        self.pwr = powerConverter
        
    def add(self, obj):
        board.DISPLAY.root_group.append(obj)
        
    def isTouched(self):
        return self.ts.touched()
    
    def waitForTouch(self):
        while not self.isTouched():
            time.sleep(0.1)
            
    def getTouch(self):
        return self.ts.touches[0]
    
    def sleepUntilTouch(self):
        self.pwr.preSleep()
        wifi.radio.enabled = False
        self.setBrightness(0.1)
        touchWake = alarm.pin.PinAlarm(pin=board.TOUCH_INT, value=False)
        alarm.light_sleep_until_alarms(touchWake)
        self.setBrightness(1.0)
        self.pwr.postSleep()
    def setBrightness(self, brightness):

        if brightness < 0.0 or brightness > 1.0:
            raise ValueError("Brightness must be between 0.0 and 1.0")
        
        voltage = self.pwr.minDisplayVoltage + (self.pwr.maxDisplayVoltage - self.pwr.minDisplayVoltage) * brightness
        self.pwr.setDisplayVoltage(voltage)

        
    

class RTC:
    def __init__(self):
        self.rtc = adafruit_pcf8563.PCF8563(i2c)
        self.ntp = adafruit_ntp.NTP(radio, tz_offset=9.5)
        self.setTime(self.ntp.datetime)

    def setTime(self, time):
        self.rtc.datetime = time

    def getTime(self):
        return self.rtc.datetime
    
def getTime(rtc):
    t = rtc.getTime()
    return f'{t.tm_hour:02}:{t.tm_min:02}:{t.tm_sec:02}'

def main():
    pwr = PowerController()
    ts = TouchScreen(pwr)
    rtc = RTC()
    t = rtc.getTime()
    
    font = terminalio.FONT
    clockLabel = Label(terminalio.FONT, text=getTime(rtc), color=0xFFFFFF, scale=5, anchor_point=(0.5, 0.5), anchored_position=(SCREEN_WIDTH // 2, SCREEN_HEIGHT // 2))
    #clockLabel.x = SCREEN_WIDTH // 2 - clockLabel.bounding_box[2] // 2
    #clockLabel.y = SCREEN_HEIGHT // 2 - clockLabel.bounding_box[3] // 2
    ts.add(clockLabel)
    while True:
        activeStart = time.monotonic()
        while time.monotonic() - activeStart < 5.0:
            t = rtc.getTime()
            clockLabel.text = getTime(rtc)
            time.sleep(0.2)
        ts.sleepUntilTouch()
    

if __name__ == "__main__":
    main()