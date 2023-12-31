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
        'ldoVcc': 0x28,
        'power': 0x30,
        'voffVoltage': 0x31,
        'control': 0x32,
        'chargeControl': 0x33,
        'rtcChargeControl': 0x35,
        'buttonParameters': 0x36,
        'dcSettings': 0x80,
        'adcEnable1': 0x82,
        'adcEnable2': 0x83,
        'gpio0': 0x90,
        'gpio0ld0': 0x91,
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
    adcItems = {
        'batteryVoltage': 0x80,
        'batteryCurrent': 0x40,
        'acVoltage': 0x20,
        'acCurrent': 0x10,
        'vbusVoltage': 0x08,
        'vbusCurrent': 0x04,
        'apsVoltage': 0x02,
        'tsPinCurrent': 0x01
    }
    
    def __init__(self):
        self.configPowerBus(enableBoost=False, vbusLimit=None, inputCurrentLimit=None)
        self.initGpio()
        self.setRtcCharge(True, 3.0, 200)
        self.setMcuVoltage(3.35)
        self.setDisplayVoltage(2.8)
        self.setPeripheralVoltage(2.8)
        self.setVibratorVoltage(2)
        self.enableSpeaker(True)
        self.enablePeripherals(True)
        self.enableLcd(True)
        self.enableLed(True)
        self.setChargeCurrent(700)
        self.setButtonParameters(startupTime=1, longPressTime=1, shutdownTime=8, keepOnForPress=False, longerPwrOkWait=True)
        self.setAdcFeatures(['batteryVoltage', 'batteryCurrent', 'vbusVoltage', 'vbusCurrent', 'acVoltage', 'acCurrent', 'tsPinCurrent', 'apsVoltage'])
        self.lcdReset()
        if self.vbusAvailable():
            self.enableVbus(True)

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
        value = int((voltage - self.converterMin) / self.converterStep)
        self.write(reg, value)
        
    def setButtonParameters(self, startupTime, longPressTime, shutdownTime, keepOnForPress=True, longerPwrOkWait=False):
        try:
            startupTime = [0.128, 0.512, 1, 2].index(startupTime)
        except ValueError:
            raise ValueError("Invalid startup time")
        try:
            longPressTime = [1, 1.5, 2, 2.5].index(longPressTime)
        except ValueError:
            raise ValueError("Invalid long press time")
        try:
            shutdownTime = [4, 6, 8, 10].index(shutdownTime)
        except ValueError:
            raise ValueError("Invalid shutdown time")

        reg = startupTime << 6 | longPressTime << 4 | shutdownTime | (not keepOnForPress) << 3 | longerPwrOkWait << 2
        self.write(self.register['buttonParameters'], reg)

    def lcdReset(self):
        self.setRegBit(self.register['signalStatus34'], 0x02, True)
        time.sleep(0.1)
        self.setRegBit(self.register['signalStatus34'], 0x02, False)
        time.sleep(0.1)

    def setAdcFeatures(self, features):
        reg = 0
        for feature in features:
            try:
                reg |= self.adcItems[feature]
            except KeyError:
                raise ValueError("Invalid ADC feature")
        self.write(self.register['adcEnable1'], reg)

    ##
    # @brief Set the RTC Charge Rate
    # @param charge: True or False, whether to enable RTC charge
    # @param voltage: Max charging voltage in volts, possible values are: 3.1, 3.0 or 2.5
    # @param chargeCurrent: Max RTC charging current (in uA) possible values are: 50, 100, 200 or 400
    def setRtcCharge(self, charge, voltage, chargeCurrent):
        reg = self.read(self.register['rtcChargeControl'], 8) & 0x1C
        if charge:
            reg |= 0x80
            try:
                voltage = [3.1, 3.0, 3.0, 2.5].index(voltage)
            except ValueError:
                raise ValueError("Invalid RTC charge voltage")
            try:
                chargeCurrent = [50, 100, 200, 400].index(chargeCurrent)
            except ValueError:
                raise ValueError("Invalid RTC charge current")
                

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
            
    def setGpio0Ld0Voltage(self, voltage):
        if voltage < 1.8 or voltage > 3.3:
            raise ValueError("Voltage must be between 1.8 and 3.3")
        # Set LD0 voltage. Uses 0.1V steps starting at 1.8V.
        value = int((voltage - 1.8) / 0.1)
        g0ld0 = self.read(self.register['gpio0ld0'], 8) & 0x0F
        self.write(self.register['gpio0ld0'], g0ld0 | value << 4)

    def enableVbus(self, enable):
        if enable:
            self.setGpio0Ld0Voltage(3.3)
            # Enable LD0 output.
            gpio0 = self.read(self.register['gpio0'], 8) & 0xF8
            self.write(self.register['gpio0'], gpio0 | 0x02)
            self.enableExternalPower(True)
        else:
            self.enableExternalPower(False)
            # Disable LD0 output.
            gpio0 = self.read(self.register['gpio0'], 8) & 0xF8
            self.write(self.register['gpio0'], gpio0 | 0x07)
            

    def setRegBit(self, reg, bit, value):
        regValue = self.read(reg, 8)
        if value:
            regValue |= bit
        else:
            regValue &= ~bit
        self.write(reg, regValue)

    def enableLed(self, enable):
        self.setRegBit(self.register['signalStatus012'], 0x02, not enable)
            
    def initGpio(self):
        gpio1 = self.read(self.register['gpio1'], 8) & 0xf8
        gpio2 = self.read(self.register['gpio2'], 8) & 0xf8
        self.write(self.register['gpio1'], gpio1)
        self.write(self.register['gpio2'], gpio2)
        #gpio34 = self.read(self.register['functionCtrl34'], 8) & 0x72
        #self.write(self.register['functionCtrl34'], gpio34 | 0x84)

    def configPowerBus(self, enableBoost=False, vbusLimit=None, inputCurrentLimit=None):
        control = self.read(self.register['power'], 8) & 0x04
        if inputCurrentLimit == None:
            control |= 0x02
        elif inputCurrentLimit == 0.1:
            control |= 0x01
        elif inputCurrentLimit == 0.5:
            control |= 0x00
        else:
            raise ValueError("Invalid input current limit")
        
        if vbusLimit == None:
            control |= 0x00
        elif vbusLimit < 4.0 or vbusLimit > 4.7:
            raise ValueError("Vbus limit must be between 4.0 and 4.7")
        else:
            vbusLimit = int((vbusLimit - 4.0) / 0.1) | 0x08
            control |= vbusLimit << 3
        
        if enableBoost:
            control |= 0x80

    def enableSpeaker(self, enable):
        self.setRegBit(self.register['signalStatus012'], 0x04, enable)
        
    def enablePeripherals(self, enable):
        self.setRegBit(self.register['vc2control'], 0x04, enable)
        
    def enableLcd(self, enable):
        self.setRegBit(self.register['vc13control'], 0x02, enable)
        
    def enableExternalPower(self, enable):
        self.setRegBit(self.register['vc2control'], 0x04, enable)

    ##
    # @brief Set the charge current for the main battery.
    # @param milliamps: The charge current in milliamps (from 100 to 700)
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
    
    def enabledPeripherals(self, enable):
        self.setRegBit(self.register['vc13control'], 0x04, enable)
        
    def enableVibrator(self, enable):
        self.setRegBit(self.register['vc13control'], 0x08, enable)
        
    def setPeripheralVoltage(self, voltage):
        if voltage < 1.8 or voltage > 3.3:
            raise ValueError("Voltage must be between 1.8 and 3.3")
        step = (voltage - 1.8) / 0.1
        reg = self.read(self.register['ldoVcc'], 8) & 0x0F
        self.write(self.register['ldoVcc'], int(step) << 4 | reg)
        
    def setVibratorVoltage(self, voltage):
        if voltage < 1.8 or voltage > 3.3:
            raise ValueError("Voltage must be between 1.8 and 3.3")
        step = (voltage - 1.8) / 0.1
        reg = self.read(self.register['ldoVcc'], 8) & 0xF0
        self.write(self.register['ldoVcc'], int(step) | reg)

    def preSleep(self):
        self.enableAdc(False)
        self.enableLed(False)
        self.enableLcd(False)
        self.enableSpeaker(False)

        
    def postSleep(self):
        self.enableLcd(True)
        self.enableLed(True)
        self.enableAdc(True)
        
    def write(self, reg, data):
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
        else:
            return 0
    
# Vibration motor control class
# Runs through a sequence in the background.
# Sequences take the format: [[power, duration], [power, duration], ...]
# A power of 0 means off.
class Vibrator:
    sequence = []
    seqStep = 0
    stepEndTime = 0
    seqRunning = False

    def __init__(self, pwrCtrl):
        self.pwrCtrl = pwrCtrl

    def runSequence(self, sequence):
        self.sequence = sequence
        self.pwrCtrl.enableVibrator(True)
        self.nextStep()
        
    def update(self):
        if not self.seqRunning:
            return
        if time.monotonic() > self.stepEndTime:
            self.nextStep()

    def nextStep(self):
        if self.seqStep >= len(self.sequence):
            self.pwrCtrl.enableVibrator(False)
            self.seqRunning = False
            return
        self.pwrCtrl.setVibratorVoltage(self.sequence[self.seqStep][0])
        self.stepEndTime = time.monotonic() + self.sequence[self.seqStep][1]
        self.seqStep += 1
        self.seqRunning = True

class MotionSensor:
    # The M5Stack Core2 has a built-in accelerometer and gyroscope.
    # It uses a MPU6886.
    # See: https://github.com/m5stack/M5Core2/blob/master/src/utility/MPU6886.cpp
    # https://m5stack.oss-cn-shenzhen.aliyuncs.com/resource/docs/datasheet/core/MPU-6886-000193%2Bv1.1_GHIC_en.pdf
    reg = {
	'ADDRESS': 0x68,
	'WHOAMI': 0x75,
	'ACCEL_INTEL_CTRL': 0x69,
	'SMPLRT_DIV': 0x19,
	'INT_PIN_CFG': 0x37,
	'INT_ENABLE': 0x38,
	'ACCEL_XOUT_H': 0x3B,
	'ACCEL_XOUT_L': 0x3C,
	'ACCEL_YOUT_H': 0x3D,
	'ACCEL_YOUT_L': 0x3E,
	'ACCEL_ZOUT_H': 0x3F,
	'ACCEL_ZOUT_L': 0x40,
	'TEMP_OUT_H': 0x41,
	'TEMP_OUT_L': 0x42,
	'GYRO_XOUT_H': 0x43,
	'GYRO_XOUT_L': 0x44,
	'GYRO_YOUT_H': 0x45,
	'GYRO_YOUT_L': 0x46,
	'GYRO_ZOUT_H': 0x47,
	'GYRO_ZOUT_L': 0x48,
	'USER_CTRL': 0x6A,
	'PWR_MGMT_1': 0x6B,
	'PWR_MGMT_2': 0x6C,
	'CONFIG': 0x1A,
	'GYRO_CONFIG': 0x1B,
	'ACCEL_CONFIG': 0x1C,
	'ACCEL_CONFIG2': 0x1D,
	'FIFO_EN': 0x23,
    'ACCEL_WOM_X_THR': 0x20,
    'ACCEL_WOM_Y_THR': 0x21,
    'ACCEL_WOM_Z_THR': 0x22,
    'ACCEL_INTEL_CTRL': 0x69,
    }
    
    accelReg = {
        'XA': 0,
        'YA': 1,
        'ZA': 2,
        'XG': 3,
        'YG': 4,
        'ZG': 5,
        'TEMP': 6,
    }
    
    fsyncReg = ['NONE', 'TEMP', 'XG', 'YG', 'ZG', 'XA', 'YA', 'ZA']
    buffer = bytearray(6)
    
    CLOCK_INTERNAL = 0x00
    CLOCK_AUTO = 0x01
    CLOCK_STOP = 0x07
    
    ACCEL_RANGE_2G = 0x00
    ACCEL_RANGE_4G = 0x01
    ACCEL_RANGE_8G = 0x02
    ACCEL_RANGE_16G = 0x03
    
    accelRangeValue = [2.0, 4.0, 8.0, 16.0]
    
    SAMPLES_4PERSEC = 0x00
    SAMPLES_8PERSEC = 0x01
    SAMPLES_16PERSEC = 0x02
    SAMPLES_32PERSEC = 0x03

    ACCEL_BANDWIDTH_5HZ = 0x06
    ACCEL_BANDWIDTH_10HZ = 0x05
    ACCEL_BANDWIDTH_21HZ = 0x04
    ACCEL_BANDWIDTH_44HZ = 0x03
    ACCEL_BANDWIDTH_99HZ = 0x02
    ACCEL_BANDWIDTH_218HZ = 0x01
    ACCEL_BANDWIDTH_420HZ = 0x07
    ACCEL_BANDWIDTH_1046HZ = 0x08
    
    GYRO_RANGE_250DPS = 0x00
    GYRO_RANGE_500DPS = 0x01
    GYRO_RANGE_1000DPS = 0x02
    GYRO_RANGE_2000DPS = 0x03
    
    gyroRangeValue = [250.0, 500.0, 1000.0, 2000.0]
    
    GYRO_BANDWIDTH_5HZ = 0x06
    GYRO_BANDWIDTH_10HZ = 0x05
    GYRO_BANDWIDTH_20HZ = 0x04
    GYRO_BANDWIDTH_41HZ = 0x03
    GYRO_BANDWIDTH_92HZ = 0x02
    GYRO_BANDWIDTH_176HZ = 0x01
    GYRO_BANDWIDTH_250HZ = 0x00
    GYRO_BANDWIDTH_3281HZ = 0x20
    GYRO_BANDWIDTH_8173HZ = 0x10
    
    FIFO_MODE_OVERWRITE = 0x00
    FIFO_MODE_STOP = 0x01
    
    INTERNAL_SAMPLE_RATE = 1000
    
    INTERRUPT_X_AXIS = 7
    INTERRUPT_Y_AXIS = 6
    INTERRUPT_Z_AXIS = 5
    INTERRUPT_FIFO_OFLOW = 4
    INTERRUPT_GDRIVE = 2
    INTERRUPT_DATA_RDY = 0
    
    WOM_OR = 0
    WOM_AND = 1
    
    lowPowerMode = False
    
    i2cAddress = 0x68
    def __init__(self):
        if not self.checkId():
            raise ValueError("Invalid sensor")
        self.reset()
        self.setPowerMode(sleep=False, cycle=False, tempDisable=False, gyroStandby=False, clockSelect=self.CLOCK_AUTO)
        self.setAccelConfig(self.ACCEL_RANGE_8G, self.SAMPLES_8PERSEC, self.ACCEL_BANDWIDTH_218HZ)
        self.setGyroConfig(self.GYRO_RANGE_2000DPS, self.GYRO_BANDWIDTH_176HZ)
        self.configureFifo(enable=False, bufferMode=self.FIFO_MODE_OVERWRITE, gyroEn=False, accelEn=False)
        self.setSampleRate(167)
        self.configureInterrupts([self.INTERRUPT_DATA_RDY], activeLow=False, openDrain=False, latch=True, clearOnRead=False, fsyncLevel=False, fsyncEnable=False)
        time.sleep(0.1)
    
    def read(self, address, buffer, length=-1):
        if length == -1:
            length = len(buffer)
        i2c.try_lock()
        i2c.writeto(self.i2cAddress, bytearray([address]))
        i2c.readfrom_into(self.i2cAddress, buffer, start=0, end=length)
        i2c.unlock()

    def write(self, address, buffer, length=-1):
        if length == -1:
            length = len(buffer)
        i2c.try_lock()
        i2c.writeto(self.i2cAddress, bytearray([address]) + buffer)
        i2c.unlock()
        time.sleep(0.001)
        
    def checkId(self):
        buffer = bytearray(1)
        self.read(self.reg['WHOAMI'], buffer, 1)
        return buffer[0] == 0x19
    
    def pollBit(self, address, bit):
        buffer = bytearray(1)
        self.read(address, buffer, 1)
        return buffer[0] & (1 << bit) != 0
    
    def waitBit(self, address, bit, value):
        while self.pollBit(address, bit) != value:
            time.sleep(0.001)

    def reset(self):
        self.write(self.reg['PWR_MGMT_1'], bytearray([0x00]))
        time.sleep(0.01)
        self.write(self.reg['PWR_MGMT_1'], bytearray([0x80]))
        time.sleep(0.01)
        self.waitBit(self.reg['PWR_MGMT_1'], 7, False)
        
    def setPowerMode(self, sleep=False, cycle=False, tempDisable=False, gyroStandby=False, clockSelect=CLOCK_AUTO):
        flags = sleep << 6 | cycle << 5 | tempDisable << 3 | gyroStandby << 4 | clockSelect
        self.write(self.reg['PWR_MGMT_1'], bytearray([flags]))
        
    def setAccelStandby(self, accelList):
        self.read(self.reg['PWR_MGMT_2'], self.buffer, 1)
        for id in accelList:
            if id in self.accelReg:
                if accelList[id] == True:
                    self.buffer[0] &= ~(1 << (5 - self.accelReg[id]))
                else:
                    self.buffer[0] |= 1 << (5 - self.accelReg[id])
        self.write(self.reg['PWR_MGMT_2'], self.buffer, 1)
                
    def setAccelConfig(self, range, sampleRate, bandwidth):
        self.write(self.reg['ACCEL_CONFIG'], bytearray([range << 3 | bandwidth]))
        self.write(self.reg['ACCEL_CONFIG2'], bytearray([sampleRate << 4 | bandwidth]))
        self.accelRange = range
        self.accelSampleRate = sampleRate
        self.accelBandwidth = bandwidth

    def setGyroConfig(self, scale, bandwidth):
        self.write(self.reg['GYRO_CONFIG'], bytearray([scale << 3 | (bandwidth >> 3)]))
        self.read(self.reg['CONFIG'], self.buffer, 1)
        self.buffer[0] &= 0b01111000
        self.buffer[0] |= (bandwidth & 0x07)
        self.write(self.reg['CONFIG'], self.buffer, 1)
        self.gyroScale = scale
        self.gyroBandwidth = bandwidth
        
    def setFsyncConfig(self, mode=None):
        self.read(self.reg['CONFIG'], self.buffer, 1)
        self.buffer[0] &= 0b11100111
        if mode and mode in self.fsyncReg:
            self.buffer[0] |= (self.fsyncReg.index(mode) << 3)
        self.write(self.reg['CONFIG'], self.buffer, 1)
        
    def copyBits(self, address, bitList):
        self.read(address, self.buffer, 1)
        for bit in bitList:
            self.buffer[0] &= ~(1 << bit)
            self.buffer[0] |= (bitList[bit] << bit)
        self.write(address, self.buffer, 1)

    def configureFifo(self, enable, bufferMode, gyroEn, accelEn):
        self.copyBits(self.reg['CONFIG'], {6: bufferMode})
        self.copyBits(self.reg['FIFO_EN'], {4: gyroEn, 3: accelEn})
        self.copyBits(self.reg['USER_CTRL'], {6: enable})
        
        
    def resetFifo(self):
        self.copyBits(self.reg['USER_CTRL'], {2: True})
        
    def setSampleRate(self, rate):
       self.buffer[0] = round((1000 / rate) - 1)
       self.write(self.reg['SMPLRT_DIV'], self.buffer, 1)
        
    def configureInterrupts(self, intList, activeLow=False, openDrain=False, latch=False, clearOnRead=False, fsyncLevel=False, fsyncEnable=False):
        self.copyBits(self.reg['INT_PIN_CFG'], {7: activeLow, 6: openDrain, 5: latch, 4: clearOnRead, 3: fsyncLevel, 2: fsyncEnable})
        flags = 0
        for interrupt in intList:
            flags |= 1 << interrupt
        self.write(self.reg['INT_ENABLE'], bytearray([flags]))
            
    def setWakeOnMotionThreshold(self, x=None, y=None, z=None):
        if x != None:
            self.write(self.reg['ACCEL_WOM_X_THR'], bytearray([x]))
        if y != None:
            self.write(self.reg['ACCEL_WOM_Y_THR'], bytearray([y]))
        if z != None:
            self.write(self.reg['ACCEL_WOM_Z_THR'], bytearray([z]))

    def setAccelIntelligence(self, womDetect=False, intelligentMode=False, outputLimit=True, womMode=WOM_OR):
        self.write(self.reg['ACCEL_INTEL_CTRL'], bytearray([womDetect << 7 | intelligentMode << 6 | (not outputLimit) << 1 | womMode]))

    def wakeOnMotion(self, threshold, x=False, y=False, z=False):
        self.setPowerMode(sleep=False, cycle=False, tempDisable=False, gyroStandby=False, clockSelect=self.CLOCK_AUTO)
        self.setAccelStandby({'XA': False, 'YA': False, 'ZA': False, 'XG': True, 'YG': True, 'ZG': True})
        self.setAccelConfig(self.ACCEL_RANGE_8G, self.SAMPLES_8PERSEC, self.ACCEL_BANDWIDTH_218HZ)
        self.configureInterrupts([self.INTERRUPT_X_AXIS, self.INTERRUPT_Y_AXIS, self.INTERRUPT_Z_AXIS])
        self.setWakeOnMotionThreshold(x=32, y=32, z=32)
        self.setAccelIntelligence(womDetect=True, intelligentMode=True, outputLimit=True, womMode=self.WOM_OR)
        self.setSampleRate(200)
        self.setPowerMode(cycle=True)
        
        
        
    ##
    #  @brief Reads the temperature from the MPU6886.
    #  @return Returns a float value representing the temperature in degrees Celsius.
    def getTemperature(self):
        self.read(self.reg['TEMP_OUT_H'], self.buffer, 2)
        return (self.buffer[0] << 8 | self.buffer[1]) / 326.8 + 25
    
    ##
    #  @brief Reads the gyroscope from the MPU6886.
    #  @return Returns a tuple of the x, y, and z gyroscope values.
    def getGyro(self):
        self.read(self.reg['GYRO_XOUT_H'], self.buffer, 6)
        data = (self.buffer[0] << 8 | self.buffer[1]), (self.buffer[2] << 8 | self.buffer[3]), (self.buffer[4] << 8 | self.buffer[5])
        scalar = self.gyroRangeValue[self.gyroScale] / 32768.0
        data = tuple(x * scalar for x in data)
        return data
    
    ##
    #  @brief Reads the accelerometer from the MPU6886.
    #  @return Returns a tuple of the x, y, and z accelerometer values.
    def getAccel(self):
        self.read(self.reg['ACCEL_XOUT_H'], self.buffer, 6)
        data = (self.buffer[0] << 8 | self.buffer[1]), (self.buffer[2] << 8 | self.buffer[3]), (self.buffer[4] << 8 | self.buffer[5])
        scalar = self.accelRangeValue[self.accelRange] / 32768.0
        data = tuple(x * scalar for x in data)
        return data


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
    #mc = MotionSensor()

    
    pwr.setVibratorVoltage(3.2)
    pwr.enableVibrator(False)
    font = terminalio.FONT
    clockLabel = Label(terminalio.FONT, text=getTime(rtc), color=0xFFFFFF, scale=5, anchor_point=(0.5, 0.5), anchored_position=(SCREEN_WIDTH // 2, SCREEN_HEIGHT // 2))
    accelLabel = Label(terminalio.FONT, text='Accel', color=0xFFFFFF, scale=2, anchor_point=(0.5, 0.5), anchored_position=(SCREEN_WIDTH // 2, SCREEN_HEIGHT // 2 + 50))
    #clockLabel.x = SCREEN_WIDTH // 2 - clockLabel.bounding_box[2] // 2
    #clockLabel.y = SCREEN_HEIGHT // 2 - clockLabel.bounding_box[3] // 2
    ts.add(clockLabel)
    ts.add(accelLabel)
    while True:
        activeStart = time.monotonic()
        pwr.enableLed(True)
        while time.monotonic() - activeStart < 5.0:
            t = rtc.getTime()
            clockLabel.text = getTime(rtc)
            #accelLabel.text = str(mc.getAccel())
            time.sleep(0.05)
        #mc.wakeOnMotion(32, x=True, y=True, z=True)
        pwr.enableLed(False)
        ts.sleepUntilTouch()
        pwr.enableVibrator(True)
        time.sleep(0.1)
        pwr.enableVibrator(False)
    

if __name__ == "__main__":
    main()