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

i2c = busio.I2C(board.SCL, board.SDA)
radio = socketpool.SocketPool(wifi.radio)
SCREEN_WIDTH = 320
SCREEN_HEIGHT = 240


class TouchScreen:
    def __init__(self):
        self.ts = adafruit_focaltouch.Adafruit_FocalTouch(i2c, debug=False)
        board.DISPLAY.root_group = displayio.Group()
    
    def add(self, obj):
        board.DISPLAY.root_group.append(obj)
        
    def isTouched(self):
        return self.ts.touched()
    
    def waitForTouch(self):
        while not self.isTouched():
            time.sleep(0.1)
            
    def getTouch(self):
        return self.ts.touches[0]
    

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
    ts = TouchScreen()
    rtc = RTC()
    t = rtc.getTime()
    
    font = terminalio.FONT
    clockLabel = Label(terminalio.FONT, text=getTime(rtc), color=0xFFFFFF, scale=5, anchor_point=(0.5, 0.5), anchored_position=(SCREEN_WIDTH // 2, SCREEN_HEIGHT // 2))
    #clockLabel.x = SCREEN_WIDTH // 2 - clockLabel.bounding_box[2] // 2
    #clockLabel.y = SCREEN_HEIGHT // 2 - clockLabel.bounding_box[3] // 2
    ts.add(clockLabel)
    while True:
        t = rtc.getTime()
        clockLabel.text = getTime(rtc)
        time.sleep(1)
    

if __name__ == "__main__":
    main()