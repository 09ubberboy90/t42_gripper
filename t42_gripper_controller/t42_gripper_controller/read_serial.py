import sys
import serial

arduino = serial.Serial(port='/dev/tty'+sys.argv[1], baudrate=115200, timeout=.1)
while True:
    msg = arduino.readline()
    if msg:
        print(msg)
