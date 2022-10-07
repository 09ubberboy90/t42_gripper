import serial

arduino = serial.Serial(port='/dev/ttyUSB0', baudrate=9600, timeout=.1)
while True:
    msg = arduino.readline()
    if msg:
        print(msg)
