import sys
import serial
import time
import numpy as np

## Used with Slave and master ino files 

# master = serial.Serial(port='/dev/ttyACM3', baudrate=2000000, timeout=.1)
slave = serial.Serial(port='/dev/ttyACM1', baudrate=2000000, timeout=.1)
average = []
for t in range(100): 
    start = time.time()
    slave.write(f"{t}\n".encode())
    while True:
        slave_msg = slave.readline()
        if slave_msg.decode().replace("\r\n", "") == str(t+1):
            stop = time.time()
            break
    average.append(stop - start)
    print(t)

print(average[0])
print(np.mean(np.array(average)))
print(np.mean(np.array(average[1:])))