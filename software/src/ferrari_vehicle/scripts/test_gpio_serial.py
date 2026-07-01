#!/usr/bin/env python3

import serial

ser = serial.Serial("/dev/serial0", 9600, timeout=1)

ser.write(b"Hello from myself!\n")
print(ser.readline())
