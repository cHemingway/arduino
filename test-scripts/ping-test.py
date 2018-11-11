#!/usr/bin/python
import serial
import syslog
import time
import json


#The following line is for serial over GPIO
port = '/dev/ttyACM0'


ard = serial.Serial(port,9600,timeout=5)
text = ""
with open('../json-schema/examples/ping-request-example.json') as j:
    text=j.read()
    json_text= json.loads(text)
    text = json.dumps(json_text)
text_bytes = text.encode()
print(text_bytes)
ard.write(text_bytes + b'\n')
time.sleep(2)
print("Res:")
res = ard.readline()
print(res)



