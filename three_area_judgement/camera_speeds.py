#!/usr/bin/python3

"""
Thorlabsに出すシリアル通信の命令フォーマットは
header部 + データ部分

header: 6byte
    ex:
      モーター速度を決定する命令 4013
      モーターのアドレス xA2
      ラズパイのアドレス x01

      order: \x13\x04\x0E\x00\xA2\x01
"""

import numpy as np
import cv2
import serial
import time
#import pickle

from utils import MotorControl

cap = cv2.VideoCapture(0)
print("Initialized Camera")

dev  = "/dev/ttyUSB0"
rate = 115200
ser  = serial.Serial(dev, rate, timeout=10)
print("Initialize serial connect")

ser.write(b'\x23\x02\x00\x00\x11\x01')
print("Thorlabs Initialize")

ser.write(b'\xF4\x04\x04\x00\xA2\x01\x01\x00\x12\x00')
print("Setted S-Curve Parameter")

controller = MotorControl(ser)

time.sleep(2)

threshould = 20000

x = []
y = []

start_time = time.time()

while(True):
    # Capture frame by CCD camera
    # ret, frame = cap.read()

    while True:
        ret, frame = cap.read()
        if ret: break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    data = np.array(gray)

    height, width = data.shape

    # convolution making
    conved = np.add.reduce(data, axis=1)

    filtered_index, = np.where(conved >= threshould)

    bottom = filtered_index[-1] if len(filtered_index) != 0 else 0
    middle = int(height/2)

    controller.update(bottom - middle)

    print("{}".format(bottom - middle))

    cv2.rectangle(frame, (0, middle),      (10, bottom),         (0, 0, 255), thickness=-1)
    cv2.rectangle(frame, (0, middle - 10), (width, middle + 10), (0, 255, 0), thickness=3)

    cv2.imshow('frame', frame)

    x.append(time.time() - start_time)
    y.append(bottom - middle)

    # break if ESC entered
    key = cv2.waitKey(1)

    if key == 27:
        break

  #  time.sleep(0.95) # stop 10 m sec

data = np.c=[x, y]
np.savetxt("Feedback_data.csv", data, delimiter = ",")


ser.write(b'\x65\x04\x01\x01\xA2\x01')
print("stopped motor")
ser.write(b'\x65\x04\x01\x01\x11\x01')
print("stopped APT controller")

ser.close()
cap.release()
cv2.destroyAllWindows()


#with open("x_array.pkl", "wb") as f:
 #   pickle.dump(x, f)


#with open("y_array.pkl", "wb") as f:
 #   pickle.dump(y, f)
