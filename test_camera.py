#!/usr/bin/python3

import numpy as np
import cv2
import serial
from time import sleep

cap = cv2.VideoCapture(0)

while(True):
    # Capture frame by CCD camera
    ret, frame = cap.read()

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    data = np.array(gray)

    height, width = data.shape

    # convolution making
    conved = np.add.reduce(data, axis=1)

    threshould = 20000
    filtered_index, = np.where(conved >= threshould)

    if len(filtered_index) == 0:
        None
    else:
        bottom = filtered_index[-1]
        middle = int(height/2)

        print("{}, {}".format(bottom, middle))
        cv2.rectangle(frame, (0, middle), (10, bottom), (0, 0, 255), thickness=-1)

    cv2.imshow('frame', frame)
    # middle of vertical lines (shape = 480 * 640)
    # gray scale threshould is 80


    # break if ESC entered
    key = cv2.waitKey(1)

    if key == 27:
        break


cap.release()
cv2.destroyAllWindows()

