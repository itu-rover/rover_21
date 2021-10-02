import cv2
import numpy as np

kamera = cv2.VideoCapture(1)
buton_cascade = cv2.CascadeClassifier("detection_models/rot_main.xml")
switch_cascade = cv2.CascadeClassifier("detection_models/rot_switch.xml")
switch2_cascade = cv2.CascadeClassifier("detection_models/rot_switch2.xml")
red_cascade = cv2.CascadeClassifier("detection_models/indicator_red.xml")
socket_cascade = cv2.CascadeClassifier("detection_models/socket.xml")

while 1:
    ret, frame = kamera.read()

    griton = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    buton = buton_cascade.detectMultiScale(griton, 1.3, 4)
    switch = switch_cascade.detectMultiScale(griton, 1.3, 4)
    switch2 = switch2_cascade.detectMultiScale(griton, 1.3, 4)
    red = red_cascade.detectMultiScale(griton, 1.3, 4)
    socket = socket_cascade.detectMultiScale(griton, 1.3, 4)

    for (x, y, w, h) in buton:
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 3)

    for (x, y, w, h) in switch:
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 3)

    for (x, y, w, h) in switch2:
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 3)

    for (x, y, w, h) in red:
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 3)

    for (x, y, w, h) in socket:
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 255), 3)

    cv2.imshow("orjinal", frame)
    if cv2.waitKey(25) & 0xFF == ord("q"):
        break
kamera.release()
cv2.destroyAllWindows()
