import cv2
import visionfunctions as vision

while True:
    try:
        cap = cv2.VideoCapture('http://10.1.66.84:1181/?action=stream')
        _, IMG = cap.read()
        isShooting = True

vision.img_filter(IMG)
vision.img_show()

cv2.imshow("OG", IMG)