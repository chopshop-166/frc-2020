from math import radians
import pyrealsense2 as rs2
import cv2
import numpy as np


def swapStream(isShooting, s):
    if isShooting:
        s.set_option(rs2.option.enable_auto_exposure, False)
        s.set_option(rs2.option.brightness, 0)
        s.set_option(rs2.option.contrast, 100)
        s.set_option(rs2.option.exposure, 156)
        s.set_option(rs2.option.gain, 75)
        s.set_option(rs2.option.gamma, 300)
        s.set_option(rs2.option.hue, 0)
        s.set_option(rs2.option.saturation, 50)
        s.set_option(rs2.option.sharpness, 50)
        s.set_option(rs2.option.white_balance, 6500)
    else:
        s.set_option(rs2.option.enable_auto_exposure, True)
        s.set_option(rs2.option.brightness, 0)
        s.set_option(rs2.option.contrast, 50)
        s.set_option(rs2.option.gamma, 300)
        s.set_option(rs2.option.hue, 0)
        s.set_option(rs2.option.saturation, 64)
        s.set_option(rs2.option.sharpness, 50)
        s.set_option(rs2.option.white_balance, 4600)


def imgfilter(IMG, LOWER_COLOR, UPPER_COLOR, debugging, WIDTH, HEIGHT):

    # Convert from RGB to HSV, helps with filtering
    HSV = cv2.cvtColor(IMG, cv2.COLOR_BGR2HSV)

    # Create mask within hsv range
    MASK = cv2.inRange(HSV, LOWER_COLOR, UPPER_COLOR)

    # Blur image (buffer for pixel imperfections)
    MEDIAN = cv2.medianBlur(MASK, 5)

    # Edge detection for use in line detection
    MED_EDGES = cv2.Canny(MEDIAN, 50, 150)

    if debugging:
        BLUR = cv2.GaussianBlur(MASK, (3, 3), 0)

        BLUR_EDGES = cv2.Canny(BLUR, 100, 200)
        MASK_EDGES = cv2.Canny(MASK, 100, 200)

        # Empty image for drawing lines (testing)
        FILTERED_LINE_IMG = np.zeros((HEIGHT, WIDTH, 3), np.uint8)
        LINE_IMG = np.zeros((HEIGHT, WIDTH, 3), np.uint8)
    # Find lines in selected image
    lines = cv2.HoughLinesP(MED_EDGES, 1, radians(0.5), 25, maxLineGap=25)
    return lines
