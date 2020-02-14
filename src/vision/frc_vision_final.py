# Contains most of the vision processing, though the most complete is under Offline_Filter
# Likely to become the final product file for vision processing
# The exposure options are likely needed to be adjusted per competition
# They are currently tuned to the testing area
# Tuning for the RealSense camera can be done easily through the ReaslSense Viewer app in a GUI
# One thing I do still need to figure out is assigning the camera to a specific IO port number
# Testing on my laptop, it is only visible when using one port

from math import degrees, radians
import pyrealsense2 as rs2
import cv2
import numpy as np
import time
from networktables import NetworkTables

# Takes in slopes x and y, tests if they are equal to each other or any previously verified line
def unequal(new, old_list):
    variance = 5
    for i in old_list:
        x3, y3, x4, y4 = i[0]
        old_slope = degrees(np.arctan((y4 - y3)/(x4 - x3)))
        min_val = old_slope - variance
        max_val = old_slope + variance
        if min_val < new < max_val:
            return False
    return True

NetworkTables.initialize(server='roborio-166-frc.local')

sd = NetworkTables.getTable('SmartDashboard')
# sd.putNumber('someNumber', 1234)
# otherNumber = sd.getNumber('otherNumber')

WIDTH = 640
HEIGHT = 480
POINT_SAMPLES = 5

pipe = rs2.pipeline()               # The camera's API sucks, but at least I can guarantee setings
config = rs2.config()
config.enable_stream(rs2.stream.color, WIDTH, HEIGHT, rs2.format.bgr8, 60)
config.enable_stream(rs2.stream.depth, WIDTH, HEIGHT, rs2.format.z16, 60)
profile = pipe.start(config)
s = profile.get_device().query_sensors()[1]
s.set_option(rs2.option.brightness, 0)
s.set_option(rs2.option.contrast, 100)
s.set_option(rs2.option.exposure, 45)
s.set_option(rs2.option.gain, 75)
s.set_option(rs2.option.gamma, 100)
s.set_option(rs2.option.hue, 0)
s.set_option(rs2.option.saturation, 50)
s.set_option(rs2.option.sharpness, 0)
s.set_option(rs2.option.white_balance, 2800)

X_VALS = []
Y_VALS = []

pointer = 0


while True:
    start_time = time.time()

    frames = rs2.composite_frame(pipe.wait_for_frames())
    frame = rs2.video_frame(frames.get_color_frame())
    if not frame:
        continue

    IMG = np.asanyarray(frame.get_data())

    # Convert from RGB to HSV, helps with filltering
    HSV = cv2.cvtColor(IMG, cv2.COLOR_BGR2HSV)

    # Define upper and lower bounds for HSV variables
    LOWER_COLOR = np.array([70, 80, 255])
    UPPER_COLOR = np.array([95, 180, 255])

    # Create mask within hsv range
    MASK = cv2.inRange(HSV, LOWER_COLOR, UPPER_COLOR)

    # Various blur method testings
    BLUR = cv2.GaussianBlur(MASK, (3, 3), 0)
    MEDIAN = cv2.medianBlur(MASK, 3)

    # Edge detection on each test for use in line detection
    BLUR_EDGES = cv2.Canny(BLUR, 100, 200)
    MASK_EDGES = cv2.Canny(MASK, 100, 200)
    MED_EDGES = cv2.Canny(MEDIAN, 50, 150)

    # Empty image for drawing lines (testing)
    FILTERED_LINE_IMG = np.zeros((HEIGHT, WIDTH, 3), np.uint8)
    LINE_IMG = np.zeros((HEIGHT, WIDTH, 3), np.uint8)

    # Find lines in selected image
    LINES = cv2.HoughLinesP(MASK_EDGES, 1, radians(.5), 25, maxLineGap=25)

    if LINES is not None:
        NUM_LINES = len(LINES)
        FILTERED_LINES = []
        X_TOTAL = 0
        Y_TOTAL = 0
        for NEW_LINE in LINES:
            x1, y1, x2, y2 = NEW_LINE[0]
            new_slope = degrees(np.arctan((y2 - y1)/(x2 - x1)))
            if FILTERED_LINES:
                if (new_slope < -40 or new_slope > 40) and unequal(new_slope, FILTERED_LINES):
                    FILTERED_LINES.append(NEW_LINE)
                    cv2.line(FILTERED_LINE_IMG, (x1, y1), (x2, y2), (0, 255, 0), 1)
                    X_TOTAL += x1 + x2
                    Y_TOTAL += y1 + y2
            else:
                if new_slope < -40 or new_slope > 40:
                    FILTERED_LINES.append(NEW_LINE)
                    cv2.line(FILTERED_LINE_IMG, (x1, y1), (x2, y2), (0, 255, 0), 1)
                    X_TOTAL += x1 + x2
                    Y_TOTAL += y1 + y2

        NUM_LINES = len(FILTERED_LINES)
        if FILTERED_LINES:
            X_AVG = 0
            Y_AVG = 0
            if len(X_VALS) == POINT_SAMPLES:
                X_VALS[pointer] = X_TOTAL/(2*NUM_LINES)
                Y_VALS[pointer] = Y_TOTAL/(2*NUM_LINES)
                for i in range(len(X_VALS)):
                    X_AVG += X_VALS[i]
                    Y_AVG += Y_VALS[i]
                X_AVG /= POINT_SAMPLES
                Y_AVG /= POINT_SAMPLES

                cv2.circle(FILTERED_LINE_IMG, (int(X_AVG), int(Y_AVG)), 5, [255, 255, 255], -1)
            else:
                X_VALS.append(int(X_TOTAL/(2*NUM_LINES)))
                Y_VALS.append(int(Y_TOTAL/(2*NUM_LINES)))

        for LINE in LINES:
            x1, y1, x2, y2 = LINE[0]
            cv2.line(LINE_IMG, (x1, y1), (x2, y2), (0, 255, 0), 1)
    end_time = time.time()

    cv2.imshow("og lines", LINE_IMG)
    cv2.imshow("lines", FILTERED_LINE_IMG)
    cv2.imshow('OG', IMG)                   # Open the gallery of all my filtered works
    cv2.imshow('Mask', MASK)
    cv2.imshow('blur', BLUR_EDGES)
    cv2.imshow('median', MEDIAN)
    cv2.imshow('med', MED_EDGES)
    cv2.imshow('Mask Edges', MASK_EDGES)

    if pointer == POINT_SAMPLES - 1:
        pointer = 0
    else:
        pointer += 1


    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    end_time = time.time()
    print(end_time - start_time)

cv2.destroyAllWindows()
pipe.stop()
