# Final FRC 2020 vision program
# Exposure setting should be tuned on a per-competition basis
# They are currently tuned to the testing area
# Tuning for the RealSense camera can be done easily through the ReaslSense Viewer app in a GUI

from math import degrees, radians
import pyrealsense2 as rs2
import cv2
import numpy as np
import time
from cscore import CameraServer
from networktables import NetworkTables

ANGLE_THRESHOLD = 40

# Resolution
WIDTH = 640
HEIGHT = 480
POINT_SAMPLES = 5
# Angle value of each pixel
FOV_ANGLE = 82.5
PIXEL_ANGLE = FOV_ANGLE/WIDTH

# Enable CameraServer
cs = CameraServer.getInstance()
cs.enableLogging()

outputStream = cs.putVideo("Color", WIDTH, HEIGHT)

# Takes in slopes x and y, tests if they are equal to each other or any previously verified line

def unequal(new, old_list):
    variance = 5
    for i in old_list:
        x1, y1, x2, y2 = i[0]
        old_slope = degrees(np.arctan((y2 - y1)/(x2 - x1)))
        if abs(new - old_slope) < variance:
            return False
    return True


def newLine(filtered_lines, new_line, filtered_line_img, x1, y1, x2, y2, X_TOTAL, Y_TOTAL):
    filtered_lines.append(new_line)
    cv2.line(filtered_line_img, (x1, y1), (x2, y2), (0, 255, 0), 1)
    X_TOTAL += x1 + x2
    Y_TOTAL += y1 + y2

    return X_TOTAL, Y_TOTAL

NetworkTables.initialize(server='pi-166-frc.local')

sd = NetworkTables.getTable('SmartDashboard')
# sd.putNumber('someNumber', 1234)
# otherNumber = sd.getNumber('otherNumber')


# Camera settings
pipe = rs2.pipeline()
config = rs2.config()
config.enable_stream(rs2.stream.color, WIDTH, HEIGHT, rs2.format.bgr8, 30)
config.enable_stream(rs2.stream.depth, WIDTH, HEIGHT, rs2.format.z16, 30)
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

VALS = []

while True:
    start_time = time.time()

    # Waits to get frames, and gets information about the frame.
    frames = rs2.composite_frame(pipe.wait_for_frames())
    frame = rs2.video_frame(frames.get_color_frame())
    depth = rs2.depth_frame(frames.get_depth_frame())

    if not frame:
        continue

    IMG = np.asanyarray(frame.get_data())

    # Convert from RGB to HSV, helps with filtering
    HSV = cv2.cvtColor(IMG, cv2.COLOR_BGR2HSV)

    # Define upper and lower bounds for HSV variables
    LOWER_COLOR = np.array([70, 80, 255])
    UPPER_COLOR = np.array([95, 180, 255])

    # Create mask within hsv range
    MASK = cv2.inRange(HSV, LOWER_COLOR, UPPER_COLOR)

    # Various blur method testings (buffer for pixel imperfections)
    BLUR = cv2.GaussianBlur(MASK, (3, 3), 0)
    MEDIAN = cv2.medianBlur(MASK, 5)

    # Edge detection on each test for use in line detection
    BLUR_EDGES = cv2.Canny(BLUR, 100, 200)
    MASK_EDGES = cv2.Canny(MASK, 100, 200)
    MED_EDGES = cv2.Canny(MEDIAN, 50, 150)

    # Empty image for drawing lines (testing)
    FILTERED_LINE_IMG = np.zeros((HEIGHT, WIDTH, 3), np.uint8)
    LINE_IMG = np.zeros((HEIGHT, WIDTH, 3), np.uint8)

    # Find lines in selected image
    LINES = cv2.HoughLinesP(MED_EDGES, 1, radians(.5), 25, maxLineGap=25)

    # If there are lines:
    if LINES is not None:
        NUM_LINES = len(LINES)
        FILTERED_LINES = []
        X_TOTAL = 0
        Y_TOTAL = 0

        for NEW_LINE in LINES:
            x1, y1, x2, y2 = NEW_LINE[0]
            new_slope = degrees(np.arctan((y2 - y1)/(x2 - x1)))

            # Checks if we have verified lines, and makes a new line based on that.
            if FILTERED_LINES:
                if (new_slope < -ANGLE_THRESHOLD or new_slope > ANGLE_THRESHOLD) and unequal(new_slope, FILTERED_LINES):
                    X_TOTAL, Y_TOTAL = newLine(
                        FILTERED_LINES, NEW_LINE, FILTERED_LINE_IMG, x1, y1, x2, y2, X_TOTAL, Y_TOTAL)
            elif new_slope < -ANGLE_THRESHOLD or new_slope > ANGLE_THRESHOLD:
                X_TOTAL, Y_TOTAL = newLine(FILTERED_LINES, NEW_LINE,
                    FILTERED_LINE_IMG, x1, y1, x2, y2, X_TOTAL, Y_TOTAL)

        NUM_LINES=len(FILTERED_LINES)
        if FILTERED_LINES:
            X_AVG=0
            Y_AVG=0

            if len(VALS) == POINT_SAMPLES:
                VALS.pop(0)
                VALS.append([X_TOTAL/(2*NUM_LINES), Y_TOTAL/(2*NUM_LINES)])

                # TODO: Test if you could keep this continuous, not re-add everything every loop
                # Subtract Popped val, add new val?
                for VAL in VALS:
                    X_VAL, Y_VAL = VAL
                    X_AVG += X_VAL
                    Y_AVG += Y_VAL

                X_AVG=int(X_AVG / POINT_SAMPLES)
                Y_AVG=int(Y_AVG / POINT_SAMPLES)

                offset = 2 * (X_AVG - (WIDTH/2)) / WIDTH
                cv2.circle(IMG, (X_AVG, Y_AVG),
                           5, [255, 255, 255], -1)
                try:
                    dist_to_target = depth.get_distance(X_AVG, Y_AVG)
                except:
                    None

                # Smart Dashboard variables
                sd.putBoolean("Sees Target", True)
                sd.putNumber("Ratio Offset", offset)
                sd.putNumber("Angle Offset", PIXEL_ANGLE * X_AVG)
                sd.putNumber("Distance To Target", dist_to_target)

            else:
                VALS.append([X_TOTAL/(2*NUM_LINES), Y_TOTAL/(2*NUM_LINES)])

        for LINE in LINES:
            x1, y1, x2, y2=LINE[0]
            cv2.line(LINE_IMG, (x1, y1), (x2, y2), (0, 255, 0), 1)

    # Open the gallery of all my filtered works
    # cv2.imshow("og lines", LINE_IMG)
    # cv2.imshow("lines", FILTERED_LINE_IMG)
    # cv2.imshow('OG', IMG)
    # cv2.imshow('Mask', MASK)
    # cv2.imshow('blur', BLUR_EDGES)
    # cv2.imshow('median', MEDIAN)
    # cv2.imshow('med', MED_EDGES)
    # cv2.imshow('Mask Edges', MASK_EDGES)

    outputStream.putFrame(IMG)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    end_time=time.time()
    # print(end_time - start_time)

cv2.destroyAllWindows()
pipe.stop()
