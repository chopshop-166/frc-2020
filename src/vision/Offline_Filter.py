# Final FRC 2020 vision program
# Exposure setting should be tuned on a per-competition basis
# They are currently tuned to the testing area
# Tuning for the RealSense camera can be done easily through the ReaslSense Viewer app in a GUI

from math import degrees, radians
import pyrealsense2 as rs2
import cv2
import numpy as np
import time

debugging = True

ANGLE_LOW_THRESHOLD = 40
ANGLE_HIGH_THRESHOLD = 70

# Resolution
WIDTH = 640
HEIGHT = 480
POINT_SAMPLES = 5

# Angle value of each pixel
FOV_ANGLE = 82.5
PIXEL_ANGLE = FOV_ANGLE / WIDTH

# Takes in slopes x and y, tests if they are equal to each other or any previously verified line

def unequal(new, old_list):
    variance = 5
    for i in old_list:
        x1, y1, x2, y2 = i[0]
        old_slope = degrees(np.arctan((y2 - y1) / (x2 - x1)))
        if abs(new - old_slope) < variance:
            return False
    return True


def newLine(
    filtered_lines, new_line, filtered_line_img, x1, y1, x2, y2, X_TOTAL, Y_TOTAL
):
    filtered_lines.append(new_line)
    if debugging:
        cv2.line(filtered_line_img, (x1, y1), (x2, y2), (0, 255, 0), 1)
    X_TOTAL += x1 + x2
    Y_TOTAL += y1 + y2

    return X_TOTAL, Y_TOTAL

VALS = []

FILTERED_LINE_IMG = np.zeros((HEIGHT, WIDTH, 3), np.uint8)

IMG = cv2.imread("Elbit3_Color.png")

# Convert from RGB to HSV, helps with filtering
HSV = cv2.cvtColor(IMG, cv2.COLOR_BGR2HSV)

# Define upper and lower bounds for HSV variables
LOWER_COLOR = np.array([29, 144, 5])
UPPER_COLOR = np.array([144, 255, 255])

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
LINES = cv2.HoughLinesP(MED_EDGES, 1, radians(0.5), 25, maxLineGap=25)

# If there are lines:
if LINES is not None:
    NUM_LINES = len(LINES)
    FILTERED_LINES = []
    X_TOTAL = 0
    Y_TOTAL = 0

    for NEW_LINE in LINES:
        x1, y1, x2, y2 = NEW_LINE[0]
        new_slope = degrees(np.arctan((y2 - y1) / (x2 - x1)))

        # Checks if we have verified lines, and makes a new line based on that.
        if FILTERED_LINES:
            if (
                ANGLE_HIGH_THRESHOLD > abs(new_slope) > ANGLE_LOW_THRESHOLD
            ) and unequal(new_slope, FILTERED_LINES):
                X_TOTAL, Y_TOTAL = newLine(
                    FILTERED_LINES,
                    NEW_LINE,
                    FILTERED_LINE_IMG,
                    x1,
                    y1,
                    x2,
                    y2,
                    X_TOTAL,
                    Y_TOTAL,
                )
        elif ANGLE_HIGH_THRESHOLD > abs(new_slope) > ANGLE_LOW_THRESHOLD:
            X_TOTAL, Y_TOTAL = newLine(
                FILTERED_LINES,
                NEW_LINE,
                FILTERED_LINE_IMG,
                x1,
                y1,
                x2,
                y2,
                X_TOTAL,
                Y_TOTAL,
            )

    NUM_LINES = len(FILTERED_LINES)
    if FILTERED_LINES:
        VALS.append([X_TOTAL / (2 * NUM_LINES), Y_TOTAL / (2 * NUM_LINES)])
        X_AVG = 0
        Y_AVG = 0

        for VAL in VALS:
            X_VAL, Y_VAL = VAL
            X_AVG += X_VAL
            Y_AVG += Y_VAL

        X_AVG = int(X_AVG)
        Y_AVG = int(Y_AVG)
            
        cv2.circle(FILTERED_LINE_IMG, (X_AVG, Y_AVG), 5, [255, 255, 255], -1)

    if debugging:
        for LINE in LINES:
            x1, y1, x2, y2 = LINE[0]
            cv2.line(LINE_IMG, (x1, y1), (x2, y2), (0, 255, 0), 1)

# Open the gallery of all my filtered works
if debugging:
    cv2.imshow("og lines", LINE_IMG)
    cv2.imshow("lines", FILTERED_LINE_IMG)
    cv2.imshow("OG", IMG)
    cv2.imshow("Mask", MASK)
    cv2.imshow("blur", BLUR_EDGES)
    cv2.imshow("median", MEDIAN)
    cv2.imshow("med", MED_EDGES)
    cv2.imshow("Mask Edges", MASK_EDGES)

cv2.waitKey(0)

cv2.destroyAllWindows()
