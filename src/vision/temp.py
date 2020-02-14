# Exists solely for me to reorganize this code and test stuff out

from math import radians, degrees   # PyLint please stop yelling at me I did nothing wrong...
import cv2
import numpy as np

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

WIDTH = 640
HEIGHT = 480

IMG = cv2.imread('Realsense_4_Color.png')

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
LINE_IMG = np.zeros((HEIGHT, WIDTH, 3), np.uint8)

# Find lines in selected image
LINES = cv2.HoughLinesP(MED_EDGES, 1, radians(.5), 15, maxLineGap=20)

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
                cv2.line(LINE_IMG, (x1, y1), (x2, y2), (0, 255, 0), 1)
                X_TOTAL += x1 + x2
                Y_TOTAL += y1 + y2
        else:
            if new_slope < -40 or new_slope > 40:
                FILTERED_LINES.append(NEW_LINE)
                cv2.line(LINE_IMG, (x1, y1), (x2, y2), (0, 255, 0), 1)
                X_TOTAL += x1 + x2
                Y_TOTAL += y1 + y2

    NUM_LINES = len(FILTERED_LINES)
    if FILTERED_LINES:
        X_CENTER = X_TOTAL/(2*NUM_LINES)
        Y_CENTER = Y_TOTAL/(2*NUM_LINES)
        LINE_IMG[int(Y_CENTER), int(X_CENTER)] = [255, 255, 255]


cv2.imshow("lines", LINE_IMG)
cv2.imshow('OG', IMG)                   # Open the gallery of all my filtered works
cv2.imshow('Mask', MASK)
cv2.imshow('blur', BLUR_EDGES)
cv2.imshow('median', MEDIAN)
cv2.imshow('med', MED_EDGES)
cv2.imshow('Mask Edges', MASK_EDGES)

cv2.waitKey(0)                          # Press any key to continue...
cv2.destroyAllWindows()                 # Leave without a trace