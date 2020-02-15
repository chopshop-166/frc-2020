# PyLint please stop yelling at me I did nothing wrong...
from math import radians, degrees
import cv2
import numpy as np


def equal(x, y, parallel):
    # Takes in slopes x and y, tests if they are equal to each other or any previously verified line
    # Tests if slopes are within 5 degrees of each other
    variance = 5
    # Come to think of it that's probably way too small
    Max = x + variance
    Min = x - variance
    # Please just trust me this thing somehow works
    if parallel:
        for i in parallel:
            # Compare to previously verified lines
            X1, Y1, X2, Y2 = i[0]
            pSlope = degrees(np.arctan((Y2 - Y1)/(X2 - X1)))
            # If the line's too similar with its partner or any previous line, fails test
            if not Min < y < Max or Min < pSlope < Max:
                return False
            # If it doesn't fail, must have passed
            return True
    # And if there aren't previous lines, don't bother comparing
    else:
        if Min < y < Max:
            return True


# Define width and height of frame
width = 640
height = 480

# Import source file
img = cv2.imread('Realsense_6_Color.png')

# Convert from RGB to HSV, helps with filltering
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

# Define upper and lower bounds for HSV variables
lower_color = np.array([70, 80, 255])
upper_color = np.array([95, 180, 255])
# Define kernel for morphologyEx
kernel = np.ones((5, 5), np.uint8)

# Create mask within hsv range
mask = cv2.inRange(hsv, lower_color, upper_color)

morphMask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

# Create color result of mask over input image (testing)
res = cv2.bitwise_and(img, img, mask=morphMask)
# Various blur method testings
blur = cv2.GaussianBlur(mask, (3, 3), 0)
median = cv2.medianBlur(morphMask, 3)

# Edge detection on each test for use in line detection
blur_edges = cv2.Canny(blur, 100, 200)
mask_edges = cv2.Canny(mask, 100, 200)
med_edges = cv2.Canny(median, 50, 150)
res_edges = cv2.Canny(res, 100, 200)

# Create empty image for lines (testing)
lineImg = np.zeros((height, width, 3), np.uint8)
lines = cv2.HoughLinesP(med_edges, 1, radians(.5), 15, maxLineGap=20)
# Find all lines in image


# Make sure lines exist (crashes otherwise)
if lines is not None:
    # New empty list to fill with valid lines
    filtered_lines = []
    for line in lines:
        # Extract points from line
        X1, Y1, X2, Y2 = line[0]
        # Calculate slope as degrees for ease of use
        deg_slope = degrees(np.arctan((Y2 - Y1)/(X2 - X1)))
        if deg_slope < -40 or deg_slope > 40:
            # Narrow slope range lines are found in
            # Add to list of verified lined
            filtered_lines.append(line)

    # TODO: check if this and the next bunch can be combined to one step

    # Check new list of lines
    numLines = len(filtered_lines)
    if numLines > 1:
        # new list to eliminate duplicate lines
        parallel = []
        Xtotal = 0
        Ytotal = 0
        # Iterate and compare
        for i in range(numLines):
            for j in range(i + 1, numLines):
                # Extract points from lines x2
                iPoints = filtered_lines[i]
                X1, Y1, X2, Y2 = iPoints[0]
                jPoints = filtered_lines[j]
                X3, Y3, X4, Y4 = jPoints[0]
                # Find slopes of lines in degrees
                iSlope = degrees(np.arctan((Y2 - Y1)/(X2 - X1)))
                # could be moved to equals function???
                jSlope = degrees(np.arctan((Y4 - Y3)/(X4 - X3)))

                if equal(iSlope, jSlope, parallel):
                    # Send to magic equals box
                    # If verified, add *ONE* to final list (TODO: rename parallel, doesn't make sense)
                    parallel.append(filtered_lines[i])
                    # Draws lines on the empty line image, probably irrelevant, can use data points
                    cv2.line(lineImg, (X1, Y1), (X2, Y2), (0, 255, 0), 1)
                    # Add all X and Y values of ONE verified line
                    Xtotal += X1
                    Xtotal += X2
                    # may work better alternating which set?
                    Ytotal += Y1
                    Ytotal += Y2
        # Average ALL X and Y values to find midpoint of filtered lines
        numLines = len(parallel)
        if numLines:
            Xcenter = Xtotal/(numLines*2)
            Ycenter = Ytotal/(numLines*2)
            lineImg[int(Ycenter), int(Xcenter)] = [255, 255, 255]

    cv2.imshow('Lines', lineImg)

# Open the gallery of all my filtered works
cv2.imshow('OG', img)
cv2.imshow('Mask', mask)
cv2.imshow('MorphMask', morphMask)
cv2.imshow('Filtered', res)
cv2.imshow('blur', blur_edges)
cv2.imshow('median', median)
cv2.imshow('med', med_edges)
cv2.imshow('res', res_edges)
cv2.imshow('Mask Edges', mask_edges)

cv2.waitKey(0)
# Press any key to continue...
cv2.destroyAllWindows()
# Leave without a trace
