from math import radians, degrees   # PyLint please stop yelling at me I did nothing wrong...
import cv2
import numpy as np

def equal(x, y, parallel):  # Takes in slopes x and y, tests if they are equal to each other or any previously verified line
    variance = 5                                                        # Tests if slopes are within 5 degrees of each other
    Max = x + variance                                                  # Come to think of it that's probably way too small
    Min = x - variance
    if parallel:                                                        # Please just trust me this thing somehow works
        for i in parallel:                                              # Compare to previously verified lines
            X1, Y1, X2, Y2 = i[0]
            pSlope = degrees(np.arctan((Y2 - Y1)/(X2 - X1)))
            if not Min < y < Max or Min < pSlope < Max:                 # If the line's too similar with its partner or any previous line, fails test
                return False
            return True                                                 # If it doesn't fail, must have passed
    else:                                                               # And if there aren't previous lines, don't bother comparing
        if Min < y < Max:
            return True

width = 640                                                             # Define width and height of frame
height = 480

img = cv2.imread('Realsense_6_Color.png')                               # Import source file

hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)                              # Convert from RGB to HSV, helps with filltering

lower_color = np.array([70, 80, 255])                                   # Define upper and lower bounds for HSV variables
upper_color = np.array([95, 180, 255])
kernel = np.ones((5, 5), np.uint8)                                      # Define kernel for morphologyEx

mask = cv2.inRange(hsv, lower_color, upper_color)                       # Create mask within hsv range

morphMask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

res = cv2.bitwise_and(img, img, mask=morphMask)                         # Create color result of mask over input image (testing)
blur = cv2.GaussianBlur(mask, (3, 3), 0)                                # Various blur method testings
median = cv2.medianBlur(morphMask, 3)

blur_edges = cv2.Canny(blur, 100, 200)                                  # Edge detection on each test for use in line detection
mask_edges = cv2.Canny(mask, 100, 200)
med_edges = cv2.Canny(median, 50, 150)
res_edges = cv2.Canny(res, 100, 200)

lineImg = np.zeros((height, width, 3), np.uint8)                        # Create empty image for lines (testing)
lines = cv2.HoughLinesP(med_edges, 1, radians(.5), 15, maxLineGap=20)   # Find all lines in image


if lines is not None:                                                   # Make sure lines exist (crashes otherwise)
    filtered_lines = []                                                 # New empty list to fill with valid lines
    for line in lines:
        X1, Y1, X2, Y2 = line[0]                                        # Extract points from line
        deg_slope = degrees(np.arctan((Y2 - Y1)/(X2 - X1)))             # Calculate slope as degrees for ease of use
        if deg_slope < -40 or deg_slope > 40:                           # Narrow slope range lines are found in
            filtered_lines.append(line)                                 # Add to list of verified lined

    # TODO: check if this and the next bunch can be combined to one step

    numLines = len(filtered_lines)                                      # Check new list of lines
    if numLines > 1:
        parallel = []                                                   # new list to eliminate duplicate lines
        Xtotal = 0
        Ytotal = 0
        for i in range(numLines):                                       # Iterate and compare
            for j in range(i + 1, numLines):
                iPoints = filtered_lines[i]                             # Extract points from lines x2
                X1, Y1, X2, Y2 = iPoints[0]
                jPoints = filtered_lines[j]
                X3, Y3, X4, Y4 = jPoints[0]
                iSlope = degrees(np.arctan((Y2 - Y1)/(X2 - X1)))        # Find slopes of lines in degrees
                jSlope = degrees(np.arctan((Y4 - Y3)/(X4 - X3)))        # could be moved to equals function???

                if equal(iSlope, jSlope, parallel):                     # Send to magic equals box
                    parallel.append(filtered_lines[i])                  # If verified, add *ONE* to final list (TODO: rename parallel, doesn't make sense)
                    cv2.line(lineImg, (X1, Y1), (X2, Y2), (0, 255, 0), 1)   # Draws lines on the empty line image, probably irrelevant, can use data points
                    Xtotal += X1                                        # Add all X and Y values of ONE verified line
                    Xtotal += X2
                    Ytotal += Y1                                        # may work better alternating which set?
                    Ytotal += Y2
        numLines = len(parallel)                                        # Average ALL X and Y values to find midpoint of filtered lines
        if numLines:
            Xcenter = Xtotal/(numLines*2)
            Ycenter = Ytotal/(numLines*2)
            lineImg[int(Ycenter), int(Xcenter)] = [255, 255, 255]

    cv2.imshow('Lines', lineImg)

cv2.imshow('OG', img)                   # Open the gallery of all my filtered works
cv2.imshow('Mask', mask)
cv2.imshow('MorphMask', morphMask)
cv2.imshow('Filtered', res)
cv2.imshow('blur', blur_edges)
cv2.imshow('median', median)
cv2.imshow('med', med_edges)
cv2.imshow('res', res_edges)
cv2.imshow('Mask Edges', mask_edges)

cv2.waitKey(0)                          # Press any key to continue...

cv2.destroyAllWindows()                 # Leave without a trace