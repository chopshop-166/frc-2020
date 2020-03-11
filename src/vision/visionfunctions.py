from math import radians
import pyrealsense2 as rs2
import cv2
from numpy import radians, degrees
from constants import *


def unequal(new, old_list):
    variance = 5
    for i in old_list:
        x1, y1, x2, y2 = i[0]
        old_slope = degrees(np.arctan((y2 - y1) / (x2 - x1)))
        if abs(new - old_slope) < variance:
            return False
    return True


def newLine(
    filtered_lines, new_line, x1, y1, x2, y2, x_total, Y_TOTAL
):
    filtered_lines.append(new_line)
    if debugging:
        cv2.line(filtered_line_img, (x1, y1), (x2, y2), (0, 255, 0), 1)
    x_total += x1 + x2
    Y_TOTAL += y1 + y2

    return x_total, Y_TOTAL


def camera_init():
    pipe = rs2.pipeline()
    config = rs2.config()
    config.enable_stream(rs2.stream.color, WIDTH, HEIGHT, rs2.format.bgr8, 30)
    config.enable_stream(rs2.stream.depth, WIDTH, HEIGHT, rs2.format.z16, 30)
    profile = pipe.start(config)
    s = profile.get_device().query_sensors()[1]
    swap_stream(False)


def get_frames():
    frames = rs2.composite_frame(pipe.wait_for_frames())
    frame = rs2.video_frame(frames.get_color_frame())
    depth = rs2.depth_frame(frames.get_depth_frame())

    if not frame:
        return False
    
    IMG = np.asanyarray(frame.get_data())


def swap_stream(isShooting):
    if isShooting:
        s.set_option(rs2.option.enable_auto_exposure, False)
        s.set_option(rs2.option.brightness, CAM_BRIGHTNESS)
        s.set_option(rs2.option.contrast, CAM_CONTRAST)
        s.set_option(rs2.option.exposure, CAM_EXPOSURE)
        s.set_option(rs2.option.gain, CAM_GAIN)
        s.set_option(rs2.option.gamma, CAM_GAMMA)
        s.set_option(rs2.option.hue, CAM_HUE)
        s.set_option(rs2.option.saturation, CAM_SATURATION)
        s.set_option(rs2.option.sharpness, CAM_SHARPNESS)
        s.set_option(rs2.option.white_balance, CAM_WHITE_BALANCE)
    else:
        s.set_option(rs2.option.enable_auto_exposure, True)
        s.set_option(rs2.option.brightness, 0)
        s.set_option(rs2.option.contrast, 50)
        s.set_option(rs2.option.gamma, 300)
        s.set_option(rs2.option.hue, 0)
        s.set_option(rs2.option.saturation, 64)
        s.set_option(rs2.option.sharpness, 50)
        s.set_option(rs2.option.white_balance, 4600)


def img_filter(debugging):

    # Convert from RGB to HSV, helps with filtering
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    # Create mask within hsv range
    mask = cv2.inRange(hsv, LOWER_COLOR, UPPER_COLOR)

    # Blur image (buffer for pixel imperfections)
    median = cv2.medianBlur(mask, 5)

    # Edge detection for use in line detection
    med_edges = cv2.Canny(median, 50, 150)

    if debugging:
        blur = cv2.GaussianBlur(mask, (3, 3), 0)

        blur_edges = cv2.Canny(blur, 100, 200)
        mask_edges = cv2.Canny(mask, 100, 200)

        # Empty image for drawing lines (testing)
        filtered_line_img = np.zeros((HEIGHT, WIDTH, 3), np.uint8)
        line_img = np.zeros((HEIGHT, WIDTH, 3), np.uint8)
    # Find lines in selected image
    lines = cv2.HoughLinesP(med_edges, 1, radians(0.5), 25, maxLineGap=25)

    return lines


def line_filter(lines):
    if lines is not None:
        num_lines = len(lines)
        filtered_lines = []
        x_total = 0
        y_total = 0

        for new_line in lines:
            x1, y1, x2, y2 = new_line[0]
            new_slope = degrees(np.arctan((y2 - y1) / (x2 - x1)))

            # Checks if we have verified lines, and makes a new line based on that.
            if filtered_lines:
                if (ANGLE_HIGH_THRESHOLD > new_slope > ANGLE_LOW_THRESHOLD) and unequal(
                    new_slope, filtered_lines
                ):
                    x_total, y_total = newLine(
                        filtered_lines,
                        new_line,
                        x1,
                        y1,
                        x2,
                        y2,
                        x_total,
                        y_total,
                    )
            elif ANGLE_HIGH_THRESHOLD > abs(new_slope) > ANGLE_LOW_THRESHOLD:
                x_total, y_total = newLine(
                    filtered_lines,
                    new_line,
                    x1,
                    y1,
                    x2,
                    y2,
                    x_total,
                    y_total,
                )
        num_lines = len(filtered_lines)
        if filtered_lines:
            x_avg = 0
            y_avg = 0

            if len(vals) == POINT_SAMPLES:
                # Pop the first value, and append the new average if the list is full
                vals.pop(0)
                vals.append([x_total / (2 * num_lines), y_total / (2 * num_lines)])

                # TODO: Test if you could keep this continuous, not re-add everything every loop
                # Subtract Popped val, add new val?
                for VAL in vals:
                    x_val, y_val = VAL
                    x_avg += x_val
                    y_avg += y_val
                x_avg = int(x_avg / POINT_SAMPLES)
                y_avg = int(y_avg / POINT_SAMPLES)

                offset = 2 * (x_avg - (WIDTH / 2)) / WIDTH
                cv2.circle(filtered_line_img, (x_avg, y_avg), 5, [255, 255, 255], -1)
                try:
                    dist_to_target = depth.get_distance(x_avg, y_avg)
                except:
                    None
                if not debugging:
                    # Smart Dashboard variables
                    sd.putBoolean("Sees Target", True)
                    sd.putNumber("Ratio Offset", offset)
                    sd.putNumber(
                        "Angle Offset", (PIXEL_ANGLE * x_avg) - (FOV_ANGLE / 2)
                    )
                    sd.putNumber("Distance To Target", dist_to_target)
            else:
                vals.append([x_total / (2 * num_lines), y_total / (2 * num_lines)])
        if debugging:
            for LINE in lines:
                x1, y1, x2, y2 = LINE[0]
                cv2.line(line_img, (x1, y1), (x2, y2), (0, 255, 0), 1)


def img_show():
    cv2.imshow("OG", img)
    cv2.imshow("Mask", mask)
    cv2.imshow("blur", blur_edges)
    cv2.imshow("median", median)
    cv2.imshow("med", med_edges)
    cv2.imshow("Mask Edges", mask_edges)
