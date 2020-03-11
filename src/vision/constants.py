import numpy as np
from networktables import NetworkTables

debugging = True

# Smart Dashboard
sd = NetworkTables.getTable("SmartDashboard")

# Resolution
WIDTH = 640
HEIGHT = 480
POINT_SAMPLES = 5

# Angle value of each pixel
FOV_ANGLE = 82.5
PIXEL_ANGLE = FOV_ANGLE / WIDTH

# Camera Variables
CAM_BRIGHTNESS = 0
CAM_CONTRAST = 100
CAM_EXPOSURE = 156
CAM_GAIN = 75
CAM_GAMMA = 300
CAM_HUE = 0
CAM_SATURATION = 50
CAM_SHARPNESS = 50
CAM_WHITE_BALANCE = 6500

# Filter Variables
LOWER_COLOR = np.array([39, 92, 18])
UPPER_COLOR = np.array([89, 255, 255])

ANGLE_HIGH_THRESHOLD = 80
ANGLE_LOW_THRESHOLD = 30

# Globals
(
    depth,
    pipe,
    s,
    img,
    hsv,
    mask,
    blur,
    median,
    med_edges,
    blur_edges,
    mask_edges,
    line_img,
    filtered_line_img,
) = None
vals = []