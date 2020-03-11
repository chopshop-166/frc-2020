import pyrealsense2 as rs2
import numpy as np
import visionfunctions as vision

while True:
    if vision.get_frames():
        print()