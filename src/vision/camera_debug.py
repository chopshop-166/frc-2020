import visionfunctions as vision

vision.camera_init()
while True:
    if vision.get_frames():
        lines = vision.img_filter(True)
        vision.line_filter(lines)
        vision.img_show()