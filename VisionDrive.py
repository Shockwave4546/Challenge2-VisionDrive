from cv2 import cv2
import numpy
import math
import threading
import time
from networktables import NetworkTables

def connectionListener(connected, info):
    print(info, "; Connection=%s" % connected)
    with cond:
        notified[0] = True
        cond.notify()

# Generate a 3d mask for use in masking step using a given image
def genMask(im, targetp1, targetp2):
    d = im.shape
    # print("Dimensions: ", d)
    # numpy.zeros - the [h,w,d] - w = width, h = height and d = # of channels
    # maskframe = numpy.zeros([480,640,3], dtype=numpy.uint8)
    maskframe = numpy.zeros(d, dtype=numpy.uint8)
    maskframe.fill(0)
    # use -1 at last parameter to do 'filled rectangle' instead of line length
    # cv2.rectangle(maskframe, (0,100), (639, 140), (255,255,255), -1)
    cv2.rectangle(maskframe, targetp1, targetp2, (255,255,255), -1)

    return maskframe

#
# Process the Image - 
# - HSV+Thresholding -> Masking 
def process(im):
    global _old_x
    global _old_y
    global _old_w
    global _old_h
    #
    # Convert the image to HSV and then apply Thresholding - OpenCV standard is Blue-Green-Red
    # 
    out = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
    #
    out = cv2.inRange(out, (_hsv_threshold_hue[0], _hsv_threshold_sat[0], _hsv_threshold_val[0]),  (_hsv_threshold_hue[1], _hsv_threshold_sat[1], _hsv_threshold_val[1]))
    #
    # masking out the area that we wanted by just taking an entire slice of the area and leave it in the 'out' image
    out = out[_y1:_y2, _x1:_x2]
    # also draw this slice on the original image in Red (BGR)
    cv2.rectangle(im, _mask_p1, _mask_p2, (0,0,255), 1)

    # Now, let's find the contour
    contours, _ =cv2.findContours(out, mode=cv2.RETR_EXTERNAL, method=cv2.CHAIN_APPROX_SIMPLE)
    # still need to do a 'STOP' condition check before going into this loop...but for now, skip that
    if len(contours) > 0:
        # locate the largest contour
        c = max(contours, key = cv2.contourArea)
        # And then find the bounding rectangle that fit this contours...this is going to enclosed everything that fits inside a rect.
        # - will be put in x, y, width and height
        #
        cb_x, cb_y, cb_w, cb_h = cv2.boundingRect(c)
        # put this into the 'current or old' variables
        _old_x = cb_x
        _old_y = cb_y
        _old_w = cb_w
        _old_z = cb_h
    else:
        # couldn't find any contours, for now, we just assumed to use the last one - note - need to perform the stop condition
        cb_x = _old_x
        cb_y = _old_y
        cb_w = _old_w
        cb_h = _old_h

    # the new bounding box = contains the min/max of the targetted/tracked line point
    # cbbox = out[cb_y:cb_y+cb_h, cb_x:cb_x+cb_w]
    # Now, draw this new bounding box on the original image in GREEN
    cv2.rectangle(im, (cb_x, cb_y + _y1), (cb_x + cb_w, cb_y + cb_h + _y1), (0, 255, 0), 1)

    sp = genMask(out, (cb_x, cb_y), (cb_x + cb_w, cb_y + cb_h))
#    cv2.rectangle(out, (cb_x, cb_y), (cb_x + cb_w, cb_y + cb_h), (0, 255, 255), 1)
    #print(contours)

    # debugging use - comment out for production
    # cv2.imshow("Filtered", out)
    # return out
    ## return cbbox
    return sp

# Parameter Block - parameters that are constants*
# Camera Port #
_portnum = 0
#
# Thresholding a HSV to isolated Black Lines only - value is 'dialed' in already but might needed additional tuning once deployed to robot/pi
_hsv_threshold_hue = [0.0, 180.0]
_hsv_threshold_sat = [0.0, 255.0]
_hsv_threshold_val = [0.0, 114.0]
#
# Mask x1,y1 and x2,y2 [upper left and lower right points]
_mask_p1 = (0,100)
_mask_p2 = (639, 140)
_x1 = 0
_x2 = 639
_y1 = 100
_y2 = 140
#
_old_h = _old_w = _old_x = _old_y = 0
#
_center_ledge = 300
_center_redge = 340
_center = 320
#
# Parameters/Variables for Vision Processing
cond = threading.Condition()
notified = [False]
#
default_drive_speed = 0.75

#
# 
# Main routine starts here
#
# Let's start up the camera 1st
camera = cv2.VideoCapture(_portnum)
# camera = cv2.VideoCapture("http://10.45.46.2:1181/stream.mjpg")
#
# Setup and Established NetworkTable connection
NetworkTables.initialize('127.0.0.1')
#NetworkTables.startClient('127.0.0.1')
NetworkTables.addConnectionListener(connectionListener, immediateNotify=True)

# Wait until we got the NetworkTable Connection completed here
with cond:
    print("Waiting for connection")
    if not notified[0]:
        cond.wait()

# Put the DriveByVision Table into x
x = NetworkTables.getTable('DriveByVision')
f = x.getNumber('Forward Speed', defaultValue=0.0)
lr = x.getNumber('Left-Right', defaultValue=0.0)
print("Initial Forward Speed: [0 to 1]", f)
print("Initial Left-Right [-1 to 1]: ", lr)

while True:
    ret, frame = camera.read()
    # maskframe = genMask(frame, _mask_p1, _mask_p2)
    processedframe = process(frame)

    # cv2.imshow("Mask", maskframe)
    cv2.imshow("Camera", frame)
    cv2.imshow("OutTrack", processedframe)
    
    # print(cv2.boundingRect(processedframe))
    # print("L - W - R - X : ", _old_x, _old_w, _old_x + _old_w, _old_x + round(_old_w/2))
    linecenter = _old_x + round(_old_w / 2)

    # If line is within the window of the left and right edge of the center, then just go forward with 0 head
    # - Scale from [_center_redge to 2* _x2] from [0 to 1] 
    # - Scale from [0 to _center_ledge] from [-1 to 0]
    if (linecenter >= _center_ledge) and (linecenter <= _center_redge):
        # print("Straight Forward")
        lr = 0
    if (linecenter < _center_ledge):
        # print("Go Left")
        lr = -1 * ((_center_ledge - linecenter) / _center)
    if (linecenter > _center_redge):
        # print("Go Right")
        lr = (linecenter - _center_redge) / _center

    speed = default_drive_speed + ((abs(lr)) * default_drive_speed)/10
    x.putNumber("Forward Speed", speed)
    x.putNumber("Left-Right", lr)

    if cv2.waitKey(1) == ord('q'):
        break

# Clean up after getting out of the loop
camera.release()
cv2.destroyAllWindows()

