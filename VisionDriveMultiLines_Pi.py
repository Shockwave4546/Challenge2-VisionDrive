from cv2 import cv2
import numpy
import math
import threading
import time
from networktables import NetworkTables

# Callback function for whenever NetworkTables connect/disconnect
def connectionListener(connected, info):
    # Print out the connect/disconnect status info
    print(info, "; Connection=%s" % connected)
    with cond:
        notified[0] = True
        cond.notify()

def dupImageByColor(in_frame, hue, sat, val, torg, t):
    out_img = cv2.cvtColor(in_frame, cv2.COLOR_BGR2HSV)
    #
    out_img = cv2.inRange(out_img, (hue[0], sat[0], val[0]),  (hue[1], sat[1], val[1]))
    out_img = cv2.blur(out_img, (3, 3))
    #
    # masking out the area that we wanted by just taking an entire slice of the area and leave it in the 'out' image
    #out_img = out_img[_y1:_y2, _x1:_x2]
    # also draw this slice on the original image in Red (BGR)
    #cv2.rectangle(out_img, _mask_p1, _mask_p2, (0,0,255), 1)

    # Now, let's find the contour - we only keeping/using contours for this time
    contours, _ =cv2.findContours(out_img, mode=cv2.RETR_EXTERNAL, method=cv2.CHAIN_APPROX_SIMPLE)

    maxcontourarea = 0.0
    if len(contours) > 0:
        c = max(contours, key = cv2.contourArea)
        maxcontourarea = cv2.contourArea(c)
    #for c in contours:   
    #    cv2.drawContours(out_img, c, 0, (0,150,150), 1)
    # out_img = cv2.cvtColor(out_img, cv2.COLOR_HSV2BGR)
    # cv2.putText(in_frame, str(t) + ": " + str(maxcontourarea), torg, cv2.FONT_HERSHEY_PLAIN, 1, (0,0,0))

    return (out_img, maxcontourarea)

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
def process(im, frame):
    global _old_x
    global _old_y
    global _old_w
    global _old_h
    global _old_lostsight
    #
    # Convert the image to HSV and then apply Thresholding - OpenCV standard is Blue-Green-Red
    # 
#    out = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)
    #
#    out = cv2.inRange(out, (_hsv_threshold_hue[0], _hsv_threshold_sat[0], _hsv_threshold_val[0]),  (_hsv_threshold_hue[1], _hsv_threshold_sat[1], _hsv_threshold_val[1]))
    #
    # masking out the area that we wanted by just taking an entire slice of the area and leave it in the 'out' image
#    out = out[_y1:_y2, _x1:_x2]
    out = im[_y1: _y2, _x1: _x2]
    # also draw this slice on the original image in Red (BGR)
    cv2.rectangle(frame, _mask_p1, _mask_p2, (0,0,255), 1)

    # Now, let's find the contour - we only keeping/using contours for this time
    contours, _ =cv2.findContours(out, mode=cv2.RETR_EXTERNAL, method=cv2.CHAIN_APPROX_SIMPLE)
    # still need to do a 'STOP' condition check before going into this loop...but for now, skip that
    if len(contours) > 0:
        # Line Merging Test
        
        # locate the largest contour - for now, we only going to use/deal with the largest contours - kinda big assumption for now
        c = max(contours, key = cv2.contourArea)
        # And then find the bounding rectangle that fit this contours...this is going to enclosed everything that fits inside a rect.
        # - will be put in x, y, width and height [_x, _y, _w and _h]
        cb_x, cb_y, cb_w, cb_h = cv2.boundingRect(c)
        # put this into the 'current or old' variables
        _old_x = cb_x
        _old_y = cb_y
        _old_w = cb_w
        _old_z = cb_h
        _old_lostsight = 0
    else:
        # couldn't find any contours, for now, we just assumed to use the last one - note - need to perform the stop condition
        cb_x = _old_x
        cb_y = _old_y
        cb_w = _old_w
        cb_h = _old_h
        _old_lostsight = _old_lostsight + 1

    # the new bounding box = contains the min/max of the targetted/tracked line point
    # cbbox = out[cb_y:cb_y+cb_h, cb_x:cb_x+cb_w]
    # Now, draw this new bounding box on the original image in GREEN
    # cv2.rectangle(im, (cb_x, cb_y + _y1), (cb_x + cb_w, cb_y + cb_h + _y1), (0, 255, 0), 1)
    cv2.rectangle(frame, (cb_x, cb_y + _y1), (cb_x + cb_w, cb_y + cb_h + _y1), (0, 255, 0), 1)

    sp = genMask(out, (cb_x, cb_y), (cb_x + cb_w, cb_y + cb_h))
    #cv2.rectangle(out, (cb_x, cb_y), (cb_x + cb_w, cb_y + cb_h), (0, 255, 255), 1)
    #print(contours)

    # debugging use - comment out for production
    # cv2.imshow("Filtered", out)
    # return out
    ## return cbbox
    return sp

# Parameter Block - parameters that are constants*
# Camera Port #
_portnum = 0
# Live or Dev
_dev = False
#
# Thresholding a HSV to isolated Black Lines only - value is 'dialed' in already but might needed additional tuning once deployed to robot/pi
_hsv_threshold_hue = [0.0, 180.0]
_hsv_threshold_sat = [0.0, 255.0]
_hsv_threshold_val = [0.0, 114.0]
# Priority 
# - Red Line (1st time only - ignore after seeing other color line for X)
# - Blue Line (1st time only - ignore after seeing other color line for X)
# - Black Line (Default)
# Thresholding a HSV to isolated Black Lines only - value is 'dialed' in already but might needed additional tuning once deployed to robot/pi
# Mod Black's Value after using with Blue and Red at the same time
_black_hue = [0.0, 180.0]
_black_sat = [0.0, 255.0]
_black_val = [0.0, 60.0]
# Blue - with Red and Black
_blue_hue = [99.0, 168.0]
_blue_sat = [67.0, 255.0]
_blue_val = [0.0, 255.0]
# Red - with Black and Blue
_red_hue = [0.0, 60.0]
_red_sat = [85.0, 255.0]
_red_val = [115.0, 255.0]
_minContourArea = 900   # 30px by 30px - pretty small but very real for now, can increase/decrease later
#
okay2SwitchRed = True       # Starting out, only okay to switch to Red
okay2SwitchBlue = False     # and not allow to switch to Blue until we see Red
okay2SwitchBlack = False    # this flag can only be set by inside Red or Blue
okaytoignore = True
# We will always starts on the Blackline from starting position
whichline = "Black"
# Blue, Green, Red, B-G, G-R
color = [(255,0,0), (0,255,0), (0,0,255), (150,150,0), (0,150,150)]
#
# Mask x1,y1 and x2,y2 [upper left and lower right points]
_x1 = 0
_x2 = 639
#_y1 = 100
#_y2 = 140
_y1 = 120
_y2 = 220
_mask_p1 = (_x1, _y1)
_mask_p2 = (_x2, _y2)
#
_old_h = _old_w = _old_x = _old_y = 0
_old_lostsight = 0
_stop = False
_quickturn = False
#
# Width of the Frame (640x480)
_width = 640
# Trackwidth = Line to Track
_trackwidth = 36
#
_center = _width / 2
#
# create a 'window' or 'bracket' for the daadzone to not move off center = 1x the width of the line to track on both side from the center
_center_ledge = _center - _trackwidth
_center_redge = _center + _trackwidth
_maxboxwidth = _center_ledge
#
# Parameters/Variables for Vision Processing
cond = threading.Condition()
notified = [False]
#
default_drive_speed = 0.2
min_drive_speed = 0.1
#
# 
# Main routine starts here
#
# Let's start up the camera 1st
if (_dev):
    # Use Portnum for Webcam testing
    camera = cv2.VideoCapture(_portnum)
else:
    # Use the camera stream from the ROMI/PI for Live Run
    camera = cv2.VideoCapture("http://10.45.46.2:1181/stream.mjpg")
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
x.putBoolean("Stop", _stop)
x.putBoolean("QuickTurn", _quickturn)

#print("Initial Forward Speed: [0 to 1]", f)
#print("Initial Left-Right [-1 to 1]: ", lr)

while True:
    ret, frame = camera.read()

    redline, red_contourarea = dupImageByColor(frame, _red_hue, _red_sat, _red_val, (50, 50), "Red")
    blueline, blue_contourarea = dupImageByColor(frame, _blue_hue, _blue_sat, _blue_val, (50, 100), "Blue")
    blackline, black_contourarea = dupImageByColor(frame, _black_hue, _black_sat, _black_val, (50, 150), "Black")
    line = blackline
    # Are we on the redline from last cycle
    if (whichline == "Red"):
        okay2SwitchRed = False
        okay2SwitchBlue = True # okay to use Blue after being on the Redline
        line = redline
        contourarea = red_contourarea
        whichline = "Red"
        # Look for next transition, which is Blackline
        if (okay2SwitchBlack == True) and (black_contourarea > _minContourArea):
            # See real blackline, switch to it
            line = blackline
            contourarea = black_contourarea
            whichline = "Black"
        else:
            # can only switch to black when we no longer see anymore blacks (minArea/100) for now
            if (black_contourarea < _minContourArea/10):
                okay2SwitchBlack = True
    else:
        # Not on Redline right now, okay to switch to Redline?
        if (okay2SwitchRed == True) and (red_contourarea > _minContourArea):
            # Swith to Redline and not allow to be on Red again
            okay2SwitchRed = False
            line = redline
            contourarea = red_contourarea
            whichline = "Red"
        else:
            # not on Red, and not able to switch to Red...see if we can switch into Blue or not then
            #
            #  Are we already on Blueline thoug...
            if (whichline == "Blue"):
                line = blueline
                contourarea = blue_contourarea
                whichline = "Blue"
                # If so, then see if we see blackline and switch to that if we are
                if (okay2SwitchBlack == True) and (black_contourarea > _minContourArea):
                    # See real blackline, switch to it
                    line = blackline
                    contourarea = black_contourarea
                    whichline = "Black"
                else:
                    # can only switch to black when we no longer see anymore blacks (minArea/100) for now
                    if (black_contourarea < _minContourArea/10):
                        okay2SwitchBlack = True
            else:
                # By now, we should be on the Blackline already so there's no switching yet...so set not allow to switch blackline again
                okay2SwitchBlack = False
                # Now, see if we need to switch to Blueline...
                if (okay2SwitchBlue == True) and (blue_contourarea > _minContourArea):
                    okay2SwitchBlue = False     # Not allow to switch to Blue anymore.  this is where we will set switch2Red if we want to allow
                    line = blueline
                    contourarea = blue_contourarea
                    whichline = "Blue"

    # Only continue to process after we pickup the right "line"
    # maskframe = genMask(frame, _mask_p1, _mask_p2)
    print("Whichline: ", whichline)
    processedframe = process(line, frame)

    # cv2.imshow("Mask", maskframe)
    ### cv2.imshow("Camera", frame)
    #cv2.imshow("Final", line)
    #cv2.imshow("Red", redline)
    #cv2.imshow("Blue", blueline)
    #cv2.imshow("Black", blackline)

    # cv2.imshow("OutTrack", processedframe)
    
    if (_old_lostsight > 50):
        _stop = True

    # print(cv2.boundingRect(processedframe))
    ### print("L - W - R - C : ", _old_x, _old_w, _old_x + _old_w, _old_x + round(_old_w/2))
    linecenter = _old_x + round(_old_w / 2)

    if (_stop == False):
        # If line is within the window of the left and right edge of the center, then just go forward with 0 head
        # - Scale from [_center_redge to 2* _x2] from [0 to 1] 
        # - Scale from [0 to _center_ledge] from [-1 to 0]
        if (linecenter >= _center_ledge) and (linecenter <= _center_redge):
            # print("Straight Forward")
            lr = 0
            # targetc is used for speed modifier calculation
            targetc = 0
            # speed = default_drive_speed
            speed = 0.10
        if (linecenter < _center_ledge):
            # print("Go Left")
            # lr = -1 * q((_center_ledge - linecenter) / _center)
            lr = ((_old_x / _center_ledge) -1) * 0.15
            # targetc is used for speed modifier calculation
            targetc = -1 * ((_center_ledge - linecenter) / _center)

            speed = 0.10
        if (linecenter > _center_redge):
            # print("Go Right")
            # lr = (linecenter - _center_redge) / _center
            lr = (((_old_x + _old_w) - _center_redge) / _center) * 0.15
            # targetc is used for speed modifier calculation
            targetc = (linecenter - _center_redge) / _center 
            speed = 0.10
        #
        # Speed = default speed + 5% modifier from line distance from target center - 25% modifier due to "target/line width"
        # Target Modifier = targetbox_width/maxboxsize
        # speed = default_drive_speed + ((abs(targetc)) * default_drive_speed * 0.05) - (0.25 * default_drive_speed * (_old_w / _maxboxwidth))
        # speed = default_drive_speed - (0.25 * default_drive_speed * (_old_w / _maxboxwidth))

        # can't go slower then the min speed
        #if speed < min_drive_speed:
        #    speed = min_drive_speed

        #if (_old_w > (1.5 * _trackwidth)):
        if True:
            _quickturn = True
        else:
            _quickturn = False
        # print(_old_w);
        # print("Component 1: ", ((abs(targetc)) * default_drive_speed * 0.05))
        # print(("Component 2: ", (0.25 * default_drive_speed * (_old_w / _maxboxwidth))))
        ### print("TW - LC - CL - CR - C", _old_w, linecenter, _center_ledge, _center_redge, _center)
        x.putNumber("Forward Speed", speed)
        x.putNumber("Left-Right", lr)
        x.putBoolean("QuickTurn", _quickturn)
        x.putBoolean("Stop", _stop)

    else:
        x.putNumber("Forward Speed", 0.0)
        x.putNumber("Left-Right", 0.0)
        x.putBoolean("QuickTurn", _quickturn)
        x.putBoolean("Stop", _stop)

    #if cv2.waitKey(1) == ord('q'):
    #    break

# Clean up after getting out of the loop
camera.release()
cv2.destroyAllWindows()

