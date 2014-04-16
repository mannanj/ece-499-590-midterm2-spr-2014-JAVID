#!/usr/bin/env python
# /* -*-  indent-tabs-mode:t; tab-width: 8; c-basic-offset: 8  -*- */
# /*
# Copyright (c) 2014, Daniel M. Lofaro <dan (at) danLofaro (dot) com>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the author nor the names of its contributors may
#       be used to endorse or promote products derived from this software
#       without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# */
import diff_drive
import ach
import sys
import time
from ctypes import *
import socket
import cv2.cv as cv
import cv2
import numpy as np
import math

dd = diff_drive
ref = dd.H_REF()
tim = dd.H_TIME()

ROBOT_DIFF_DRIVE_CHAN   = 'robot-diff-drive'
ROBOT_CHAN_VIEW_R   = 'robot-vid-chan-r'
ROBOT_CHAN_VIEW_L   = 'robot-vid-chan-l'
ROBOT_TIME_CHAN  = 'robot-time'
# CV setup 
cv.NamedWindow("wctrl_L", cv.CV_WINDOW_AUTOSIZE)
cv.NamedWindow("wctrl_R", cv.CV_WINDOW_AUTOSIZE)
#capture = cv.CaptureFromCAM(0)
#capture = cv2.VideoCapture(0)

# added
##sock.connect((MCAST_GRP, MCAST_PORT))
newx = 320
newy = 240

nx = 320
ny = 240

r = ach.Channel(ROBOT_DIFF_DRIVE_CHAN)
r.flush()
vl = ach.Channel(ROBOT_CHAN_VIEW_L)
vl.flush()
vr = ach.Channel(ROBOT_CHAN_VIEW_R)
vr.flush()
t = ach.Channel(ROBOT_TIME_CHAN)
t.flush()

i=0


print '======================================'
print '============= Robot-View ============='
print '========== Daniel M. Lofaro =========='
print '========= dan@danLofaro.com =========='
print '======================================'
while True:
    # Get Frame
    imgL = np.zeros((newx,newy,3), np.uint8)
    imgR = np.zeros((newx,newy,3), np.uint8)
    c_image = imgL.copy()
    c_image = imgR.copy()
    vidL = cv2.resize(c_image,(newx,newy))
    vidR = cv2.resize(c_image,(newx,newy))
    [status, framesize] = vl.get(vidL, wait=False, last=True)
    if status == ach.ACH_OK or status == ach.ACH_MISSED_FRAME or status == ach.ACH_STALE_FRAMES:
        vid2 = cv2.resize(vidL,(nx,ny))
        imgL = cv2.cvtColor(vid2,cv2.COLOR_BGR2RGB)
        cv2.imshow("wctrl_L", imgL)
        cv2.waitKey(10)
    else:
        raise ach.AchException( v.result_string(status) )
    [status, framesize] = vr.get(vidR, wait=False, last=True)
    if status == ach.ACH_OK or status == ach.ACH_MISSED_FRAME or status == ach.ACH_STALE_FRAMES:
        vid2 = cv2.resize(vidR,(nx,ny))
        imgR = cv2.cvtColor(vid2,cv2.COLOR_BGR2RGB)
        cv2.imshow("wctrl_R", imgR)
        cv2.waitKey(10)
    else:
        raise ach.AchException( v.result_string(status) )


    [status, framesize] = t.get(tim, wait=False, last=True)
    if status == ach.ACH_OK or status == ach.ACH_MISSED_FRAME or status == ach.ACH_STALE_FRAMES:
        pass
        #print 'Sim Time = ', tim.sim[0]
    else:
        raise ach.AchException( v.result_string(status) )

#-----------------------------------------------------
#-----------------------------------------------------
#-----------------------------------------------------
    # Def:
    # ref.ref[0] = Right Wheel Velos
    # ref.ref[1] = Left Wheel Velos
    # tim.sim[0] = Sim Time
    # imgL       = cv image in BGR format (Left Camera)
    # imgR       = cv image in BGR format (Right Camera)
    [status, framesize] = t.get(tim, wait=False, last=True)
    ts=tim.sim[0]
    frequency=0.1
  
    ref.ref[0] = -0.5
    ref.ref[1] = 0.5

    # Commands Robot
    #r.put(ref)
    # Sleeps
    time.sleep(0.1)  

    #mask the green color
    hsvL = cv2.cvtColor(imgL, cv2.COLOR_BGR2HSV)
    hsvR = cv2.cvtColor(imgR, cv2.COLOR_BGR2HSV)
    lower_g = np.array([0, 50, 50], dtype=np.uint8)
    upper_g = np.array([60,255,255], dtype=np.uint8)
    maskg = cv2.inRange(hsvL, lower_g, upper_g) 
    maskgR = cv2.inRange(hsvR, lower_g, upper_g) 
    cv2.imshow('MaskL',maskg)
    cv2.imshow('MaskR',maskgR)
    momentsl = cv2.moments(maskg)
    momentsr = cv2.moments(maskgR)


    #check if moments > 0 and get centroid
    if momentsl['m00'] > 0 and momentsr['m00'] > 0:
        cnt=momentsl['m00'] #I think this is not the correct area 
        cntr=momentsr['m00'] #I think this is not the correct area 
	height=math.sqrt(cnt/3.14)*2
	heightr=math.sqrt(cntr/3.14)*2
	#print "height in pixels", height
        if cnt != 0.0 and cntr !=0.0:
          cx = momentsl['m10']/momentsl['m00']
          cy = momentsl['m01']/momentsl['m00']  
          cxr = momentsr['m10']/momentsr['m00']
          cyr = momentsr['m01']/momentsr['m00']
          centroid= (cx, cy)
	  centroidr=(cxr,cyr)
          #print "cx", cx, "cx r ", cxr  
          angle=math.atan((cyr-cy)/(cxr-cx))
          distance=math.tan(angle) * 0.4*1000


    #wait for object to get to center and move towards or away from it
    if (momentsl['m00'] >0 and momentsr['m00'] >0):
        if (cx<=324/2 and distance>=5.0):
            print "moving forward, distance = ", distance, " m"
            ref.ref[0] = 0.6
            ref.ref[1] = 0.6   
            r.put(ref);   
            frequency=0.25   
        elif (cx>=325/2 and distance>=5.0):
            print "moving backward, distance = ", distance, " m"
            ref.ref[0]= -0.6
            ref.ref[1]= -0.6
            r.put(ref);
            frequency=0.25
        if (cx<=324/2 and distance<=3.0):
            print "moving forward, distance = ", distance, " m"
            ref.ref[0] = 0.25
            ref.ref[1] = 0.25   
            r.put(ref);   
            frequency=0.25   
        elif (cx>=325/2 and distance<=3.0):
            print "moving backward, distance = ", distance, " m"
            ref.ref[0]= -0.25
            ref.ref[1]= -0.25
            r.put(ref);
            frequency=0.25
        elif (cx<=324/2):
            print "turning right ", cx
            ref.ref[0] = 0.1
            ref.ref[1] = -0.1    
            r.put(ref);   
            if (cx<=309/2):
                ref.ref[0] = 0.35
                ref.ref[1] = -0.35  
                r.put(ref);    
            if (cx<=281/2):
                ref.ref[0]= 0.65
                ref.ref[1]= -0.65    
                r.put(ref);      
        elif (cx>=325/2):
            print "turning left", cx
            ref.ref[0]= -0.1
            ref.ref[1]= 0.1
            r.put(ref);
            if (cx>=340/2):
                ref.ref[0]= -0.35
                ref.ref[1]= 0.35
                r.put(ref);
                if (cx>=375/2):
                    ref.ref[0]= -0.65
                    ref.ref[1]= 0.65
                    r.put(ref);

    #get sim times and calculate difference+error
    ts2=tim.sim[0]
    difference=abs(ts-ts2) 
    while (frequency > difference):   
        [status, framesize] = t.get(tim, wait=False, last=True)
        ts2=tim.sim[0]
        difference=abs(ts-ts2)


#-----------------------------------------------------
#-----------------------------------------------------
#-----------------------------------------------------
