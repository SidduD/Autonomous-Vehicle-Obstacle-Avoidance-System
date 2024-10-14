from pymavlink import mavutil
import RPi.GPIO as GPIO
import time
import cv2
import numpy as np
import math
import struct

cap = cv2.VideoCapture(0)
StepSize = 5

#set GPIO Pins
GPIO_TRIGGER = 23
GPIO_ECHO = 24

def ultrasonic_setup():
    #GPIO Mode (BOARD / BCM)
    GPIO.setmode(GPIO.BCM)
    #set GPIO direction (IN / OUT)
    GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
    GPIO.setup(GPIO_ECHO, GPIO.IN)

def distance():
    # set Trigger to HIGH
    GPIO.output(GPIO_TRIGGER, True)

    # set Trigger after 0.01ms to LOW
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)

    StartTime = time.time()
    StopTime = time.time()

    # save StartTime
    while GPIO.input(GPIO_ECHO) == 0:
        StartTime = time.time()

    # save time of arrival
    while GPIO.input(GPIO_ECHO) == 1:
        StopTime = time.time()

    # time difference between start and arrival
    TimeElapsed = StopTime - StartTime
    # multiply with the sonic speed (34300 cm/s)
    # and divide by 2, because there and back
    distance = (TimeElapsed * 34300) / 2

    return distance

def getChunks(l, n):
    a = []
    for i in range(0, len(l), n):
        a.append(l[i:i + n])
    return a

def process_camera_frame():
    ret,frame = cap.read()
    img = cv2.flip(frame, 0)
    blur = cv2.bilateralFilter(img,9,40,40)
    edges = cv2.Canny(blur,50,100)
    img_h = img.shape[0] - 1
    img_w = img.shape[1] - 1
    EdgeArray = []
    
    for j in range(0,img_w,StepSize):
        pixel = (j,0)
        for i in range(img_h-5,0,-1):
            if edges.item(i,j) == 255:
                pixel = (j,i)
                EdgeArray.append(pixel)
                break

    if len(EdgeArray) != 0:
        chunks = getChunks(EdgeArray, math.ceil(len(EdgeArray)/3))
    else:
        return
    
    #c = []
    distance = []
    for i in range(len(chunks)):
        x_vals = []
        y_vals = []
        for (x,y) in chunks[i]:
            x_vals.append(x)
            y_vals.append(y)
        avg_x = int(np.average(x_vals))
        avg_y = int(np.average(y_vals))
        #c.append([avg_y,avg_x])
        distance.append(math.sqrt((avg_x - 320)**2 + (avg_y - 640)**2))
        cv2.line(img, (320, 640), (avg_x,avg_y), (0,0,255), 2)
    
    cv2.imshow("frame", img)
    cv2.waitKey(5)

    if(distance[0] < distance[1]):
        if(distance[0] < distance[2]):
            return 0
        else:
            return 2
    else:
        if(distance[1] < distance[2]):
            return 1
        else:
            return 2

# Start a connection
the_connection = mavutil.mavlink_connection('/dev/ttyACM0')
# Wait for the first heartbeat
# This sets the system and component ID of remote system for the link
the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (the_connection.target_system,
the_connection.target_component))

angle = 90
speed = 0
ultrasonic_setup();
while True:
    dist = distance()
    if(dist < 50):
        if(dist < 15):
            speed = 0
        else:
            #speed = 30
            #speed = (int(((dist-15)*float(90))//35)%10)*10
            if dist > 40:
                speed = 70
            elif dist > 30:
                speed = 40
            elif dist > 20:
                speed = 20
            elif dist >=15:
                speed = 20
            speed = 20
    else:
        speed = 90
    
    ret = process_camera_frame()
    if ret == 0:
        print("Left direction is preferred");
        angle = 3 
    elif ret == 1:
        print("Forward direction is preferred");
        angle = 2 
    elif ret == 2:
        print("Right direction is preferred");
        angle = 1 
        
    message = speed + angle
    print ("Measured Distance = %.1f cm" % dist)
    print ("Adjusted Speed = %d" % speed)
    print ("Adjusted angle = %d" % angle)
    print ("Sent message = %d" % message)
    time.sleep(0.1)  
    
    #packed_message = struct.pack('BB', angle, speed) #first byte is angle, second byte is speed
    #int_packed_message = int.from_bytes(packed_message, byteorder='big')
    send_message = mavutil.mavlink.MAVLink_debug_message(0, angle, speed)
    the_connection.mav.send(send_message)
   