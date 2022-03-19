#!/usr/bin/env python3
"""
Pi Camera Live Image Classification

Detects objects in continuous stream of images from Pi Camera. Use Edge Impulse
Runner and downloaded .eim model file to perform inference. Bounding box info is
drawn on top of detected objects along with framerate (FPS) in top-left corner.

Author: EdgeImpulse, Inc.
Date: August 3, 2021
License: Apache-2.0 (apache.org/licenses/LICENSE-2.0)
"""

import cv2
import numpy as np
import pyzbar.pyzbar as qr
import smtplib
import os, sys, time
from picamera import PiCamera
from picamera.array import PiRGBArray
from edge_impulse_linux.image import ImageImpulseRunner
import serial
import math


# Settings
model_file = "modelfile.eim"             # Trained ML model from Edge Impulse
res_width = 96                          # Resolution of camera (width)
res_height = 96                         # Resolution of camera (height)
rotation = 0                            # Camera rotation (0, 90, 180, or 270)

# The ImpulseRunner module will attempt to load files relative to its location,
# so we make it load files relative to this program instead
dir_path = os.path.dirname(os.path.realpath(__file__))
model_path = os.path.join(dir_path, model_file)

# Load the model file
runner = ImageImpulseRunner(model_path)

# Initialize model (and print information if it loads)
try:
    model_info = runner.init()
    print("Model name:", model_info['project']['name'])
    print("Model owner:", model_info['project']['owner'])
    
# Exit if we cannot initialize the model
except Exception as e:
    print("ERROR: Could not initialize model")
    print("Exception:", e)
    if (runner):
            runner.stop()
    sys.exit(1)

# Initial framerate value


# Initial framerate value
ser=serial.Serial('/dev/ttyACM0',9600, timeout = 1)
'''
try:
    ser=serial.Serial('/dev/ttyACM0',9600, timeout = 1)
except:
    try:
        ser=serial.Serial('/dev/ttyACM1',9600, timeout = 1)
    except:
        try:
            ser=serial.Serial('/dev/ttyACM2',9600, timeout = 1)
        except:
            try:
                ser=serial.Serial('/dev/ttyACM3',9600, timeout = 1)
            except:
                print("Please check port !")
   
'''
def getcharacter():

    # Start the camera
    counter = 0
    AccX = 0
    fps = 0
    with PiCamera() as camera:
        
        # Configure camera settings
        camera.resolution = (res_width, res_height)
        camera.rotation = rotation
        
        # Container for our frames
        raw_capture = PiRGBArray(camera, size=(res_width, res_height))

        # Continuously capture frames (this is our while loop)
        for frame in camera.capture_continuous(raw_capture, 
                                                format='bgr', 
                                                use_video_port=True):
                                                
            # Get timestamp for calculating actual framerate
            timestamp = cv2.getTickCount()
            
            # Get Numpy array that represents the image
            img = frame.array
            
            # Convert image to RGB
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            
            # Encapsulate raw image values into array for model input
            features, cropped = runner.get_features_from_image(img)
            
            # Perform inference
            res = None
            try:
                res = runner.classify(features)
            except Exception as e:
                print("ERROR: Could not perform inference")
                print("Exception:", e)
                
            # Display predictions and timing data
            print("-----")
            results = res['result']['classification']
            for label in results:
                prob = results[label]
                #print(label + ": " + str(round(prob, 3)))
            #print("FPS: " + str(round(fps, 3)))
            
            # Find label with the highest probability
            max_label = max(results, key=results.get)
            print("the character is",max_label)
            
            x=bytes(max_label,'utf-8')
            ser.write(x)
            time.sleep(0.1)

            # Draw max label on preview window
            cv2.putText(img,
                        max_label,
                        (0, 12),
                        cv2.FONT_HERSHEY_PLAIN,
                        1,
                        (255, 255, 255))
                        
            # Draw max probability on preview window
            cv2.putText(img,
                        str(round(results[max_label], 2)),
                        (0, 24),
                        cv2.FONT_HERSHEY_PLAIN,
                        1,
                        (255, 255, 255))
            
            # Show the frame
            cv2.imshow("Frame", img)
            
            # Clear the stream to prepare for next frame
            raw_capture.truncate(0)
            
            # Calculate framrate
            frame_time = (cv2.getTickCount() - timestamp) / cv2.getTickFrequency()
            fps = 1 / frame_time
            
            #counter = counter + 1
            break
            #AccX = AccX + int.from_bytes(x, "big")
            #if (counter >= 10):
            #    if(AccX / 10 == int.from_bytes(x, "big")):
            #        break
            #    else:
            #        AccX = int.from_bytes(x, "big")
            
    # Clean up
    cv2.destroyAllWindows()
    return x




while True:
    receivedChar = ser.read()
    time.sleep(0.01)
    
    
    if (receivedChar == b'A'):
        print(receivedChar)
        
        a = getcharacter()
        #print(a)

        ser.write(a)
    elif (receivedChar == b'B'):
        print(receivedChar)
        b = getcharacter()
        #print(b)
        ser.write(b)
    receivedChar = ""
""" 

while True:
    receivedChar = ser.read()
    time.sleep(0.01)
    
    
    if (receivedChar == b'A'):
        print(receivedChar)
        
        a = b'hulk'
        #print(a)

        ser.write(a)
    elif (receivedChar == b'B'):
        print(receivedChar)
        b = b'captain_america'
        #print(b)
        ser.write(b)
    receivedChar = ""
    
"""    
    
    
    