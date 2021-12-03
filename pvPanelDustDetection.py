# Author: Muhammad Hassaan Bashir
# Dated: 26-12-2020
# PV Panel Dust Detection Tensorflow Model Class

# Importing Libraries
import numpy as np
# import tensorflow as tf
import tflite_runtime.interpreter as tflite

import cv2
import pathlib

class PvPanelDustDetection:
    # Constructor / Initialization function
    def __init__(self):
        global interpreter
        global input_details
        global output_details

        # Setting Font and text for CV2
        # font 
        # self.font = cv2.FONT_HERSHEY_SIMPLEX 
        # # org 
        # self.org = (5, 30) 
        # # fontScale 
        # self.fontScale = 0.2
        # # Blue color in BGR 
        # self.color = (255, 0, 0) 
        # # Line thickness of 2 px 
        # self.thickness = 1
        # Load TFLite model and allocate tensors.
        
        # interpreter = tf.lite.Interpreter(model_path="D:\Hassaan Bashir\Embedded C Course\Python Training\\fyp\model.tflite")
        interpreter = tflite.Interpreter(model_path="/home/pi/fyp_finalCode/model.tflite")
        
        # Get input and output tensors.
        input_details = interpreter.get_input_details()
        output_details = interpreter.get_output_details()

        interpreter.allocate_tensors()

        # input details
        print(input_details)
        # output details
        print(output_details)

    # Pollution Detection Function
    def pvPollutionDetection(self, img):
        # Resizing Image to required size
        new_img = cv2.resize(img, (224, 224))
        new_img = new_img.astype(np.float32)
        interpreter.set_tensor(input_details[0]['index'], [new_img])

        interpreter.invoke()
        rects = interpreter.get_tensor(
            output_details[0]['index'])

        output_data = interpreter.get_tensor(output_details[0]['index'])
        # print("For file {}, the output is {}".format(file.stem, output_data))

        detectState = output_data[0]
        if (max(detectState) > 0.93):
            detectState = np.argmax(detectState)

            print(detectState)
            if (detectState == 0):
                # Using cv2.putText() method 
                # new_img = cv2.putText(new_img, 'Cleaned Panel', self.org, self.font, self.fontScale, self.color, self.thickness, cv2.LINE_AA) 
                return "Cleaned Panel"
            else:
                # new_img = cv2.putText(new_img, 'Dirty Panel', self.org, self.font, self.fontScale, self.color, self.thickness, cv2.LINE_AA)
                return "Dirty Panel"   
            # cv2.imshow("image", new_img)
        
        else:
            # new_img = cv2.putText(new_img, 'Unable to detect Panel', self.org, self.font, self.fontScale, self.color, self.thickness, cv2.LINE_AA)
            return "No PV Panel in frame"
        # return new_img