# Author: Muhammad Hassaan Bashir
# Dated: 25-12-2020

# Main Program with UI for Controlling QuadCopter

# Importing Libraries
import os
import pygubu
import tkinter as tk
from PIL import Image, ImageTk
import cv2
import droneControl
import queue
import threading
import pvPanelDustDetection

import RPi.GPIO as GPIO
import time


GPIO.setmode(GPIO.BOARD)
GPIO.cleanup()
# Servo Set angle Function
def SetAngle(angle, motor):
    global servopin
    # Check which motor has been selected
    if motor == "M1":
        servopin = int(servo1_pin)
    else:
        servopin = int(servo2_pin)
    
    print(servopin)
    # Duty Cycle calculation for Angle
    duty = angle / 18 + 2
    print(f"\nSetting Angle {angle} on Motor: {motor} with dutycycle {duty}")
    GPIO.setup(servopin, GPIO.OUT)
    pwm = GPIO.PWM(servopin,50)
    pwm.start(0)
    pwm.ChangeDutyCycle(duty)
    time.sleep(1)
    pwm.ChangeDutyCycle(2.5)
    print(f"\nAngle set:  {angle} on Motor: {motor} with dutycycle {duty}")
    pwm.ChangeDutyCycle(0)
    pwm.stop()

# Servo RPI
global servo1_pin
global servo2_pin
global servopin
servopin = 0
servo1_pin = 37
servo2_pin = 35
global angle1
global angle2
angle1 = 120
angle2 = 15




# Threads for Connectivity Takeoff and other FC commands
def createConnectionThread(queue):

    textValue = "Flight Controller State: Connecting..."
    flightControllerConnectionState.config(text = textValue, background="#E8B828")

    quadCopter.connectFlightController()
    print("Here in FC Button")
    if quadCopter:
        queue.put(quadCopter)
        print(f"Value  of {altValue.get()}")

        textValue = "Flight Controller State: Connected"
        flightControllerConnectionState.config(text = textValue, background="#125500")
        initializeDrone.config(state = "normal")
    
    else:
        connecttoFlightController.config(state = "normal")
    
    getGPSinfoButton.config(state = 'normal')


# Initialize Drone Function
def initializeDroneFunction(queue):
    isDroneInitialize = quadCopter.initDrone()
    print("Here in Initialize Drone Button")
    if isDroneInitialize:
        armCopterButton.config(state = "normal")

# Arm Drone Function
def armingDroneFunction(queue):
    isDroneArmed = quadCopter.armingDrone()
    print("Here in Arming Drone Button")
    if isDroneArmed:
        takingOffButton.config(state = "normal")

# TakingOff drone Function
def takingoffDroneFunction(queue):
    isDroneReachAltitude = quadCopter.takeoffDrone(2)
    print("Here in Taking off Button")
    if isDroneReachAltitude:
        landButton.config(state = "normal")
        rtlButton.config(state = "normal")

# Landing at current position function
def landingDroneFunction(queue):
    isDroneLanded = quadCopter.landDrone("LAND")
    print("Here in Landing Button")
    if isDroneLanded:
        initializeDrone.config(state = "normal")

# Landing at Launch position function
def landingRTLDroneFunction(queue):
    isDroneLanded = quadCopter.landDrone("RTL")
    print("Here in Landing RTL Button")
    if isDroneLanded:
        initializeDrone.config(state = "normal")


# Thread for Real time video stream and detection window
def pvPanelDetectionthreadFunction(self, queue):
    while True:
        ret, frame = cap.read()
        if ret:
            # frame = cv2.flip(frame, 1)
            # cv2image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGBA)
            cv2imageStatus = "Detection Disabled"
            cv2imageStatus = pvPanelDetector.pvPollutionDetection(frame)
            print(f"Image Status {cv2imageStatus}")

            cv2image = cv2.resize(frame, (250, 250))
            cv2image = cv2.cvtColor(cv2image, cv2.COLOR_BGR2RGBA)
            if isPVPanelPerformanceDetectionEnabled:
                cv2image = cv2.putText(cv2image,cv2imageStatus, self.org, self.font, self.fontScale, self.color, self.thickness, cv2.LINE_AA)
            img = Image.fromarray(cv2image)
            imgtk = ImageTk.PhotoImage(image=img)
            pvImageViewer.imgtk = imgtk
            pvImageViewer.configure(image=imgtk)

# Setting Up project Path for UI
PROJECT_PATH = os.path.dirname(__file__)
PROJECT_UI = os.path.join(PROJECT_PATH, "droneControlUI.ui")

# Camera OpenCV Settings
IMAGE_RESIZE_FACTOR = 0.1
width, height = 250, 250
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

# UI Class
class DroneApp:
    def __init__(self, parent):
        # GPIO.cleanup()
        self.builder = builder = pygubu.Builder()
        builder.add_resource_path(PROJECT_PATH)
        builder.add_from_file(PROJECT_UI)
        self.mainwindow = builder.get_object('pv_Panel_Detection_Control_Panel', parent)
        builder.connect_callbacks(self)
        

         # Setting Font and text for CV2
        # font 
        self.font = cv2.FONT_HERSHEY_SIMPLEX 
        # org 
        self.org = (3, 240) 
        # fontScale 
        self.fontScale = 0.4
        # Blue color in BGR 
        self.color = (255, 0, 0) 
        # Line thickness of 2 px 
        self.thickness = 1
        # Load TFLite model and allocate tensors.

        # Global Variables Declaration
        global pvImageViewer
        global cameraUpDownControl
        global cameraLeftRightControl
        global connecttoFlightController
        global quadCopter
        global initializeDrone
        global armCopterButton
        global takingOffButton
        global getDroneInfoButton
        global getGPSinfoButton
        global rtlButton
        global landButton
        global altValue
        global flightControllerConnectionState
        global pvPanelDetectionButton
        global queue
        global thread
        global pvPanelDetector
        global droneInfoTextBox
        
        # Global Variable for detection of Dust/Pollution on Panel
        global isPVPanelPerformanceDetectionEnabled
        isPVPanelPerformanceDetectionEnabled = False

        # Initializing Quadcopter and PV Panel Detector to NONE
        quadCopter = None
        pvPanelDetector = None
        
        # Setting up pvPanelDetector
        pvPanelDetector = pvPanelDustDetection.PvPanelDustDetection()

        # Global Variable Initialization        
        self.parent = parent
        # Image Viewer Variable
        pvImageViewer = tk.Label(builder.get_object('imageViewer'), font="Arial 10") 
        pvImageViewer.pack()
        # Gimbal Up/Down Slider
        cameraUpDownControl = builder.get_object('setGimbalUpDown')
        cameraUpDownControl.bind("<ButtonRelease-1>", self.setCameraGimbalUpDown)
        cameraUpDownControl.set(15)
        # Gimbal Left/Right SlideBar
        cameraLeftRightControl = builder.get_object('setGimbalLeftRight')
        cameraLeftRightControl.bind("<ButtonRelease-1>", self.setCameraGimbalLeftRight)
        cameraLeftRightControl.set(120)
        # QuadCopter Connectibity
        connecttoFlightController = builder.get_object('connectFC')
        # Initialize Drone
        initializeDrone = builder.get_object('initializeDrone')
        # Arming Copter button
        armCopterButton = builder.get_object('armDrone')
        # Taking off Button
        takingOffButton = builder.get_object('takeOff')
        # Get Drone info Button
        getDroneInfoButton = builder.get_object('getInfo')
        # Get GPS Info Button
        getGPSinfoButton = builder.get_object('getGPSinfo')
        getGPSinfoButton.config(state = 'normal')
        # RTL Button
        rtlButton = builder.get_object('rTL')
        # land Button
        landButton = builder.get_object('land')
        # PV Panel Detection button
        pvPanelDetectionButton = builder.get_object('startDetection')
        # altValue spinbox
        altValue = builder.get_object('altValue')
        # FC Connection State
        flightControllerConnectionState = builder.get_object('fcState')
        # Drone info Text Box
        droneInfoTextBox = builder.get_object('droneInfo')
        
        SetAngle(10,"M1")
        SetAngle(90,"M2")

        self.runImage()
        # self.runImage2()

    # ----------------- Buttons Callback functions ----------------- 
    def setCameraGimbalUpDown(self, event):
        # Get Angle Value
        val = cameraUpDownControl.get()
        global angle1

        # Check if angle is change or not
        if (angle1 != val):
            cameraUpDownControl.config(state="disable")
            print("\nValue of Angle1 "+ str(val))
            angle1 = val
            # Setting Desired Angle by calling SetAngle Function
            SetAngle(angle1, "M1")
            cameraUpDownControl.config(state="normal")
        # pass

    def setCameraGimbalLeftRight(self,event):
        # Get Angle Value
        val = cameraLeftRightControl.get()
        global angle2

        # Check if angle is change or not
        if (angle2 != val):
            cameraLeftRightControl.config(state="disable")
            print("\nValue of Angle2 "+ str(val))
            angle2 = val
            # Setting Desired Angle by calling SetAngle Function
            SetAngle(angle2, "M2")
            cameraLeftRightControl.config(state="normal")
        # pass

    def connectToFlightController(self):
        connecttoFlightController.config(state = "disable")
        getGPSinfoButton.config(state = 'disable')
        global queue
        global thread

        queue = queue.Queue()
        thread = threading.Thread(target=createConnectionThread,
                              args=(queue,))
        thread.daemon = True  # so you can quit the demo program easily :)
        thread.start()
        # pass

    def initializeDrone(self):
        global queue
        global thread

        initializeDrone.config(state = "disable")
        thread = threading.Thread(target=initializeDroneFunction,
                              args=(queue,))
        thread.daemon = True  # so you can quit the demo program easily :)
        thread.start()

    def armDrone(self):
        global queue
        global thread
        armCopterButton.config(state = "disable")
        thread = threading.Thread(target=armingDroneFunction,
                              args=(queue,))
        thread.daemon = True  # so you can quit the demo program easily :)
        thread.start()
        # pass

    def getDroneInfo(self):

        pass

    def droneTakeOff(self):
        global queue
        global thread
        takingOffButton.config(state = "disable")
        thread = threading.Thread(target=takingoffDroneFunction,
                              args=(queue,))
        thread.daemon = True  # so you can quit the demo program easily :)
        thread.start()
        # pass

    def getGPSinfo(self):
        getGPSinfoButton.config(state = 'disable')
        droneInfoTextBox.config(state = 'normal')
        textBoxValue = quadCopter.getGPSinformation()
        # textBoxValue = droneInfoTextBox.get("1.0",tk.END)
        # sprint(f"here now {textBoxValue}")
        droneInfoTextBox.insert(tk.END,textBoxValue)
        getGPSinfoButton.config(state = 'normal')

    def returnToLunch(self):
        global queue
        global thread
        landButton.config(state = "disable")
        rtlButton.config(state = "disable")
        thread = threading.Thread(target=landingRTLDroneFunction,
                              args=(queue,))
        thread.daemon = True  # so you can quit the demo program easily :)
        thread.start()
        # pass

    def droneLand(self):
        global queue
        global thread
        landButton.config(state = "disable")
        rtlButton.config(state = "disable")
        thread = threading.Thread(target=landingDroneFunction,
                              args=(queue,))
        thread.daemon = True  # so you can quit the demo program easily :)
        thread.start()
        # pass

    def startDustDetectionModel(self):
        global isPVPanelPerformanceDetectionEnabled
        # Check wheteher Detection state is enabled or not
        if isPVPanelPerformanceDetectionEnabled:
            pvPanelDetectionButton.config(text="Start PV Panel Dust Detection")
            isPVPanelPerformanceDetectionEnabled = False
        else:
            pvPanelDetectionButton.config(text="Stop PV Panel Dust Detection")
            isPVPanelPerformanceDetectionEnabled = True

        # pass

    # ----------------- Buttons Callback functions ----------------- 

    def run(self):
        self.mainwindow.mainloop()

    
    def runImage2(self):
        thread = threading.Thread(target=pvPanelDetectionthreadFunction,
                              args=(self,queue,))
        thread.daemon = True  # so you can quit the demo program easily :)
        thread.start()

    def runImage(self):
        
        
        ret, frame = cap.read()
        if ret:
            # frame = cv2.flip(frame, 1)
            # cv2image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGBA)
            cv2imageStatus = None
            cv2image = frame
            if isPVPanelPerformanceDetectionEnabled:
                cv2imageStatus = pvPanelDetector.pvPollutionDetection(frame)
                # print(f"\n{cv2imageStatus}")        
            cv2image = cv2.resize(cv2image, (224, 224))
            cv2image = cv2.putText(cv2image,cv2imageStatus, self.org, self.font, self.fontScale, self.color, self.thickness, cv2.LINE_AA)
            cv2image = cv2.cvtColor(cv2image, cv2.COLOR_BGR2RGBA)
            img = Image.fromarray(cv2image)
            imgtk = ImageTk.PhotoImage(image=img)
            pvImageViewer.imgtk = imgtk
            pvImageViewer.configure(image=imgtk)
        pvImageViewer.after(10, self.runImage)       
        
        # cap = cv2.VideoCapture(0)
        # img = cv2.cvtColor(cap.read()[1], cv2.COLOR_BGR2RGB) # to RGB
        # photo = PIL.ImageTk.PhotoImage(image = PIL.Image.fromarray(img))

        # pvImageViewer.create_image(0, 0, image=photo, anchor=tk.NW)

if __name__ == '__main__':
    root = tk.Tk()
    app = DroneApp(root)
    quadCopter = droneControl.DroneControl()
    app.run()
