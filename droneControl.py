# Author: Muhammad Hassaan Bashir
# Dated: 26-12-2020
# Dronekit API helper and connectivity Class

# Importing Libraries
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time

# Drone Controlling Helper Class
class DroneControl:

    # Drone Control Class initialization function
    # Setting Up variables in this function
    def __init__(self):
        global vehicle
        self.vehicle = None

    # Vehicle Connectivity method
    def connectFlightController(self):
        # Initializing and Connecting to Vehicle
        print("Initializing and Connecting Vehicle\n")
        # Connecting to Quad Copter Flight Controller
        print("---------------------------------------------------\nConnecting to vehicle on /dev/Serial0\n---------------------------------------------------\n")
        self.vehicle = connect('/dev/ttyACM0', wait_ready=True, baud=9600)
        if self.vehicle != None:
            # vehicle = connect('192.168.18.8:14550', wait_ready=True)
            print("Connected\n")
            # GPS Settings
            print("---------------------- GPS Settings --------------------------\n")
            print("GPS: %s" % self.vehicle.gps_0)
            print("GPS: %s" % self.vehicle.mode.name) 
            print("--------------------------------------------------------------\n")

    # ============================================================
    # Quadcopter Control Functions using DroneKit APIs Starts Here
    # 1. Initialoze Drone
    # 2. armingDrone
    # 3. takeoffDrone
    # 4. landDrone
    # 5. Get GPS info
    # 1. Arming Function armingDrone
    def initDrone(self):
        print ("Basic pre-arm checks")
        # First Get and Set the Home Location
        while not self.vehicle.home_location:
            cmds = self.vehicle.commands
            cmds.download()
            cmds.wait_ready()
            if not self.vehicle.home_location:
                print(" Waiting for home location ...")
        # We have a home location, so print it!        
        print("\n Home location: %s" % self.vehicle.home_location)
        
        # Don't let the user try to arm until autopilot is ready
        while not self.vehicle.is_armable:
            print (" Waiting for vehicle to initialise...")
        print (self.vehicle.is_armable)
        time.sleep(1)    
        return True

    # 2. Arming Function armingDrone
    def armingDrone(self):
        print ("Arming motors")
        # Copter should arm in GUIDED mode
        self.vehicle.mode    = VehicleMode("GUIDED")
        print(self.vehicle.mode.name)
        self.vehicle.armed   = True

        while not self.vehicle.armed:
            print ("Waiting for arming...")
        time.sleep(1)
        return True

    # 3. Takingoff Function takeoffDrone
    def takeoffDrone(self,aTargetAltitude):
        print ("Taking off!")
        self.vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude  
        # Check that vehicle has reached takeoff altitude
        while True:
            print (" Altitude: ", self.vehicle.location.global_relative_frame.alt) 
            #Break and return from function just below target altitude.        
            if self.vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: 
                print ("Reached target altitude")
                break
            elif self.vehicle.location.global_relative_frame.alt<0:
                print("Negative Values")
                break
            time.sleep(1)
        time.sleep(0.5)
        return True

    # 4. landingDrone function landDrone
    def landDrone(self,mode):
        ##This function ensures that the vehicle has landed (before vechile.close is called)
        print("Landing")
        ##thread_distance.join()
        time.sleep(1)
        print(f"Landing using Mode: {mode}")
        self.vehicle.mode = VehicleMode(mode)
        # vehicle.mode = VehicleMode("LAND")
        while self.vehicle.armed:
            time.sleep(1)
        return True

    # 5. Getting GPS info and Home Location
    def getGPSinformation(self):
        # This functions retrives the GPS information and home location
        print("\n......Getting GPS info......")
        textBox = "\nGlobal Location: " + str(self.vehicle.location.global_frame)
        textBox += "\nGlobal Location (relative altitude): " + str(self.vehicle.location.global_relative_frame)
        textBox += "\nLocal Location: " + str(self.vehicle.location.local_frame)
        textBox += "\nAttitude :" + str(self.vehicle.attitude)
        print(f"\n{textBox}")
        return textBox
    # Quadcopter Control Functions using DroneKit APIs Ends Here
    # ============================================================
