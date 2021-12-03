# RaspberryPi-Controlled-Quadcopter-using-Dronekit to Detect the Dusted PV (Solar) Cell



## Project Idea and Background
* Solar energy is a free, inconsumable and clean energy source. 
* Due to these advantages, a number of studies have been conducted throughout the world on efficient photovoltaic (PV) modules to make more use of solar energy. 
* Below are some of the crucial parameters that can influence the conversion efficiency of photovoltaic systems. 
  * The type of PV Cell
  * Orientation and inclination angle of PV module
  * Location
  * Cell temperature
  * Shading
  * Deposition of dust and pollution on module surface.


The purpose of this project is to design an autonomous drone to detect the dusted/rusted PV cell in a solar park that contains thousands of PV panels. And send the detected panel issue to the server/ control room that monitors the performance of PV cell based on the output power. And if needed the automated cleaning solution will be triggered that will clean the specific PV cell

## Components Uses
* **RaspberryPi 4 B+**
* **PixHawk Flight Controller with external GPS and builtin compass**
* **USB WebCam**
* **Quadcopter frame F450**
* **ESC for BLDC**
* **4 BLDC**
* **LiPO battery 3S**


## Project Architecture
The project Architecture contains 5 files

* **dronePanel.py**
* **droneControl.py** (helper class that contains function to talk to Flight Controller to control the drone using [DroneKit](https://dronekit-python.readthedocs.io/en/latest/) APIs)
* **dronControlUI.ui** (UI design/layout file created using [PyGubu Desinger](https://github.com/alejandroautalan/pygubu-designer), that is very helpful tool in designing UI of python project with just drag and drop and then you can easily render it with Tkinter library and bind the button and view port accordinly)
* **pvPanelDustDetection.py** (another helper class that contains the function to detect the dusted PV panel )
* **model.tflite** (Trained model based on the research paper:  and converted to tflite for raspberry pi)

## RPi UI and Conrols
