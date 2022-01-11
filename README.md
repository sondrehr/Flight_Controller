# Flight_Controller
The code used on my STM32 diy drone flight controller

## Installation and running
Download and add the repository https://github.com/rogerclarkmelbourne/Arduino_STM32 to devices folder within the Arduino IDE folder to access the boards hardware settings.
With this done one can use the Arduino IDE to upload the code directly to the board. 

## Drone design
### Drone design v1.0
<img src="https://github.com/sondrehr/Flight_Controller/blob/master/Drone%20Pictures/IMG_20200118_231401.jpg" alt="Drone version 1" width="800"/>



### Drone design v2.0
<img src="https://github.com/sondrehr/Flight_Controller/blob/master/Drone%20Pictures/IMG_20210730_162126.jpg" alt="Drone version 2" width="800"/>
Redesigned most parts and added lidar and another GPS.

## Circuit board 
### Circuit board v1.0(left) vs v2.0(right)

#### Backside
Focused on more solid soldering. In addition to cleaning up in v2.0

<img src="https://github.com/sondrehr/Flight_Controller/blob/master/Drone%20Pictures/IMG_20210723_120347.jpg" alt="Drone version 2" width="700"/>

#### Frontside
Added more light to display different states in v2.0

<img src="https://github.com/sondrehr/Flight_Controller/blob/master/Drone%20Pictures/IMG_20210723_120402.jpg" alt="Drone version 2" width="700"/>

### Circuit board v3.0

This is going to be a PCB instead a diy soldered board

#### Schematic made in KiCad

<img src="https://github.com/sondrehr/Flight_Controller/blob/master/PCB%20kiCAD/FC_schematic.png" alt="pcb" width="700"/>

#### PCB model made in KiCad

Made on a 4 layer PCB where layer 2 and 3(not included in the img) are VCC and GND. The traces are roughly split into a vertical layer(1) and horizontal layer(4) to make tracing easier.

<img src="https://github.com/sondrehr/Flight_Controller/blob/master/PCB%20kiCAD/FC_pcb.PNG" alt="pcb" width="700"/>

#### 3D model from KiCad

<img src="https://github.com/sondrehr/Flight_Controller/blob/master/PCB%20kiCAD/FC_3D.png" alt="pcb" width="700"/>

#### Picture of PCB v3.0 with soldered components

<img src="https://github.com/sondrehr/Flight_Controller/blob/master/Drone%20Pictures/Capture.JPG" alt="pcb" width="700"/>



