# Machine vision and camera integration - pick and place application

This example shows how to interface a camera (Datalogic T47) with the Eva Python SDK for dynamically moving Eva to an object detected by the camera and picking it up.

## Requirements

This example assumes you have a working machine vison camera. The camera is trained through its native software to give the X and Y coordinates and rotation angle of a detected object relative to the camera frame. 
These values are then used to perform the pick and place operation by using the inverse kinematics function.

The communication between the camera and the robot is based on a TCP/IP server, the parameters of which have been set up in the camera software. 
A string containing the X, Y and angle parameters is generated and streamed to Eva.

The relative orientation of the camera with respect to the robot position is obtained through a calibration procedure. 
This example assumes that the camera's XY plane is parallel to Eva's XY plane. Their relative rotation can be input in the script.

## Supported Version

Requirements to run this script as is:

- Eva software version: 3.x.x
- Eva Python SDK: 2.x.x

## Project description

__[main.py](main.py)__

This file is the main script and contains the logic to connect to Eva and to communicate with the camera. This will perform the pick and place task, as a function of the objects sensed by the camera. 

You can use this file to modify the toolpath settings (speed, additional waypoints, IOs, etc).

__[evaUtilities.py](evaUtilities.py)__

This file contains the auxiliary functions needed by the **main.py** script (frame of reference handling, angle and quaternion conversion, inverse kinematics, string reading, string parsing). You should not changed this file.


__[config/config_manager.py](config/config_manager.py)__

This file loads the YAML parameter file. You should not changed this file.

__[config/use_case_config.yaml](config/use_case_config.yaml)__

This YAML file contains the parameters needed for the simulation, which are initialized to zero by default. 

In detailed, you will have to input:
- EVA communication setup (host, token)
- TCP/IP communication setup (server, port)
- objects characteristic (number, name, size) 
- EVA configurations (home, guess, drop-off). 

**NOTE: without changing these parameters, EVA will automatically set its home position in the upright configuration**
