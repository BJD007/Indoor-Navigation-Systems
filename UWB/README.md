# UWB
## The setup:
- Multiple UWB anchors placed at known positions in the indoor environment
- One UWB tag mounted on the drone
- Mini-computer (e.g., Raspberry Pi) connected to the UWB tag and Pixhawk
- Pixhawk autopilot for drone control


This script provides a basic framework for indoor navigation using UWB PulseON modules. 

## Here's an explanation of the key components:
- Library Imports: We use pymavlink for communication with the Pixhawk and serial for reading data from the UWB modules.
- Constants: Define the update rate, number of UWB anchors, and their positions in the environment.
- Connections: Establish connections to both the Pixhawk and the UWB module.
- UWB Distance Reading: The read_uwb_distances() function reads distance measurements from the UWB modules.
- Trilateration: The trilaterate() function estimates the drone's position based on the UWB distance measurements. Note that this is a simplified placeholder and should be replaced with a more robust algorithm for production use.
- Position Target Sending: The send_position_target() function sends the estimated position to the Pixhawk as a target for the drone to move towards.
- Main Loop: Continuously reads UWB distances, estimates position, and sends position targets to the Pixhawk.

## To use this script:
- Install required libraries: pip install pymavlink pyserial
- Adjust the serial port names (/dev/ttyACM0 for Pixhawk, /dev/ttyUSB0 for UWB module) according to your setup.
- Modify the ANCHOR_POSITIONS to match the actual positions of your UWB anchors in the environment.
- Implement a more sophisticated trilateration or localization algorithm to improve position estimation accuracy.
- Run the script on your mini-computer.

## Notes
- Obstacle Avoidance: Implementing obstacle avoidance requires additional sensors (e.g., LiDAR, sonar) and algorithms (e.g., potential fields, vector field histograms).
- Path Planning: Implementing path planning might involve algorithms like A*, D*, RRT, or other motion planning algorithms.
- Calibration and Fine-Tuning: Ensure that the UWB system and Pixhawk are calibrated and fine-tuned for your specific environment and hardware.
- Safety Measures: Always have appropriate safety measures in place when testing the system, especially for indoor flights.

Created on 2015-10-01