# Indoor-lights

make sure you have the following prerequisites:
PX4Flow sensor connected to the Pixhawk
Pixhawk connected to the mini-computer via a serial connection
Python environment with necessary libraries installed

The script does the following:
Imports necessary libraries (pymavlink for communication with Pixhawk).
Sets up a connection to the Pixhawk.
Defines functions for:
Requesting message intervals from the Pixhawk
Sending velocity commands to the Pixhawk
The main loop:
Requests OPTICAL_FLOW messages at the specified update rate
Reads OPTICAL_FLOW messages from the Pixhawk
Uses a PID controller to calculate corrections based on the flow in the X direction
Sends velocity commands to follow the line (lights on the ceiling)
Maintains a consistent update rate

To use this script:
Install the required library: pip install pymavlink
Adjust the serial port name ('/dev/ttyACM0') and baud rate (115200) according to your setup.
Modify the PID constants (KP, KI, KD) and MAX_VELOCITY as needed for your specific drone and environment.
Run the script on your mini-computer.
Note that this implementation assumes:
The PX4Flow sensor is mounted facing upwards towards the ceiling.
The lights on the ceiling form a straight line that the drone should follow.
The drone's forward direction is aligned with the line.
You may need to adjust the logic depending on your specific setup and requirements. Also, ensure that your Pixhawk is configured correctly for indoor flight and that you have appropriate safety measures in place when testing the system.
Created on 2021-04-01