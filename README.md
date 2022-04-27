# Controlling-Robot-Gripper-using-ROS
- Controlling a two fingered robot gripper using 2 dynamixel RX-24F motors and Potentiometers to calculate the stiffness of the object.
## For controlling dynamixel RX-24f
- Download the dynamixel_sdk package for ROS which has all the libraries to work with.(http://wiki.ros.org/dynamixel_sdk)
- Download and copy my code to the *DynamxelSDK/ROS/dynamixel_sdk_examples/src*
- For cpp files required changes are to be made in the cmakelist and the package.xml files.
## For reading potentiometer through phidget_22 
- Download the phidget ROS package (http://wiki.ros.org/phidgets)
- Download and copy the my potentiometer reading code in *phidgets_drivers/phidgets_ik/src*
