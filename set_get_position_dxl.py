#!/usr/bin/env python

import os
import time
import rospy
from dynamixel_sdk import *
from dynamixel_sdk_examples.srv import *
from dynamixel_sdk_examples.msg import *

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

# Control table address
ADDR_TORQUE_ENABLE      = 24               # Control table address is different in Dynamixel model
ADDR_GOAL_POSITION      = 30
ADDR_PRESENT_POSITION   = 36

# Protocol version
PROTOCOL_VERSION            = 1.0               # See which protocol version is used in the Dynamixel

# Default setting
DXL_ID                      = 1                 # Dynamixel ID : 1
BAUDRATE                    = 1000000             # Dynamixel default baudrate : 57600
DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
                                                # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 0               # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 1000            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 20                # Dynamixel moving status threshold

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

current_position = 0

def set_goal_pos_callback(data):
    print("Setting Goal Position of ID %s = %s" % (data.id, data.position))
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, data.id, ADDR_GOAL_POSITION, data.position)

def get_present_pos(req):
    global current_position
    dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, req.id, ADDR_PRESENT_POSITION)
    print("Present Position of ID %s = %s" % (req.id, dxl_present_position))
    current_position = dxl_present_position
    return dxl_present_position

# def read_write_py_node():
    
    # rospy.spin()

def main():
    try:
       portHandler.openPort()
       print("Succeeded to open the port")
    except:
        print("Failed to open the port")
        print("Press any key to terminate...")
        getch()
        quit()

    # Set port baudrate
    try:
        portHandler.setBaudRate(BAUDRATE)
        print("Succeeded to change the baudrate")
    except:
        print("Failed to change the baudrate")
        print("Press any key to terminate...")
        getch()
        quit()
    rospy.sleep(2)

    ##ENABLING TORQUE

    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    print("dxl_comm_result", dxl_comm_result)
    if dxl_comm_result != COMM_SUCCESS:
        # print("dxl_comm_result", dxl_comm_result)
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        print("Press any key to terminate...")
        getch()
        quit()
    elif dxl_error != 0:
        
        print("%s" % packetHandler.getRxPacketError(dxl_error))
        print("Press any key to terminate...")
        getch()
        quit()
    else:
        print("DYNAMIXEL has been successfully connected")

    print("Ready to set Position.")

    rospy.init_node('testing_dynamixel_2',anonymous=True)
    topic_position = "/set_position"
    pub = rospy.Publisher(topic_position,  SetPosition, queue_size=10)
    msg_position = SetPosition()
    rospy.Subscriber('set_position', SetPosition, set_goal_pos_callback)
    rospy.Service('get_position', GetPosition, get_present_pos)
    
    # rospy.Service('get_position', GetPosition, get_present_pos)
    
    time.sleep(0.5)
    
    # dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION)

    

    msg_position.id = 1
    msg_position.position = 950    
    pub.publish(msg_position)
    
    elapsed_time = 0
    rospy.sleep(2)
    communication_time = 0.01
    loop_rate = rospy.Rate(100)
    # position = [940,930,920]

    for i in range(1000,0,-10):   


        msg_position.id = 1
        msg_position.position = i
        pub.publish(msg_position)

        loop_rate.sleep()
        dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION)

        
        print("the current position of motor is ", dxl_present_position)
    
    
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    portHandler.closePort()

if __name__ == '__main__':
    main()
