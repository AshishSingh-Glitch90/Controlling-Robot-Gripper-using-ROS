#!/usr/bin/env python
import os
import numpy as np
import time
import rospy
from dynamixel_sdk import *
from dynamixel_sdk_examples.srv import *
from dynamixel_sdk_examples.msg import *
import math
import matplotlib.pyplot as plt
import pandas as pd

from std_msgs.msg import Float32 #importing library for reading potentiometer

from geometry_msgs.msg import Twist
from rospy.topics import Publisher, Subscriber


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
DXL_ID_2                    = 2                 # Dynamixel ID : 2
BAUDRATE                    = 1000000           # Dynamixel default baudrate : 57600
DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller
TORQUE_ENABLE               = 1                 # Value for enabling the torque
TORQUE_DISABLE              = 0                 # Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = 0               # Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 1000            # and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 5               # Dynamixel moving status threshold

#INITIALIZING GLOBAL VARIABLE 

finger_current_position = 0.0  #in metres
finger_current_position_analog_0 = 0.0 # in metres
total_length_potentiometer = 20 #centimeters
total_voltage_potentiometer = 5 #volts
distance_every_volt = total_length_potentiometer / total_voltage_potentiometer
force = []
force_analog_0 = []
distance_id1 = []
finger_position = []
finger_position_analog_0 = []
total_force_gripper = []


portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)


def potentiometer_read_callback_2(data):
    global finger_current_position
    global distance_every_volt
    finger_current_position = distance_every_volt * data.data / 100

def potentiometer_read_callback(data_0):
    global distance_every_volt
    global finger_current_position_analog_0
    finger_current_position_analog_0 = distance_every_volt * data_0.data / 100


def set_goal_pos_callback(data):
    print("Setting Goal Position of ID %s = %s \n" % (data.id, data.position))
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, data.id, ADDR_GOAL_POSITION, data.position)


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
    dxl_comm_result_2, dxl_error_2= packetHandler.write1ByteTxRx(portHandler, DXL_ID_2, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    print(dxl_comm_result, dxl_error)
    print(dxl_comm_result_2, dxl_error_2)

    if (dxl_comm_result and dxl_comm_result_2 != COMM_SUCCESS):
    # if (dxl_comm_result != COMM_SUCCESS):
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result_2))
        print("Press any key to terminate...")
        getch()
        quit()
    elif (dxl_error and dxl_error_2 != 0):
    # elif (dxl_error != 0):
        print("%s" % packetHandler.getRxPacketError(dxl_error))
        # print("%s" % packetHandler.getRxPacketError(dxl_error_2))
        print("Press any key to terminate...")
        getch()
        quit()
    else:
        print("DYNAMIXELS has been successfully connected")

    print("Ready to set Position.")

    rospy.init_node('stiffness_algorithm',anonymous=True)

    global finger_current_position #important to keep the current position of finger
    global finger_current_position_analog_0
    
    potentiometer_topic = '/analog_input00'
    
    potentiometer_topic_2 = '/analog_input01'
    
    sub = rospy.Subscriber(potentiometer_topic, Float32, potentiometer_read_callback)
    
    Sub_2 = rospy.Subscriber(potentiometer_topic_2, Float32, potentiometer_read_callback_2)    
    
    topic_position = "/set_position"
    pub = rospy.Publisher(topic_position,  SetPosition, queue_size=10)
    msg_position = SetPosition()
    msg_position_2 = SetPosition()
    rospy.Subscriber('set_position',SetPosition, set_goal_pos_callback)

    time.sleep(.5) #very important to put in as this gives time for the publisher and subcriber to setup
    
    ## constant variables for both dynamixels

    c_1 = 27.34 #Newton
    c_2 = -168.15 # per metre
    total_degree = 300.0
    total_bits = 1023.0
    radius = 25.46/2 #millimeter

    length = radius * (total_degree/total_bits) * math.pi / 180

    #changing the datatype of the position to Unassigned int according to the required dynamixel motor

    position_ID_2 = 0
    position_ID_1 = 1023
    position_ID_1 = np.int16(position_ID_1)
    position_ID_2 = np.int16(position_ID_2)

    print("opening the gripper \n")

    msg_position.id = 1
    msg_position_2.id = 2

    msg_position.position = position_ID_1
    msg_position_2.position = position_ID_2
    pub.publish(msg_position)
    pub.publish(msg_position_2)

    rospy.sleep(1)

    #Making sure that the dynamixel has reached to the extreme position to open gripper

    dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION)
    dxl_present_position_2, dxl_comm_result_2, dxl_error_2 = packetHandler.read2ByteTxRx(portHandler, DXL_ID_2, ADDR_PRESENT_POSITION)

    while (abs(dxl_present_position_2 - position_ID_2) > DXL_MOVING_STATUS_THRESHOLD and abs(dxl_present_position - position_ID_1) > DXL_MOVING_STATUS_THRESHOLD):
        pub.publish(msg_position)
        pub.publish(msg_position_2)

    
    rospy.sleep(0.5) #important to put it here other before publishing program moves forward

    print("Gripper is open press key to continue")

    getch()


    loop_rate = rospy.Rate(100) # Operating Frequency
    
    dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION)
    dxl_present_position_2, dxl_comm_result_2, dxl_error_2 = packetHandler.read2ByteTxRx(portHandler, DXL_ID_2, ADDR_PRESENT_POSITION)

    z = 0
    # for z in range(0,15,1):
    # while(z<1):


    ## Maximum position at 100 hz = 12.8898 so change in position should be less than that 
    #  
    for position_ID_2 in range (0,600,10):    
        # msg_position.id = 1
        msg_position_2.id = 2
        # position_ID_1 = position_ID_1 + 10
        # msg_position.position = position_ID_1
        msg_position_2.position = position_ID_2
        pub.publish(msg_position)
        pub.publish(msg_position_2)

        # rospy.sleep(0.5)

        loop_rate.sleep() 

        # potentiometer_read()
        # rospy.sleep(0.5) #very important as this gives time for the motor to rotate
        dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read2ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION)
        dxl_present_position_2, dxl_comm_result_2, dxl_error_2 = packetHandler.read2ByteTxRx(portHandler, DXL_ID_2, ADDR_PRESENT_POSITION)
        
        # rospy.sleep(.5)
        print("the current position of motor is ", dxl_present_position)
        print("current position of motor ID _2", dxl_present_position_2)


        # quit()

        # if dxl_present_position_2 < 1025 and dxl_present_position < 1025:
        
        length_dynamixel_ID1 = 1023 - dxl_present_position
        
        current_length_moved_1 = (length * length_dynamixel_ID1)/1000 #metres
        

        #metres and ID1 starts from 15cm mark and reduces from there also the width of the finger is 1 cm till face of magnet

        S_i_2 = .121 - (finger_current_position + 0.01 + current_length_moved_1) 
        
        
        current_length_moved_2 = (length * dxl_present_position_2)/1000 #metres
        
        #ID2 starts from 4cm mark also the width of the finger is 1cm till the face of magnet 

        S_o_2 = finger_current_position - (.043 + .01 + current_length_moved_2)


        delta_force = abs((c_1* math.exp(c_2 * S_o_2)) - (c_1* math.exp(c_2 * S_i_2)))

        ## FOR ANALOG 0 POTENTIOMETER AND FINGER

        # motor is already at 4.3 cms and also the width of the finger is 1cm till the face of the magnet

        S_o_2_analog0 = finger_current_position_analog_0 - (0.043 + 0.01 + current_length_moved_2)


        ## motor is already at 4cms and also the width of the finger is 1cm till the face of the magnet 

        S_i_2_analog0 = .123 - (finger_current_position_analog_0 + 0.01 + current_length_moved_1) 

        delta_force_analog_0 = abs((c_1* math.exp(c_2 * S_o_2_analog0)) - (c_1* math.exp(c_2 * S_i_2_analog0)))

        finger_position_analog_0.append(finger_current_position_analog_0)

        finger_position.append(finger_current_position)

        #REAL TIME PLOTTING
        force_analog_0.append(delta_force_analog_0)
        force.append(delta_force)

        print("delta force",delta_force)
        print("delta force analog 0", delta_force_analog_0)
                

            ### GRAPH FOR ANALOG 1 and ANALOG 0

            # plt.clf()
            # plt.subplot(2,1,1)
            # plt.scatter(finger_position,force)
            # plt.plot(finger_position,force)
            # plt.xlabel('Position of analog 1 finger')
            # plt.ylabel("Force applied by analog 0 finger")
            # plt.tight_layout()
            # plt.draw()

            # plt.subplot(2,1,2)
            # plt.scatter(finger_position_analog_0,force_analog_0)
            # plt.plot(finger_position_analog_0,force_analog_0)
            # plt.xlabel('Position of analog 0 finger')
            # plt.ylabel("Force applied by analog 0 finger")
            # plt.tight_layout()
            # plt.draw()

            # plt.pause(.0000001)

            # break
            #Putting threshold for individual finger
        if (delta_force > 5 and delta_force_analog_0 > 5):
            print("Object has been in touch of both fingers exiting the FOR loop")
            break
        # else:
        #     pass

    
    #SAVING DATA IN CSV FILE

    # df = pd.DataFrame(list(zip(finger_position,force,finger_position_analog_0,force_analog_0)),columns=['analog_1_finger','force_analog_1','analog_0_finger','force_analog_0'])

    # df.to_csv('force_profile.csv', header=True, index=False)

    # print(df)

    print("press key to exit program")
    getch()

    print("PROGRAM HAS EXITED")

    plt.subplot(2,1,1)
    plt.scatter(finger_position,force)
    plt.plot(finger_position,force)
    plt.xlabel('Position of analog 1 finger')
    plt.ylabel("Force applied by analog 1 finger")
    plt.tight_layout()
    plt.draw()

    plt.subplot(2,1,2)
    plt.scatter(finger_position_analog_0,force_analog_0)
    plt.plot(finger_position_analog_0,force_analog_0)
    plt.xlabel('Position of analog 0 finger')
    plt.ylabel("Force applied by analog 0 finger")
    plt.tight_layout()
    plt.draw()

    plt.show()

    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    dxl_comm_result_2, dxl_error_2 = packetHandler.write1ByteTxRx(portHandler, DXL_ID_2, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
    portHandler.closePort()

def draw_graph(z):
    global distance_id1
    global force
    plt.cla()
    plt.plot(distance_id1,force)

if __name__ == '__main__':        
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
