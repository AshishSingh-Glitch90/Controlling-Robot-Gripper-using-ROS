#!/usr/bin/env python
import rospy
import sys
sys.path.append("/home/ashish/catkin_ws/src/variable_server")
# import var_serv
from std_msgs.msg import Float32

# finger_current_position = 0

def potentiometer_read_callback(data):
    # print("reading of potentiometer is /analog_7",data.data)
    finger_current_position = distance_every_volt * data.data / 100
    print("current position of finger analog_0",finger_current_position)
    # finger_current_position = distance_every_volt

def potentiometer_read_callback_2(data):
    # global finger_current_position
    # print("reading of potentiometer /analog_6 is",data.data)
    finger_current_position_analog_6 = distance_every_volt * data.data / 100

    print("current position of finger analog 1",finger_current_position_analog_6)
    # position = var_serv.variable_server()
    # position.set_finger_pos(finger_current_position)
    # finger_position = get_finger_current_position()
    # print(finger_position)
    
    # return finger_current_position

# def get_finger_current_position():
#     global finger_current_position
#     return finger_current_position

if __name__ == '__main__':
    try:
        total_length_potentiometer = 20 #centimeters
        total_voltage_potentiometer = 5 #volts
        distance_every_volt = total_length_potentiometer / total_voltage_potentiometer
        potentiometer_topic = '/analog_input00'
        potentiometer_topic_2 = '/analog_input01'
        rospy.init_node('potentiometer_read_Gripper', anonymous=True)
        sub = rospy.Subscriber(potentiometer_topic, Float32, potentiometer_read_callback)
        Sub_2 = rospy.Subscriber(potentiometer_topic_2, Float32, potentiometer_read_callback_2)    
        # rospy.Rate(10)
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")