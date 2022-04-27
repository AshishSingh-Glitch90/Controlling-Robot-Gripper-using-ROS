#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32

def sensor_callback(data):
    print("reading of potentiometer is",data.data)

def sensor_read():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    potentiometer_topic = '/analog_input00'
    rospy.init_node('potentiometer_read', anonymous=True)
    sub = rospy.Subscriber(potentiometer_topic, Float32, sensor_callback)
    # spin() simply keeps python from exiting until this node is stopped

if __name__ == '__main__':
    try:
        potentiometer_topic = '/analog_input00'
        rospy.init_node('potentiometer_read', anonymous=True)
        sub = rospy.Subscriber(potentiometer_topic, Float32, sensor_callback)
        rospy.spin()
    except rospy.ROSInterruptException:

        rospy.loginfo("node terminated.")