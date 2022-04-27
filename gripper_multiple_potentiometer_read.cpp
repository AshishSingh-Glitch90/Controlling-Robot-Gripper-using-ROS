#include <ros/ros.h>
#include "std_msgs/Float32.h"
#include "phidgets_ik/ik_ros_i.h"

void potentiometer_read_callback(const std_msgs::Float32::ConstPtr& msg)
{
    printf("I heard from first potentiometer: %f \n", msg->data);
}

void potentiometer_read_callback_2(const std_msgs::Float32::ConstPtr& msg)
{
    printf("I heard from 2nd potentiometer: %f \n", msg->data);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gripper_potentiometer_read");
    
    //ros::NodeHandle nh;
    //ros::NodeHandle nh_private("~");
    //phidgets::IKRosI ik(nh, nh_private);
    
    ros::NodeHandle node;
    ros::Subscriber sub = node.subscribe("/analog_input07",1000,potentiometer_read_callback);
    ros::Subscriber sub_2 = node.subscribe("/analog_input06",1000,potentiometer_read_callback_2);
    ros::spin();
    return 0;
}