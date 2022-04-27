#include "ros/ros.h"
#include "std_msgs/Float32.h"


void sensor_callback(const std_msgs::Float32::ConstPtr& msg)
{
    printf("I heard: %f \n", msg->data);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "potentiometer_read");
    ros::NodeHandle node;

    ros::Subscriber sub = node.subscribe("/analog_input07",1000,sensor_callback);

    ros::spin();
    return 0;
}
