#include <ros/ros.h>
#include <iostream>
#include "std_msgs/String.h"
#include <tuple>
#include "dynamixel_sdk_examples/GetPosition.h"
#include "dynamixel_sdk_examples/SetPosition.h"
#include "dynamixel_sdk/dynamixel_sdk.h"
#include <vector>

using namespace dynamixel;
using namespace std;

// Control table address
#define ADDR_TORQUE_ENABLE    24
#define ADDR_GOAL_POSITION    30
#define ADDR_PRESENT_POSITION 36

// Protocol version
#define PROTOCOL_VERSION      1.0             // Default Protocol version of DYNAMIXEL X series.

// Default setting
#define DXL1_ID               1               // DXL1 ID
#define BAUDRATE              1000000           // Default Baudrate of DYNAMIXEL X series
#define DEVICE_NAME           "/dev/ttyUSB0"  // [Linux] To find assigned port, use "$ ls /dev/ttyUSB*" command

#define TORQUE_ENABLE                1                 // Value for enabling the torque
#define TORQUE_DISABLE               0                 //Value for disabling the torque
#define DXL_MINIMUM_POSITION_VALUE   0               //Dynamixel will rotate between this value
#define DXL_MAXIMUM_POSITION_VALUE   1000            //and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
#define DXL_MOVING_STATUS_THRESHOLD  20                //Dynamixel moving status threshold

PortHandler * portHandler;
PacketHandler * packetHandler;
ros::Publisher set_position_pub;
ros::Subscriber set_position_sub;


void setPositionCallback(const dynamixel_sdk_examples::SetPosition::ConstPtr & msg);

int main(int argc, char ** argv)
{ 

  typedef vector< tuple<int,string> > tuple_list;

  int32_t position = 0;
  
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  portHandler = PortHandler::getPortHandler(DEVICE_NAME);
  packetHandler = PacketHandler::getPacketHandler(PROTOCOL_VERSION);

  if (!portHandler->openPort()) {
    ROS_ERROR("Failed to open the port!");
    return -1;
  }
  std::cout << "succeded to open port \n";
    //return 0 ;
  

  if (!portHandler->setBaudRate(BAUDRATE)) {
    ROS_ERROR("Failed to set the baudrate!");
    return -1;
  } 
  cout << "succeded to set baudrate \n";
    //return 0 ;
  

  dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL1_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
  if (dxl_comm_result != COMM_SUCCESS) {
    ROS_ERROR("Failed to enable torque for Dynamixel ID %d", DXL1_ID);
    return -1;
  }
  cout << "succeded to enable torque \n";
  cout << "ready to set position \n";

  ros::init(argc, argv, "gripper_controller");
  ros::NodeHandle nh;
  set_position_pub = nh.advertise<dynamixel_sdk_examples::SetPosition>("/set_position",10);
  set_position_sub = nh.subscribe("/set_position", 10, setPositionCallback);
  dynamixel_sdk_examples::SetPosition msg_position;
  
  //ros::Duration(5.0).sleep(); //sleep for 5 sec
  sleep(1); //very important to put in as this gives time for the pub/sub to setup


  ros::Rate loop_rate(100); //publishes 10 messages per second
  msg_position.id = 1;
  msg_position.position = 1023;
  set_position_pub.publish(msg_position);
  ros::spinOnce();
  sleep(3);
  
  for (int i = 1000;i>0;)
  {
  
    msg_position.id = DXL1_ID;

    msg_position.position = i;

    set_position_pub.publish(msg_position);
    // sleep(1); //very important to put in as this gives time for the publisher to initiate
    cout << msg_position << endl;
    // cout << set_position_pub.getNumSubscribers() <<endl;
    ros::spinOnce();
    loop_rate.sleep();
    i = i - 10;

    dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL1_ID, ADDR_PRESENT_POSITION, (uint32_t *)&position, &dxl_error);
    if (dxl_comm_result == COMM_SUCCESS) {
      ROS_INFO("getPosition : [ID:%d] -> [POSITION:%d]", DXL1_ID, position);
    }
  }
  
  portHandler->closePort();
  return 0;
}


void setPositionCallback(const dynamixel_sdk_examples::SetPosition::ConstPtr & msg)
{
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  // Position Value of X series is 4 byte data. For AX & MX(1.0) use 2 byte data(uint16_t) for the Position Value.

  uint32_t position = (unsigned int)msg->position; // Convert int32 -> uint32

  // Write Goal Position (length : 4 bytes)
  // When writing 2 byte data to AX / MX(1.0), use write2ByteTxRx() instead.
  dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, (uint8_t)msg->id, ADDR_GOAL_POSITION, position, &dxl_error);
  if (dxl_comm_result == COMM_SUCCESS) {
    ROS_INFO("setPosition : [ID:%d] [POSITION:%d]", msg->id, msg->position);
  } else {
    ROS_ERROR("Failed to set position! Result: %d", dxl_comm_result);
  }
}
