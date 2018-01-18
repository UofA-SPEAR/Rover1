#include <iostream>
#include <rover1/controller.h>
#include <rover1/drive_cmd.h>
#include <string>
#include "ros/ros.h"
#include "serial/serial.h"
#include "std_msgs/Int32.h"

class Drive_Serial {
 public:
    Drive_Serial();
    Drive_Serial(std::string port, uint32_t baud);

 private:
    int mtude_, dir_;
    const std::string port_;
    uint32_t baud_;

    ros::NodeHandle nh_;
    ros::Subscriber control_cmd_sub;
    serial::Serial my_serial;

    void centralControlCallback(const rover1::drive_cmd::ConstPtr& msg);
};


// Legacy Drive_Serial Object Constructor
Drive_Serial::Drive_Serial():
  mtude_(0),
  dir_(0) {
  // Initialize the control_cmd_sub
  control_cmd_sub = nh_.subscribe("/drive_topic", 10,
      &Drive_Serial::centralControlCallback, this);
}


// Drive_Serial Object Constructor with Port + Baud
Drive_Serial::Drive_Serial(const std::string port_num, uint32_t baud_num):
  mtude_(0),
  dir_(0),
  port_(port_num),
  baud_(baud_num),
  my_serial(this->port_, (uint32_t)this->baud_,
      serial::Timeout::simpleTimeout(1000)) {
  
  // Initialize the control_cmd_sub
  control_cmd_sub = nh_.subscribe("/drive_topic", 10,
      &Drive_Serial::centralControlCallback, this);
}


// Callback of the topic /numbers
void Drive_Serial::centralControlCallback(
    const rover1::drive_cmd::ConstPtr& msg) {

  ROS_INFO("[DRIVE] Magnitude  [%f]", msg->magnitude);
  ROS_INFO("[DRIVE] Polar Angle  [%f]", msg->polar_angle);

  // TODO(jordan/mark): Forward this data to the Arduino
}


int main(int argc, char **argv) {
  // Initializing ROS node with a name of demo_topic_subscriber
  ros::init(argc, argv, "drive_serial");

  // Initialize the Serial Node object
  Drive_Serial serial_node;

  /* Initialize the Serial Node with port + baud */
  //Drive_Serial serial_node(argv[1], argv[2]);

  ros::spin();
  return 0;
}


