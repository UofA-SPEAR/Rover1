#include <iostream>
#include <rover1/controller.h>
#include <string>
#include "ros/ros.h"
#include "serial/serial.h"
#include "std_msgs/Int32.h"

/*
 * TODO: STILL CURRENTLY USING THE DRIVE DEFINED MSG, NEED TO DEFINE
 * A NEW MSG TYPE FOR THE ARM
 */


class Arm_Serial {
 public:
    Arm_Serial();
    Arm_Serial(std::string port, uint32_t baud);

 private:
    // Place holders until we determine what actual values we want to send to
    // the Arm Ardino
    double motor1_, motor2_, motor3_, motor4_, motor5_, motor6_;
    const std::string port_;
    uint32_t baud_;

    ros::NodeHandle nh_;
    ros::Subscriber arm_cmd_sub;
    serial::Serial my_serial;

    void centralControlCallback(const rover1::controller::ConstPtr& msg);
};


// Legacy Arm_Serial Object Constructor
Arm_Serial::Arm_Serial():
  motor1_(0),
  motor2_(0),
  motor3_(0),
  motor4_(0),
  motor5_(0),
  motor6_(0) {
  // Initialize the control_cmd_sub
  arm_cmd_sub = nh_.subscribe("/arm_topic", 10,
      &Arm_Serial::centralControlCallback, this);
}


// Arm_Serial Object Constructor with Port + Baud
Arm_Serial::Arm_Serial(std::string port_num, uint32_t baud_num):
  motor1_(0),
  motor2_(0),
  motor3_(0),
  motor4_(0),
  motor5_(0),
  motor6_(0),
  port_(port_num),
  baud_(baud_num),
  my_serial(port_, baud_, serial::Timeout::simpleTimeout(1000)) {
  // Initialize the control_cmd_sub
  arm_cmd_sub = nh_.subscribe("/arm_topic", 10,
      &Arm_Serial::centralControlCallback, this);
}


// Callback of the topic /numbers
void Arm_Serial::centralControlCallback(
    const rover1::controller::ConstPtr& msg) {

  ROS_INFO("[ARM] X Coord  [%f]", msg->x_coord);
  ROS_INFO("[ARM] Y Coord  [%f]", msg->y_coord);

  // TODO(jordan/mark): Forward this data to the Arduino
}


int main(int argc, char **argv) {
  /*
   * CURRENT THOUGHT:The Arduino on the arm may be using C++ and 
   * the Arduino IDE, so it may be better to use ROS Serial (IDK)
   * Otherwise, we just need to decode the data on the arduino side
   */

  // Initializing ROS node with a name of demo_topic_subscriber
  ros::init(argc, argv, "arm_serial");

  // Initialize the Serial Node object
  Arm_Serial serial_node;

  /*Arm_Serial serial_node(argv[1], argv[2]); */

  ros::spin();
  return 0;
}


