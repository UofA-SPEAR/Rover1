#include <dirent.h>
#include <iostream>
#include <rover1/controller.h>
#include <string>
#include <sstream>
#include <unistd.h>
#include "ros/ros.h"
#include "serial/serial.h"
#include "std_msgs/Int32.h"

/*
 * TODO: STILL CURRENTLY USING THE DRIVE DEFINED MSG, NEED TO DEFINE
 * A NEW MSG TYPE FOR THE ARM
 */


class Arm_Serial {
 public:
    Arm_Serial(std::string port_num, uint32_t baud_num);

 private:
    // Place holders until we determine what actual values we want to send to
    // the Arm Ardino
    int angle_, stepper_num_;
    const std::string port_;
    uint32_t baud_;

    ros::NodeHandle nh_;
    ros::Subscriber arm_cmd_sub;
    serial::Serial my_serial;

    void centralControlCallback(const rover1::controller::ConstPtr& msg);
};

// Arm_Serial Object Constructor with Port + Baud
Arm_Serial::Arm_Serial(std::string port_num, uint32_t baud_num):
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

  // TODO(jordan): change specific message sent to arm

  size_t bytes_sent;
  std::string buf;
  std::stringstream stream;

  ROS_INFO("[ARM] Left Stick  [%f]", msg->left_stick);
  ROS_INFO("[ARM] Right stick  [%f]", msg->right_stick);

  // Create msg that is sent to the Arduino
  stream << std::fixed << std::setprecision(5) << msg->left_stick
    << " " << msg->right_stick << std::endl;

  const std::string str = stream.str();

  ROS_INFO("[ARM] Sending [%s]", str.c_str());

  bytes_sent = this->my_serial.write(str);
  buf = this->my_serial.readline();

  ROS_INFO("[ARM] Read %s [%zu bytes]", buf.c_str(), buf.length());
}


int main(int argc, char **argv) {
  // Initializing ROS node with a name of demo_topic_subscriber
  ros::init(argc, argv, "arm_serial");

//  std::string arduino_port;
//  uint32_t def_baud = 9600;

  // TODO(jordan): Probs need to change this if we have two arduinos connected

  // Check if the arduino is connected to ttyUSB0
//  DIR* dir = opendir("/dev/ttyUSB0");
//  if (ENOENT != errno) {
//    arduino_port = "/dev/ttyUSB0";
//  }

//  dir = opendir("/dev/ttyACM0");
//  if (ENOENT != errno) {
//    arduino_port = "/dev/ttyACM0";
//  }

//  if (arduino_port.empty()) {
//    ROS_ERROR("NO ARDUINO CONNECTED. PLEASE RESTART THE PROGRAM "
//        "WITH ARDUINO CONNECTED TO THE USB PORT");
//    exit(EXIT_FAILURE);
//  }

  // Initialize the Serial Node object
//  Arm_Serial serial_node(arduino_port, def_baud);

  ros::spin();
  return 0;
}


