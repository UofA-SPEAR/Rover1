#include <dirent.h>
#include <iostream>
#include <rover1/controller.h>
#include <string>
#include <sstream>
#include "ros/ros.h"
#include "serial/serial.h"
#include "std_msgs/Int32.h"
#include <unistd.h>

class Drive_Serial {
 public:
    Drive_Serial(std::string port, uint32_t baud);

 private:
    int mtude_, dir_;
    const std::string port_;
    uint32_t baud_;

    ros::NodeHandle nh_;
    ros::Subscriber control_cmd_sub;
    serial::Serial my_serial;

    void centralControlCallback(const rover1::controller::ConstPtr& msg);
};


// Drive_Serial Object Constructor with specified Port + Num. Currently not used
Drive_Serial::Drive_Serial(const std::string port_str, uint32_t baud_num):
  mtude_(0),
  dir_(0),
  port_(port_str),
  baud_(baud_num),
  my_serial(this->port_, baud_,
  serial::Timeout::simpleTimeout(1)) {
  // TODO(Jordan): May want to send handshake msg with Arduino
  // AKA double check that it isn't a different device on that port

  ROS_INFO("Port = %s", port_.c_str());
  ROS_INFO("Baud = %d", baud_);
  ROS_INFO("Port Open? [%s]", my_serial.isOpen() ? "Yes" : "No");

  // Initialize the control_cmd_sub
  control_cmd_sub = nh_.subscribe("/drive_topic", 10,
      &Drive_Serial::centralControlCallback, this);
}


// Callback
void Drive_Serial::centralControlCallback(
    const rover1::controller::ConstPtr& msg) {
  size_t bytes_read;
  size_t bytes_sent;
  std::string buf;
  std::stringstream stream;

  ROS_INFO("[DRIVE] Left Stick  [%f]", msg->left_stick);
  ROS_INFO("[DRIVE] Right Stick  [%f]", msg->right_stick);

  stream << std::fixed << std::setprecision(5) << msg->left_stick
    << " " << msg->right_stick << std::endl;
  const std::string str = stream.str();

  ROS_INFO("[DRIVE] Sending [%s]", str.c_str());

  bytes_sent = this->my_serial.write(str);
  buf = this->my_serial.readline();

  /* ROS_INFO("[DRIVE] Bytes Sent [%zu]", bytes_sent); */
  ROS_INFO("[DRIVE] Read %s [%zu bytes]", buf.c_str(), buf.length());
}


int main(int argc, char **argv) {
  // Initializing ROS node with a name of demo_topic_subscriber
  ros::init(argc, argv, "drive_serial");

  // TODO(jordan/mark) restructure the code for two arduinos

  std::string arduino_port;
  uint32_t def_baud = 9600;

  // Check if the arduino is connected to ttyUSB0
  DIR* dir = opendir("/dev/ttyUSB0");
  if (ENOENT != errno) {
     arduino_port = "/dev/ttyUSB0";
  }

  // Check if the arduino is connected to ttyACM0
  dir = opendir("/dev/ttyACM0");
  if (ENOENT != errno) {
     arduino_port = "/dev/ttyACM0";
  }

  if (arduino_port.empty()) {
    ROS_ERROR("NO ARDUINO CONNECTED. PLEASE RESTART THE PROGRAM "
      "WITH ARDUINO CONNECTED TO THE USB HUB");
    exit(EXIT_FAILURE);
  }

  // Initialize the Serial Node object
  Drive_Serial serial_node(arduino_port, def_baud);

  // TODO(mark/jordan): Restructure code to while loop and ros:spinOnce()
  // Therefore, we can constantly send msgs to the arduino
  // (instead of only sending msgs when the callback is called)
  
  ros::spin();
  return 0;
}


