#include <dirent.h>
#include <iostream>
#include <string>
#include <algorithm>
#include <cmath>
#include <sstream>
#include <unistd.h>

#include "ros/ros.h"
#include <rover1/input_drive.h>
#include "serial/serial.h"
#include "std_msgs/Int32.h"


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

    void centralControlCallback(const rover1::input_drive::ConstPtr& msg);
    void command(char cmd, int8_t val);
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
  control_cmd_sub = nh_.subscribe("/drive_topic", 50,
      &Drive_Serial::centralControlCallback, this);
}


// Callback
void Drive_Serial::centralControlCallback(
    const rover1::input_drive::ConstPtr& msg) {

  int8_t left = 127 * msg->left;
  int8_t right = 127 * msg->right;
  ROS_INFO("[DRIVE] Sending {L:[%d] R:[%d]}", left, right);

  command('L', left);
  usleep(10000); // sleep for x seconds
  command('R', right);

}

void Drive_Serial::command(char cmd, int8_t val){
  const uint8_t byte[5] = {2, (uint8_t)cmd, (uint8_t)val, 
      (uint8_t) (((uint8_t)cmd)+((uint8_t)val)), 3};
  this->my_serial.write(byte, 5);
}

int main(int argc, char **argv) {
  // Initializing ROS node with a name of demo_topic_subscriber
  ros::init(argc, argv, "drive_serial");


  std::string arduino_port;
  uint32_t def_baud = 9600;

  DIR* dir = opendir("/dev/serial_drive");
  if (ENOENT != errno) {
     arduino_port = "/dev/serial_drive";
  }

  if (arduino_port.empty()) {
    ROS_ERROR("NO ARDUINO CONNECTED. PLEASE RESTART THE PROGRAM "
      "WITH ARDUINO CONNECTED TO THE USB HUB");
  }

  // Initialize the Serial Node object
  Drive_Serial serial_node(arduino_port, def_baud);

  ros::spin();
  return 0;
}


