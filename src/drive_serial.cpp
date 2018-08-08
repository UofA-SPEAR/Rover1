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

    int8_t left;
    int8_t right;

    ros::NodeHandle nh_;
    ros::Subscriber control_cmd_sub;
    serial::Serial my_serial;

    void centralControlCallback(const rover1::input_drive::ConstPtr& msg);
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
  left = 0;
  right = 0;
}

template <typename T> int sign(T val){
    return (T(0) < val) - (val < T(0));
}

// Callback
void Drive_Serial::centralControlCallback(
    const rover1::input_drive::ConstPtr& msg) {
  size_t bytes_sent;
  int8_t msgleft = msg->left * 127;
  int8_t msgright = msg->right * 127;

  if(msg->left == 0 && msg->right == 0){
      left = 0;
      right = 0;
  }else{
      left += sign(msgleft - left);
      right += sign(msgright - right);
  }


  ROS_INFO("[DRIVE] Sending {L:[%d] R:[%d]}", left, right);

  uint8_t L[1] = {'L'};
  uint8_t l[1] = {((uint8_t) left)};
  uint8_t R[1] = {'R'};
  uint8_t r[1] = {((uint8_t) right)};
  bytes_sent = this->my_serial.write(L, 1);
  this->my_serial.flush();
  bytes_sent = this->my_serial.write(l, 1);
  this->my_serial.flush();
  bytes_sent = this->my_serial.write(R, 1);
  this->my_serial.flush();
  bytes_sent = this->my_serial.write(r, 1);
  this->my_serial.flush();

  /*
  bytes_sent = this->my_serial.write('L');
  bytes_sent = this->my_serial.write(left);
  bytes_sent = this->my_serial.write('R');
  bytes_sent = this->my_serial.write(right);
  */
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


