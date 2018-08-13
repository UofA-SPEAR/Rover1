#include <dirent.h>
#include <iostream>
#include <string>
#include <sstream>
#include <unistd.h>
#include <cmath>

#include "ros/ros.h"
#include <rover1/input_arm.h>
#include "serial/serial.h"

#define HOME_SIGNAL 254


class Arm_Serial {
 public:
    Arm_Serial(std::string port_num, uint32_t baud_num);
    void centralControlCallback(const rover1::input_arm::ConstPtr& msg);
    void sendCommand(char command, uint32_t angle);

 private:
    // Place holders until we determine what actual values we want to send to
    // the Arm Ardino
    int angle_, stepper_num_;
    const std::string port_;
    uint32_t baud_;

    ros::NodeHandle nh_;
    ros::Subscriber arm_cmd_sub;
    serial::Serial my_serial;

};

// Arm_Serial Object Constructor with Port + Baud
Arm_Serial::Arm_Serial(std::string port_num, uint32_t baud_num):
  port_(port_num),
  baud_(baud_num),
  my_serial(port_, baud_, serial::Timeout::simpleTimeout(1000)) {
  // Initialize the control_cmd_sub
  arm_cmd_sub = nh_.subscribe("/user_arm_commands", 50,
      &Arm_Serial::centralControlCallback, this);

  ROS_INFO("Port = %s", port_num.c_str());
  ROS_INFO("Baud = %d", baud_num);
  ROS_INFO("Port Open? [%s]", my_serial.isOpen() ? "Yes" : "No");
}


// Callback of the topic /numbers
void Arm_Serial::centralControlCallback(
    const rover1::input_arm::ConstPtr& msg) {

  size_t bytes_sent;

  //ROS_INFO("[ARM] {base, shoulder, elbow, wrist_pitch, wrist_roll, fingers} = {%d, %d, %d, %d, %d, %d}", bytes[0], bytes[1], bytes[2], bytes[3], bytes[4], bytes[5]);

  sendCommand(1, msg->pot);

}

void Arm_Serial::sendCommand(char command, uint32_t angle){
    uint8_t b1 = angle & 0x000000FF;
    uint8_t b2 = (angle & 0x0000FF00) >> 8;
    uint8_t b3 = (angle & 0x00FF0000) >> 16;
    uint8_t b4 = (angle & 0xFF000000) >> 24;

    ROS_INFO("[ARM] %d [%d| %d %d %d %d]", command, angle, b1, b2, b3, b4);

    uint8_t bytes[8] = {2, command, b1, b2, b3, b4, (uint8_t)(b1 + b2 + b3 + b4), 3};
    my_serial.write(bytes, 8);
}


int main(int argc, char **argv) {
  // Initializing ROS node with a name of arm_serial
  ros::init(argc, argv, "arm_serial");

  std::string arduino_port;
  uint32_t def_baud = 9600;

  DIR* dir = opendir("/dev/serial_arm");
  if (ENOENT != errno) {
    arduino_port = "/dev/serial_arm";
  }

  if (arduino_port.empty()) {
    ROS_ERROR("NO ARDUINO CONNECTED. PLEASE RESTART THE PROGRAM "
        "WITH ARDUINO CONNECTED TO THE USB PORT");
    //exit(EXIT_FAILURE);
  }

  // Initialize the Serial Node object
  Arm_Serial serial_node(arduino_port, def_baud);

  ros::spin();
  return 0;
}
