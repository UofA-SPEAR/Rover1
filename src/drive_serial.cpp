#include <iostream>
#include <rover1/controller.h>
#include <rover1/drive_cmd.h>
#include <string>
#include <sstream>
#include "ros/ros.h"
#include "serial/serial.h"
#include "std_msgs/Int32.h"
#include <unistd.h>

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
  dir_(0),
  port_("/dev/ttyUSB0"),
  baud_(9600), 
  my_serial(port_, baud_, serial::Timeout::simpleTimeout(1000)) {

  /* nh_.param("port", port_, port_); */
  /* nh_.param("baud", baud_, baud_); */
  std::string is_open = "No";
  if(my_serial.isOpen())
    is_open = "Yes";
  ROS_INFO("Port = %s", port_.c_str());
  ROS_INFO("Baud = %d", baud_);
  ROS_INFO("Port Open? [%s]", is_open.c_str());

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
  std::stringstream stream;
  stream << std::fixed << std::setprecision(5) << msg->magnitude;
  const std::string str = stream.str();

  ROS_INFO("[DRIVE] Sending [%s]", str.c_str());

  uint8_t buf;
  size_t bytes_sent = this->my_serial.write(str);
  size_t bytes_read = this->my_serial.read(&buf, str.length());

  ROS_INFO("[DRIVE] Bytes Sent [%zu]", bytes_sent);
  ROS_INFO("[DRIVE] Read [%s] [%zu bytes]", &buf, bytes_read);
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


