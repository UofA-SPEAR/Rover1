#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <rover1/controller.h>
#include <rover1/input_arm.h>
#include <rover1/input_drive.h>
#include <rover1/drive_cmd.h>
#include <iostream>
#include <math.h>

#define Pi acos(-1.0)

class Central_Control {
 public:
    Central_Control();
    void recToPolar(double x, double y, double &r, double &theta);

 private:
    ros::NodeHandle nh_;

    ros::Subscriber usercmd_sub;
    ros::Subscriber user_drive_sub;
    ros::Subscriber user_arm_sub;

    ros::Publisher drive_pub;
    ros::Publisher arm_pub;

    void cmdCallback(const rover1::controller::ConstPtr& msg);
    void userDriveCallback(const rover1::input_drive::ConstPtr& msg);
    void userArmCallback(const rover1::input_arm::ConstPtr& msg);
};


Central_Control::Central_Control() {
  // Initialize publisher
  drive_pub = nh_.advertise<rover1::drive_cmd>("/drive_topic", 10);
  arm_pub = nh_.advertise<rover1::controller>("/arm_topic", 10);

  // Intialize subscriber
  usercmd_sub = nh_.subscribe("/user_commands", 10,
      &Central_Control::cmdCallback, this);

  // Subscribers for user input from server controller
  user_drive_sub = nh_.subscribe("/user_drive_commands", 10,
      &Central_Control::userDriveCallback, this);

  user_arm_sub = nh_.subscribe("/user_arm_commands", 10,
      &Central_Control::cmdCallback, this);
}

// Command Callback
void Central_Control::cmdCallback(const rover1::controller::ConstPtr& msg) {
  // Send the msg to the drive system or arm based on the current state
  if (msg->state) {
    arm_pub.publish(msg);
  } else {
    drive_pub.publish(msg);
  }
}

void Central_Control::userDriveCallback(const rover1::input_drive::ConstPtr& msg) {
  // TODO handle msg
}

void Central_Control::userArmCallback(const rover1::input_arm::ConstPtr& msg) {
  // TODO handle msg
}


void Central_Control::recToPolar(double x, double y,
    double &r, double &theta) {
  const double toDegrees = 180.0/Pi;

  r = sqrt((pow(x, 2))+(pow(y, 2)));
  theta = atan(y/x) * toDegrees;

  // Convert theta to the correct quadrant
  if ((x <= 0 && y >= 0) || (x <= 0 && y <= 0)) {
    theta = theta + 180;  // Quad 2 or 3
  } else if (x >= 0 && y < 0) {
    theta = theta + 360;  // Quad 4
  }
}

int main(int argc, char **argv) {
  // Initializing ROS node with a name of demo_topic_subscriber
  ros::init(argc, argv, "central_control");

  Central_Control control_node;

  // Spinning the node
  ros::spin();
  return 0;
}

