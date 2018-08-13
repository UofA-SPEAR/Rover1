#include <iostream>
#include <math.h>

#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <rover1/input_arm.h>
#include <rover1/input_drive.h>

#define Pi acos(-1.0)

class Central_Control {
 public:
    Central_Control();
    void recToPolar(double x, double y, double &r, double &theta);

 private:
    ros::NodeHandle nh_;

    ros::Subscriber user_drive_sub;
    ros::Subscriber user_arm_sub;

    ros::Publisher drive_pub;
    ros::Publisher arm_pub;

    void userDriveCallback(const rover1::input_drive::ConstPtr& msg);
    void userArmCallback(const rover1::input_arm::ConstPtr& msg);
};


Central_Control::Central_Control() {
  // Initialize publisher
  drive_pub = nh_.advertise<rover1::input_drive>("/drive_topic", 50);
  arm_pub = nh_.advertise<rover1::input_arm>("/arm_topic", 50);


  // Subscribers for user input from server controller
  user_drive_sub = nh_.subscribe("/user_drive_commands", 50,
      &Central_Control::userDriveCallback, this);

  user_arm_sub = nh_.subscribe("/user_arm_commands", 50,
      &Central_Control::userArmCallback, this);
}

void Central_Control::userDriveCallback(const rover1::input_drive::ConstPtr& msg) {
    drive_pub.publish(msg);
}

void Central_Control::userArmCallback(const rover1::input_arm::ConstPtr& msg) {
    arm_pub.publish(msg);
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

