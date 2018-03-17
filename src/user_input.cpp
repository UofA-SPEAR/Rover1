#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include <rover1/controller.h>
#include <iostream>
#include <math.h>
#include <unistd.h>

#define Pi acos(-1.0)
#define PS3_AXIS_STICK_LEFT_UPWARDS 1
#define PS3_AXIS_STICK_RIGHT_UPWARDS 3

class User_Input {
 public:
    User_Input();

 private:
    // indices of the x,y,(analog sticks), and the Green Triangle button
    // from the Joy's axes array
    int x_idx_, y_idx_, tri_idx_;
    double x_scale_, y_scale_;
    // State of the controller - 0 for drive, 1 for arm
    uint8_t state_;

    ros::NodeHandle nh_;
    ros::Publisher controller_cmd_pub;
    ros::Subscriber ps3_sub;

    void ps3Callback(const sensor_msgs::Joy::ConstPtr& ps3_msg);
};


User_Input::User_Input():
  x_idx_(2),
  y_idx_(1),
  tri_idx_(12),
  x_scale_(1),
  y_scale_(1) {
  // Initialize publ
    isher
  controller_cmd_pub = nh_.advertise<rover1::controller>
    ("/user_commands", 10);

  // Initialize subscriber
  ps3_sub = nh_.subscribe("joy", 10, &User_Input::ps3Callback, this);
}


void User_Input::ps3Callback(const sensor_msgs::Joy::ConstPtr& ps3_msg) {
  rover1::controller msg;
  size_t size_axis;
  size_t size_button;

  if (ps3_msg->buttons[tri_idx_]) {
    state_ = state_ ^ 1;
    //TODO: Experiment with this
    usleep(50000); 
  }

  size_axis = sizeof(ps3_msg->axes);
  size_button = sizeof(ps3_msg->buttons);
  msg.left_stick = x_scale_*ps3_msg->axes[PS3_AXIS_STICK_LEFT_UPWARDS];
  msg.right_stick = y_scale_*ps3_msg->axes[PS3_AXIS_STICK_RIGHT_UPWARDS];
  msg.state = state_;

  ROS_INFO("MSG STATE %i", msg.state);

  // Publish the ps3 controller's velocity parameters
  controller_cmd_pub.publish(msg);
}


int main(int argc, char** argv) {
  // Initializing ROS node
  ros::init(argc, argv, "user_input");

  // Initialize User_Input Node object
  User_Input user_input;

  ros::spin();
  return 0;
}
