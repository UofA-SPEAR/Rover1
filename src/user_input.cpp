#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include <rover1/controller.h>
#include <iostream>
#include <math.h>
#include <unistd.h>

#define Pi acos(-1.0)

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
  x_scale_(-1),
  y_scale_(1){
  /* nh_.param("axis_linear", linear_, linear_); */

  // Initialize publisher
  controller_cmd_pub = nh_.advertise<rover1::controller>
    ("/user_commands", 10);

  // Initialize subscriber
  ps3_sub = nh_.subscribe("joy", 10, &User_Input::ps3Callback, this);
}


void User_Input::ps3Callback(const sensor_msgs::Joy::ConstPtr& ps3_msg) {
  rover1::controller msg;

  if(ps3_msg->axes[tri_idx_]){
    state_ = state_ ^ 1; //XOR
    // Delay to keep ROS from reading the button multiple times when pressed
    // Number probs need to be change / finalized
    usleep(600); 
  }

  msg.x_coord = x_scale_*ps3_msg->axes[x_idx_];
  msg.y_coord = y_scale_*ps3_msg->axes[y_idx_];
  
  msg.state = state_;

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
