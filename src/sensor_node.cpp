#include "ros/ros.h"
#include <unistd.h>
#include <rover1/output_sensors.h>


class Sensor_Publisher {
 public:
    Sensor_Publisher();
    void publish();

 private:
    double latitude;
    double longtitude;
    ros::NodeHandle nh_;

    ros::Publisher sense_pub;
};


Sensor_Publisher::Sensor_Publisher() {
  // Initialize publisher
  sense_pub = nh_.advertise<rover1::output_sensors>("/sensor_out", 10);
}
void Sensor_Publisher::publish(){
    rover1::output_sensors msg;
    msg.latitude = 5;
    msg.longtitude = 5;

    sense_pub.publish(msg);
}



int main(int argc, char **argv) {
  ros::init(argc, argv, "sensor_publisher");

  Sensor_Publisher sense_pub;

  while(1){
      usleep(1*1000000); // sleep for x seconds
      sense_pub.publish();
  }

  return 0;
}

