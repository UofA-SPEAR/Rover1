#include <unistd.h>

#include "ros/ros.h"
#include <rover1/output_sensors.h>


class Sensor_Publisher {
 public:
    Sensor_Publisher();
    void updateGPS(double lon, double lat);
    void updateUV(double uv);
    void updateTemperature(double tmp);
    void updateHumidity(double hum);

 private:
    void publish();
    // Start Sensor data ////////////////
    // GPS
    double latitude;
    double longtitude;
    // UV
    double uv;
    // temperature probe
    double temperature;
    double humidity;
    // End Sensor data ////////////////
    ros::NodeHandle nh_;

    ros::Publisher sense_pub;
};


Sensor_Publisher::Sensor_Publisher() {
  // Initialize publisher
  sense_pub = nh_.advertise<rover1::output_sensors>("/sensor_out", 10);
}
void Sensor_Publisher::publish(){
    rover1::output_sensors msg;

    msg.latitude = this->latitude;
    msg.longtitude = this->longtitude;
    msg.uv = this->uv;
    msg.temperature = this->temperature;
    msg.humidity = this->humidity;

    sense_pub.publish(msg);
}

void Sensor_Publisher::updateGPS(double lon, double lat){ 
   this->latitude = lat; 
   this->longtitude = lon; 
   this->publish();
}

void Sensor_Publisher::updateUV(double uv){
   this->uv = uv; 
   this->publish();
}

void Sensor_Publisher::updateTemperature(double tmp){ 
    this->temperature = tmp;
    this->publish();
}

void Sensor_Publisher::updateHumidity(double hum){ 
    this->humidity = hum;
    this->publish();
}



int main(int argc, char **argv) {
  ros::init(argc, argv, "sensor_publisher");

  Sensor_Publisher sense_pub;

  usleep(1*1000000); // sleep for x seconds
  sense_pub.updateGPS(123,119);
  usleep(1*1000000); // sleep for x seconds
  sense_pub.updateUV(3);
  usleep(1*1000000); // sleep for x seconds
  sense_pub.updateTemperature(32);
  usleep(1*1000000); // sleep for x seconds
  sense_pub.updateHumidity(-2);

  ros::spin();
  return 0;
}

