#include <MS5611.h>
#include <ros.h>
#include <std_msgs/Int32.h>

MS5611 baro;
int32_t pressure;
ros::NodeHandle nh;

std_msgs::Int32 pressure_msg;
ros::Publisher pressure_data_pub("pressure_data", &pressure_msg);

void setup() {
  // Start barometer
  baro = MS5611();
  baro.begin();
  // Start serial (UART)
  Serial.begin(57600);
  nh.initNode();
  nh.advertise(pressure_data_pub);
}

void loop() {
  // Read pressure
  // pressure = baro.getPressure();
  // Send pressure via serial (UART);
  // Serial.println(pressure);
  pressure_msg.data = baro.getPressure();
  pressure_data_pub.publish( &pressure_msg );
  nh.spinOnce();
}
