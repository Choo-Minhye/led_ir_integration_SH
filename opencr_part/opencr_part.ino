

#include <ros.h>
#include<std_msgs/String.h>
#include <sensor_msgs/Range.h>
#include <ros/time.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Byte.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Vector3.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

#include <turtlebot3_msgs/SensorState.h>
#include <TurtleBot3.h>

#define SERIAL Serial2

/*----Docking----*/
std_msgs::Int16 dockdata_T;
ros::Publisher pubdockdata_T("dockdata_T", &dockdata_T);
char data ;
String _data, _dockdata;
bool buttonState = LOW;
ros::NodeHandle nh;
ros::Time current_time;

int led_data;

void led_CB(const std_msgs::Int32& led_cb) {
  led_data = led_cb.data;
  SERIAL.println(led_data);
}
ros::Subscriber<std_msgs::Int32> send_led_data("led_change", &led_CB);



void setup()
{
  SERIAL.begin(115200);
  nh.initNode();
  nh.subscribe(send_led_data);
  nh.advertise(pubdockdata_T);
}

/*******************************************************************************
  Loop function
*******************************************************************************/
void loop()
{
  pub_dock();
  nh.spinOnce();
}



void pub_dock() {

  if (SERIAL.available() > 0 ) {
    unsigned int dockdata = 0;

    data = SERIAL.read();
    _data += data;

    if (data == '\n') {
      _dockdata = _data;
      dockdata = _dockdata.toInt();

      _data = "";
      _dockdata = "";

      dockdata_T.data = dockdata;
      pubdockdata_T.publish(&dockdata_T);

    }

  }

}
