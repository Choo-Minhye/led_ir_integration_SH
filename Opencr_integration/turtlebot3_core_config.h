
#ifndef TURTLEBOT3_CORE_CONFIG_H_
#define TURTLEBOT3_CORE_CONFIG_H_

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


#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <Barometer.h>
#include <MotorControl.h>
#include "infra_red.h"
#include "motors.h"
#include "barometer.h"
#include "ultra_sound.h"


#include <BatteryStatus.h>
#include <CAN.h>
#define BATTERY_ID 0x460
#define SEND_INTERVAL_MS 1000
#define REQUEST_FRAME 0x60
#define BATTERY_HZ 1


#include <math.h>

#define IR_READ_PIN A2
#define IR_LIGHT_PIN A3
#define US0_ANALOG_PIN A0
#define US1_ANALOG_PIN A1
#define LINEAR_ACTUATOR_L1 4
#define LINEAR_ACTUATOR_L2 7
#define LINEAR_ACTUATOR_R1 8
#define LINEAR_ACTUATOR_R2 9
#define CONVEYOR_PIN1 10
#define CONVEYOR_PIN2 11
#define ROLLSHUTTER_PIN1 5
#define ROLLSHUTTER_PIN2 6
const int buttonPin = A5; // for bumpercase
const int relayPin = A4; // 13; // for relaycase

#define SERIAL Serial2

#define DEG2RAD(x)                       (x * 0.01745329252)  // *PI/180
#define RAD2DEG(x)                       (x * 57.2957795131)  // *180/PI

#define TICK2RAD                         0.001533981  // 0.087890625[deg] * 3.14159265359 / 180 = 0.001533981f

#define TEST_DISTANCE                    0.300     // meter
#define TEST_RADIAN                      3.14      // 180 degree

#define DEBUG_SERIAL                     SerialBT2

// Callback function prototypes
void resetCallback(const std_msgs::Empty& reset_msg);

// Function prototypes
void publishImuMsg(void);


ros::Time rosNow(void);
ros::Time addMicros(ros::Time & t, uint32_t _micros); // deprecated

void updateVariable(bool isConnected);
void updateTime(void);
void updateGyroCali(bool isConnected);
void updateTFPrefix(bool isConnected);

void initOdom(void);
void initJointStates(void);

bool calcOdometry(double diff_time);

void sendLogMsg(void);
void waitForSerialLink(bool isConnected);



char data ;
String _data , _dockdata;
long range_time;
bool buttonState = LOW;
bool docking_info = 0;



unsigned long IMU_COUNT = 0;
unsigned long BARO_COUNT = 1;
unsigned long SONAR_1_COUNT = 1;
unsigned long BUTTON_COUNT = 1;


/*******************************************************************************
  ROS NodeHandle
*******************************************************************************/
ros::NodeHandle nh;
ros::Time current_time;
uint32_t current_offset;

/*******************************************************************************
  ROS Parameter
*******************************************************************************/
char get_prefix[10];
char* get_tf_prefix = get_prefix;

char odom_header_frame_id[30];
char odom_child_frame_id[30];

char imu_frame_id[30];

char joint_state_header_frame_id[30];

/*******************************************************************************
  Subscriber
*******************************************************************************/
/*----Motors----*/
MotorController mc;
void motorControlCB(const james_msgs::MotorControl &msg)
{
  mc.control(msg.motor, static_cast<Direction>(msg.direction), msg.pwm);
}
ros::Subscriber<james_msgs::MotorControl> sub_motors("motor_control", &motorControlCB);

/*----Bumper Relay----*/
void relayCB(const std_msgs::Bool& relay_cb) {
  if (relay_cb.data == true) {
    digitalWrite(relayPin, LOW);
  }
  else {
    digitalWrite(relayPin, HIGH);
  }
}
ros::Subscriber<std_msgs::Bool> sub_relay("relay", &relayCB);

/*----LED Strips----*/
int led_data;

void led_CB(const std_msgs::Int32& led_cb) {
  led_data = led_cb.data;
  SERIAL.println(led_data);
}
ros::Subscriber<std_msgs::Int32> send_led_data("led_change", &led_CB);


/*******************************************************************************
  Publisher
*******************************************************************************/

// IMU of Turtlebot3
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu", &imu_msg);

/*----baromter----*/
james_msgs::Barometer baro_msg;
ros::Publisher pub_baro("/barometer", &baro_msg);
Barometer baro;

/*----Ultrasound----*/
sensor_msgs::Range range_msg;
sensor_msgs::Range range_msg1;
UltraSound us0(US0_ANALOG_PIN), us1(US1_ANALOG_PIN);
ros::Publisher pub_range0( "/sonar0", &range_msg);
ros::Publisher pub_range1( "/sonar1", &range_msg1);


/*----IR----*/
bool IR_lightControl(bool val)
{
  digitalWrite(IR_LIGHT_PIN, val);
  return true;
}

bool IR_isOccupied()
{
  return !digitalRead(IR_READ_PIN);
}

void lightControl(const std_srvs::SetBool::Request& req, std_srvs::SetBool::Response &res) {
  IR_lightControl(req.data);
  res.success = true;
  if (req.data)
    res.message = "Light on";
  else
    res.message = "Light off";
}

void isOccupied(const std_srvs::Trigger::Request& req, std_srvs::Trigger::Response &res) {
  res.success = IR_isOccupied();
  if (res.success)
    res.message = "There is a package";
  else
    res.message = "There is not a package";
}

ros::ServiceServer<std_srvs::SetBool::Request, std_srvs::SetBool::Response> light_server("/ir_light", &lightControl);
ros::ServiceServer<std_srvs::Trigger::Request, std_srvs::Trigger::Response> ir_server("/ir", &isOccupied);

/*----Docking----*/
std_msgs::Int16 dockdata_T;
ros::Publisher pubdockdata_T("dockdata_T", &dockdata_T);

/*----Bumper----*/
std_msgs::Bool bumper_state;
ros::Publisher pub_bumper("bumper_state", &bumper_state);

/*----Battery----*/
james_msgs::BatteryStatus battery_msg;
can_message_t tx_msg, rx_msg;
ros::Publisher pub_bms("/bms_status", &battery_msg);
const unsigned char option0 = 0x1 ;
const unsigned char option1 = 0x2 ;
const unsigned char option2 = 0x4 ;
const unsigned char option3 = 0x8 ;
const unsigned char option4 = 0x10;
const unsigned char option5 = 0x20;
const unsigned char option6 = 0x40;
const unsigned char option7 = 0x80;

unsigned long BATTERY_COUNT = 0;



void sendStatusRequest() {
  tx_msg.id = BATTERY_ID;
  tx_msg.format = CAN_STD_FORMAT;
  tx_msg.data[0] = REQUEST_FRAME ;
  tx_msg.length = 1;
  CanBus.writeMessage(&tx_msg);
}
void parsingFrame() {
  if (CanBus.availableMessage())
  {
    if (CanBus.readMessage(&rx_msg))
    {
      if (rx_msg.data[1] == 0x01) {

        battery_msg.voltage = ((int)rx_msg.data[3] * 256 + (int)rx_msg.data[2]) * 0.01;
        float current = ((int)rx_msg.data[5] * 256 + (int)rx_msg.data[4]);
        if (current > 32767) {
          current = current - 65536;
          current = current * 0.01;
        }
        else
        {
          current = current * 0.01;
        }
        battery_msg.current = current;
        // status
        if ((int)rx_msg.data[7]&option0) {
          battery_msg.status_voltage = 1;
        }
        else if (rx_msg.data[7]&option1) {
          battery_msg.status_voltage = -1;
        }
        else if (rx_msg.data[6]&option0) {
          battery_msg.status_voltage = 2;
        }
        else if (rx_msg.data[6]&option1) {
          battery_msg.status_voltage = -2;
        }
        else {
          battery_msg.status_voltage = 0;
        }
        if (rx_msg.data[7]&option2) {
          battery_msg.status_current = 1;
        }
        else if (rx_msg.data[7]&option3) {
          battery_msg.status_current = -1;
        }
        else if (rx_msg.data[6]&option2) {
          battery_msg.status_current = 2;
        }
        else if (rx_msg.data[6]&option3) {
          battery_msg.status_current = -2;
        }
        else {
          battery_msg.status_current = 0;
        }
        if (rx_msg.data[7]&option4) {
          battery_msg.status_temperature = 1;
        }
        else if (rx_msg.data[7]&option5) {
          battery_msg.status_temperature = -1;
        }
        else if (rx_msg.data[6]&option4) {
          battery_msg.status_temperature = 2;
        }
        else if (rx_msg.data[6]&option5) {
          battery_msg.status_temperature = -2;
        }
        else {
          battery_msg.status_temperature = 0;
        }
        if (rx_msg.data[7]&option6) {
          battery_msg.status_BMS = 1;
        }
      }
      else if (rx_msg.data[1] == 0x02) {
        battery_msg.time_to_fill       = ((int)rx_msg.data[3] * 256 + (int)rx_msg.data[2]) * 1;
        battery_msg.time_to_empty      = ((int)rx_msg.data[5] * 256 + (int)rx_msg.data[4]) * 1;
        battery_msg.state_of_charge    = rx_msg.data[6];
        battery_msg.state_of_health    = rx_msg.data[7];
      }
      else if (rx_msg.data[1] == 0x03) {
        battery_msg.remaining_capacity = ((int)rx_msg.data[3] * 256 + (int)rx_msg.data[2]) * 0.01;
        battery_msg.available_energy   = ((int)rx_msg.data[5] * 256 + (int)rx_msg.data[4]) * 0.1;
        battery_msg.temprature         = ((int)rx_msg.data[7] * 256 + (int)rx_msg.data[6]) * 0.1;
      }
      battery_msg.stamp               = nh.now();
    }
  }
}





void pub_Baro(unsigned long BARO_FREQUENCY) {

  unsigned long Baro_CYCLE_T = 1000 / BARO_FREQUENCY;

  if (millis() >= Baro_CYCLE_T * BARO_COUNT) {
    baro_msg.header.frame_id = "/barometer";
    baro_msg.header.stamp = nh.now();

    baro_msg.temperature = baro.getTemperature();
    baro_msg.absolute_alt = baro.getAbsoluteAltitude();
    baro_msg.relative_alt = baro.getRelativeAltitude();
    pub_baro.publish(&baro_msg);

    BARO_COUNT++;
  }
}

void pub_Sonar(unsigned long SONAR_1_FREQUENCY) {

  unsigned long SONAR_1_CYCLE_T = 1000 / SONAR_1_FREQUENCY;

  if (millis() >= SONAR_1_CYCLE_T * SONAR_1_COUNT) {

    range_msg.range = us0.getDistance();
    range_msg.header.frame_id = "sonar0";
    range_msg.header.stamp = nh.now();
    pub_range0.publish(&range_msg);

    range_msg1.range = us1.getDistance();
    range_msg1.header.frame_id = "sonar1";
    range_msg1.header.stamp = nh.now();
    pub_range1.publish(&range_msg1);

    SONAR_1_COUNT++;
  }
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
      if (dockdata == 0) {
        docking_info = 0;
      }
      else {
        docking_info = 1;
      }
    }

  }

}


void pub_Bumper(unsigned long BUTTON_FREQUENCY) {

  unsigned long BUTTON_CYCLE_T = 1000 / BUTTON_FREQUENCY;

  if (millis() >= BUTTON_CYCLE_T * BUTTON_COUNT) {

    if (buttonState == LOW) {
      bumper_state.data = true;
      if (docking_info == 1) {
        digitalWrite(relayPin, LOW);
      }
    }
    else if (buttonState == HIGH) {
      bumper_state.data = false;
      digitalWrite(relayPin, HIGH);
    }
    pub_bumper.publish(&bumper_state);
    // when switch pressed, LOW
    // when switch non pressed, HIGH

    BUTTON_COUNT++;
  }
}

void pub_Battery(unsigned long BATTERY_FREQUENCY) {
  unsigned long BATTERY_CYCLE_T = 1000 / BATTERY_FREQUENCY; // 1Hz
  if (millis() >= BATTERY_CYCLE_T * BATTERY_COUNT)
  {
    sendStatusRequest();
    parsingFrame(); // 3 frames
    parsingFrame();
    parsingFrame();
    pub_bms.publish(&battery_msg);
    BATTERY_COUNT++;
  }
}

/*******************************************************************************
  Transform Broadcaster
*******************************************************************************/
// TF of Turtlebot3
geometry_msgs::TransformStamped odom_tf;
tf::TransformBroadcaster tf_broadcaster;

/*******************************************************************************
  SoftwareTimer of Turtlebot3
*******************************************************************************/
static uint32_t tTime[10];

/*******************************************************************************
  Declaration for sensors
*******************************************************************************/
Turtlebot3Sensor sensors;
unsigned long prev_update_time;


/*******************************************************************************
  Publish msgs (IMU data: angular velocity, linear acceleration, orientation)
*******************************************************************************/
void pub_IMU(const unsigned long IMU_FREQUENCY)
{
  unsigned long IMU_CYCLE_T = 1000 / IMU_FREQUENCY;

  if (millis() >= IMU_CYCLE_T * IMU_COUNT) {

    imu_msg = sensors.getIMU();

    imu_msg.header.stamp    = rosNow();
    imu_msg.header.frame_id = imu_frame_id;

    imu_pub.publish(&imu_msg);

    IMU_COUNT++;

  }
}


/*******************************************************************************
  Callback function for reset msg
*******************************************************************************/
void resetCallback(const std_msgs::Empty& reset_msg)
{
  char log_msg[50];

  (void)(reset_msg);

  sprintf(log_msg, "Start Calibration of Gyro");
  nh.loginfo(log_msg);

  sensors.calibrationGyro();

  sprintf(log_msg, "Calibration End");
  nh.loginfo(log_msg);

}

/*******************************************************************************
  Update TF Prefix
*******************************************************************************/
void updateTFPrefix(bool isConnected)
{
  static bool isChecked = false;
  char log_msg[50];

  if (isConnected)
  {
    if (isChecked == false)
    {
      nh.getParam("~tf_prefix", &get_tf_prefix);

      if (!strcmp(get_tf_prefix, ""))
      {
        sprintf(odom_header_frame_id, "odom");
        sprintf(odom_child_frame_id, "base_footprint");

        sprintf(imu_frame_id, "imu_link");
        sprintf(joint_state_header_frame_id, "base_link");
      }
      else
      {
        strcpy(odom_header_frame_id, get_tf_prefix);
        strcpy(odom_child_frame_id, get_tf_prefix);

        strcpy(imu_frame_id, get_tf_prefix);
        strcpy(joint_state_header_frame_id, get_tf_prefix);

        strcat(odom_header_frame_id, "/odom");
        strcat(odom_child_frame_id, "/base_footprint");

        strcat(imu_frame_id, "/imu_link");
        strcat(joint_state_header_frame_id, "/base_link");
      }

      sprintf(log_msg, "Setup TF on IMU [%s]", imu_frame_id);
      nh.loginfo(log_msg);

      sprintf(log_msg, "Setup TF on JointState [%s]", joint_state_header_frame_id);
      nh.loginfo(log_msg);

      isChecked = true;
    }
  }
  else
  {
    isChecked = false;
  }
}


/*******************************************************************************
  Update variable (initialization)
*******************************************************************************/
void updateVariable(bool isConnected)
{
  static bool variable_flag = false;

  if (isConnected)
  {
    if (variable_flag == false)
    {
      sensors.initIMU();
      variable_flag = true;
    }
  }
  else
  {
    variable_flag = false;
  }
}

/*******************************************************************************
  Wait for Serial Link
*******************************************************************************/
void waitForSerialLink(bool isConnected)
{
  static bool wait_flag = false;

  if (isConnected)
  {
    if (wait_flag == false)
    {
      delay(10);

      wait_flag = true;
    }
  }
  else
  {
    wait_flag = false;
  }
}

/*******************************************************************************
  Update the base time for interpolation
*******************************************************************************/
void updateTime()
{
  current_offset = millis();
  current_time = nh.now();
}

/*******************************************************************************
  ros::Time::now() implementation
*******************************************************************************/
ros::Time rosNow()
{
  return nh.now();
}
/*******************************************************************************
  Start Gyro Calibration
*******************************************************************************/
void updateGyroCali(bool isConnected)
{
  static bool isEnded = false;
  char log_msg[50];

  (void)(isConnected);

  if (nh.connected())
  {
    if (isEnded == false)
    {
      sprintf(log_msg, "Start Calibration of Gyro");
      nh.loginfo(log_msg);

      sensors.calibrationGyro();

      sprintf(log_msg, "Calibration End");
      nh.loginfo(log_msg);

      isEnded = true;
    }
  }
  else
  {
    isEnded = false;
  }
}



#endif // TURTLEBOT3_CORE_CONFIG_H_