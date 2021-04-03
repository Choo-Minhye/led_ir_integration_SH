#include "turtlebot3_core_config.h"

/*******************************************************************************
  Setup function
*******************************************************************************/

void setup()
{
  DEBUG_SERIAL.begin(115200);
  SERIAL.begin(115200);

  nh.initNode();
  nh.subscribe(sub_motors);
  nh.subscribe(sub_relay);
  nh.subscribe(send_led_data);

  nh.getHardware()->setBaud(115200);
  nh.advertise(imu_pub);
  nh.advertise(pub_baro);
  baro.setReference();

  mc.addMotor(new PairedBaseMotor(LINEAR_ACTUATOR_L1, LINEAR_ACTUATOR_L2, LINEAR_ACTUATOR_R1, LINEAR_ACTUATOR_R2));
  mc.addMotor(new BaseMotor(CONVEYOR_PIN1, CONVEYOR_PIN2));
  mc.addMotor(new BaseMotor(ROLLSHUTTER_PIN1, ROLLSHUTTER_PIN2));

  nh.advertiseService(light_server);
  nh.advertiseService(ir_server);
  nh.advertise(pubdockdata_T);
  nh.advertise(pub_bumper);
  nh.advertise(pub_bms);

  CanBus.begin(CAN_BAUD_500K, CAN_STD_FORMAT);
  CanBus.configFilter(BATTERY_ID, 0, CAN_STD_FORMAT);


  tf_broadcaster.init(nh);

  // Setting for IMU
  sensors.init();
  prev_update_time = millis();

  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(relayPin, OUTPUT);
  pinMode(IR_READ_PIN, INPUT);
  pinMode(IR_LIGHT_PIN, OUTPUT);

  digitalWrite(buttonPin, HIGH); // for setting
  digitalWrite(relayPin, HIGH);

  pinMode(LED_WORKING_CHECK, OUTPUT);

}

/*******************************************************************************
  Loop function
*******************************************************************************/
void loop()
{
  buttonState = digitalRead(buttonPin);
  uint32_t t = millis();
  updateTime();
  updateVariable(nh.connected());
  updateTFPrefix(nh.connected());

  pub_IMU(100);
  pub_Baro(14);
  pub_Bumper(100);
  pub_dock();
  //  pub_Battery(1);

  // Update the IMU unit
  sensors.updateIMU();
  // Start Gyro Calibration after ROS connection
  updateGyroCali(nh.connected());


  nh.spinOnce();
  waitForSerialLink(nh.connected());
}
