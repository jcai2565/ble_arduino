#include "BLECStringCharacteristic.h"
#include "EString.h"
#include "RobotCommand.h"
#include <ArduinoBLE.h>
#include <math.h>
#include <Wire.h>
#include "cmd_types.h"
#include "pins.h"
#include "ble.h"
#include "motors.hpp"
#include "utils.hpp"
#include "led.hpp"
#include "pid.hpp"
#include "imu.hpp"
#include "config.hpp"
#include "tof.hpp"
#include "kalman.hpp"

//////////// Global Variables ////////////
BLEService testService(BLE_UUID_TEST_SERVICE);
BLECStringCharacteristic rx_characteristic_string(BLE_UUID_RX_STRING, BLEWrite, MAX_MSG_SIZE);
BLEFloatCharacteristic tx_characteristic_float(BLE_UUID_TX_FLOAT, BLERead | BLENotify);
BLECStringCharacteristic tx_characteristic_string(BLE_UUID_TX_STRING, BLERead | BLENotify, MAX_MSG_SIZE);

// RX
RobotCommand robot_cmd(":|");

// TX
EString tx_estring_value;
float tx_float_value = 0.0;

long interval = 500;
static long previousMillis = 0;
unsigned long currentMillis = 0;

//PID speed test;
bool doSpeedTest = false;
unsigned long testStartTime;
int controlLoopCounter = 0;
int measurementCounter = 0;

//Kalman filter step response flag
bool doOpenLoop = false;
unsigned long openLoopStartTime;
int openLoopIndex = 0;

//KalmanFilter pos_kf(dt, mass, dist, sigma_meas, sigma_proc_1, sigma_proc_2);
// sigma_proc = sqrt(20^2 * 10) = 63.24
// sigma_proc = sqrt(10^2 * 10) = 31.62
// in mm units
KalmanFilter pos_kf(0.00856, 0.000258, 0.000339, 20, 31.62, 31.62);
// KalmanFilter pos_kf(0.00856, 0.000258, 0.000339, 20, 63.24, 63.24);

//Lab 8 stunts
int stunt_tof_index = 0;
int stunt_kf_index = 0;
unsigned long stunt_start_time;

//////////// Global Variables ////////////

void handle_command()
{
  // Set the command string from the characteristic value
  robot_cmd.set_cmd_string(rx_characteristic_string.value(),
                           rx_characteristic_string.valueLength());

  bool success;
  int cmd_type = -1;

  // Get robot command type (an integer)
  /* NOTE: THIS SHOULD ALWAYS BE CALLED BEFORE get_next_value()
   * since it uses strtok internally (refer RobotCommand.h and
   * https://www.cplusplus.com/reference/cstring/strtok/)
   */
  success = robot_cmd.get_command_type(cmd_type);

  // Check if the last tokenization was successful and return if failed
  if (!success)
  {
    return;
  }

  // Handle the command type accordingly
  switch (cmd_type)
  {

  case SET_POS_GAINS:
  {
    float kp_val, ki_val, kd_val;
    bool success_kp = robot_cmd.get_next_value(kp_val);
    bool success_ki = robot_cmd.get_next_value(ki_val);
    bool success_kd = robot_cmd.get_next_value(kd_val);

    if (success_kp && success_ki && success_kd)
    {
      pos_pid.setGains(kp_val, ki_val, kd_val);
      Serial.println("Position PID gains set successfully.");
    }
    else
    {
      Serial.println("Failed to set Position PID gains. Ensure all values are provided.");
    }
    break;
  }

  case SET_POS_SETPOINT:
  {
    float set;
    bool success_set = robot_cmd.get_next_value(set);

    if (success_set)
    {
      pos_pid.setSetpoint(set);
      Serial.println("Position setpoint set successfully.");
    }
    else
    {
      Serial.println("Failed to set Position Setpoint");
    }
    break;
  }

  case START_POS_PID:
  {
    pos_pid.do_pid = true;
    pos_pid.reset();

    float d1 = getTof1IfReady();
    while (d1 == -1.0){
      delay(5);
      Serial.println("START_POS_PID: WAITING FOR FIRST TOF DATA");
      d1 = getTof1IfReady();
    }
    pos_kf.initialize(d1);
    break;
  }
  case STOP_POS_PID:
  {
    pos_pid.do_pid = false;
    stop();
    break;
  }

case SET_ANGLE_GAINS:
  {
    float kp_val, ki_val, kd_val;
    bool success_kp = robot_cmd.get_next_value(kp_val);
    bool success_ki = robot_cmd.get_next_value(ki_val);
    bool success_kd = robot_cmd.get_next_value(kd_val);

    if (success_kp && success_ki && success_kd)
    {
      angle_pid.setGains(kp_val, ki_val, kd_val);
      Serial.println("Angle PID gains set successfully.");
    }
    else
    {
      Serial.println("Failed to set Angle PID gains. Ensure all values are provided.");
    }
    break;
  }

  case SET_ANGLE_SETPOINT:
  {
    float set;
    bool success_set = robot_cmd.get_next_value(set);

    if (success_set)
    {
      angle_pid.setSetpoint(set);
      Serial.println("Angle setpoint set successfully.");
    }
    else
    {
      Serial.println("Failed to set Angle Setpoint");
    }
    break;
  }

  case START_ANGLE_PID:
  {
    angle_pid.do_pid = true;
    angle_pid.reset();
    break;
  }
  case STOP_ANGLE_PID:
  {
    angle_pid.do_pid = false;
    stop();
    break;
  }

  case SEND_TOF_DATA:
  {
    //Sends all data from TOF and Control during that time. Assumes that no other data was collected.
    for (int i=0; i<array_size; i++){
      tx_estring_value.clear();
      tx_estring_value.append("T:");
      tx_estring_value.append(pos_pid.time_array[i]);
      tx_estring_value.append("|");
      tx_estring_value.append("D:");
      tx_estring_value.append(pos_pid.meas_array[i]);
      tx_estring_value.append("|");
      tx_estring_value.append("Set:");
      tx_estring_value.append(pos_pid.setpoint_array[i]);
      tx_characteristic_string.writeValue(tx_estring_value.c_str());
    }
    break;
  }
  case SEND_POS_PID_CONTROL_DATA:
  {
    for (int i=0; i<pid_array_size; i++){
      tx_estring_value.clear();
      tx_estring_value.append("T:");
      tx_estring_value.append(pos_pid.pid_timestamp_array[i]);
      tx_estring_value.append("|");
      tx_estring_value.append("P:");
      tx_estring_value.append(pos_pid.p_array[i]);
      tx_estring_value.append("|");
      tx_estring_value.append("I:");
      tx_estring_value.append(pos_pid.i_array[i]);
      tx_estring_value.append("|");
      tx_estring_value.append("D:");
      tx_estring_value.append(pos_pid.d_array[i]);
      tx_estring_value.append("|");
      tx_estring_value.append("KF:");
      tx_estring_value.append(pos_kf.position_array[i]);
      tx_characteristic_string.writeValue(tx_estring_value.c_str());
    }
    break;
  }
  case SEND_IMU_DATA:
  {
    for (int i=0; i<array_size; i++){
      tx_estring_value.clear();
      tx_estring_value.append("T:");
      tx_estring_value.append(angle_pid.time_array[i]);
      tx_estring_value.append("|");
      tx_estring_value.append("A:");
      tx_estring_value.append(angle_pid.meas_array[i]);
      tx_estring_value.append("|");
      tx_estring_value.append("Set:");
      tx_estring_value.append(angle_pid.setpoint_array[i]);
      tx_characteristic_string.writeValue(tx_estring_value.c_str());
    }
    break;
  }
  case SEND_ANGLE_PID_CONTROL_DATA:
  {
    for (int i=0; i<pid_array_size; i++){
      tx_estring_value.clear();
      tx_estring_value.append("T:");
      tx_estring_value.append(angle_pid.pid_timestamp_array[i]);
      tx_estring_value.append("|");
      tx_estring_value.append("P:");
      tx_estring_value.append(angle_pid.p_array[i]);
      tx_estring_value.append("|");
      tx_estring_value.append("I:");
      tx_estring_value.append(angle_pid.i_array[i]);
      tx_estring_value.append("|");
      tx_estring_value.append("D:");
      tx_estring_value.append(angle_pid.d_array[i]);
      tx_characteristic_string.writeValue(tx_estring_value.c_str());
    }
    break;
  }
  case PID_SPEED_TEST:
  {
    doSpeedTest = true;
    controlLoopCounter = 0;
    measurementCounter = 0;
    testStartTime = millis();
    break;
  }
  case PING_SPEED_TEST:
  {
    tx_estring_value.clear();
    tx_estring_value.append("Main:");
    tx_estring_value.append(controlLoopCounter);
    tx_estring_value.append("|");
    tx_estring_value.append("Measurement:");
    tx_estring_value.append(measurementCounter);
    tx_characteristic_string.writeValue(tx_estring_value.c_str());
    break;
  }
  case START_OPEN_LOOP:
  {
    openLoopStartTime = millis();
    doOpenLoop = true;
    openLoopIndex = 0;
    for (int i = 0; i < array_size; i++){
      distance1_array[i] = 0;
      timestamp_array[i] = 0;
    }

    break;
  }
  case FETCH_D1:
  {
    for (int i = 0; i < array_size; i++){
      tx_estring_value.clear();
      tx_estring_value.append("T:");
      tx_estring_value.append(timestamp_array[i]);
      tx_estring_value.append("|");
      tx_estring_value.append("P:");
      tx_estring_value.append(distance1_array[i]);
      tx_characteristic_string.writeValue(tx_estring_value.c_str());
    }
    break;
  }
  case START_STUNT:
  {
    stunt_tof_index = 0;
    stunt_kf_index = 0;
    
    //TOF and slower timestamp array
    for (int i = 0; i < array_size; i++){
      timestamp_array[i] = 0;
      distance1_array[i] = 0;
    }

    //KF timestamp array
    for (int i = 0; i < pid_array_size; i++){
      big_timestamp_array[i] = 0;
    }

    // Initialize with first measurement
    float d1 = getTof1IfReady();
    while (d1 == -1.0){
      delay(5);
      Serial.println("START_STUNT: WAITING FOR FIRST TOF DATA");
      d1 = getTof1IfReady();
    }
    pos_kf.initialize(d1);
    
    stunt_start_time = millis();
    timestamp_array[stunt_tof_index] = stunt_start_time;
    distance1_array[stunt_tof_index] = d1;
    stunt_tof_index++;

    // Call stunt function.
    stuntOpenLoop();

    // Just to make sure we stop after stunt is over.
    stop();
    break;
  }
  case FETCH_STUNT_KF:
  {
    for (int i = 0; i < pid_array_size; i++){
      tx_estring_value.clear();
      tx_estring_value.append("T:");
      tx_estring_value.append(big_timestamp_array[i]);
      tx_estring_value.append("|");
      tx_estring_value.append("P:");
      tx_estring_value.append(pos_kf.position_array[i]);
      tx_characteristic_string.writeValue(tx_estring_value.c_str());
    }
    break;
  }
  default:
  {
    Serial.print("Invalid Command Type: ");
    Serial.println(cmd_type);
    break;
  }
  }
}

void write_data()
{
  currentMillis = millis();
  if (currentMillis - previousMillis > interval)
  {

    tx_float_value = tx_float_value + 0.5;
    tx_characteristic_float.writeValue(tx_float_value);

    if (tx_float_value > 10000)
    {
      tx_float_value = 0;
    }

    previousMillis = currentMillis;
  }
}

void read_data()
{
  // Query if the characteristic value has been written by another BLE device
  if (rx_characteristic_string.written())
  {
    handle_command();
  }
}

void setup()
{
  Wire.begin();
  Serial.begin(115200);
  ledSetup();
  motorSetup();
  tofSetup();
  imuSetup();

  // myICM.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial
  BLE.begin();

  // Set advertised local name and service
  BLE.setDeviceName("Artemis BLE");
  BLE.setLocalName("Artemis BLE");
  BLE.setAdvertisedService(testService);

  // Add BLE characteristics
  testService.addCharacteristic(tx_characteristic_float);
  testService.addCharacteristic(tx_characteristic_string);
  testService.addCharacteristic(rx_characteristic_string);

  // Add BLE service
  BLE.addService(testService);

  // Initial values for characteristics
  // Set initial values to prevent errors when reading for the first time on central devices
  tx_characteristic_float.writeValue(0.0);

  // Output MAC Address
  Serial.print("Advertising BLE with MAC: ");
  Serial.println(BLE.address());

  BLE.advertise();
  gyr_last = micros();
}

void debugPrint()
{
  if (DO_DEBUG){
    //print some stuff
  }
}

/*
* Called during main loop to execute open loop for lab 7. CANNOT be called concurrently with angle PID.
*/
void handleOpenLoop()
{
  unsigned long current = millis();
  unsigned long diff = current - openLoopStartTime;
  if (diff > 4000) {
    stop();
    return;
  } 
  else {
    if (diff > 2000) {
      stop();
    }
    else {
      drive(FORWARD, 150);
    }
    
    float d1 = getTof1IfReady();
    if (d1 != -1.0) {
      timestamp_array[openLoopIndex] = current;
      distance1_array[openLoopIndex] = d1;
      openLoopIndex++;
    }
  }

  return;
}

/*
* Called during main loop to execute positional PID. CAN be called concurrently with angle PID.
*/
void posPidControlLoop()
{
  if (doSpeedTest)
  {
    controlLoopCounter++;
    float d1 = getTof1IfReady();
    if (d1 != -1.0) measurementCounter++;

    if (millis() - testStartTime > 2000)
    {
      doSpeedTest = false;
    }
    return;
  }

  if (!pos_pid.do_pid) // Using pos_pid instead of global do_pid
  {
    if (!angle_pid.do_pid){
      stop();
    }
    return;
  }

  if (millis() - pos_pid.pid_start_time > 8000)
  {
    pos_pid.do_pid = false;
  }

  // TOF returns -1.0 if NOT ready
  float d1 = getTof1IfReady();
  Serial.println(d1);

  int pwm;

  // TOF data is ready: record fresh data
  if (d1 != -1.0)
  {
    pos_kf.update(d1); //update Kalman filter with new data value

    if (pos_pid.pid_index >= array_size) // Prevent array overflow
    {
      Serial.println("ARRAY FULL! Cannot write to array. Continuing PID control.");
    }
    else
    {
      pwm = pos_pid.compute(pos_kf.getPosition()); // Get position according to KF

      pos_pid.time_array[pos_pid.pid_index] = millis();
      pos_pid.meas_array[pos_pid.pid_index] = d1;
      pos_pid.pwm_history[pos_pid.pid_index] = pwm;
      pos_pid.setpoint_array[pos_pid.pid_index] = pos_pid.setpoint;
      pos_pid.pid_index++;
    }
  }
  else //no data from TOF
  {
    pwm = (pos_pid.pid_index == 0) ? 0 : pos_pid.compute(pos_kf.getPosition());
  }

  // Has a lower bound of DEADBAND. Also calls update to KF
  executePosPid(pwm); 
  return;
}

/*
* Called during main loop to execute angle PID. CAN be called concurrently with position PID.
*/
void anglePidControlLoop()
{
  if (doSpeedTest)
  {
    controlLoopCounter++;
    float yaw = getDmpYaw();
    if (isValidYaw(yaw)) measurementCounter++;

    if (millis() - testStartTime > 2000)
    {
      doSpeedTest = false;
    }
    return;
  }

  if (!angle_pid.do_pid)
  {
    if (!pos_pid.do_pid){
      stop();
    }
    return;
  }

  if (millis() - angle_pid.pid_start_time > 10000)
  {
    angle_pid.do_pid = false;
  }

  float angle = getDmpYaw();
  if (isValidYaw(angle)){
    Serial.print("PID Control Loop: Yaw is: ");
    Serial.println(angle);
  }
  int pwm;

  if (isValidYaw(angle))
  {
    if (angle_pid.pid_index >= array_size) // Prevent array overflow
    {
      Serial.println("ARRAY FULL! Cannot write to array. Continuing PID control.");
    }
    else
    {
      angle_pid.time_array[angle_pid.pid_index] = millis();
      angle_pid.meas_array[angle_pid.pid_index] = angle;
      pwm = angle_pid.compute(angle); 
      angle_pid.pwm_history[angle_pid.pid_index] = pwm;
      angle_pid.setpoint_array[angle_pid.pid_index] = angle_pid.setpoint;
      angle_pid.pid_index++;
    }
  }
  else
  {
    pwm = angle_pid.pid_index == 0 ? 0 : angle_pid.compute(angle_pid.meas_array[angle_pid.pid_index - 1]);
  }

  executeAnglePid(pwm); 
  Serial.print("Executing spin: ");
  Serial.println(pwm);
  return;
}

void loop()
{
  ledCheckAndSet(); // LED blink loop component
  
  // Listen for connections
  BLEDevice central = BLE.central();
  // handleOpenLoop();
  posPidControlLoop();
  anglePidControlLoop();
  
  debugPrint();

  // If a central is connected to the peripheral
  if (central)
  {
    Serial.print("Connected to: ");
    Serial.println(central.address());

    // While central is connected
    while (central.connected())
    {
      ledCheckAndSet(); // LED blink loop component

      // Send data
      write_data();

      // Read data
      read_data();
      
      // handleOpenLoop();
      posPidControlLoop();
      anglePidControlLoop();

      debugPrint();
    }

    // Exiting: stop motors
    stop();
    Serial.println("Disconnected");
  }
}

  
