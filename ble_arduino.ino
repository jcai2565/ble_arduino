#include "BLECStringCharacteristic.h"
#include "EString.h"
#include "RobotCommand.h"
#include <ArduinoBLE.h>
//lab2
// #include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU
#include <math.h>
//lab3 TOF
#include <Wire.h>
#include "SparkFun_VL53L1X.h" //Click here to get the library: http://librarymanager/All#SparkFun_VL53L1X

#include "cmd_types.h"
#include "pins.h"
#include "ble.h"
#include "motors.hpp"
#include "utils.hpp"
#include "led.hpp"
#include "pid.hpp"
#include "imu.hpp"
#include "config.hpp"


// #ifdef USE_SPI
//   ICM_20948_SPI myICM; // If using SPI create an ICM_20948_SPI object
// #else
//   ICM_20948_I2C myICM; // Otherwise create an ICM_20948_I2C object
// #endif

//TOF
SFEVL53L1X distanceSensor1;
SFEVL53L1X distanceSensor2(Wire, XSHUT);

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

//CONFIGS
const float alpha = 0.284;
const float alpha_complementary = 0.1;
float Kp = 0;
float Ki = 0;
float Kd = 0;

// // IMU
// int timestamp_array[array_size];
// float temperature_array[array_size];
// float acc_roll_array[array_size];
// float acc_pitch_array[array_size];

// float gyr_roll_array[array_size];
// float gyr_pitch_array[array_size];
// float gyr_yaw_array[array_size];

// float acc_roll_array_lowpass[array_size];
// float acc_pitch_array_lowpass[array_size];

// float complementary_roll_array[array_size];
// float complementary_pitch_array[array_size];

// TOF
float distance1_array[array_size];
float distance2_array[array_size];


// // Low pass variables
// float lowpass_pitch[2];
// float lowpass_roll[2];

// float gyr_pitch = 0 , gyr_roll = 0, gyr_yaw = 0, dt = 0;
// float complementary_pitch = 0, complementary_roll = 0;
// unsigned long gyr_last;
int write_ind = 0;
bool write_flag = false;

//////////// Global Variables ////////////

void
handle_command()
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
    if (!success) {
        return;
    }

    // Handle the command type accordingly
    switch (cmd_type) {
        case GET_IMU_DATA:
        {
          float gyr_pitch = 0 , gyr_roll = 0, gyr_yaw = 0, dt = 0;
          float complementary_pitch = 0, complementary_roll = 0;
          gyr_last = micros();
          
          for (int i = 0; i < array_size; i++)
          {
            if (myICM.dataReady())
            {
              myICM.getAGMT();
              float theta = atan2((&myICM)->accX(), (&myICM)->accZ())  * (180.0 / M_PI);
              float phi = atan2((&myICM)->accY(), (&myICM)->accZ()) * (180.0 / M_PI);
              acc_pitch_array[i] = theta;
              acc_roll_array[i] = phi;
              acc_pitch_array_lowpass[i] = lowPassPitch(theta);
              acc_roll_array_lowpass[i] = lowPassRoll(phi);
              timestamp_array[i] = (int) millis();
              
              dt = ( micros() - gyr_last) / 1000000.;
              gyr_last = micros();
              gyr_pitch = gyr_pitch - myICM.gyrY()*dt; //negate gyro reading according to sign conv. defined on IMU
              gyr_roll = gyr_roll + myICM.gyrX()*dt;
              gyr_yaw = gyr_yaw + myICM.gyrZ()*dt;
              gyr_roll_array[i] = gyr_roll;
              gyr_pitch_array[i] = gyr_pitch; 
              gyr_yaw_array[i] = gyr_yaw;

              complementary_pitch = (complementary_pitch - myICM.gyrY()*dt)*(1-alpha_complementary) + theta * alpha_complementary; //also negate gyro
              complementary_roll = (complementary_roll + myICM.gyrX()*dt)*(1-alpha_complementary) + phi * alpha_complementary; 

              complementary_pitch_array[i] = complementary_pitch;
              complementary_roll_array[i] = complementary_roll;
            }
          }

          //Return data as "T:{}|accP:{}|accPlp:{}|accR:{}|accRlp:{}|gyrP:{}|gyrR:{}|gyrY:{}|compP:{}|compR:{}"
          for (int j = 0; j < array_size; j++) {
            send_all_data(j);
          }
          break;
        }
        case WRITE_IMU_DATA:
        {
          write_flag = true;
          if (write_ind == array_size - 1){
            write_ind = 0;
          }
          break;
        }
        case SEND_IMU_DATA:
        {
          for (int j = 0; j < array_size; j++){
            send_all_data(j);
          }
          break;
        }
        case GET_TOF_DATA:
        {
          distanceSensor1.setDistanceModeShort();
          for (int i = 0; i < array_size; i++){

            distanceSensor1.startRanging();  //Write config bytes

            while (!distanceSensor1.checkForDataReady()){
              delay(1); //dubious
            }
            
            distance1_array[i] = distanceSensor1.getDistance();
            distanceSensor1.clearInterrupt();
            distanceSensor1.stopRanging();

            timestamp_array[i] = millis();
          }

          for (int j = 0; j < array_size; j++){
            tx_estring_value.clear();
            tx_estring_value.append("T:");
            tx_estring_value.append(timestamp_array[j]);
            tx_estring_value.append("|");
            tx_estring_value.append("D:");
            tx_estring_value.append(distance1_array[j]);
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
          }

          break;
        }
        case GET_SENSOR_DATA:
        {
          // Serial.println("sensordata start ");
          float complementary_pitch = 0, complementary_roll = 0; gyr_yaw = 0, dt = 0;
          gyr_last = micros();
          distanceSensor1.setDistanceModeShort();
          distanceSensor2.setDistanceModeShort();
          for (int i = 0; i < array_size; i++){

            distanceSensor1.startRanging();  //Write config bytes
            distanceSensor2.startRanging();  //Write config bytes

            while (!distanceSensor1.checkForDataReady()){
              delay(1); //dubious
            }
            while (!distanceSensor2.checkForDataReady()){
              delay(1); 
            }
            distance1_array[i] = distanceSensor1.getDistance();
            distanceSensor1.clearInterrupt();
            distanceSensor1.stopRanging();

            distance2_array[i] = distanceSensor2.getDistance();
            distanceSensor2.clearInterrupt();
            distanceSensor2.stopRanging();

            // Serial.print("dist1:");
            // Serial.println(distance1_array[i]);
            // Serial.print("dist2:");
            // Serial.println(distance2_array[i]);

            if (myICM.dataReady()) 
            {
              myICM.getAGMT();
              float theta = atan2((&myICM)->accX(), (&myICM)->accZ())  * (180.0 / M_PI);
              float phi = atan2((&myICM)->accY(), (&myICM)->accZ()) * (180.0 / M_PI);
              dt = ( micros() - gyr_last) / 1000000.;
              gyr_last = micros();
              gyr_yaw = gyr_yaw + myICM.gyrZ()*dt;
              
              complementary_pitch = (complementary_pitch - myICM.gyrY()*dt)*(1-alpha_complementary) + theta * alpha_complementary; //also negate gyro
              complementary_roll = (complementary_roll + myICM.gyrX()*dt)*(1-alpha_complementary) + phi * alpha_complementary; 

              complementary_roll_array[i] = complementary_roll;
              complementary_pitch_array[i] = complementary_pitch;
              gyr_yaw_array[i] = gyr_yaw;

              // Serial.print("pitch:");
              // Serial.println(complementary_pitch_array[i]);
            }
            timestamp_array[i] = (int) millis();
            // Serial.print("time: ");
            // Serial.println(timestamp_array[i]);
          }

          for (int j = 0; j < array_size; j++){
            tx_estring_value.clear();
            tx_estring_value.append("T:");
            tx_estring_value.append(timestamp_array[j]);
            tx_estring_value.append("|");
            tx_estring_value.append("D1:");
            tx_estring_value.append(distance1_array[j]);
            tx_estring_value.append("|");
            tx_estring_value.append("D2:");
            tx_estring_value.append(distance2_array[j]);
            tx_estring_value.append("|");
            tx_estring_value.append("Roll:");
            tx_estring_value.append(complementary_roll_array[j]);
            tx_estring_value.append("|");
            tx_estring_value.append("Pitch:");
            tx_estring_value.append(complementary_pitch_array[j]);
            tx_estring_value.append("|");
            tx_estring_value.append("Yaw:");
            tx_estring_value.append(gyr_yaw_array[j]);
            tx_characteristic_string.writeValue(tx_estring_value.c_str());
          }

          break;
        }
        case SET_P: 
        {
          float val;
          success = robot_cmd.get_next_value(val);
          if (success)
          {
            Kp = val;
          } else 
          {
            Serial.println("Setting Kp failed, KP NOT SET");
          }
          
          break;
        }
        case SET_I:
        {
          float val;
          success = robot_cmd.get_next_value(val);
          if (success)
          {
            Ki = val;
          } else 
          {
            Serial.println("Setting Kp failed, KP NOT SET");
          }
          
          break;
        }
        case SET_D:
        {
          float val;
          success = robot_cmd.get_next_value(val);
          if (success)
          {
            Kd = val;
          } else 
          {
            Serial.println("Setting Kp failed, KP NOT SET");
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

void
write_data()
{
    currentMillis = millis();
    if (currentMillis - previousMillis > interval) {

        tx_float_value = tx_float_value + 0.5;
        tx_characteristic_float.writeValue(tx_float_value);

        if (tx_float_value > 10000) {
            tx_float_value = 0;
            
        }

        previousMillis = currentMillis;
    }
}

void
read_data()
{
    // Query if the characteristic value has been written by another BLE device
    if (rx_characteristic_string.written()) {
        handle_command();
    }
}


void
setup()
{
  Wire.begin();
  Serial.begin(115200);
  
  
  ledSetup();
  motorSetup();

  // TOF
  pinMode(XSHUT, OUTPUT); //write, to control XSHUT
  distanceSensor1.init();
  
  // Configure TOF1, disable TOF2
  digitalWrite(XSHUT, LOW);
  distanceSensor1.setI2CAddress(ALT_I2C); // set TOF 1 (the one without XSHUT) to ALT_I2C
  // Serial.println(distanceSensor1.getI2CAddress(), HEX);
  // Serial.println(distanceSensor2.getI2CAddress(), HEX);
  delay(10); // Allow TOF1 to boot

  int status; 
  status = distanceSensor1.begin();
  if (status != 0)
  {
    Serial.println("Distance sensor 1 failed to begin. Please check wiring. Freezing...");
    while(1);
  }

  // Reset XSHUT so that TOF2 is active
  digitalWrite(XSHUT, HIGH);
  delay(10); // Allow TOF2 to boot
  distanceSensor2.init();

  status = distanceSensor2.begin();
  if (status != 0)
  {
    Serial.println("Distance sensor 2 failed to begin. Please check wiring. Trying again...");
    while(1);
  }

  Serial.println("Both TOF sensors initialized successfully!");

  #ifdef USE_SPI
    SPI_PORT.begin();
  #else
    WIRE_PORT.begin();
    WIRE_PORT.setClock(400000);
  #endif

  //myICM.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial

  // SENSOR INITIALIZATION CHECKS
  bool initialized = false;
  while (!initialized)
  {
    #ifdef USE_SPI
        myICM.begin(CS_PIN, SPI_PORT);
    #else
        myICM.begin(WIRE_PORT, AD0_VAL);
    #endif

    SERIAL_PORT.print(F("Initialization of the sensor returned: "));
    SERIAL_PORT.println(myICM.statusString());
    if (myICM.status != ICM_20948_Stat_Ok)
    {
      SERIAL_PORT.println("Trying again...");
      delay(500);
    } 
    else
    {
      initialized = true;
    }
  }

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

void
loop()
{   
    ledCheckAndSet(); //LED blink loop component
    motorOpenLoop();
    // Listen for connections
    BLEDevice central = BLE.central();

    // If a central is connected to the peripheral
    if (central) {
        Serial.print("Connected to: ");
        Serial.println(central.address());

        // While central is connected
        while (central.connected()) {
            ledCheckAndSet(); //LED blink loop component

            // Send data
            write_data();

            // Read data
            read_data();
        }

        //Exiting: stop motors if no PID.
        stop();
        Serial.println("Disconnected");
    }

    /*
    * The problem: I think sending a BLE command blocks the loop. So I think I have to refactor so that
    the complementary and gyro do update. And then when the command is called, it's still the GET_IMU_DATA
    structure... But for now I comment out.
    */

    // if (myICM.dataReady())
    // {
    //   myICM.getAGMT();
    //   float theta = atan2((&myICM)->accX(), (&myICM)->accZ())  * (180.0 / M_PI);
    //   float phi = atan2((&myICM)->accY(), (&myICM)->accZ()) * (180.0 / M_PI);
      
    //   dt = ( micros() - gyr_last) / 1000000.;
    //   gyr_last = micros();
    //   gyr_pitch = gyr_pitch - myICM.gyrY()*dt; //negate gyro reading according to sign conv. defined on IMU
    //   gyr_roll = gyr_roll + myICM.gyrX()*dt;
    //   gyr_yaw = gyr_yaw + myICM.gyrZ()*dt;

    //   complementary_pitch = (complementary_pitch - myICM.gyrY()*dt)*(1-alpha_complementary) + theta * alpha_complementary; //also negate gyro
    //   complementary_roll = (complementary_roll + myICM.gyrX()*dt)*(1-alpha_complementary) + phi * alpha_complementary; 

    //   if (write_flag){
    //     acc_pitch_array[write_ind] = theta;
    //     acc_roll_array[write_ind] = phi;
    //     acc_pitch_array_lowpass[write_ind] = lowPassPitch(theta);
    //     acc_roll_array_lowpass[write_ind] = lowPassRoll(phi);
    //     timestamp_array[write_ind] = (int) millis();

    //     gyr_roll_array[write_ind] = gyr_roll;
    //     gyr_pitch_array[write_ind] = gyr_pitch; 
    //     gyr_yaw_array[write_ind] = gyr_yaw;

    //     complementary_pitch_array[write_ind] = complementary_roll;
    //     complementary_roll_array[write_ind] = complementary_roll;
    //     write_ind++;
    //   }
    //   if (write_ind == array_size - 1){
    //     write_flag = false;
    //   }

    //TOF DEBUG
    // distanceSensor1.startRanging(); 
    // distanceSensor2.startRanging();

    // if (!distanceSensor1.checkForDataReady() || !distanceSensor2.checkForDataReady()){
    //   return; //next loop if not ready
    // }

    // int d1 = distanceSensor1.getDistance();
    // distanceSensor1.clearInterrupt();
    // distanceSensor1.stopRanging();
    // Serial.print("Distance1 (mm): ");
    // Serial.print(d1);
    // Serial.print(" | ");
    // // TOF2 distance
    // int d2 = distanceSensor2.getDistance(); 
    // distanceSensor2.clearInterrupt();
    // distanceSensor2.stopRanging();
    // Serial.print("Distance2 (mm): ");
    // Serial.println(d2);
}

//
//
//
//
//
//
//
//
// IMU Helper Functions

// Functions to calibrate
float calibratedRoll(float val){
  const float measuredMinus90 = -87.5;
  const float measuredPlus90 = 88; 

  return -90 + (val - measuredMinus90) * (180 / (measuredPlus90 - measuredMinus90));
}

float calibratedPitch(float val){
  const float measuredMinus90 = -85;
  const float measuredPlus90 = 87; 

  return -90 + (val - measuredMinus90) * (180 / (measuredPlus90 - measuredMinus90));
}


float lowPassPitch(float val){
  if (!DO_LOWPASS) return val;

  bool isFirst = (lowpass_pitch == 0);

  lowpass_pitch[1] = alpha * val + (1 - alpha) * lowpass_pitch[0];
  lowpass_pitch[0] = lowpass_pitch[1]; 

  //To prevent large vertical spike from 0 -> first value
  if (isFirst) return val; 

  return lowpass_pitch[1];
}

float lowPassRoll(float val){
  if (!DO_LOWPASS) return val;

  bool isFirst = (lowpass_pitch == 0);

  lowpass_roll[1] = alpha * val + (1 - alpha) * lowpass_roll[0];
  lowpass_roll[0] = lowpass_roll[1];

  if (isFirst) return val; 
  return lowpass_roll[1];
}

/*
End Code from lab 2 
*/

void send_all_data(int j){
  //Return data as "T:{}|accP:{}|accPlp:{}|accR:{}|accRlp:{}|gyrP:{}|gyrR:{}|gyrY:{}|compP:{}|compR:{}"
  tx_estring_value.clear();
  tx_estring_value.append("T:");
  tx_estring_value.append(timestamp_array[j]);
  tx_estring_value.append("|");
  tx_estring_value.append("accP:");
  tx_estring_value.append(acc_pitch_array[j]);
  tx_estring_value.append("|");
  tx_estring_value.append("accPlp:");
  tx_estring_value.append(acc_pitch_array_lowpass[j]);
  tx_estring_value.append("|");
  tx_estring_value.append("accR:");
  tx_estring_value.append(acc_roll_array[j]);
  tx_estring_value.append("|");
  tx_estring_value.append("accRlp:");
  tx_estring_value.append(acc_roll_array_lowpass[j]);
  tx_estring_value.append("|");
  tx_estring_value.append("gyrP:");
  tx_estring_value.append(gyr_pitch_array[j]);
  tx_estring_value.append("|");
  tx_estring_value.append("gyrR:");
  tx_estring_value.append(gyr_roll_array[j]);
  tx_estring_value.append("|");
  tx_estring_value.append("gyrY:");
  tx_estring_value.append(gyr_yaw_array[j]);
  tx_estring_value.append("|");
  tx_estring_value.append("compP:");
  tx_estring_value.append(complementary_pitch_array[j]);
  tx_estring_value.append("|");
  tx_estring_value.append("compR:");
  tx_estring_value.append(complementary_roll_array[j]);
  tx_characteristic_string.writeValue(tx_estring_value.c_str());
}
