// Arduino sketch that returns calibration offsets for MPU6050 //   Version 1.1  (31th January 2014)
// Done by Luis Ródenas <luisrodenaslorda@gmail.com>
// Based on the I2Cdev library and previous work by Jeff Rowberg <jeff@rowberg.net>
// Updates (of the library) should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
 
// These offsets were meant to calibrate MPU6050's internal DMP, but can be also useful for reading sensors.
// The effect of temperature has not been taken into account so I can't promise that it will work if you
// calibrate indoors and then use it outdoors. Best is to calibrate and use at the same room temperature.
 
 
#include <EEPROM.h>
// I2Cdev and MPU6050 must be installed as libraries
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

 #define LED_PIN A2 //

///   CONFIGURATION   /
//Change this 3 variables if you want to fine tune the skecth to your needs.
int averageAccount = 1000;   //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
int acel_deadzone = 8;   //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
int giro_deadzone = 1;   //Giro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)

// default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
//MPU6050 accelgyro;
MPU6050 accelgyro(0x68); // <-- use for AD0 low
 
int16_t ax, ay, az, gx, gy, gz;
 
int16_t mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz, state = 0;
int16_t ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset;

int l = 0;
///   SETUP   
void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();
  // COMMENT NEXT LINE IF YOU ARE USING ARDUINO DUE
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz). Leonardo measured 250kHz.
 


  // configure LED for output
  pinMode(LED_PIN, OUTPUT);


  digitalWrite(LED_PIN, HIGH);
  delay(500);
  digitalWrite(LED_PIN, LOW);
 
  // initialize device
  accelgyro.initialize();
  
  digitalWrite(LED_PIN, HIGH);
  delay(500);
  digitalWrite(LED_PIN, LOW);
  
  delay(100);
  
  // verify connection
  if(!accelgyro.testConnection())
    {
      while(1)
  {
    digitalWrite(LED_PIN, l = 1-l);
    delay(100);
  }
    }

  
  // reset offsets
  accelgyro.setXAccelOffset(0);
  accelgyro.setYAccelOffset(0);
  accelgyro.setZAccelOffset(0);
  accelgyro.setXGyroOffset(0);
  accelgyro.setYGyroOffset(0);
  accelgyro.setZGyroOffset(0);
  


//  int16_t dat;
//  EEPROM.get(0, dat);  // read flag bit in the address 0
//  if(dat == initial_value)
//    {
//      while(1)
//      {
//          digitalWrite(LED_PIN, l = 1-l);
//          delay(1000);
//      }
//    }
    
  delay(2000);
  
  digitalWrite(LED_PIN, LOW);
  delay(300);
  digitalWrite(LED_PIN, HIGH);
  delay(300);
  
}
 
///   LOOP   

void loop() {
  if (state == 0) {
//    Serial.println("\nReading sensors for first time...");
    meansensors();
    state++;
    delay(100);
  }
 
  if (state == 1) {
//    Serial.println("\nCalculating offsets...");
    calibration();
    state++;
    delay(100);
  }
 
  if (state == 2) {
    meansensors();

  }
  saveOffsetToEEPROM();
  while(1)
  {
    digitalWrite(LED_PIN, l = 1-l);
    delay(300);
  }
}

 
void meansensors() {
  long i = 0, buff_ax = 0, buff_ay = 0, buff_az = 0, buff_gx = 0, buff_gy = 0, buff_gz = 0;
 
  while (i < (averageAccount + 101)) {
    // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
 
    if (i > 100 && i <= (averageAccount + 100)) { //First 100 measures are discarded
      buff_ax = buff_ax + ax;
      buff_ay = buff_ay + ay;
      buff_az = buff_az + az;
      buff_gx = buff_gx + gx;
      buff_gy = buff_gy + gy;
      buff_gz = buff_gz + gz;
    }
    if (i == (averageAccount + 100)) {
      mean_ax = buff_ax / averageAccount;
      mean_ay = buff_ay / averageAccount;
      mean_az = buff_az / averageAccount;
      mean_gx = buff_gx / averageAccount;
      mean_gy = buff_gy / averageAccount;
      mean_gz = buff_gz / averageAccount;
    }
    i++;
    delay(2); //Needed so we don't get repeated measures
  }
}
 
void calibration() {
  int led = 0;
  ax_offset = -mean_ax / 8;
  ay_offset = -mean_ay / 8;
  az_offset = (16384 - mean_az) / 8;
 
  gx_offset = -mean_gx / 4;
  gy_offset = -mean_gy / 4;
  gz_offset = -mean_gz / 4;
  
  while (1) {
    int ready = 0;
    accelgyro.setXAccelOffset(ax_offset);
    accelgyro.setYAccelOffset(ay_offset);
    accelgyro.setZAccelOffset(az_offset);
 
    accelgyro.setXGyroOffset(gx_offset);
    accelgyro.setYGyroOffset(gy_offset);
    accelgyro.setZGyroOffset(gz_offset);
 
    meansensors();
    
    digitalWrite(LED_PIN, led = 1 - led);
    
    if (abs(mean_ax) <= acel_deadzone) ready++;
    else ax_offset = ax_offset - mean_ax / acel_deadzone;
 
    if (abs(mean_ay) <= acel_deadzone) ready++;
    else ay_offset = ay_offset - mean_ay / acel_deadzone;
 
    if (abs(16384 - mean_az) <= acel_deadzone) ready++;
    else az_offset = az_offset + (16384 - mean_az) / acel_deadzone;
 
    if (abs(mean_gx) <= giro_deadzone) ready++;
    else gx_offset = gx_offset - mean_gx /giro_deadzone;
 
    if (abs(mean_gy) <= giro_deadzone) ready++;
    else gy_offset = gy_offset - mean_gy /giro_deadzone;
 
    if (abs(mean_gz) <= giro_deadzone) ready++;
    else gz_offset = gz_offset - mean_gz /giro_deadzone;
 
    if (ready == 6) break;
  }
}

void saveOffsetToEEPROM()
{
  int eeAddress = 0;
  
  
//  Serial.print("write the offset to EEPROM: ");
//  printOffset();
//  EEPROM.put(eeAddress, initial_value);
//  
//  eeAddress = sizeof(int16_t); //Move address to the next byte after a int16_t data.  
  
  EEPROM.put(eeAddress, ax_offset);
  eeAddress += sizeof(int16_t); //Move address to the next byte after a int16_t data.
  EEPROM.put(eeAddress, ay_offset);
  eeAddress += sizeof(int16_t); 
  EEPROM.put(eeAddress, az_offset);
  eeAddress += sizeof(int16_t); 
  EEPROM.put(eeAddress, gx_offset);
  eeAddress += sizeof(int16_t);
  EEPROM.put(eeAddress, gy_offset);
  eeAddress += sizeof(int16_t); 
  EEPROM.put(eeAddress, gz_offset);
}
