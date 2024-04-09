/*
  This project contains two parts of functions:
  1.using the Hardware I2C to get the IMU data from the Slaves.
  2.using BLE to send the IMU data to PC.
*/
#include <ArduinoBLE.h>
#include <Wire.h>

#include "LSM6DS3.h"
LSM6DS3 myIMU(I2C_MODE, 0x6A);    //I2C device address 0x6A

#include "SoftwareI2C.h"  
SoftwareI2C WireS1;

#include <QMC5883LCompass.h>
QMC5883LCompass compass;

// I2C initial setup
#define buffer_size 26
#define slave_num 9
char buff[slave_num][buffer_size] = {""};

String str_slave = "";
String str_master = "";
int count = 0;

unsigned long myTime;

//BLE initial setup
BLEService MonitorService("00001101-0000-1000-8000-00805f9b34fb");  // setup the uuid
BLEStringCharacteristic txString("00001143-0000-1000-8000-00805f9b34fb", BLERead | BLENotify, 512);

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup()
{
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);
  digitalWrite(LEDR, LOW);
  digitalWrite(LEDG, HIGH);
  digitalWrite(LEDB, HIGH);
  
  // Initialize the magnetometer QMC5883L
  compass.initSoftwareI2C(&WireS1, 3, 2);     // initSoftwareI2C, sda, scl
  compass.init();
  compass.setCalibration(-1180, 1192, -1777, 651, -1136, 1233);
  // Define OSR = 512, Full Scale Range = 8 Gauss, ODR = 200Hz, set continuous measurement mode
  compass.setMode(0x01, 0x0C, 0x10, 0x00);   // compass.setMode(MODE, ODR, RNG, OSR);
  compass.setSmoothing(10,true);  
  delay(500);
  
  // Initialize the hardware I2C bus and Serial port
  Wire.begin();       
  Serial.begin(9600);
  delay(500);

//Call .begin() to configure the IMUs
  if (myIMU.begin() != 0) 
  {
    Serial.println("Device error");
    while(1);
  } else {
    digitalWrite(LEDR, HIGH);
    delay(500);
    digitalWrite(LEDR, LOW);
    delay(500);
    Serial.println("Device OK!");
  }
  delay(500);
  
/*******BLE Initialization*********/
  if (!BLE.begin()) 
  {
    digitalWrite(LEDR, HIGH);
    digitalWrite(LEDB, HIGH);
    delay(500);
    digitalWrite(LEDB, LOW);
    delay(500);
    Serial.println("Starting BLE failed!");
    while (1);
  }
  
// BLE initialization
  BLE.setLocalName("CurveMonitor");
  BLE.setAdvertisedService(MonitorService); // add the service UUID
  MonitorService.addCharacteristic(txString); // add the characteristic
  BLE.addService(MonitorService); // Add the service
  // start advertising
  BLE.advertise();

  Serial.println("Bluetooth device active, waiting for connections...");
  Serial.print("value size = ");
  Serial.println(txString.valueSize());

  digitalWrite(LEDR, HIGH);
  digitalWrite(LEDB, LOW);
}


// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
void loop()
{  
  // wait for a BLE central
  BLEDevice central = BLE.central();

  // if a central is connected to the peripheral:
  if (central) 
  {
    Serial.print("Connected to central: ");
    Serial.println(central.address());
    digitalWrite(LEDB, HIGH);    
    digitalWrite(LEDG, LOW);
    // while the central is connected:
    while (central.connected()) 
    {
      myTime = millis();
      updateSlaveIMUs();
      updateMasterIMU();
      txString.writeValue(String(1000/(millis() - myTime)) + " fps");
    }
    
    // when the central disconnects, turn off the LED:
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
    digitalWrite(LEDG, HIGH);
    digitalWrite(LEDB, LOW);
  }

}


// ================================================================
// ===                    Custom Functions                      ===
// ================================================================

void updateSlaveIMUs() 
{
  uint8_t node_count = 0;
  
  for(uint8_t i=0; i<slave_num; i++)
  {    
      node_count ++;
      Wire.requestFrom(i, buffer_size);    // request buffer_size bytes from slave device #xxx
      count = 0;

      while(Wire.available())    // slave may send less than requested
       {
          buff[i][count++] = Wire.read();    // receive a byte as character
          if(count > buffer_size)
            break;
        }

      str_slave += buff[i];
      str_slave += ";";
    
      if(node_count % 9 == 0)
      {
         txString.writeValue(str_slave); 
         Serial.println(str_slave);
         str_slave = "";
      }
   }
   
   txString.writeValue(str_slave); 
   Serial.println(str_slave);
   str_slave = "";
}

void updateMasterIMU()
{
//  Ax
  str_master += String(myIMU.readFloatAccelX());
  str_master += " ";
//  Ay
  str_master += String(myIMU.readFloatAccelY());
  str_master += " ";
//  Az
  str_master += String(myIMU.readFloatAccelZ());
  str_master += " ";
//  Gx
  str_master += String(myIMU.readFloatGyroX());
  str_master += " ";
//  Gy
  str_master += String(myIMU.readFloatGyroY());
  str_master += " ";
//  Gz
  str_master += String(myIMU.readFloatGyroZ());
  str_master += " ";

  compass.read();
//  Mx
  str_master += String(compass.getX());
  str_master += " ";
//  My
  str_master += String(compass.getY());
  str_master += " ";
//  Mz
  str_master += String(compass.getZ());
  str_master += " ";
//  Azimuth
  str_master += String(compass.getAzimuth());
  Serial.println(compass.getAzimuth());

  txString.writeValue(str_master);
  str_master = "";
}
