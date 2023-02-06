/*
  This project contains three parts of functions:
  1.using Hardware Serial Port to communicate with the previous slave node(or master node) for getting the I2C address
  2.using the Hardware I2C to communicate with MPU6050
  3.using the Software I2C to communicate with I2C Master and transmit the imu data
*/

/*
Note:
  Before using this code, please burn the "Calibration_mpu6050toEEPROM.ino" into your arduino to preset the IMU's offsets, which 
  are saved in the EEPROM. So this program would read IMU's offsets from EEPROM firstly.
*/

#include <EEPROM.h>
//#include <MsTimer2.h>

// ================================================================
// ===                       MPU6050 setup                      ===
// ================================================================
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "MPU6050.h"
#include "Wire.h"

#define LED_PIN A2 //
//#define LED_PIN LED_BUILTIN 

MPU6050 accelgyro(0x68);

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
bool dmpRead_flag = true; // set true if IIC have sent IMU data
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;   // [w, x, y, z]         quaternion container
char imu_data[26] = "no data"; // must be initialized  
char w[6], x[6], y[6], z[6];

//int16_t ax, ay, az, gx, gy, gz;
int16_t ax_offset=0, ay_offset=0, az_offset=0, gx_offset=0, gy_offset=0, gz_offset=0;


// INTERRUPT DETECTION ROUTINE
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() 
{
    mpuInterrupt = true;
}


// ================================================================
// ===                  Software I2C setup                      ===
// ================================================================

//-----------------------------------------------//
//The I2C address should be changed manually here.
//此处需手动修改软件I2C地址
//逆时针对应偶数号，顺时针对应奇数号
#define soft_I2C_SLAVE_ADDRESS 25
char addr[] = "25";
//-----------------------------------------------//

//逆时针，偶数号
//#define soft_SCL_BIT PC0  
//#define soft_SDA_BIT PC1  

//顺时针，奇数号
#define soft_SCL_BIT PC1  
#define soft_SDA_BIT PC0 


#define soft_I2C_DDR DDRC
#define soft_I2C_OUT PORTC
#define soft_I2C_IN  PINC

extern "C" 
{
  volatile uint8_t byteN=0; // receiving byte n (0..39)
  volatile uint8_t I2C_data;

// This function would be triggered while receiving a byte data from the I2C Master
  void InComingData(void) 
  {
//    calibration_flag = I2C_data;
  }

 // This function would be triggered while receiving a request from the I2C Master and will send a byte data to the Master
  void OutGoingData(void) 
  {
    dmpRead_flag = true;
    I2C_data = imu_data[byteN++];
  }
}



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
void setup() 
{   
    // initialize serial communication
//    Serial.begin(38400);  // 8MHz at 3.3v,  38400 or slower
       
    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);
    
    // configure hardware IIC, used for reading IMU data
    Wire.begin();
    TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)

//    delay(5000);
    // initialize IMU device
    accelgyro.initialize();
      // verify connection
  if(!accelgyro.testConnection())
    {
      while(1)
      {
        digitalWrite(LED_PIN, LOW);
        delay(100);
        digitalWrite(LED_PIN, HIGH);
        delay(100);
      }
    }
    
    readOffsetFromEEPROM();

//     open DMP in the IMU
    devStatus = accelgyro.dmpInitialize();

    

    SetIMUOffset();
    
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) 
    {
        // turn on the DMP, now that it's ready
        accelgyro.setDMPEnabled(true);
        // enable Arduino interrupt detection
        attachInterrupt(1, dmpDataReady, RISING);  // connect the Atmega328p INT1 pin for interrupt
        mpuIntStatus = accelgyro.getIntStatus();
        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = accelgyro.dmpGetFIFOPacketSize();
    } 
    else 
    {
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
//      Serial.print(F("DMP Initialization failed (code "));
//      Serial.print(devStatus);
//      Serial.println(F(")"));
      while(1)
        {
            digitalWrite(LED_PIN, LOW);
            delay(500);
            digitalWrite(LED_PIN, HIGH);
            delay(500);
          }
      }
      
      digitalWrite(LED_PIN, HIGH);

    
    // SoftwareI2C Initialization
    //    TIMSK0 = 0;  // disable the Arduino's Timer 0 (NOTE: delay() and millis() will no longer work)
      __asm__ __volatile__ (
              "cbi %[soft_I2CDDR], %[soft_SCLBIT]"                                                "\n"
              "cbi %[soft_I2COUT], %[soft_SCLBIT]"                                                "\n"
              "cbi %[soft_I2CDDR], %[soft_SDABIT]"                                                "\n"
              "cbi %[soft_I2COUT], %[soft_SDABIT]"                                                "\n"
       :: [soft_I2CDDR] "I" (_SFR_IO_ADDR(soft_I2C_DDR)),
          [soft_I2COUT] "I" (_SFR_IO_ADDR(soft_I2C_OUT)),
          [soft_SCLBIT] "I" (soft_SCL_BIT),
          [soft_SDABIT] "I" (soft_SDA_BIT)
      );

}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
//float current_time = 0;
//float end_time = 0;

void loop() 
{
    if(dmpRead_flag)  
      ReadIMUbyDMP();  
      
    dmpRead_flag = false;
    
    noInterrupts();
    byteN = 0;
    SoftI2C_writeANDread();
    interrupts();


//    Serial.println(dmpRead_flag);

//    current_time = millis();   // 毫秒读数.
//    end_time = millis();   // 毫秒读数.
//    Serial.print("----------------------IMU time: "); 
//    Serial.println((end_time - current_time), 1); 
    
}


// ================================================================
// ===                    Custom Functions                      ===
// ================================================================
// Read IMU data 
//void ReadIMU()
//{
//  String data = "";
//    // read raw accel/gyro measurements from device
//  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
////
////  float accx = ax / AcceRatio;              //x轴加速度
////  float accy = ay / AcceRatio;              //y轴加速度
////  float accz = az / AcceRatio;              //z轴加速度
//
//  data += String(addr);
//  data += " ";
//  data += String(ax);
//  data += " ";
//  data += String(ay);
//  data += " ";
//  data += String(az);
//  data += " ";
//  data += String(gx);
//  data += " ";
//  data += String(gy);
//  data += " ";
//  data += String(gz);
//
//  int str_len = data.length() + 1;
//  char buffer_data[str_len];
//  data.toCharArray(buffer_data, str_len);
//  
//  strcpy(imu_data, buffer_data);
//}

// Read IMU data by DMP
void ReadIMUbyDMP()
{
    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = accelgyro.getIntStatus();
    // get current FIFO count
    fifoCount = accelgyro.getFIFOCount();
    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) 
    {
        // reset so we can continue cleanly
        accelgyro.resetFIFO();
    } 
    else if (mpuIntStatus & 0x02) 
    {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) 
          fifoCount = accelgyro.getFIFOCount();

        // read a packet from FIFO
        accelgyro.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        // quaternion values in matrix form: w x y z
        accelgyro.dmpGetQuaternion(&q, fifoBuffer);
        dataConversion(q.w, q.x, q.y, q.z);  // convert and merge the quaternion into the imu_data[](char)
        
//        Serial.println(imu_data); //serial outputs would spend much time.
    }
  }


// This function would convert and merge the quaternion into the imu_data[](char)
void dataConversion(float qw, float qx, float qy, float qz)
{
  // note: the function sprintf() is not compatiable with AVR's arduino.
  // dtostrf() is suitable to convert float to char for AVR's arduino.
  int i = 0;
  char data[30] = ""; //must be initialized

  dtostrf(qw, 1, 2, w);  
  dtostrf(qx, 1, 2, x);
  dtostrf(qy, 1, 2, y);
  dtostrf(qz, 1, 2, z);

//
//  sprintf(w, "%d", ax_offset);  
//  sprintf(x, "%d", ay_offset);
//  sprintf(y, "%d", gx_offset);
//  sprintf(z, "%d", gy_offset);

  strcat(data, addr);
  strcat(data, " ");

  strcat(data, w);
  strcat(data, " ");
  strcat(data, x);
  strcat(data, " ");
  strcat(data, y);
  strcat(data, " ");
  strcat(data, z);
  
  strcpy(imu_data, data);


}

// update the IMU's Offset
void SetIMUOffset()
{
  accelgyro.setXAccelOffset(ax_offset);
  accelgyro.setYAccelOffset(ay_offset);
  accelgyro.setZAccelOffset(az_offset);
  accelgyro.setXGyroOffset(gx_offset);
  accelgyro.setYGyroOffset(gy_offset);
  accelgyro.setZGyroOffset(gz_offset);  
//  
//  accelgyro.setXAccelOffset(10000);
//  accelgyro.setYAccelOffset(-1000);
//  accelgyro.setZAccelOffset(10600);
//  accelgyro.setXGyroOffset(1000);
//  accelgyro.setYGyroOffset(-1000);
//  accelgyro.setZGyroOffset(1000);  
}


// This function would trigger the functions of InComingData() and OutGoingData(), the I2C_data representes the software I2C's data transmission register
void SoftI2C_writeANDread()
{
    __asm__ __volatile__ (
    "HandleTransaction:"                                                             "\n"  // rcall 相对子程序调用 ，调用I2C_activity子程序
           "rcall I2C_activity"                                                      "\n"
           "brtc HandleTransaction"                                                  "\n"
    "StartCondition:"                                                                "\n"
           "ldi R16, 0"                                                              "\n"
           "sts byteN, R16"                                                          "\n"
           "rcall wait_SCL_low"                                                      "\n"
           "rcall slave_readByte"                                                    "\n"
           "brts StartCondition"                                                     "\n"
           "brhs StopCondition"                                                      "\n"
           "mov R19, R18"                                                            "\n"
           "lsr R19"                                                                 "\n"
           "cpi R19, %[soft_SLAVE_ADDR]"                                                  "\n"
           "brne StopCondition"                                                      "\n"
           "rcall slave_writeACK"                                                    "\n"
           "andi R18, 0b00000001"                                                    "\n"
           "brne MasterRead"                                                         "\n"
    "MasterWrite:"                                                                   "\n"
           "rcall slave_readByte"                                                    "\n"
           "brts StartCondition"                                                     "\n"
           "brhs StopCondition"                                                      "\n"
           "rcall slave_writeACK"                                                    "\n"
           "sbi %[soft_I2CDDR], %[soft_SCLBIT]"                                                "\n"
           "sts I2C_data, R18"                                                       "\n"
           "call InComingData"                                                       "\n"
           "cbi %[soft_I2CDDR], %[soft_SCLBIT]"                                                "\n"
           "rjmp MasterWrite"                                                        "\n"
    "MasterRead:"                                                                    "\n"
           "sbi %[soft_I2CDDR], %[soft_SCLBIT]"                                                "\n"
           "call OutGoingData"                                                       "\n"
           "lds R18, I2C_data"                                                       "\n"
           "cbi %[soft_I2CDDR], %[soft_SCLBIT]"                                                "\n"
           "rcall slave_writeByte"                                                   "\n"
           "brts StartCondition"                                                     "\n"
           "brhs StopCondition"                                                      "\n"
           "rcall slave_readACK"                                                     "\n"
           "breq MasterRead"                                                         "\n"
    "StopCondition:"                                                                 "\n"
           "rjmp DoneTransaction"                                                    "\n"

    "I2C_activity:"                                                                  "\n"
           "in R16, %[soft_I2CIN]"                                                        "\n"  // in I/O口输入
           "andi R16, (1<<%[soft_SCLBIT] | 1<<%[soft_SDABIT])"                                 "\n"
    "ac1:" "in R17, %[soft_I2CIN]"                                                        "\n"
           "andi R17, (1<<%[soft_SCLBIT] | 1<<%[soft_SDABIT])"                                 "\n"
           "cp R16, R17"                                                             "\n"
           "breq ac1"                                                                "\n"
           "clh"                                                                     "\n"
           "clt"                                                                     "\n"
           "sbrs R16, %[soft_SCLBIT]"                                                     "\n"
           "rjmp ac2"                                                                "\n"
           "sbrs R17, %[soft_SCLBIT]"                                                     "\n"
           "rjmp ac2"                                                                "\n"
           "sbrs R17, %[soft_SDABIT]"                                                     "\n"
           "set"                                                                     "\n"
           "sbrc R17, %[soft_SDABIT]"                                                     "\n"
           "seh"                                                                     "\n"
    "ac2:" "ret"                                                                     "\n"

    "slave_readByte:"                                                                "\n"
           "ldi R18, 0b00000001"                                                     "\n"
    "rb1:" "rcall wait_SCL_high"                                                     "\n"
           "in R19, %[soft_I2CIN]"                                                        "\n"
           "rcall I2C_activity"                                                      "\n"
           "brts rb2"                                                                "\n"
           "brhs rb2"                                                                "\n"
           "sec"                                                                     "\n"
           "sbrs R19, %[soft_SDABIT]"                                                     "\n"
           "clc"                                                                     "\n"
           "rol R18"                                                                 "\n"
           "brcc rb1"                                                                "\n"
           "clz"                                                                     "\n"
           "clh"                                                                     "\n"
    "rb2:" "ret"                                                                     "\n"

    "slave_writeByte:"                                                               "\n"
           "ldi R19, 8"                                                              "\n"
    "wb1:" "lsl R18"                                                                 "\n"
           "brcs wb2"                                                                "\n"
           "sbi %[soft_I2CDDR], %[soft_SDABIT]"                                                "\n"
    "wb2:" "brcc wb3"                                                                "\n"
           "cbi %[soft_I2CDDR], %[soft_SDABIT]"                                                "\n"
    "wb3:" "rcall wait_SCL_high"                                                     "\n"
           "rcall I2C_activity"                                                      "\n"
           "brts wb4"                                                                "\n"
           "brhs wb4"                                                                "\n"
           "dec R19"                                                                 "\n"
           "brne wb1"                                                                "\n"
    "wb4:" "cbi %[soft_I2CDDR], %[soft_SDABIT]"                                                "\n"
           "ret"                                                                     "\n"

    "skipPulse:"                                                                     "\n"
           "rcall wait_SCL_high"                                                     "\n"
    "wait_SCL_low:"                                                                  "\n"
           "sbic %[soft_I2CIN], %[soft_SCLBIT]"                                                "\n"
           "rjmp wait_SCL_low"                                                       "\n"
           "ret"                                                                     "\n"

    "wait_SCL_high:"                                                                 "\n"
           "sbis %[soft_I2CIN], %[soft_SCLBIT]"                                                "\n"
           "rjmp wait_SCL_high"                                                      "\n"
           "ret"                                                                     "\n"

    "slave_writeACK:"                                                                "\n"
           "sbi %[soft_I2CDDR], %[soft_SDABIT]"                                                "\n"
           "rcall skipPulse"                                                         "\n"
           "cbi %[soft_I2CDDR], %[soft_SDABIT]"                                                "\n"
           "ret"                                                                     "\n"

    "slave_readACK:"                                                                 "\n"
           "rcall wait_SCL_high"                                                     "\n"
           "sez"                                                                     "\n"
           "sbic %[soft_I2CIN], %[soft_SDABIT]"                                                "\n"
           "clz"                                                                     "\n"
           "rcall wait_SCL_low"                                                      "\n"
           "ret"                                                                     "\n"

    "DoneTransaction:"                                                               "\n"
  :: [soft_I2CIN] "I" (_SFR_IO_ADDR(soft_I2C_IN)),
     [soft_I2COUT] "I" (_SFR_IO_ADDR(soft_I2C_OUT)),
     [soft_I2CDDR] "I" (_SFR_IO_ADDR(soft_I2C_DDR)),
     [soft_SCLBIT] "I" (soft_SCL_BIT),
     [soft_SDABIT] "I" (soft_SDA_BIT),
     [soft_SLAVE_ADDR] "M" (soft_I2C_SLAVE_ADDRESS),
     "e" (InComingData),
     "e" (OutGoingData),
     [I2C_data] "label" (I2C_data),
     [byteN] "label" (byteN)
  );
}

// ================================================================
// ===                      EEPROM FUNCTIONS                    ===
// ================================================================

void readOffsetFromEEPROM()
{
//  int eeAddress = sizeof(byte);
  int eeAddress = 0;

  EEPROM.get(eeAddress, ax_offset);
  eeAddress += sizeof(int16_t); //Move address to the next byte after a int16_t data.
  EEPROM.get(eeAddress, ay_offset);
  eeAddress += sizeof(int16_t); 
  EEPROM.get(eeAddress, az_offset);
  eeAddress += sizeof(int16_t); 
  EEPROM.get(eeAddress, gx_offset);
  eeAddress += sizeof(int16_t);
  EEPROM.get(eeAddress, gy_offset);
  eeAddress += sizeof(int16_t); 
  EEPROM.get(eeAddress, gz_offset);

//  Serial.print("read the offset for EEPROM: ");
//  printOffset();
}
