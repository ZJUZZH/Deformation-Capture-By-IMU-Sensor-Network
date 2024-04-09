#include <Wire.h>

// IIC initial setup
#define buffer_size 17

#define initial_node 0
#define slave_num 10

byte buff[slave_num][buffer_size] = {""};

String str = "";
uint8_t count = 0;

unsigned long myTime;

// Euler angles
float roll, pitch, yaw;

union BytesToFloat {
    float value;
    byte bytes[4];
};
BytesToFloat q[4];

byte i2c_address[slave_num];

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

  Wire.begin();        // join i2c bus (address optional for master)
  Serial.begin(2000000);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  
  digitalWrite(LEDR, HIGH);
  digitalWrite(LEDG, LOW);
  digitalWrite(LEDB, HIGH);  

  delay(1000);
}


// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
void loop()
{
  // myTime = millis();
  updateSlaveIMUs();
  // Serial.println("  " + String(1000/(millis() - myTime)) + " fps");
  delay(100);
}


// ================================================================
// ===                    Custom Functions                      ===
// ================================================================

void updateSlaveIMUs() 
{
  for(uint8_t i=initial_node; i<slave_num+initial_node; i++)
  {    
    Wire.requestFrom(i, buffer_size);    // request buffer_size bytes from slave device #xxx
    count = 0;

    while(Wire.available())    // slave may send less than requested
    {
      buff[i-initial_node][count++] = Wire.read();    // receive a byte
      if(count >= buffer_size)
        break;
    }

    // i2c address
    i2c_address[i-initial_node] = buff[i-initial_node][0];
    str += String(i2c_address[i-initial_node]);

    // quaternion data: w, x , y, z
    for(uint8_t j = 0; j < 4; j++)
    {
        q[j].bytes[0] = buff[i-initial_node][j * 4 + 1];
        q[j].bytes[1] = buff[i-initial_node][j * 4 + 2];
        q[j].bytes[2] = buff[i-initial_node][j * 4 + 3];
        q[j].bytes[3] = buff[i-initial_node][j * 4 + 4];

        str += ' ';
        str += String(q[j].value);
    }
    str += ";";
  }
   Serial.println(str);
   str = "";    // refresh the string buffer
}