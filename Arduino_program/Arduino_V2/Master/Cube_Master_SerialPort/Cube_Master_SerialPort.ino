#include <Wire.h>

// IIC initial setup
#define BUFFER_SIZE 47
#define SLAVE_NUMBER 24
char buff[SLAVE_NUMBER][BUFFER_SIZE] = {""};

String str_slave = "";

unsigned long myTime;

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
  delay(5000);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  
  digitalWrite(LEDR, HIGH);
  digitalWrite(LEDG, LOW);
  digitalWrite(LEDB, HIGH);  
}


// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
void loop()
{
    // myTime = millis();
    str_slave  = updateSlaveIMUs();
    Serial.println(str_slave);
    str_slave  = "";
    // Serial.println(String(1000/(millis() - myTime)) + " fps");
}


// ================================================================
// ===                    Custom Functions                      ===
// ================================================================

String updateSlaveIMUs() 
{
  String str = "";
  uint8_t count = 0;
  for(uint8_t i=0; i<SLAVE_NUMBER; i++)
  {
    Wire.requestFrom(i, BUFFER_SIZE);    // request buffer_size bytes from slave device #xxx
    count = 0;

    while(Wire.available())    // slave may send less than requested
      {
        buff[i][count++] = Wire.read();    // receive a byte as character
        if(count > BUFFER_SIZE)
          break;
      }

    str += buff[i];
    str += ";";
  }

  return str;
}
