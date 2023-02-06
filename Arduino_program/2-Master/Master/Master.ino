#include <Wire.h>

// IIC initial setup
#define maxlength 128
#define buffer_size 26
#define slave_num 30
char realNodeNumber[maxlength];  // the maximum quantity of IIC slave node is 128
char buff[maxlength][buffer_size] = {""}; 

String str = "";
int count = 0;

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup()
{
  Wire.begin();        // join i2c bus (address optional for master)
  Serial.begin(2000000);
    delay(3000);
//  while (!Serial) {
//    ; // wait for serial port to connect. Needed for native USB port only
//  }

}


// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
void loop()
{
  updateIMUsMonitor();

}


// ================================================================
// ===                    Custom Functions                      ===
// ================================================================

void updateIMUsMonitor() 
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

      str += buff[i];
      str += ";";
    
      delayMicroseconds(1); 
      
   }
   Serial.println(str);
   str = "";
}
