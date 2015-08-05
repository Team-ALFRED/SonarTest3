/*
Generic example for the SRF modules 02, 08, 10 and 235.
Only the SRF08 uses the light sensor so when any other 
range finder is used with this code the light reading will 
be a constant value. 
*/

#define cmdByte     0x00       // Command byte
#define lightByte   0x01       // Byte to read light sensor
#define rangeByte   0x02       // Byte for start of ranging data
#define srf3Address 0x79       // F2
#define srf2Address 0x71       // E2
#define srf1Address 0x70       // E0
#define buttonPin   2          // number of E-Stop

#include <Wire.h>
#include <ros.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Int16MultiArray.h>

ros::NodeHandle  n;

std_msgs::Int16MultiArray ultra;
ros::Publisher sensor("sensor", &ultra); 
  
void setup(){
  Wire.begin();             // Initialize the I2C bus
  n.initNode();             // init ros node
  n.advertise(sensor);      // set ros node name   
//  changeAddress(srf2Address,0xE0);
  ultra.data_length = 5;
  pinMode(buttonPin, INPUT);     
}
 
void loop(){
  ultra.data[1] = getRange(srf1Address);        // Calls a function to get the range data 
  ultra.data[2] = getRange(srf2Address); 
  ultra.data[3] = getRange(srf3Address);
  ultra.data[4] = digitalRead(buttonPin); 
  sensor.publish(&ultra);
  n.spinOnce();
  delay(10);                                      // Wait before looping
}

void changeAddress(byte oldAddress, byte newAddress)
{
  Wire.beginTransmission(oldAddress);
  Wire.write(byte(0x00));
  Wire.write(byte(0xA0));
  Wire.endTransmission();

  Wire.beginTransmission(oldAddress);
  Wire.write(byte(0x00));
  Wire.write(byte(0xAA));
  Wire.endTransmission();

  Wire.beginTransmission(oldAddress);
  Wire.write(byte(0x00));
  Wire.write(byte(0xA5));
  Wire.endTransmission();

  Wire.beginTransmission(oldAddress);
  Wire.write(byte(0x00));
  Wire.write(newAddress);
  Wire.endTransmission();
}


 
int getRange(int address){                                   // This function gets a ranging from the SRF08
  int range = 0; 
 
  Wire.beginTransmission(address);             // Start communicating with SRF08
  Wire.write((byte)cmdByte);                             // Send Command Byte
  Wire.write(0x51);                                // Send 0x51 to start a ranging in cm
  Wire.endTransmission();
 
  delay(100);                                     // Wait for ranging to be complete
 
  Wire.beginTransmission(address);             // start communicating with SRFmodule
  Wire.write(rangeByte);                           // Call the register for start of ranging data
  Wire.endTransmission();
 
  Wire.requestFrom(address, 2);                // Request 2 bytes from SRF module
  while(Wire.available() < 2);                    // Wait for data to arrive
  byte highByte = Wire.read();                 // Get high byte
  byte lowByte = Wire.read();                  // Get low byte
 
  range = (highByte << 8) + lowByte;              // Put them together
   
  return(range);                                  // Returns Range
}
