/*
Generic example for the SRF modules 02, 08, 10 and 235.
Only the SRF08 uses the light sensor so when any other 
range finder is used with this code the light reading will 
be a constant value. 
*/

#define cmdByte 0x00                              // Command byte
#define lightByte 0x01                            // Byte to read light sensor
#define rangeByte 0x02                            // Byte for start of ranging data
#define srf1Address 0x79
#define srf2Address 0x70

#include <Wire.h>
#include <ros.h>
#include <sensor_msgs/Range.h>
#include <std_msgs/Int64.h>

ros::NodeHandle  n1;
ros::NodeHandle  n2; 

std_msgs::Int64 ultra_1;
//std_msgs::Int32 ultra_2; 
ros::Publisher sensor1("sensor1", &ultra_1);
//ros::Publisher sensor2("sensor2", &ultra_2); 
  
void setup(){
  Wire.begin();                                   // Initialize the I2C bus
  n1.initNode();                                  // init ros node
  //n2.initNode();                                  // init ros node
  n1.advertise(sensor1);                          // set ros node name   
  //n2.advertise(sensor2);                          // set ros node name 
  //Serial.begin(57600); 
}
 
void loop(){
  ultra_1.data = getRange(srf1Address)<<7;// + getRange(srf2Address);                      // Calls a function to get the range data 
  ultra_1.data = ultra_1.data + getRange(srf2Address); 
  //ultra_2.data = getRange(srf2Address);                      // Calls a function to get the range data 
  
  
  
  sensor1.publish(&ultra_1);
  //Serial.println("here");  
  //sensor2.publish(&ultra_2);
  n1.spinOnce(); 
  //n2.spinOnce(); 
  delay(1000);                                      // Wait before looping
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
