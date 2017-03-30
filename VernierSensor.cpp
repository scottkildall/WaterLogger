/*
  VernierSensor: Class to encapsulate Vernier sensor
*/

#include "Arduino.h"
#include "VernierSensor.h"
#include "Wire.h"

//-- TCA9548A Mulitplexer address, used in the tcaselect() function below

#define TCAADDR 0x70

VernierSensor::VernierSensor() {
    
}

void VernierSensor::setChannelNum(int _channelNum) {
  channelNum = _channelNum;
}

// loads basic sensor info (name, etc) - usually needs only to be done once
void VernierSensor::loadBasicInfo() {
  tcaselect(channelNum);
   Wire.begin(); //start I2C communication
  // BTAResistorSensorID(Channel);
    
    //if (SensorNumber[Channel]==0)
     //   DigitalSensorID( Channel);// if no resistorID, check for digital ID
  // PrintSensorInfo();// this line can be commented out if you do not need all this info!!!

   Wire.end(); // end I2C communication
  
}

// returns analog data
int VernierSensor::query() {
  //Turn off the I2C communication
    pinMode(A4, INPUT); 
    pinMode(A5, INPUT); 

    return analogRead(getAPin());
}

// returns A0, A1, A2 based on analog pin
int VernierSensor::getAPin() {
  if( channelNum == 0 )
      return A0;
  else if( channelNum == 1 )
      return A1;
  else if( channelNum == 2 )
      return A2;

  else
    return -1;
}

void VernierSensor::tcaselect(uint8_t i) {
  if (i > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}


