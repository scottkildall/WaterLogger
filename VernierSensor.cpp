/*
  VernierSensor: Class to encapsulate Vernier sensor
*/

#include "Arduino.h"
#include "VernierSensor.h"
#include "Wire.h"

//-- TCA9548A Mulitplexer address, used in the tcaselect() function below

#define TCAADDR (0x70)
#define DEVICE (0x50)



VernierSensor::VernierSensor() {
    //Serial.print("VenierSensor constructor");
}

void VernierSensor::setChannelNum(uint8_t ch) {
  Serial.print("setting channel num = ");
  channelNum = ch;
  Serial.println(channelNum);
  tcaNum = channelNum+1;
}

// loads basic sensor info (name, etc) - usually needs only to be done once
void VernierSensor::loadBasicInfo() {
  //int tcaNum = channelNum+1;
  tcaselect(tcaNum);
   Wire.begin(); //start I2C communication
   
   DigitalSensorID();
   PrintSensorInfo();// this line can be commented out if you do not need all this info!!!

   Wire.end(); // end I2C communication
  
}

// returns analog data
int VernierSensor::query() {
  //Turn off the I2C communication
    pinMode(A3, INPUT); 
    pinMode(A4, INPUT); 
    pinMode(A5, INPUT); 
    
    return analogRead(getAPin());
}

uint8_t VernierSensor::getSensorNum()
{ 
  return sensorNum;
}


// returns A0, A1, A2 based on analog pin
int VernierSensor::getAPin() {
  Serial.print("channel: ");
  Serial.println(channelNum);
  
  if( channelNum == 0 )
      return A0;
  else if( channelNum == 1 )
      return A1;
  else if( channelNum == 2 )
      return A2;

  else {
    return -1;
    Serial.println("bad APin");
  }
}

void VernierSensor::tcaselect(uint8_t i) {
Serial.print("tcaselect: ");
Serial.println(i);

  if (i > 7) return;


  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}

 void VernierSensor::DigitalSensorID()
    { 
      int i;
      int x;
        // check for digital ID sensor:
        for (i=0; i<133; i++)  // clear our digital ID sensor data
         { 
           sensordata[i]=0; 
         } 
       //Now check for Digital AutoID sensor:
      Wire.begin(); // join i2c bus (address optional for master) !!!
      //Reading device first time... ;
      Wire.beginTransmission(DEVICE); // Now we're going to read it back
      Wire.write(0x0);               // Sending address 0, so it knows where we'll want
      Wire.endTransmission();       // to read from
      x =Wire.requestFrom(DEVICE,32);    // Start new transmission and keep reading for 128 bytes
      i=1;
      while(x>1)    
        {  
          x=Wire.available();
           byte a = Wire.read();    // Read a byte and write it out to the Serial port
           sensordata [i]=a;
           i++;  
         } 
      //Reading device second time... ;
      Wire.beginTransmission(DEVICE); // Now we're going to read it back
      Wire.write(0x20);               // Sending address 0, so it knows where we'll want
      Wire.endTransmission();       // to read from
      x =Wire.requestFrom(DEVICE,32);    // Start new transmission and keep reading for 128 bytes
      i=1;
      while(x>1)    
       {  
         x=Wire.available();
         byte c = Wire.read();    // Read a byte and write it out to the Serial port
         sensordata [i+32]=c;
         i++;  
        }  
       //Reading device third time... ;
       Wire.beginTransmission(DEVICE); // Now we're going to read it back
       Wire.write(0x40);               // Sending address 0, so it knows where we'll want
       Wire.endTransmission();       // to read from
       x =Wire.requestFrom(DEVICE,32);    // Start new transmission and keep reading for 128 bytes
       i=1;
       while(x>1)    
        {  
          x=Wire.available();
          byte c = Wire.read();    // Read a byte and write it out to the Serial port
          sensordata [i+64]=c;
          i++;  
         }      
       //Reading device a forth time... ;
       Wire.beginTransmission(DEVICE); // Now we're going to read it back
       Wire.write(0x60);               // Sending address 0, so it knows where we'll want
       Wire.endTransmission();       // to read from
       x =Wire.requestFrom(DEVICE,32);    // Start new transmission and keep reading for 128 bytes
       i=1;
       while(x>1)    
        {  
          x=Wire.available();
          byte c = Wire.read();    // Read a byte and write it out to the Serial port
          sensordata [i+96]=c;
          i++;  
        }      
      /*Print out array:  // remove *'s to get this display for diagnostics
      Serial.print("array: ");  
      for (i = 68; i<121;i++)
       {
         Serial.print (i);
         Serial.print (" ");
         Serial.print(sensordata[i]);  
         Serial.print (" ");
         Serial.println(char(sensordata[i])); 
        }
       */
         //******************************************************************
          //Determine sensor number:  
         //  VoltageID[Channel]=-1;// indicated resistor ID not used
        sensorNum =sensordata[2];  

  /*
        //Determine the sensor name:    
        Name="";
        for (i = 0; i<20;i++)
         {
           char c =  sensordata[i+9]; 
           Name[Channel] += c;
         }
        Name[Channel] += '\0';    //add terminating character
        
        //Determine the short name:  
         ShortName[Channel]="";
         for (i = 0; i<12;i++)
         {
           char c =  sensordata[i+28]; 
           ShortName[Channel] += c;
         }
        ShortName[Channel] += '\0';    //add terminating character
    */   
        //Determine the calibration equation type:  
        //CalEquationType[Channel]=sensordata[57]; 
        
        //Determines the  calibration page:  
        //Page[Channel]= sensordata[70];  
  
        // the code below uses the calibration page set:
        // Intercept starts at 71 for page 1, 90 for p2, and 109 for p3
        
        //Determines intercept:   
        /*
        for (i = 0; i<4;i++)
          {
            floatbyte[i]=sensordata[i+71+(Page[Channel])*19];
          }
         float j = *(float*) &floatbyte;  
         Intercept[Channel] =j;
         */
         //Determines slope:
         // slope starts at 75 for page 1, 94 for p2, and 113 for p3
         /*
         for (i = 0; i<4;i++)
          {
            floatbyte[i]=sensordata[i+75+(Page[Channel])*19];
          }  
         float y = *(float*) &floatbyte;  
         Slope[Channel] =y;
         */
         
         //Determines the units:  
         // the cryptic code in the next line just uses the
         //correct bytes, depending on the page selected.
         // units start at 83 for page 1, 102 for p2, and 121 for p3
         /*for (i= 0; i<7;i++)
           {
            char c =  sensordata[83+i+(Page[Channel])*19]; 
             Units[Channel] += c;
           }
         Units[Channel] += '\0';    //add terminating character
         */
     } //end of checking for digital ID sensor
    
void VernierSensor::PrintSensorInfo()
 {// print out information about sensor:
  //(This code is commented out, but add it for more feedback)
   Serial.println(" "); 
   Serial.print("BTA Channel #: ");
   Serial.println(tcaNum);
   Serial.print("sensor ID number: "); 
   Serial.println(sensorNum);
    
   if( sensorNum == SENSOR_NUM_PH )
     Serial.println( "PH Sensor");
   
   else if( sensorNum == SENSOR_NUM_EC )
     Serial.println( "Conductivity Sensor");

   else if( sensorNum == SENSOR_NUM_DO )
     Serial.println( "Dissolved Oxygen Sensor");

   else if( sensorNum == 0 )
     Serial.println( "No Sensor");

    else
     Serial.println( "Unknown Sensor");
   }// end of PrintSensorInfo


