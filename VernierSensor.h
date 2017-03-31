/*
  Class for a Vernier Sensor run through the Adafruit Multiplexer
  
  Written by Scott Kildall
  www.kildall.com

  Important: call the constructor, THEN call setChannelNm
*/


#ifndef CLASS_Vernier_Sesnor
#define CLASS_Vernier_Sesnor

class VernierSensor
{
  public:
    // the channel number will correspond to the pin numbers of the multiplexer, i.e. SD0 and SC0 will correspond to channel = 0
    VernierSensor();

    // call this FIRST. To avoid dynamic alloc, I have this separate accessor function, but there is no error-checking due to memory
    // constraints
    void setChannelNum(uint8_t ch);

    // loads basic sensor info (name, etc) - usually needs only to be done once
    void loadBasicInfo();

    // returns analog data
    int query();
    
  private:
    void tcaselect(uint8_t i);
    int getAPin();    // return analog pin (A0, A1, A2, etc that corresponds to the sensor)

    void DigitalSensorID();
    void PrintSensorInfo();

    byte sensordata [128];      // could make this a static to optimize memory, if needed
    
    uint8_t channelNum;
    uint8_t tcaNum;
    uint8_t sensorNum;
};

#endif // CLASS_Vernier_Sesnor



