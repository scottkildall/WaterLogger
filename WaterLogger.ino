/*
 *  WaterLogger
 *  Arduino GPS Data Logger using Vernier Sensors
 *  by Scott Kildall
 *  
 *  Adapted from the Ladaada's logger, modified by Bill Greiman to use the SdFat library
 *  Works with the Adafruit Ultimate GPS Shield using MTK33x9 chipset
 * 
 *  Also uses the Adafruit i2c Multiplexer, conneced to Vernier BTA-ELV analog ports
 *  
 *  The 3 sensor we are using are: pH, Conductivity and Dissolved Oxygen
 *  
 *  And it used modified Adafruit GPS logging libraries, due to a parsing error
 * 
 *  ------------------------------------------------------------------------
 *  
 *  Data-logging fields will be:
 *  
 *  Time (elapsed ms), Latitude, Longitude, Elevation, pH, Conductivity, DO
 *  
 *  Exact format of each is still in progress
 * 
 *  Lat, Long, shoud be something like: 35.655856,-105.977923
 *  
 *  To do:
 *  - Clean up GPS-logger
 *  - lat/long conversion logging
 *  - Solder jumper wires
 *  - Wire in Arduino breadboards
 *  
 *  -----------
 *  For Mega 2560, refer here: https://learn.adafruit.com/adafruit-shield-compatibility/ultimate-gps-shield
 *  Create a Jumper from pin 8 on the shield to RX1 on the Mega
 *  Create a Jumper from pin 7 on the shield to TX1 on the Mega
  */

#include <SPI.h>
#include <SoftwareSerial.h>
#include <SD.h>


//-- this is a modified version of the Adafruit_GPS libraries. There is a parsing error and a bit of extra code, which I have trimmed out
//-- due to memory constraints
#include "Adafruit_GPS_mod.h"

//-- my custom time class
#include "MSTimer.h"


#include "VernierSensor.h"


// Uno, use this
//SoftwareSerial mySerial(8, 7);

// Arduino Mega 2560, use this
#define mySerial Serial1
Adafruit_GPS GPS(&mySerial);

// PINS
#define redLEDPin (6)
#define greenLEDPin (4)
#define switchPin  (9)
#define chipSelectPin (10)
#define builtinledPin (13)

// Switch Debounce
#define debounceMS (100)

// Data file
File datafile;
MSTimer dataSaveTimer;          // time for how often we save GPS info
#define dataSaveTime (2000)     // intervals for saving GPS data (in ms)

// timer how often we save to a new file (in case of crashes or other problems)
MSTimer newFileTimer;
#define TIME_BEFORE_NEWFILE (60000 * 10)

// Vernier Sensors
#define numSensors (3)
VernierSensor sensors[numSensors];


// status of the system
short status;
#define statusStartup (0)
#define statusWaiting (1)
#define statusRecording (2)
#define statusError (3)

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO  true
/* set to true to only log to SD when GPS has a fix, for debugging, keep it false */
#define LOG_FIXONLY false  

#define SERIAL_DEBUG true

void setup() {
  status = statusStartup;
  
  // connect at 115200 so we can read the GPS fast enough and echo without dropping chars
  // also spit it out
  if(SERIAL_DEBUG) {
    Serial.begin(115200);
    Serial.println("\r\nWater Logger by Scott Kildall");
  }

  
  pinMode(builtinledPin, OUTPUT);
  pinMode(redLEDPin, OUTPUT);    // Your LED pin, check out global variables above
  pinMode(greenLEDPin, OUTPUT);    // Your LED pin, check out global variables above
  pinMode(switchPin, INPUT);  // Your switch pin (though technically speaking, this code is not needed)
  startupBlink();
  
  adjustLEDs();
  
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(chipSelectPin, OUTPUT);

  //-- initailze the SD Card    
  initSDCard();
  createSDFile();

  initGPS();

  initSensors();
  
  dataSaveTimer.setTimer(dataSaveTime);
  dataSaveTimer.start();

  newFileTimer.setTimer(TIME_BEFORE_NEWFILE);
  newFileTimer.start();

  if(SERIAL_DEBUG) 
    Serial.println("Ready!");

  status = statusWaiting;
  adjustLEDs();
}


void loop() {
  boolean switchPressed = checkSwitchPressed();

     // channel numbers start at 0, check wiring on the multiplexer
     //Serial.println("----------");
    
        
    if( status != statusRecording ) {
     
       // channel numbers start at 0, check wiring on the multiplexer
    for(int i = 0; i < numSensors; i++ ) {
      sensors[i].setChannelNum(i);
      sensors[i].loadBasicInfo();
    }
    delay(1000);
      return;
    }

    if( switchPressed ) {
       for(int i = 0; i < numSensors; i++ )
        sensors[i].loadBasicInfo();
    }
    
    readGPSData();
    
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    
    if(SERIAL_DEBUG) 
      Serial.println("GPS - received");
    
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trying to print out data
    
    // Don't call lastNMEA more than once between parse calls!  Calling lastNMEA 
    // will clear the received flag and can cause very subtle race conditions if
    // new data comes in before parse is called again.
    char *stringptr = GPS.lastNMEA();
    
    if (!GPS.parse(stringptr))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another

    // Sentence parsed! 
    if(SERIAL_DEBUG) 
      Serial.println("OK");
    if (LOG_FIXONLY && !GPS.fix) {
      if(SERIAL_DEBUG)  
        Serial.print("No Fix");
      return;
    }
   
    printGPSData();

    if( dataSaveTimer.isExpired() ) {
       Serial.println("timer expired");

     
    
        if( GPS.latitude != 0.0 ) {
           Serial.println("writing data");

          // grab sensor data, write to data file
          int data[numSensors];
          for(int i = 0; i < numSensors; i++ ) {
            data[i] = sensors[i].query();

            //-- special-case debug until we get this sorted out
            Serial.print("A");
             Serial.print(i);
            Serial.print("= ");
            Serial.println(data[i]);
          }
          
          digitalWrite(redLEDPin, true);
          writeData();
          digitalWrite(redLEDPin, false);
        }
        
        dataSaveTimer.start();
    }

    // create a new SD file
    if( newFileTimer.isExpired() ) {
      createSDFile();
      newFileTimer.start();
    }

    // not sure what's going on here - this can be pulled out
//    if (strstr(stringptr, "RMC") || strstr(stringptr, "GGA")) 
//      datafile.flush();

    if(SERIAL_DEBUG) 
      Serial.println();
  }
}

boolean checkSwitchPressed() {
   if( digitalRead(switchPin) == true  ) {   // button is pressed
        delay(debounceMS); // debounce
         
         // wait for button to be released
         while( digitalRead(switchPin) == true )
           ;  // do nothing
         
        delay(debounceMS); // debounce
         
        status = (status == statusWaiting) ? statusRecording: statusWaiting;

        adjustLEDs();
   }
}

void startupBlink() {
  for( int i = 0; i < 3; i++ ) {
     digitalWrite(greenLEDPin, HIGH);   
     digitalWrite(redLEDPin, HIGH); 
     delay(100);
     digitalWrite(greenLEDPin, LOW);   
     digitalWrite(redLEDPin, LOW); 
     delay(100);
  }
}

void adjustLEDs() {
  if( status == statusStartup ) {
    digitalWrite(greenLEDPin, HIGH);   
    digitalWrite(redLEDPin, HIGH);
  }
  else if( status == statusWaiting ) {
    digitalWrite(greenLEDPin, LOW);   
    digitalWrite(redLEDPin, HIGH);
  }
  else if( status == statusRecording ) {
    digitalWrite(greenLEDPin, HIGH);   
    digitalWrite(redLEDPin, LOW);
  }
  else if( status == statusError ) {
    digitalWrite(greenLEDPin, HIGH);   
    digitalWrite(redLEDPin, HIGH);
  }
}

void writeData() {
  datafile.print(millis());
  datafile.print(",");
  datafile.print(GPS.latitude,8);
  datafile.print(",");
  datafile.println(GPS.longitude,8);
  datafile.flush();
}
void printGPSData() {
  if(SERIAL_DEBUG) {
   Serial.print("\nTime: ");
    Serial.print(GPS.hour, DEC); Serial.print(':');
    Serial.print(GPS.minute, DEC); Serial.print(':');
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality); 
    
    //if (GPS.fix) {
      Serial.print("Location: ");
       
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(", "); 
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
      
      Serial.print("Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
    //}

  }
}

// read a Hex value and return the decimal equivalent
uint8_t parseHex(char c) {
  if (c < '0')
    return 0;
  if (c <= '9')
    return c - '0';
  if (c < 'A')
    return 0;
  if (c <= 'F')
    return (c - 'A')+10;
}


void initSDCard() {
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelectPin, 11, 12, 13)) {
    //if (!SD.begin(chipSelect)) {      // if you're using an UNO, you can use this line instead
    
    Serial.println("Card init. failed!");
    error(2);
  }
}


void createSDFile() {
  char filename[32];
  int fileNum = 1;
  
  // The SD Card needs an all caps filename
   while( true ) {
     sprintf(filename, "DATA_%d.CSV", fileNum);
     if( SD.exists(filename) == false ) {
       Serial.print("Opening new file: ");
       Serial.println(filename);
       datafile = SD.open(filename, FILE_WRITE);
       break;
     }
  
     fileNum++;
   }

   if( !datafile ) {
    if(SERIAL_DEBUG) {
      Serial.print("Couldnt create "); 
      Serial.println(filename);
    }
   }
}


void initGPS() {
  // connect to the GPS at the desired rate
  GPS.begin(9600);

  // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // uncomment this line to turn on only the "minimum recommended" data
  //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  // For logging data, we don't suggest using anything but either RMC only or RMC+GGA
  // to keep the log files at a reasonable size
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 100 millihertz (once every 10 seconds), 1Hz or 5Hz update rate

  // Turn off updates on antenna status, if the firmware permits it
  GPS.sendCommand(PGCMD_NOANTENNA);
}

void initSensors() {
   // channel numbers start at 0, check wiring on the multiplexer
  for(int i = 0; i < numSensors; i++ )
    sensors[i].setChannelNum(i);
    
  // channel numbers start at 0, check wiring on the multiplexer
  for(int i = 0; i < numSensors; i++ )
    sensors[i].loadBasicInfo();
}

void readGPSData() {
  // read data from the GPS in the 'main loop'
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    if (GPSECHO) {
      if (c) {
          if(SERIAL_DEBUG) 
            Serial.print(c);
      }
    }
}

// blink out an error code on the default pin
void error(uint8_t errno) {
  while(1) {
    uint8_t i;
    for (i=0; i<errno; i++) {
      digitalWrite(builtinledPin, HIGH);
      delay(100);
      digitalWrite(builtinledPin, LOW);
      delay(100);
    }
    for (i=errno; i<10; i++) {
      delay(200);
    }
  }
}
