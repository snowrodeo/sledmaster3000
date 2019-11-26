#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_GPS.h>
#include <Arduino.h>
#include "HX711.h"
#include <SPI.h>
#include <SD.h>



// what's the name of the hardware serial port?
#define GPSSerial Serial1
// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences
#define GPSECHO false

// SD OpenLog info
/*
#define STATUS_SD_INIT_GOOD 0
#define STATUS_LAST_COMMAND_SUCCESS 1
#define STATUS_LAST_COMMAND_KNOWN 2
#define STATUS_FILE_OPEN 3
#define STATUS_IN_ROOT_DIRECTORY 4
*/

/* Set the delay between fresh samples */
// IMU will be the fastest, and the load cell will be read at the same time
// GPS will go at it's own rate, but a multiple of the IMU, but not faster than 1 HZ
uint16_t GPS_MIN_SAMPLE_MS = 1000;
uint16_t IMU_SAMPLE_RATE_MS = 25;
uint16_t GPS_SAMPLE_RATE_MULTIPLIER = 40 ;
uint16_t GPS_SAMPLE_RATE_COUNTER = 1;
uint32_t timer = millis();

bool GPS_startup_error = false;
bool IMU_startup_error = false;
bool Logger_startup_error = false;
bool RTC_time_good = false;
bool HW_good = true;
bool System_ready = true;

const int chipSelectForSD = 4;

// DEFINE THE INTERFACES OF INTEREST
/*
    Key interfaces and their usage:
    Serial : USB serial for external comms
    I2C:  IMU (address 28)
    SPI: Bluetooth module
    Serial1: GPS feather
    ??? : SD card on 32u4 feather
*/

// OPen the IMU
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// Connect to the GPS on the hardware port
Adafruit_GPS GPS(&GPSSerial);

// set up variables using the SD utility library functions:
SdVolume volume;
SdFile root;
File SDLog;

// Various helper functions
void printEventToSD(sensors_event_t* event) {

   double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
   if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION ) {
      SDLog.print(" - acceleration");
      x = event->acceleration.x;
      y = event->acceleration.y;
      z = event->acceleration.z;
   }
   else if (event->type == SENSOR_TYPE_ORIENTATION) {
      SDLog.print(" - Orientation");
      x = event->orientation.x;
      y = event->orientation.y;
      z = event->orientation.z;
   }
   else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
      SDLog.print(" - Mag field");
      x = event->magnetic.x;
      y = event->magnetic.y;
      z = event->magnetic.z;
   }
   else if ((event->type == SENSOR_TYPE_GYROSCOPE) || (event->type == SENSOR_TYPE_ROTATION_VECTOR)) {
      SDLog.print(" - Gyro");
      x = event->gyro.x;
      y = event->gyro.y;
      z = event->gyro.z;
   }

   SDLog.print(": x= ");
   SDLog.print(x);
   SDLog.print(" | y= ");
   SDLog.print(y);
   SDLog.print(" | z= ");
   SDLog.print(z);
   SDLog.flush();

}

// Various helper functions
void printEvent(sensors_event_t* event) {
  //Serial.print(event->type);
  double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION ) {
    Serial.print(" - acceleration");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_ORIENTATION) {
    Serial.print(" - Orientation");
    x = event->orientation.x;
    y = event->orientation.y;
    z = event->orientation.z;
  }
  else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
    Serial.print(" - Mag field");
    x = event->magnetic.x;
    y = event->magnetic.y;
    z = event->magnetic.z;
  }
  else if ((event->type == SENSOR_TYPE_GYROSCOPE) || (event->type == SENSOR_TYPE_ROTATION_VECTOR)) {
    Serial.print(" - Gyro");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }

  Serial.print(": x= ");
  Serial.print(x);
  Serial.print(" | y= ");
  Serial.print(y);
  Serial.print(" | z= ");
  Serial.print(z);
}


// blink out an error code
void error(uint8_t errno) {
  while(1) {
    uint8_t i;
    for (i=0; i<errno; i++) {
      digitalWrite(13, HIGH);
      delay(100);
      digitalWrite(13, LOW);
      delay(100);
    }
    for (i=errno; i<10; i++) {
      delay(200);
    }
  }
}

void setup(void)
{
   // initial setup, check some numbers

   // Ensure the GPS Sample multiplier results in a sample rate  greater than 1000 ms
   if ( (IMU_SAMPLE_RATE_MS * GPS_SAMPLE_RATE_MULTIPLIER) < GPS_MIN_SAMPLE_MS) 
   {
      while ( (IMU_SAMPLE_RATE_MS * GPS_SAMPLE_RATE_MULTIPLIER) < GPS_MIN_SAMPLE_MS) {
         GPS_SAMPLE_RATE_MULTIPLIER++;
      }
   }
   
   // Bring up interfaces
   //Wire.begin();

   // Step 1, external serial - SHOULD BE ABLE TO turn this off at compile time I guess
   Serial.begin(115200);
   while(!Serial) delay(10); 
   Serial.println("Bruce's little helper - v0.51");  
   Serial.println("Device setup begins"); 
   pinMode(13, OUTPUT);


/* OLD VERSIONS HERE
   if (!SD.begin(chipSelectForSD)) {
      Serial.println("   Data logger init failed....");
      Logger_startup_error = true;
   } else {
      Serial.println("   Data logger device init successful...");
      Serial.println("   Opening log file...");
      SDLog = SD.open("brucelog.txt", FILE_WRITE);
      if (SDLog) {
         Serial.println("   Log file opened successful...");
      } else {
         Serial.println("   Log file opened failed...");
         Logger_startup_error = true;
      }
   }
  */

   // BRING UP IMU, er, UNIT
   Serial.println("Starting IMU");
   if (!bno.begin())
   {
      /* There was a problem detecting the BNO055 ... check your connections */
      Serial.println("   IMU not detected");
      IMU_startup_error = true;
   } else {
         Serial.println("   IMU setup complete ");
   }

   // BRING UP GPS UNIT
   // This always seems to return true no matter what... 
   Serial.println("Starting GPS");
   if( !GPS.begin(9600)) {
      /* There was a problem detecting the GPS ... check your connections */
      Serial.println("   GPS not detected");
      GPS_startup_error = true;
   } else {
      // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
      GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
      // Set the update rate (uncomment the one you want.)
      GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ); //  update rate
      GPS.sendCommand(PMTK_API_SET_FIX_CTL_5HZ);
      Serial.println("   GPS setup complete ");
      GPS_startup_error = false;
   }
   
   // BRING UP THE LOGGER
  
   
   Serial.println("\nInitializing Data logger...");
   if (!SD.begin(chipSelectForSD)) {
      Serial.println("   Data logger init failed....");
      Logger_startup_error = true;
      error(2);
   } 
   
   // Create the log file name
   char filename[15];
   strcpy(filename, "/DatLog00.TXT");
   for (uint8_t i = 0; i < 100; i++) {
      filename[7] = '0' + i/10;
      filename[8] = '0' + i%10;
      // create if does not exist, do not open existing, write, sync after write
      if (! SD.exists(filename)) {
         break;
      }
   }
   delay(250);

   // Open the file for writing
   SDLog = SD.open(filename, FILE_WRITE);
   delay(250);
   if( ! SDLog ) {
      Serial.print("   Data logger : could not create "); 
      Serial.println(filename);
      error(3);
      Logger_startup_error = true;
   } else {
      Serial.print("   Data logger : writing to "); 
      Serial.println(filename);
   }


   // wait for a bit to get started
   delay(1000);

   // REPORT SOME STATUS

   if ( Logger_startup_error || IMU_startup_error || GPS_startup_error) {
      Serial.println("Device setup failed ");
      HW_good = false;
   } else {
      Serial.println("Device setup complete ");
   }
}

void loop() 
{

   // Do any BT comm stuff here I guess

   if (!HW_good) 
   {
      // Check for and display any system errors
      delay(1);

   }
   else if (!System_ready) 
   {
      // Wait for the systme to get ready in needed
         // Try to get or set the RTC value
   } 
   else  // do stuff here
   {

      // Decide if we pull GPS data
      /*
      Serial.print("GPS sample counter is ");
      Serial.println(GPS_SAMPLE_RATE_COUNTER);
      Serial.print("GPS rate multiplier  is ");
      Serial.println(GPS_SAMPLE_RATE_MULTIPLIER);
      */
      char c = GPS.read();
      if ((c) && (GPSECHO))
         Serial.write(c);
      if (timer > millis()) timer = millis();

      if (millis() - timer > IMU_SAMPLE_RATE_MS) {
         timer = millis(); // reset the timer to now     


         if ( GPS_SAMPLE_RATE_COUNTER == GPS_SAMPLE_RATE_MULTIPLIER) {
            // pull a GPS Sample here
            GPS_SAMPLE_RATE_COUNTER = 1;
            if (GPS.newNMEAreceived()) {
               //Serial.println("GPS new NMEA received");

               // a tricky thing here is if we print the NMEA sentence, or data
               // we end up not listening and catching other sentences!
               // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
               //Serial.println(GPS.lastNMEA()); // this also sets the newNMEAreceived() flag to false
               if (!GPS.parse(GPS.lastNMEA())) { // this also sets the newNMEAreceived() flag to false
                  //Serial.println("GPS parse failed");
                  return; // we can fail to parse a sentence in which case we should just wait for another
               }
               float s = GPS.seconds + GPS.milliseconds/1000. + GPS.secondsSinceTime(); 
               int m = GPS.minute;
               int h = GPS.hour;
               int d = GPS.day;
               // Adjust time and day forward to account for elapsed time.
               // This will break at month boundaries!!! Humans will have to cope with April 31,32 etc.
               while(s > 60){ s -= 60; m++; }    
               while(m > 60){ m -= 60; h++; }
               while(h > 24){ h -= 24; d++; }
               // ISO Standard Date Format, with leading zeros https://xkcd.com/1179/ 
               Serial.print(millis());
               Serial.print(" : GPS :");
               Serial.print(GPS.year+2000, DEC); Serial.print("-");
               if(GPS.month < 10) Serial.print("0");
               Serial.print(GPS.month, DEC); Serial.print("-");
               if(d < 10) Serial.print("0");
               Serial.print(d, DEC);
               Serial.print(",");
               if(h < 10) Serial.print("0");
               Serial.print(h, DEC); Serial.print(':');
               if(m < 10) Serial.print("0");
               Serial.print(m, DEC); Serial.print(':');
               if(GPS.seconds < 10) Serial.print("0");
               Serial.print(GPS.seconds, DEC);  
               /*
               if (GPS.milliseconds < 10) {
                  Serial.print("00");
               } else if (GPS.milliseconds > 9 && GPS.milliseconds < 100) {
                  Serial.print("0");
               }
               Serial.print(GPS.milliseconds);
               */
               Serial.print(",");
               if (GPS.fix) {
                  //Serial.print("Location: ");
                  Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
                  Serial.print(",");
                  Serial.print(GPS.longitude, 4); Serial.print(GPS.lon);
                  //Serial.print("Speed (knots): "); Serial.println(GPS.speed);
                  //Serial.print("Angle: "); Serial.println(GPS.angle);
                  //Serial.print("Altitude: "); Serial.println(GPS.altitude);
                  //SDLog.print("Satellites: "); Serial.println((int)GPS.satellites);
               }
               Serial.println();
            } else {
               //Serial.println("GPS new NMEA not received");
            }
         } else {
            GPS_SAMPLE_RATE_COUNTER++;
         }

         // Grab the IMU data
         sensors_event_t orientationData , angVelocityData , linearAccelData;
         bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
         bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
         bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);

         // Grab the Load Cell data

         // write the data here

         // First write the IMU data
         
         
         
         SDLog.print(millis());
         SDLog.print(" : IMU :");
         printEventToSD(&orientationData);
         printEventToSD(&angVelocityData);
         printEventToSD(&linearAccelData);
         SDLog.println();
         
         
         //  write the IMU data
        
         Serial.print(millis());
         Serial.print(" : IMU :");
         printEvent(&orientationData);
         Serial.print(",");
         printEvent(&angVelocityData);
         Serial.print(",");
         printEvent(&linearAccelData);
         Serial.println();
         
         /*
         int8_t boardTemp = bno.getTemp();
         Serial.print(F("temperature: "));
         Serial.println(boardTemp);
         Serial.println();
         */
      }
   }
}
