/*  
    April 6, 2013
    Code for the microSD card, XBee, LCD screen, GPS position, speed, accelerator pedal position, brake pedal position, steering wheel position, motor temperature, battery temperature, motor current, and battery voltage.
    Designed to work on the Arduino Mega Pro 3.3V.

*/

#include <SoftwareSerial.h>
#include <Adafruit_MAX31855.h>
#include <Wire.h>
#include <Adafruit_ADS1015.h>
#include <Adafruit_GPS.h>
#include <SD.h>

Adafruit_GPS GPS(&Serial3);  // Tells the Arduino to use the TX3 and RX3 ports when communicating with the GPS module.

// Pin assignments for the thermocouple.

int thermoDO = 33;
int thermoCS = 32;
int thermoCLK = 34;

// Pin assignments for the micro SD card.

int chipSelect = 53;
int myData = 0; // Not a pin, but a variable.

// Variables created for gathering GPS data.

int month;
int day;
int year;
int hour;
int minute;
int seconds;
int milliseconds;
int fix_2;
int fix;
int quality;
double latitude;
double longitude;
char lat_dir;
char long_dir;
double _speed;
double angle;
double altitude;
int satellites;

// Variables created for calculation purposes for the battery temperature subsystem.

int Vo;    // Integer value of voltage reading, 0-1023. 1023 = 3.3V. This variable is also used for other subsystems.
int Vtop;  // Integer value of voltage reading, 0-1023. 1023 = 3.3V.
int Vbot;  // Integer value of voltage reading, 0-1023. 1023 = 3.3V.
double Rt1;  // Computed resistance of thermistor #1.
double Rt2;  // Computed resistance of thermistor #2.
double Rt3;  // Computed resistance of thermistor #3.
double Rt4;  // Computed resistance of thermistor #4.
double Rt5;  // Computed resistance of thermistor #5.
double Rt6;  // Computed resistance of thermistor #6.
double A = -0.068670774;  // Constant A in the Steinhart-Hart equation.
double B = 0.014595474;  // Constant B in the Steinhart-Hart equation.
double C = -8.59438e-5;  // Constant C in the Steinhart-Hart equation.
int T1;  // Temperature of battery #1.
int T2;  // Temperature of battery #2
int T3;  // Temperature of battery #3.
int T4;  // Temperature of battery #4.
int T5;  // Temperature of battery #5.
int T6;  // Temperature of battery #6.

// Variables created for calculation purposes for the steering wheel position subsystem.

double steering_pos;
int steering_angle;

// Variables created for calculation purposes for the accelerator pedal position subsystem.

int accel_position;

// Variables created for calculation purposes for the brake pedal position subsystem.

int brake_pressure;
int brake_position;

// Variables created for calculation purposes for the battery voltage subsystem.

double vbat_1;
double vbat_2;
double vbat_3;
double vbat_4;
double vbat_5;
double vbat_6;

// Variables created for calculation purposes for the pedal position potentiometer.

double results;
double percentage;

// Function prototypes.

SoftwareSerial xbee(2, 3); // RX, TX
Adafruit_MAX31855 thermocouple(thermoCLK, thermoCS, thermoDO);
Adafruit_ADS1115 ads1115_no1(0x48);  // Construct an ads1115 at the default address, 0x48
Adafruit_ADS1115 ads1115_no2(0x49);  // Construct an ads1115 at address 0x49
#define GPSECHO  false  // For the GPS module. Set to 'false' to turn off echoing the GPS data to the Serial console. Set to 'true' if you want to debug and listen to the raw GPS sentences. 
boolean usingInterrupt = false;  // For the GPS module.
void useInterrupt(boolean);  // For the GPS module.

// Interrupt is called once a millisecond, looks for any new GPS data, and stores it.

SIGNAL(TIMER0_COMPA_vect)
{
  char c = GPS.read();
  // If you want to debug, this is a good time to do it!
  if (GPSECHO)
    if (c) UDR0 = c;  
    // Writing direct to UDR0 is much much faster than Serial.print, but only one character can be written at a time. 
}

void useInterrupt(boolean v)
{
  if (v)
  {
    // Timer0 is already used for millis() - we'll just interrupt somewhere in the middle and call the "Compare A" function above.
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  }
  else
  {
    // Do not call the interrupt function COMPA anymore.
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
} 

uint32_t timer = millis();

// Default file name for the file to be created on the microSD card.

char filename[] = "DATA0000.CSV";

// Array for temporary use when appending doubles to the data string.

static char dtostrfbuffer[15];

void setup()
{
   Serial.begin(57600);  // Should be the same as the baud rate setting in the computer's Serial Monitor window. 
   Serial.println( "Arduino started sending bytes via XBee." );  // Only seen on the computer's Serial Monitor window.
   xbee.begin(19200);  // Initialize XBee module.
   GPS.begin(9600);  // Initialize GPS module.
   GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);  // Turn on RMC (recommended minimum) and GGA (fix data) including altitude.
   GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);  // 1 Hz update rate.
   useInterrupt(true);
   ads1115_no1.begin();
   ads1115_no2.begin();

   // Make sure that the default chip select pin is set to output, even if you don't use it.
   
   pinMode(chipSelect, OUTPUT);

   // See if the card is present and can be initialized.
   
   Serial.print("Initializing SD card...");
   xbee.print("Initializing SD card...");
   if (!SD.begin(chipSelect))
   {
     Serial.println("Card failed, or not present.");
     xbee.println("Card failed, or not present.");
     // Don't do anything else.
     return;
   }
   Serial.println("Card initialized.");
   xbee.println("Card initialized.");
  
   String dataString = "Date,Time,Latitude,Longitude,Speed,Throttle_Position,Motor_Temperature";  // The header for the data to be collected.
   File logfile;

   // Adjusting the file name for the file to be created on the microSD card in this session to prevent overwriting an existing file.
  
   for (uint8_t i = 0; i < 10000; i++)
   {
     filename[4] = i/1000 + '0';
     filename[5] = i/100 + '0';
     filename[6] = i/10 + '0';
     filename[7] = i%10 + '0';
     if (! SD.exists(filename))
     {
       // Only open a new file if it doesn't exist.
       logfile = SD.open(filename, FILE_WRITE); 
       break;  // Leave the loop!
     }
   }
  
   Serial.print("Logging to: ");
   Serial.println(filename);
   xbee.print("Logging to: ");
   xbee.println(filename);

   // If the file is available, write to it: 
   if (logfile)
   {
     logfile.println(dataString);
     logfile.close();
     // Print to the serial port too:
     Serial.println(dataString);
     xbee.println(dataString);
   }  
   // If the file isn't open, display up an error:  
   else
   {
     Serial.println("Error opening file.");
     xbee.println("Error opening file.");
   }   
}

void loop()
{
  // Make a string for assembling the data to log.

  String dataString = "";
  
  // GPS code.
  
  if (! usingInterrupt)
  {
    // Read data from the GPS in the 'main loop'.
    char c = GPS.read();
    // If you want to debug, this is a good time to do it!
    if (GPSECHO)
      if (c) UDR0 = c;
      // Writing direct to UDR0 is much much faster than Serial.print, but only one character can be written at a time. 
  }
  
  // If a sentence is received, we can check the checksum, parse it...
  
  if (GPS.newNMEAreceived())
  {
    if (!GPS.parse(GPS.lastNMEA()))  // This sets the newNMEAreceived() flag to false.
      return;  // We can fail to parse a sentence, in which case we should just wait for another one.
  }

  // Serial. ==> GPS data that gets printed to the computer's Serial Window.
  // xbee. ==> GPS data that gets transmitted via the XBee to the computer.
  
  month = (int)GPS.month;
  day = (int)GPS.day;
  year = (int)GPS.year;
  
  // Adjusting the date to account for Greenwich Mean Time.
  if (GPS.hour <= 4)
    day = (int)GPS.day - 1;
    // Note that this causes problems on the first of the month!

  // Adjusting the Greenwich Mean Time reported by the GPS to Eastern Standard Time.
  if (GPS.hour >= 4)
    hour = (int)GPS.hour - 4;
  else
    hour = (int)GPS.hour - 4 + 24;

  minute = (int)GPS.minute;
  seconds = (int)GPS.seconds;
  milliseconds = (int)GPS.milliseconds;
  fix = (int)GPS.fix;
  quality = (int)GPS.fixquality;
    
  // Serial transmissions.
  Serial.print("\nDate: ");
  Serial.print(month, DEC); Serial.print('/');
  Serial.print(day, DEC); Serial.print("/20");
  Serial.println(year, DEC);  
  Serial.print("Time: ");
  Serial.print(hour, DEC); Serial.print(':');
  Serial.print(minute, DEC); Serial.print(':');
  Serial.print(seconds, DEC); Serial.print('.');
  Serial.println(milliseconds);
  Serial.print("Fix: "); Serial.print(fix);
  Serial.print(" Quality: "); Serial.println(quality);

  // XBee transmissions.
  xbee.print("\nDate: ");
  xbee.print(month, DEC); xbee.print('/');
  xbee.print(day, DEC); xbee.print("/20");
  xbee.println(year, DEC);   
  xbee.print("Time: ");
  xbee.print(hour, DEC); xbee.print(':');
  xbee.print(minute, DEC); xbee.print(':');
  xbee.print(seconds, DEC); xbee.print('.');
  xbee.println(milliseconds);
  xbee.print("Fix: "); xbee.print(fix);
  xbee.print(" Quality: "); xbee.println(quality);

  //if (GPS.fix)  
//  {
    latitude = (double)GPS.latitude;
    lat_dir = (char)GPS.lat;
    longitude = (double)GPS.longitude;
    long_dir = (char)GPS.lon;
    _speed = GPS.speed*1.15;
    angle = (double)GPS.angle;
    altitude = (double)GPS.altitude;
    satellites = (int)GPS.satellites;
        
    // Serial transmissions.
    Serial.print("Location: ");
    Serial.print(latitude, 4); Serial.print(lat_dir);
    Serial.print(", "); 
    Serial.print(longitude, 4); Serial.println(long_dir);
    Serial.print("Speed (MPH): "); Serial.println(_speed);
    Serial.print("Angle: "); Serial.println(angle);
    Serial.print("Altitude: "); Serial.println(altitude);
    Serial.print("Satellites: "); Serial.println(satellites);
      
    // XBee transmissions.
    xbee.print("Location: ");
    xbee.print(latitude, 4); xbee.print(lat_dir);
    xbee.print(", "); 
    xbee.print(longitude, 4); xbee.println(long_dir);
    xbee.print("Speed (MPH): "); xbee.println(_speed);
    xbee.print("Angle: "); xbee.println(angle);
    xbee.print("Altitude: "); xbee.println(altitude);
    xbee.print("Satellites: "); xbee.println(satellites);
//  }

  // Appends the GPS data to the string to be written to the file on the microSD card.  
  
  dataString += month; dataString += '/';
  dataString += day; dataString += "/20";
  dataString += year;
  dataString += ",";
  dataString += hour; dataString += ':';
  dataString += minute; dataString += ':';
  dataString += seconds; dataString += '.';
  dataString += milliseconds;
  dataString += ",";
  
  for (uint8_t i = 0; i < 15; i++)  // Clear the buffer.
  {
     dtostrfbuffer[i] = 0;
  }
  dtostrf(latitude, 10, 4, dtostrfbuffer);  // dtostrf(variable, number of characters including comma + 1, number of characters after decimal point, buffer)
  dataString += dtostrfbuffer;
  dataString += lat_dir;
  dataString += ",";

  for (uint8_t i = 0; i < 15; i++)  // Clear the buffer.
  {
     dtostrfbuffer[i] = 0;
  }
  dtostrf(longitude, 10, 4, dtostrfbuffer);
  dataString += dtostrfbuffer;
  dataString += long_dir;
  dataString += ",";

  for (uint8_t i = 0; i < 15; i++)  // Clear the buffer.
  {
     dtostrfbuffer[i] = 0;
  }
  dtostrf(_speed, 6, 2, dtostrfbuffer);
  dataString += dtostrfbuffer;
  dataString += ",";

  // End of GPS code.

  // Battery temperature code.
  
  Vo = ads1115_no1.readADC_Differential_0_1();
  Rt1 = (Vo*1000)/(1023.0-Vo);  // Solves for Rt1 from the equation Vo = Vin * Rt1/(Rt1+R), where R = 1000 ohms, Rt1 is the resistance of the thermistor, and Vin = 3.3V = 1023.0. Voltages are scaled values from 0 to 1023.0, where 1023.0 = 3.3V for the Arduino Mega Pro 3.3V. Vo is converted into a double value before the division is perfomed to enable floating point math.
  // Performing the division as floating point numbers instead of integers prevents rounding and therefore preserves precision.
  T1 = (1.0/(A + B*log(Rt1) + C*pow(log(Rt1),3))) - 273.15;

  Vo = ads1115_no1.readADC_Differential_2_3();
  Rt2 = (double(Vo)*1000)/(1023.0-double(Vo));
  T2 = (1.0/(A + B*log(Rt2) + C*pow(log(Rt2),3))) - 273.15;
  
  Vo = ads1115_no2.readADC_Differential_0_1();
  Rt3 = (double(Vo)*1000)/(1023.0-double(Vo));
  T3 = (1.0/(A + B*log(Rt3) + C*pow(log(Rt3),3))) - 273.15;
  
  Vo = ads1115_no2.readADC_Differential_2_3();
  Rt4 = (double(Vo)*1000)/(1023.0-double(Vo));
  T4 = (1.0/(A + B*log(Rt4) + C*pow(log(Rt4),3))) - 273.15;
  
  Vtop = analogRead(6);
  Vbot = analogRead(7);
  Vo = Vtop - Vbot;
  Rt5 = (double(Vo)*1000)/(1023.0-double(Vo));
  T5 = (1.0/(A + B*log(Rt5) + C*pow(log(Rt5),3))) - 273.15;
  
  Vtop = analogRead(8);
  Vbot = analogRead(9);
  Vo = Vtop - Vbot;
  Rt6 = (double(Vo)*1000)/(1023.0-double(Vo));
  T6 = (1.0/(A + B*log(Rt6) + C*pow(log(Rt6),3))) - 273.15;
  
  // End of battery temperature code.
  
  // Steering wheel position code.
  
  Vo = analogRead(13);
  steering_pos = double(Vo)/1023.0;
  steering_angle = (steering_pos - 0.5)*180;
  // Steering wheel is turned to the right, for a steering angle of 0 to 90 degrees.
  // Steering wheel is in the center. Steering angle is 0 degrees.
  // Steering wheel is turned to the left, for a steering angle of -90 to 0 degrees.
  
  // End of steering wheel position code.
  
  // Accelerator pedal position code.
  
  // 4.18V at 100% throttle, 0.830V at 0% throttle
  // 4.18V at input = 3.15779V at voltage divider
  
  Vo = analogRead(15);
  accel_position = ((Vo*(3.3/1023.0))/3.15779)*100;
  
  // End of accelerator pedal position code.
  
  // Brake pedal position code.

  // The sensor is rated for 3,000 PSI, so 3.3V at the voltage divider corresponds to 3,000 PSI. 
  // brake pressure = (voltage - 0.5)/4 * 3000; range of 0.5V to 4.5V ratiometric
  // 0.520V = 15 PSI = 0% brake pedal position, 3.18V = 2,010 PSI = maximum system pressure = 100% brake pedal position
  // 4.5V at input = 3.40V at voltage divider = 3,000 PSI
  // 3.30V at voltage divider = 2,910.75 PSI
  // 3.18V at input = 2.40V at voltage divider = analogRead of 746 
  
  Vo = analogRead(14);
  brake_pressure = Vo*(3.3/1023.0)*(3000/3.4);
  brake_position = (brake_pressure/2010)*100;
  
  // End of brake pedal position code.
  
  // Battery voltage code.
  
  // Maximum readable voltage for battery #1 = 3.3/(10090/(10090+2178+32850)) = 14.75613479
  // Maximum readable voltage for battery #2 = 3.3/(10070/(10070+2178+32800)) = 14.76250248
  // Maximum readable voltage for battery #3 = 3.3/(10060/(10060+2183+32900)) = 14.80833996
  // Maximum readable voltage for battery #4 = 3.3/(10040/(10040+2176+32910)) = 14.832251
  // Maximum readable voltage for battery #5 = 3.3/(10040/(10040+2175+32870)) = 14.8187749
  // Maximum readable voltage for battery #6 = 3.3/(10080/(10090+2175+32910)) = 14.78616071
  
  Vo = analogRead(0);
  vbat_1 = Vo*(3.3/1023.0)*(14.75613479/3.3);

  Vo = analogRead(1);
  vbat_2 = Vo*(3.3/1023.0)*(14.76250248/3.3);  

  Vo = analogRead(2);
  vbat_3 = Vo*(3.3/1023.0)*(14.80833996/3.3);

  Vo = analogRead(3);
  vbat_4 = Vo*(3.3/1023.0)*(14.832251/3.3);

  Vo = analogRead(4);
  vbat_5 = Vo*(3.3/1023.0)*(14.8187749/3.3);

  Vo = analogRead(5);
  vbat_6 = Vo*(3.3/1023.0)*(14.78616071/3.3);

  // End of battery voltage code.
  
/*  
  // Pedal position code.
  
  results = ads1015.readADC_Differential_0_1();  // The currently installed potentiometer on the breadboard has results vary between 4100 and 3050.
  percentage = (((4100-results)/1050)*100);  // Scales the results reading to a percentage of the pedal position. 

  if (percentage <= 99.5)
  {
     Serial.print(percentage);
     Serial.println("%"); 
     xbee.print("Throttle: ") ;
     xbee.print(percentage);
     xbee.println("%");    
  }
  else if (percentage >= 99.5 && percentage <= 100)  // Account for error. The potentiometer doesn't otherwise show a reading of 100%.
  {
     Serial.print(100);
     Serial.println("%");
     xbee.print("Throttle: ") ;
     xbee.print(100);
     xbee.println("%");
  }
  else  // Account for error. The potentiometer gives strange readings when it is close to 0%.
  {
     Serial.println("%");
     xbee.print("Throttle: ") ;
     xbee.print(0);
     xbee.println("%");
  }
  
  // Appends the pedal position data to the string to be written to the file on the microSD card.

  for (uint8_t i = 0; i < 15; i++)  // Clear the buffer.
  {
     dtostrfbuffer[i] = 0;
  }
  dtostrf(percentage, 8, 2, dtostrfbuffer);
  dataString += dtostrfbuffer;
  dataString += ",";
  
  for (uint8_t i = 0; i < 15; i++)  // Clear the buffer, since this is the last piece of data being added in this session. Ensures that GPS location data will be null if no fix can be obtained.
  {
     dtostrfbuffer[i] = 0;
  }

  // End of potentiometer code.

*/

  // Thermocouple code.
  
  double thermocoupleTemperature = thermocouple.readFarenheit();
  xbee.print("Motor Temp: ");
  xbee.print(thermocoupleTemperature);
  xbee.println("F");
  xbee.println(" ");
  Serial.println(thermocoupleTemperature);
  
  // Appends the thermocouple data to the string to be written to the file on the microSD card.
  
  dtostrf(thermocoupleTemperature, 6, 2, dtostrfbuffer);
  dataString += dtostrfbuffer;
  for (uint8_t i = 0; i < 15; i++)  // Clear the buffer.
  {
     dtostrfbuffer[i] = 0;
  }
  
  delay(1000);
  // End of thermocouple code.
  
  // Open the file. Note that only one file can be open at a time, so you have to close this one before opening another.
  File logfile = SD.open(filename, FILE_WRITE);

  // If the file is available, write to it:
  if (logfile)
  {
    logfile.println(dataString);
    logfile.close();
    // Print to the serial port too:
    Serial.println(dataString);
    xbee.println(dataString);
    xbee.println();
  }
  // If the file isn't open, display an error:
  else
  {
    Serial.println("Error opening file.");
    xbee.println("Error opening file.");
    xbee.println();
  }

  // Pauses the reception and transmission of data for 90 seconds, so that the microSD card can be removed without corrupting any files.

  if(Serial.available() > 0)
  {
    myData = Serial.read();
    if(myData == '9')
    {
     Serial.print("Data Paused");
     delay(90000);
    }
   }  
}
