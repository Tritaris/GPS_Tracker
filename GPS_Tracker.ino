#include <SD.h>
#include <SPI.h>
#include <Wire.h>
#include <TinyGPS++.h>
#include <ADXL345.h>
#include <HMC5883L.h>

// Set the pins used
#define cardSelect 4
#define VBATPIN A7

#define dwellSeconds 1

static const uint32_t GPSBaud = 57600;

unsigned long trackPointNum = 1;

// The TinyGPS++ object
TinyGPSPlus gps;

HMC5883L compass;
ADXL345 accelerometer;

File logfile;

float heading1;
float heading2;

float BatteryLevel(){
  float measuredvbat = analogRead(VBATPIN);
  measuredvbat *= 2;    // we divided by 2, so multiply back
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage

  return measuredvbat;
  //Serial.print("VBat: " ); Serial.println(measuredvbat);
}

// No tilt compensation
float noTiltCompensate(Vector mag){
  float heading = atan2(mag.YAxis, mag.XAxis);
  return heading;
}

// Tilt compensation
float tiltCompensate(Vector mag, Vector normAccel){
  // Pitch & Roll 

  float roll;
  float pitch;

  roll = asin(normAccel.YAxis);
  pitch = asin(-normAccel.XAxis);

  if (roll > 0.78 || roll < -0.78 || pitch > 0.78 || pitch < -0.78){
    return -1000;
  }

  // Some of these are used twice, so rather than computing them twice in the algorithem we precompute them before hand.
  float cosRoll = cos(roll);
  float sinRoll = sin(roll);  
  float cosPitch = cos(pitch);
  float sinPitch = sin(pitch);

  // Tilt compensation
  float Xh = mag.XAxis * cosPitch + mag.ZAxis * sinPitch;
  float Yh = mag.XAxis * sinRoll * sinPitch + mag.YAxis * cosRoll - mag.ZAxis * sinRoll * cosPitch;

  float heading = atan2(Yh, Xh);

  return heading;
}

float compassHeading(){
  // Read vectors
  Vector mag = compass.readNormalize();
  Vector acc = accelerometer.readScaled();

  // Calculate headings
  heading1 = noTiltCompensate(mag);
  heading2 = tiltCompensate(mag, acc);

  if (heading2 == -1000)
  {
    heading2 = heading1;
  }

  // Set declination angle on your location and fix heading
  // You can find your declination on: http://magnetic-declination.com/
  // (+) Positive or (-) for negative
  // For Bytom / Poland declination angle is 4'26E (positive)
  // Formula: (deg + (min / 60.0)) / (180 / M_PI);
  float declinationAngle = (-7.0 + (59.0 / 60.0)) / (180 / M_PI);
  heading1 += declinationAngle;
  heading2 += declinationAngle;

  // Correct for heading < 0deg and heading > 360deg
  heading1 = correctAngle(heading1);
  heading2 = correctAngle(heading2);

  // Convert to degrees
  heading1 = heading1 * 180/M_PI; 
  heading2 = heading2 * 180/M_PI;

  return heading2;
}

// Correct angle
float correctAngle(float heading)
{
  if (heading < 0) { heading += 2 * PI; }
  if (heading > 2 * PI) { heading -= 2 * PI; }

  return heading;
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

void setupAccel(){
  // Initialize ADXL345
  Serial.println(F("Initialize ADXL345"));
  while (!accelerometer.begin())
  {
    Serial.println(F("Could not find a valid ADXL345 sensor, check wiring!"));
    delay(500);
  }

  accelerometer.setRange(ADXL345_RANGE_2G);
}

void setupMag(){
  // Initialize HMC5883L
  Serial.println("Initialize HMC5883L");
  while (!compass.begin())
  {
    Serial.println(F("Could not find a valid HMC5883L sensor, check wiring!"));
    delay(500);
  }

  //Set measurement range
  compass.setRange(HMC5883L_RANGE_1_3GA);

  //Set measurement mode
  compass.setMeasurementMode(HMC5883L_CONTINOUS);

  //Set data rate
  compass.setDataRate(HMC5883L_DATARATE_30HZ);

  //Set number of samples averaged
  compass.setSamples(HMC5883L_SAMPLES_8);

  //Set calibration offset. See HMC5883L_calibration.ino
  compass.setOffset(0, 0); 
}

void setupSD(){
  //See if the card is present and can be initialized:
  if (!SD.begin(cardSelect)) {
    Serial.println(F("Card init. failed!"));
    error(2);
    }
   
  char filename[15];
  strcpy(filename, "GPSLOG00.TXT");
    
  for (uint8_t i = 0; i < 100; i++) {
    filename[6] = '0' + i/10;
    filename[7] = '0' + i%10;
    // create if does not exist, do not open existing, write, sync after write
    if (! SD.exists(filename)) {
      break;
    }
  }

  logfile = SD.open(filename, FILE_WRITE);
  
  if( ! logfile ) {
    Serial.print(F("Couldnt create ")); 
    Serial.println(filename);
    error(3);
  }

  Serial.print(F("Writing to ")); 
  Serial.println(filename);

  //Write header to file
  logfile.println("trackpoint,time,longitude,latitude,elevation,heading,speed,batlvl");
}

void setup() {
	//Connect at 115200 so we can read the GPS fast enough and echo without dropping chars
	//also spit it out
	Serial.begin(115200);
	Serial.println(F("\r\nGPS Track Logger"));
	
	//connect at GPSBaud to the GPS
  Serial1.begin(GPSBaud);
 
	pinMode(13, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(A7, INPUT);	
 
	setupAccel();
	setupMag();
	setupSD();	
	
	Serial.println(F("Ready!"));
}

void loop() {
	while (Serial1.available() > 0)
		gps.encode(Serial1.read());   

  //If GPS has fix then write data to SD card.
	if (gps.location.isValid()){
	  digitalWrite(8, HIGH);//Turn Green LED ON to indicate writing to SD
    
    //Trackpoint Number
    logfile.print(trackPointNum++); logfile.print(F(","));

    //Date & Time in GMT time zone
    logfile.print(gps.date.year()); logfile.print(F("-"));
    logfile.print(gps.date.month()); logfile.print(F("-"));
    logfile.print(gps.date.day()); logfile.print(F(" "));
    
    if (gps.time.hour() < 10) logfile.print(F("0"));
    logfile.print(gps.time.hour());
    logfile.print(F(":"));
    if (gps.time.minute() < 10) logfile.print(F("0"));
    logfile.print(gps.time.minute());
    logfile.print(F(":"));
    if (gps.time.second() < 10) logfile.print(F("0"));
    logfile.print(gps.time.second()); logfile.print(F(","));

    //Longitude
		logfile.print(gps.location.lng(), 6); logfile.print(F(","));
    
    //Latitude
    logfile.print(gps.location.lat(), 6); logfile.print(F(","));

    //Elevation above sea level in meters
		logfile.print(gps.altitude.meters()); logfile.print(F(","));
   
    //Compass Heading in degrees  (0 - 360)
    logfile.print(compassHeading()); logfile.print(F(","));

    //Speed in kmph
    logfile.print(gps.speed.kmph()); logfile.print(F(","));

    //Battery Level
    logfile.print(BatteryLevel()); logfile.println(F(" "));

    //Force write to SD card
		logfile.flush();
    
		digitalWrite(8, LOW); //Turn Green LED OFF
	}

  //If connected to the computer output the data to the Serial Console
	if(Serial){
    //Trackpoint Number
    Serial.print(trackPointNum); Serial.print(F(","));

    //Date & Time in GMT time zone
    Serial.print(gps.date.year()); Serial.print(F("-"));
    Serial.print(gps.date.month()); Serial.print(F("-"));
    Serial.print(gps.date.day()); Serial.print(F(" "));
    
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second()); Serial.print(F(","));

    //Longitude
    Serial.print(gps.location.lng(), 6); Serial.print(F(","));
    
    //Latitude
    Serial.print(gps.location.lat(), 6); Serial.print(F(","));

    //Elevation above sea level in meters
    Serial.print(gps.altitude.meters()); Serial.print(F(","));
   
    //Compass Heading in degrees  (0 - 360)
    Serial.print(compassHeading()); Serial.print(F(","));

    //Speed in kmph
    Serial.print(gps.speed.kmph()); Serial.println(F(" "));
	}
     
	delay(dwellSeconds * 1000);
}


