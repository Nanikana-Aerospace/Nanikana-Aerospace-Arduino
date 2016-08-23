/*
Orbitron-WispDDE-Arduino Communication Bridge v1.3a

JF Nadeau 18 avr 2015

www.nanikana-aerospace.org
*/
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include "ADXL345.h"
const float alpha = 0.5;
int activeant; 
int ledstart = 3; // Si elevation sat superieur a 0 degrée, ledstart On
int ledupel = 4; // Si elevation sat superieur a elevation ant, ledupel On
int leddwel = 5; // Si elevation sat inferieur a elevation ant, leddwel On
int ledupaz = 6; // Si azimuth sat superieur a azimuth, ant ledupaz On
int leddwaz = 7; // Si azimuth sat inferieur a azimuth, ant leddwaz On
int ledfix = 2; //Fix Antenna on Sat

int pitchint;
int headingint;

double fXg = 0;
double fYg = 0;
double fZg = 0;

ADXL345 acc;

/* Assign a unique ID to this sensor at the same time */
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
#define Addr 0x1E               // 7-bit address of HMC5883 compass
// set the LCD address to 0x27 for a 20 chars 4 line display
// Set the pins on the I2C chip used for LCD connections:
//                    addr, en,rw,rs,d4,d5,d6,d7,bl,blpol
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address

void displaySensorDetails(void)
{
  sensor_t sensor;
  mag.getSensor(&sensor);
 // Serial.println("------------------------------------");
  //Serial.print  ("Sensor:       "); Serial.println(sensor.name);
 // Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
 // Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
 // Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" uT");
 // Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" uT");
 // Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" uT");  
 // Serial.println("------------------------------------");
  //Serial.println("");
  delay(500);
}

void setup() {
Serial.begin(9600);
Wire.begin();
acc.begin();

pinMode(ledstart, OUTPUT); // initialize the digital pin3 as an output. Blue LED
pinMode(ledupel, OUTPUT); // initialize the digital pin4 as an output. Yellow
pinMode(leddwel, OUTPUT); // initialize the digital pin5 as an output. Yellow
pinMode(ledupaz, OUTPUT); // initialize the digital pin6 as an output. Green
pinMode(leddwaz, OUTPUT); // initialize the digital pin7 as an output. Green
pinMode(ledfix, OUTPUT);  // initialize dig pin 2 Red 


lcd.begin(16,2);   // initialize the lcd for 16 chars 2 lines, turn on backlight

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("NKA TRACK v1.3b");
  delay(3000);
  lcd.setCursor(0,0);
  lcd.print("System :REBOOT.");
  delay(3000);
lcd.backlight(); // finish with backlight on
// Display. (Set Serial Monitor option to "No Line Ending")
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("System: READY");

if (Serial.available() > 0) 
  {
  delay(2000);
  lcd.setCursor(0,1); //Start at character 0 on line 0
  lcd.print("Processing...");
  Serial.println ("Processing Data...");
  delay(2000);
  lcd.clear();}
  else
  {
    lcd.setCursor(0,1); //Start at character 0 on line 0
    lcd.print("Waiting...");
    Serial.println ("Waiting Data");
    while (!Serial.available());  //Stop until DATA receive from Orbitron-DDE
   } 
   /* Initialise the sensor */
  if(!mag.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("HMC5883: FAIL ");
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("System: ERROR");
    while(1);
  }
  Serial.println (" Nanikana-Aerospace Antenna Tracking ");
  Serial.println (" SAT AZ    SAT EL   ANT AZ   ANT EL  STATUS   MOVE");
  Serial.println ("--------------------------------------------------------");

  /* Display some basic information on this sensor */
  displaySensorDetails();
}
void loop() {
  int runtime;
 runtime = runtime +1; 
/* Get a new sensor event */ 
  sensors_event_t event; 
  mag.getEvent(&event);
 
  /* Display the results (magnetic vector values are in micro-Tesla (uT)) */

  // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float heading = atan2(event.magnetic.y, event.magnetic.x);
  
  // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
  // Find yours here: http://www.magnetic-declination.com/
  // Mine is: (-12° 59') in radians W, which is ~13 Degrees, or (which we need) 192.3 radians
  // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
  float declinationAngle = 192.3/1000;
  heading -= declinationAngle;
  
  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
    
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
   
  // Convert radians to degrees for readability.
  float headingDegrees = heading * 180/M_PI; 
  
  //Serial.print("Heading (degrees): "); Serial.println(headingDegrees);
  double pitch, roll, Xg, Yg, Zg;
	acc.read(&Xg, &Yg, &Zg);

	//Low Pass Filter to smooth out data
	fXg = Xg * alpha + (fXg * (1.0 - alpha));
	fYg = Yg * alpha + (fYg * (1.0 - alpha));
	fZg = Zg * alpha + (fZg * (1.0 - alpha));

	//Roll and Pitch Equations
	roll  = (atan2(-fYg, fZg)*180.0)/M_PI;
	pitch = (atan2(fXg, sqrt(fYg*fYg + fZg*fZg))*180.0)/M_PI;

	//Serial.print(pitch);
//int activeant = 0;
int satazimuth = 0;
int satelevation = 0;
if (Serial.find("Z")) {
satazimuth = Serial.parseInt(); // parses numeric characters before the space
satelevation = Serial.parseInt();// parses numeric characters after the space&EL
// print the results:
Serial.print("  ");
Serial.print(satazimuth);
Serial.print("       ");
Serial.print(satelevation);
Serial.print("       ");
Serial.print(headingDegrees, 0);
Serial.print("     ");
Serial.print(pitch, 0);
Serial.print("     ");
 
if (satelevation > 0) { digitalWrite(ledstart, HIGH); Serial.print ("Active "); }// turn ledstart on
else { digitalWrite(ledstart, LOW); Serial.print ("Standby");}

pitchint = pitch;
headingint = headingDegrees;

if (satelevation > pitchint && satelevation > 0 ) { digitalWrite(ledupel, HIGH); Serial.print ("UP "); }  // up antenna
else { digitalWrite(ledupel, LOW);  }

if (satelevation < pitchint && satelevation > 0) { digitalWrite(leddwel, HIGH); Serial.print ("DW "); }  // down antenna
else { digitalWrite(leddwel, LOW);  }

if (satazimuth > headingint && satelevation > 0) { digitalWrite(ledupaz, HIGH); Serial.print ("R "); }  // turn left ant
else { digitalWrite(ledupaz, LOW);  }

if (satazimuth < headingint && satelevation > 0) { digitalWrite(leddwaz, HIGH); Serial.print ("L "); }  // turn right ant
else { digitalWrite(leddwaz, LOW);  }

if (satazimuth == headingint && satelevation == pitchint){ digitalWrite(ledfix, HIGH); Serial.print ("FIX "); }  // fix
else { digitalWrite(ledfix, LOW);  }

Serial.println("");
if (Serial.available() > 0) 
  {
    //Serial.println ("PROCESSING DATA...");} //DEBUG MODE
  else
  {
    lcd.clear();
    lcd.setCursor(0,1); //Start at character 0 on line 0
    lcd.print("NO DATA");
    //Serial.println ("Waiting Data"); //DEBUG MODE
    while (!Serial.available());  //Stop until DATA receive from Orbitron-DDE
   }  
 
//Serial.print(cor, 0);

lcd.setCursor(0,0);
lcd.print("SAT AZ");
lcd.setCursor(7,0);
  lcd.print(satazimuth);
lcd.setCursor(11,0);
lcd.print("EL   ");
lcd.setCursor(14,0);
  lcd.print(satelevation);
  lcd.setCursor(0,1);
lcd.print("ANT AZ");
lcd.setCursor(7,1);
  lcd.print(headingDegrees, 0);
lcd.setCursor(11,1);
lcd.print("EL   ");
lcd.setCursor(14,1);
  lcd.print(pitch, 0);
 // lcd.clear();
 if (runtime == 10) { lcd.clear();  runtime == 0; } //refresh lcd
}
}
void writeRegister(int deviceAddress, byte address, byte val) {
    Wire.beginTransmission(deviceAddress); // start transmission to device 
    Wire.write(address);       // send register address
    Wire.write(val);         // send value to write
    Wire.endTransmission();     // end transmission
}

int readRegister(int deviceAddress, byte address){

    int v;
    Wire.beginTransmission(deviceAddress);
    Wire.write(address); // register to read
    Wire.endTransmission();

    Wire.requestFrom(deviceAddress, 1); // read a byte

    while(!Wire.available()) {
        // waiting
    }

    v = Wire.read();
    return v;
}
