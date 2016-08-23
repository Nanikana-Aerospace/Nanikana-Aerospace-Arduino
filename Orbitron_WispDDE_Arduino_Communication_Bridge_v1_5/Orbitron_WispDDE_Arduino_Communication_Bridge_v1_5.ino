/*
Orbitron-WispDDE-Arduino Communication Bridge v1.5

JF Nadeau 24 avr 2015

www.nanikana-aerospace.org
*/
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include "ADXL345.h"
const float alpha = 0.5;
int activeant; 
int ledfix = 2; //Fix Antenna on Sat
int azimuthpotpin = A0;  //set pin for pot red value
int ledstart = 3; // Si elevation sat superieur a 0 degrée, ledstart On
int ledupel = 4; // Si elevation sat superieur a elevation ant, ledupel On
int leddwel = 5; // Si elevation sat inferieur a elevation ant, leddwel On
int ledupaz = 6; // Si azimuth sat superieur a azimuth, ant ledupaz On
int leddwaz = 7; // Si azimuth sat inferieur a azimuth, ant leddwaz On
int manualmode = 8; // selection Manual ou Auto mode.
int relayup = 9;
int relaydown = 10;
int relayleft = 11;
int relayright = 12;
int manualled = 13;// red led manual selection
int runtime;
int pitchint;
int headingint;

int satazimuth;
int satelevation;

unsigned long time; // Time for WatchDog DATA
unsigned long watchdogdata;

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
 
  delay(500);
}

void setup() {
Serial.begin(9600);
Wire.begin();
acc.begin();

pinMode (azimuthpotpin, INPUT); //init A0 input azimuth pot.

pinMode(ledfix, OUTPUT);  // initialize dig pin 2 Red
pinMode(ledstart, OUTPUT); // initialize the digital pin3 as an output. Blue LED
pinMode(ledupel, OUTPUT); // initialize the digital pin4 as an output. Yellow
pinMode(leddwel, OUTPUT); // initialize the digital pin5 as an output. Yellow
pinMode(ledupaz, OUTPUT); // initialize the digital pin6 as an output. Green
pinMode(leddwaz, OUTPUT); // initialize the digital pin7 as an output. Green
pinMode(manualmode, INPUT_PULLUP); // init dig pin 8 Manual/Auto Mode
pinMode(relayup, OUTPUT); // init D9 relay
pinMode(relaydown, OUTPUT); // init D10 relay
pinMode(relayleft, OUTPUT); // init D11 relay
pinMode(relayright, OUTPUT); // init D12 relay
pinMode(manualled, OUTPUT); //init dig pin 13 out red led manual running
digitalWrite(ledupel, LOW);
digitalWrite(relayup, HIGH);
digitalWrite(leddwel, LOW);
digitalWrite(relaydown, HIGH);
digitalWrite(ledupaz, LOW);
digitalWrite(relayleft, HIGH);
digitalWrite(leddwaz, LOW);
digitalWrite(relayright, HIGH);
digitalWrite(manualled, LOW);

lcd.begin(16,2);   // initialize the lcd for 16 chars 2 lines, turn on backlight

  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("NKA TRACK v1.5");
  delay(1000);
  lcd.setCursor(0,0);
  lcd.print("System: REBOOT.");
  Serial.println("System: REBOOT.");
  delay(2000);
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
  delay(1000);
  lcd.clear();}
  else
  {
    lcd.setCursor(0,1); //Start at character 0 on line 0
    lcd.print("Waiting for DATA");
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
  watchdogdata = time;
  int manualmodesensor = digitalRead(manualmode);
 
 if (manualmodesensor == LOW) { 
  digitalWrite(manualled, HIGH); 
  //lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("MANUAL MODE ACTIVE");
}
 else { digitalWrite(manualled, LOW);}
  
 runtime = runtime + 1; 
 //Serial.println (runtime); DEBUG MODE
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
//int activeant ;

int azimuthpot = analogRead(azimuthpotpin);
//int satazimuth;
//int satelevation;
if (Serial.find("Z")) {
satazimuth = Serial.parseInt(); // parses numeric characters before the space
satelevation = Serial.parseInt();// parses numeric characters after the space&EL
}
if (Serial.find("STOP")) {
    satelevation = 0;
    satazimuth = 0; 
    lcd.setCursor(0,0); 
    lcd.print("**STOP COMMAND**"); 
    lcd.setCursor(0,1); 
    lcd.print("*WAIT: RESET DATA*");
    digitalWrite(ledfix, LOW);
    digitalWrite(leddwaz, LOW);
    digitalWrite(ledupaz, LOW);
    digitalWrite(ledupel, LOW);
    digitalWrite(leddwel, LOW);
    digitalWrite(ledstart, LOW);
    digitalWrite(manualled, LOW);
    digitalWrite(relayup, HIGH); 
    digitalWrite(relaydown, HIGH); 
    digitalWrite(relayleft, HIGH); 
    digitalWrite(relayright, HIGH); 
    satelevation ==0;
    satazimuth ==0;
    Serial.flush();
    delay(10000);
    lcd.clear();
}
if (Serial.find("PARK")) {
    satelevation = 360;
    satazimuth = 85; 
    lcd.setCursor(0,0); 
    lcd.print("**PARK COMMAND**"); 
    lcd.setCursor(0,1); 
    lcd.print("*WAIT: RESET DATA*");
    digitalWrite(ledfix, LOW);
    digitalWrite(leddwaz, LOW);
    digitalWrite(ledupaz, LOW);
    digitalWrite(ledupel, LOW);
    digitalWrite(leddwel, LOW);
    digitalWrite(ledstart, LOW);
    digitalWrite(manualled, LOW);
    digitalWrite(relayup, HIGH); 
    digitalWrite(relaydown, HIGH); 
    digitalWrite(relayleft, HIGH); 
    digitalWrite(relayright, HIGH); 
    satelevation ==360;
    satazimuth ==85;
    //Serial.flush();
    //delay(10000);
    //lcd.clear();
}
// print the results:
pitchint = pitch; //resultat en int
headingint = headingDegrees; //resultat en int
headingint = headingint - (headingint % 5); //arrondir a 5-
//pitchint = pitchint * -1; Remove comment if Revese Pitch
pitchint = pitchint -(pitchint % 5); //arrondir a 5-
Serial.print("AZ");
Serial.print(satazimuth);
Serial.print("EL");
Serial.print(satelevation);
Serial.print("ANTAZ");
Serial.print(headingint);
Serial.print("ANTEL");
Serial.print(pitchint);
Serial.print(" ");
Serial.print("POT");
Serial.print (azimuthpot);
 
if (satelevation > 0) { digitalWrite(ledstart, HIGH); Serial.print ("ACT"); Serial.print ("1"); }// turn ledstart on
else { digitalWrite(ledstart, LOW); Serial.print ("STBY"); Serial.print("0");}
Serial.print("MOV");


if (satelevation > pitchint && satelevation > 0 && satelevation < 81) { digitalWrite(ledupel, HIGH); digitalWrite(relayup, LOW); Serial.print (""); Serial.print ("8"); }  // up antenna
else { digitalWrite(ledupel, LOW); digitalWrite(relayup, HIGH); Serial.print ("1"); }

if (satelevation < pitchint && satelevation > 0) { digitalWrite(leddwel, HIGH); digitalWrite(relaydown, LOW); Serial.print (""); Serial.print ("8");}  // down antenna
else { digitalWrite(leddwel, LOW);  digitalWrite(relaydown, HIGH); Serial.print ("1");}

if (satazimuth > headingint && satelevation > 0) { digitalWrite(ledupaz, HIGH); digitalWrite(relayleft, LOW); Serial.print (""); Serial.print ("8");}  // turn left ant
else { digitalWrite(ledupaz, LOW);  digitalWrite(relayleft, HIGH); Serial.print ("1");}

if (satazimuth < headingint && satelevation > 0) { digitalWrite(leddwaz, HIGH); digitalWrite(relayright, LOW); Serial.print (""); Serial.print ("8");}  // turn right ant
else { digitalWrite(leddwaz, LOW);  digitalWrite(relayright, HIGH); Serial.print ("1");}

if (satazimuth == headingint && satelevation == pitchint){ digitalWrite(ledfix, HIGH); Serial.print ("8");}  // fix
else { digitalWrite(ledfix, LOW); Serial.print ("0");  }

Serial.println("");

 
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
  lcd.print(headingint);
lcd.setCursor(11,1);
lcd.print("EL   ");
lcd.setCursor(14,1);
  lcd.print(pitchint);
 
 if (runtime > 10) { lcd.clear();  runtime = 0; } //refresh lcd
//}
delay(500);
// IF SERIAL TRACKING AND MANUAL MODE ENABLE :
//  ...A TRAVAILLER ..... WORK IN PROGRESS

// STOP IF NO NEW SERIAL DATA FROM ORBITRON OR MANUAL
if (Serial.available() == 0 && manualmodesensor == HIGH) 
  {
    digitalWrite(ledfix, LOW);
    digitalWrite(leddwaz, LOW);
    digitalWrite(ledupaz, LOW);
    digitalWrite(ledupel, LOW);
    digitalWrite(leddwel, LOW);
    digitalWrite(ledstart, LOW);
    digitalWrite(manualled, LOW);
    digitalWrite(relayup, HIGH); 
    digitalWrite(relaydown, HIGH); 
    digitalWrite(relayleft, HIGH); 
    digitalWrite(relayright, HIGH); 
    int satazimuth = 0;
    int satelevation = 0;
    lcd.clear();
    lcd.setCursor(0,1); //Start at character 0 on line 0
    lcd.print("NO DATA");
    //Serial.println ("Waiting Data"); //DEBUG MODE
    while (!Serial.available());  //Stop until DATA receive from Orbitron-DDE
    }
  else
  {
    //lcd.clear();
    //lcd.setCursor(0,1); //Start at character 0 on line 0
    //lcd.print("DATA ERROR");
    
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
