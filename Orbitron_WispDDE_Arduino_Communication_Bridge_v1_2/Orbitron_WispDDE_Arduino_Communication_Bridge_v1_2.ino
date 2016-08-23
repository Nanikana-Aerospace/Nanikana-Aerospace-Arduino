/*
Orbitron-WispDDE-Arduino Communication Bridge v1.0

JF Nadeau 22 feb 2015

www.nanikana-aerospace.org
*/
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include "ADXL345.h"
const float alpha = 0.5;

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
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" uT");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" uT");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" uT");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void setup() {
Serial.begin(9600);
Wire.begin();
acc.begin();
lcd.begin(16,2);   // initialize the lcd for 16 chars 2 lines, turn on backlight

// HMC5883 compass  Set operating mode to continuous
 // Wire.beginTransmission(Addr); 
 // Wire.write(byte(0x02));
 // Wire.write(byte(0x00));
 // Wire.endTransmission();
 // Serial.println("HMC5883 : Starting");
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("System :REBOOT");
  delay(3000);
lcd.backlight(); // finish with backlight on
// Display. (Set Serial Monitor option to "No Line Ending")
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("System: READY");  
delay(2000);
  lcd.setCursor(0,1); //Start at character 0 on line 0
  lcd.print("System: WAIT");
  delay(2000);
  lcd.clear();
   /* Initialise the sensor */
  if(!mag.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("HMC5883: FAIL ");
    while(1);
  }
  
  /* Display some basic information on this sensor */
  displaySensorDetails();
}
void loop() {
  
/* Get a new sensor event */ 
  sensors_event_t event; 
  mag.getEvent(&event);
 
  /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
  Serial.print("X: "); Serial.print(event.magnetic.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event.magnetic.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event.magnetic.z); Serial.print("  ");Serial.println("uT");

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
  
  Serial.print("Heading (degrees): "); Serial.println(headingDegrees);
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

int satazimuth = 0;
int satelevation = 0;
if (Serial.find("Z")) {
satazimuth = Serial.parseInt(); // parses numeric characters before the space
satelevation = Serial.parseInt();// parses numeric characters after the space&EL
// print the results:

Serial.print("SAT AZ: " );
Serial.print(satazimuth);
Serial.print ("  ANT AZ: "); Serial.println (headingDegrees);
Serial.print("SAT EL: ");
Serial.print(satelevation);
Serial.print ("  ANT EL: "); Serial.println (pitch);
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
