/*
Orbitron-WispDDE-Arduino Communication Bridge v1.0

JF Nadeau 22 feb 2015

www.nanikana-aerospace.org
*/
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#define Addr 0x1E               // 7-bit address of HMC5883 compass
// set the LCD address to 0x27 for a 20 chars 4 line display
// Set the pins on the I2C chip used for LCD connections:
//                    addr, en,rw,rs,d4,d5,d6,d7,bl,blpol
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address


void setup() {
Serial.begin(9600);
Wire.begin();
lcd.begin(16,2);   // initialize the lcd for 16 chars 2 lines, turn on backlight

// HMC5883 compass  Set operating mode to continuous
  Wire.beginTransmission(Addr); 
  Wire.write(byte(0x02));
  Wire.write(byte(0x00));
  Wire.endTransmission();
  Serial.println("HMC5883 : Starting");
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
  
}
void loop() {
  
int xm, ym, zm;

  // Initiate communications with HMC5883 compass
  Wire.beginTransmission(Addr);
  Wire.write(byte(0x03));       // Send request to X MSB register
  Wire.endTransmission();

  Wire.requestFrom(Addr, 6);    // Request 6 bytes; 2 bytes per axis
  if(Wire.available() <=6) {    // If 6 bytes available
    xm = Wire.read() << 8 | Wire.read();
    zm = Wire.read() << 8 | Wire.read();
    ym = Wire.read() << 8 | Wire.read();
    //lcd.clear();
  }  

int satazimuth = 0;
int satelevation = 0;
if (Serial.find("Z")) {
satazimuth = Serial.parseInt(); // parses numeric characters before the space
satelevation = Serial.parseInt();// parses numeric characters after the space&EL
// print the results:

Serial.print("SAT AZ: " );
Serial.print(satazimuth);
Serial.print ("  ANT AZ: "); Serial.println (xm);
Serial.print("SAT EL: ");
Serial.print(satelevation);
Serial.print ("  ANT EL: "); Serial.println (ym);
lcd.setCursor(0,0);
lcd.print("SAT AZ");
lcd.setCursor(7,0);
  lcd.print(satazimuth);
lcd.setCursor(11,0);
lcd.print("EL");
lcd.setCursor(14,0);
  lcd.print(satelevation);
  lcd.setCursor(0,1);
lcd.print("ANT AZ");
lcd.setCursor(7,1);
  lcd.print(xm);
lcd.setCursor(11,1);
lcd.print("EL");
lcd.setCursor(14,1);
  lcd.print(ym);
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
