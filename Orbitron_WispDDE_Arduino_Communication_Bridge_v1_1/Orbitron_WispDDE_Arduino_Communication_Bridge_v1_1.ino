/*
Orbitron-WispDDE-Arduino Communication Bridge v1.0

JF Nadeau 22 feb 2015

www.nanikana-aerospace.org
*/
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#define Addr 0x1E               // 7-bit address of HMC5883 compass


void setup() {
Serial.begin(9600);
Wire.begin();

// HMC5883 compass  Set operating mode to continuous
  Wire.beginTransmission(Addr); 
  Wire.write(byte(0x02));
  Wire.write(byte(0x00));
  Wire.endTransmission();
  Serial.println("HMC5883 : Starting");


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
