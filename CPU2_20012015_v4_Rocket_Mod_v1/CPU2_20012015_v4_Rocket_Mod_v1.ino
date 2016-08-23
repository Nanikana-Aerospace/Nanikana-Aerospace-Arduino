#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <SFE_BMP180.h>
#include <Wire.h>
/*
********Projet Otachi-Arduino par Nanikana-Aerospace*********
Sonde stratosphérique avec Arduino Uno R3  + open log + Ublox 6 + Gy-80 10DOF
Code basé sur des sketchs sous licence Open Source. TinyGPS++ library by Mikal Hart
ML8511 UV Sensor Read Example By: Nathan Seidle
Assemblage et programmation par Josianne Gilbert et JF Nadeau
Plus d'info sur le site web- More informations on web site. 
         www.nanikana-aerospace.org
***  Otachi Flight COMP Prog. v.20012015-D  (Rocket Mod)***
 For informations : info@nanikana-aerospace.org
 
 Ublox6 GPS
 RX = D11
 TX = D12
 
 GY-80
 A4
 A5
 
 OpenLog/Radio TX
 D1 (TX) 
*/
#define DEVICE (0x53)    //ADXL345 device address
#define TO_READ (6)        //num of bytes we are going to read each time (two bytes for each axis)

#define Addr 0x1E               // 7-bit address of HMC5883 compass

#define CTRL_REG1 0x20 //
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
#define CTRL_REG5 0x24

int L3G4200D_Address = 105; //I2C address of the L3G4200D

int xg; //xg yg zg = 3 Axis from Gyroscope
int yg;
int zg;


byte buff[TO_READ] ;    //6 bytes buffer for saving data read from the device
char str[512];                      //string buffer to transform data before sending it to the serial port

int incomingByte = 0;   // for incoming serial data


static const int RXPin = 11, TXPin = 12;
static const uint32_t GPSBaud = 9600;

//Uno, Redboard, Pro:        A4   A5 BMP180
SFE_BMP180 pressure;

double baseline; // baseline pressure

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

unsigned long time; // variable pour definir le running time du prog.
#define ALTITUDE 307.0 // Altitude of NKA HQ in Valdor in meters

long readVcc() {
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1126400L / result; // Back-calculate AVcc in mV
  return result/10;
}

void setup()
{
  Wire.begin();        // join i2c bus (address optional for master)
  
  Serial.begin(9600);
  ss.begin(GPSBaud);
  Serial.println("Otachi Flight COMP Prog. v.20012015-D (Rocket Mod)");
  Serial.println("Nanikana-Aerospace 2015");
  Serial.println("Booting.....");
  
  Serial.println("L3G4200D : Starting");
  setupL3G4200D(2000); // Configure L3G4200  - 250, 500 or 2000 deg/sec
  
  // HMC5883 compass  Set operating mode to continuous
  Wire.beginTransmission(Addr); 
  Wire.write(byte(0x02));
  Wire.write(byte(0x00));
  Wire.endTransmission();
  Serial.println("HMC5883 : Starting");
  
  
  //Turning on the ADXL345
  writeTo(DEVICE, 0x2D, 0);      
  writeTo(DEVICE, 0x2D, 16);
  writeTo(DEVICE, 0x2D, 8);
  Serial.println("ADXL345 : Starting");
  Serial.println("BMP085  : Starting");
  smartDelay(5000);
// Get the baseline pressure:
  if (pressure.begin())
    Serial.println("BMP180 : Ready");
  else
  {
    // Oops, something went wrong, this is usually a connection problem,
    // see the comments at the top of this sketch for the proper connections.

    Serial.println("BMP180 : init fail");
    while(1); // Pause forever.
  }

  Serial.println();
  Serial.println("All system READY");
}

void loop()
{
  static const double LONDON_LAT = 51.508131, LONDON_LON = -0.128002;
  char status;
  double T,P,p0,a;
  
  Serial.print("");
  time = millis(); //prints time since program started
  time = time /1000;
  Serial.print ("RUNTIME: ");
  Serial.println(time); //Serial.print ("");
  Serial.print ("{TIMEPLOT:DATA2|DATA|FIX|T|");
  printInt(gps.satellites.value(), gps.satellites.isValid(), 5); Serial.println ("}");
  
  Serial.print ("{MAP|SET|NKA01|");
  
  printFloat(gps.location.lat(), gps.location.isValid(), 11, 6); Serial.print ("|");
  printFloat(gps.location.lng(), gps.location.isValid(), 12, 6); Serial.println ("}"); 
  //printDateTime(gps.date, gps.time); Serial.print (" , ");
  
  Serial.print ("{TIMEPLOT:DATA1|DATA|ALTG|T|");
  printFloat(gps.altitude.meters(), gps.altitude.isValid(), 7, 2); Serial.println ("}");
  
  Serial.print ("{TIMEPLOT:DATA2|DATA|KMH|T|");
  printFloat(gps.speed.kmph(), gps.speed.isValid(), 6, 2); Serial.println ("}");
    //Serial.print ("GPS CARD: ");
  //printStr(gps.course.isValid() ? TinyGPSPlus::cardinal(gps.course.value()) : "*** ", 6); Serial.println ("");

// You must first get a temperature measurement to perform a pressure reading.
  
  // Start a temperature measurement:
  // If request is successful, the number of ms to wait is returned.
  // If request is unsuccessful, 0 is returned.

  status = pressure.startTemperature();
  if (status != 0)
  {
    // Wait for the measurement to complete:
    delay(status);

    // Retrieve the completed temperature measurement:
    // Note that the measurement is stored in the variable T.
    // Function returns 1 if successful, 0 if failure.

    status = pressure.getTemperature(T);
    if (status != 0)
    {
      // Print out the measurement:
      Serial.print("{TIMEPLOT:DATA2|DATA|TEMP|T|");
      Serial.print(T,2);
      Serial.println("}");
      //Serial.print((9.0/5.0)*T+32.0,2);
      //Serial.println(" deg F");
      
      // Start a pressure measurement:
      // The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
      // If request is successful, the number of ms to wait is returned.
      // If request is unsuccessful, 0 is returned.

      status = pressure.startPressure(3);
      if (status != 0)
      {
        // Wait for the measurement to complete:
        delay(status);

        // Retrieve the completed pressure measurement:
        // Note that the measurement is stored in the variable P.
        // Note also that the function requires the previous temperature measurement (T).
        // (If temperature is stable, you can do one temperature measurement for a number of pressure measurements.)
        // Function returns 1 if successful, 0 if failure.

        status = pressure.getPressure(P,T);
        if (status != 0)
        {
          // Print out the measurement:
          Serial.print("{TIMEPLOT:DATA1|DATA|PRESS|T|");
          Serial.print(P,2);
          Serial.println("}");
         // Serial.print(P*0.0295333727,2);
         // Serial.println(" inHg");

          // The pressure sensor returns abolute pressure, which varies with altitude.
          // To remove the effects of altitude, use the sealevel function and your current altitude.
          // This number is commonly used in weather reports.
          // Parameters: P = absolute pressure in mb, ALTITUDE = current altitude in m.
          // Result: p0 = sea-level compensated pressure in mb

          p0 = pressure.sealevel(P,ALTITUDE); // we're at 307 meters (Valdor, Qc)
          //Serial.print("rel.pressure: ");
          //Serial.print(p0,2);
          //Serial.print(" mb, ");
          //Serial.print(p0*0.0295333727,2);
          //Serial.println(" inHg");

          // On the other hand, if you want to determine your altitude from the pressure reading,
          // use the altitude function along with a baseline pressure (sea-level or other).
          // Parameters: P = absolute pressure in mb, p0 = baseline pressure in mb.
          // Result: a = altitude in m.

          a = pressure.altitude(P,p0);
          Serial.print("{TIMEPLOT:DATA1|DATA|ALTB|T|");
          Serial.print(a,0);
          Serial.println("}");
          //Serial.print(a*3.28084,0);
          //Serial.println(" feet");
        }
        else Serial.println("error retrieving pressure measurement\n");
      }
      else Serial.println("error starting pressure measurement\n");
    }
    else Serial.println("error retrieving temperature measurement\n");
  }
  else Serial.println("error starting temperature measurement\n");

  Serial.print("{TIMEPLOT:DATA1|DATA|Vin|T|");
  Serial.print( readVcc(), DEC ); Serial.print("}");
  Serial.write(10);
  
  int regAddress = 0x32;    //first axis-acceleration-data register on the ADXL345
  int xa, ya, za;
  
  readFrom(DEVICE, regAddress, TO_READ, buff); //read the acceleration data from the ADXL345
  
   //each axis reading comes in 10 bit resolution, ie 2 bytes.  Least Significat Byte first!!
   //thus we are converting both bytes in to one int
  xa = (((int)buff[1]) << 8) | buff[0];   
  ya = (((int)buff[3])<< 8) | buff[2];
  za = (((int)buff[5]) << 8) | buff[4];
   //we send the x y z values as a string to the serial port
  //sprintf(str, "%d %d %d", x, y, z);  
  //Serial.print(str);
  Serial.print ("{TIMEPLOT:ACC|DATA|Xa|T|");
  Serial.print (xa-19); Serial.println("}");
  Serial.print("{TIMEPLOT:ACC|DATA|Ya|T|");
  Serial.print (ya+250); Serial.println("}");
  Serial.print("{TIMEPLOT:ACC|DATA|Za|T|");
  Serial.print (za); Serial.println("}");
  
  getGyroValues();  // This will update xg, yg, and zg with new values

Serial.print ("{TIMEPLOT:ACC|DATA|Xg|T|");
  Serial.print (xg/10); Serial.println("}");
  Serial.print("{TIMEPLOT:ACC|DATA|Yg|T|");
  Serial.print (yg/10); Serial.println("}");
  Serial.print("{TIMEPLOT:ACC|DATA|Zg|T|");
  Serial.print (zg/10); Serial.println("}");

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
  
  // Print raw values of HMC5883
  
  Serial.print ("{TIMEPLOT:ACC|DATA|Xm|T|");
  Serial.print (xm-160); Serial.println("}");
  Serial.print("{TIMEPLOT:ACC|DATA|Ym|T|");
  Serial.print (ym-550); Serial.println("}");
  Serial.print("{TIMEPLOT:ACC|DATA|Zm|T|");
  Serial.print (zm+210); Serial.println("}");
  
 


    
Serial.println("");
  smartDelay(125);

  if (millis() > 5000 && gps.charsProcessed() < 10)
    Serial.println(F("No GPS data received"));
}

// This custom version of delay() ensures that the gps object
// is being "fed".
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}

static void printFloat(float val, bool valid, int len, int prec)
{
  if (!valid)
  {
    while (len-- > 1)
      Serial.print('*');
    Serial.print(' ');
  }
  else
  {
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i=flen; i<len; ++i)
      Serial.print(' ');
  }
  smartDelay(0);
}

static void printInt(unsigned long val, bool valid, int len)
{
  char sz[32] = "*****************";
  if (valid)
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i=strlen(sz); i<len; ++i)
    sz[i] = ' ';
  if (len > 0) 
    sz[len-1] = ' ';
  Serial.print(sz);
  smartDelay(0);
}


static void printDateTime(TinyGPSDate &d, TinyGPSTime &t)
{
  if (!d.isValid())
  {
    Serial.print(F("********** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
    Serial.print(sz);
  }
  
  if (!t.isValid())
  {
    Serial.print(F("******** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
    Serial.print(sz);
  }

  printInt(d.age(), d.isValid(), 5);
  smartDelay(0);
}

static void printStr(const char *str, int len)
{
  int slen = strlen(str);
  for (int i=0; i<len; ++i)
    Serial.print(i<slen ? str[i] : ' ');
  smartDelay(0);
}
//---------------- Functions
//Writes val to address register on device
void writeTo(int device, byte address, byte val) {
   Wire.beginTransmission(device); //start transmission to device 
   Wire.write(address);        // send register address
   Wire.write(val);        // send value to write
   Wire.endTransmission(); //end transmission
}

//reads num bytes starting from address register on device in to buff array
void readFrom(int device, byte address, int num, byte buff[]) {
  Wire.beginTransmission(device); //start transmission to device 
  Wire.write(address);        //sends address to read from
  Wire.endTransmission(); //end transmission
  
  Wire.beginTransmission(device); //start transmission to device
  Wire.requestFrom(device, num);    // request 6 bytes from device
  
  int i = 0;
  while(Wire.available())    //device may send less than requested (abnormal)
  { 
    buff[i] = Wire.read(); // receive a byte
    i++;
  }
  Wire.endTransmission(); //end transmission
}
//Takes an average of readings on a given pin
//Returns the average
int averageAnalogRead(int pinToRead)
{
  byte numberOfReadings = 8;
  unsigned int runningValue = 0; 

  for(int x = 0 ; x < numberOfReadings ; x++)
    runningValue += analogRead(pinToRead);
  runningValue /= numberOfReadings;

  return(runningValue);  
}

//The Arduino Map function but for floats
//From: http://forum.arduino.cc/index.php?topic=3922.0
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
void getGyroValues(){

  byte xMSB = readRegister(L3G4200D_Address, 0x29);
  byte xLSB = readRegister(L3G4200D_Address, 0x28);
  xg = ((xMSB << 8) | xLSB);

  byte yMSB = readRegister(L3G4200D_Address, 0x2B);
  byte yLSB = readRegister(L3G4200D_Address, 0x2A);
  yg = ((yMSB << 8) | yLSB);

  byte zMSB = readRegister(L3G4200D_Address, 0x2D);
  byte zLSB = readRegister(L3G4200D_Address, 0x2C);
  zg = ((zMSB << 8) | zLSB);
}

int setupL3G4200D(int scale){
  //From  Jim Lindblom of Sparkfun's code

  // Enable x, y, z and turn off power down:
  writeRegister(L3G4200D_Address, CTRL_REG1, 0b00001111);

  // If you'd like to adjust/use the HPF, you can edit the line below to configure CTRL_REG2:
  writeRegister(L3G4200D_Address, CTRL_REG2, 0b00000000);

  // Configure CTRL_REG3 to generate data ready interrupt on INT2
  // No interrupts used on INT1, if you'd like to configure INT1
  // or INT2 otherwise, consult the datasheet:
  writeRegister(L3G4200D_Address, CTRL_REG3, 0b00001000);

  // CTRL_REG4 controls the full-scale range, among other things:

  if(scale == 250){
    writeRegister(L3G4200D_Address, CTRL_REG4, 0b00000000);
  }else if(scale == 500){
    writeRegister(L3G4200D_Address, CTRL_REG4, 0b00010000);
  }else{
    writeRegister(L3G4200D_Address, CTRL_REG4, 0b00110000);
  }

  // CTRL_REG5 controls high-pass filtering of outputs, use it
  // if you'd like:
  writeRegister(L3G4200D_Address, CTRL_REG5, 0b00000000);
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
