// Projet Otachi-Arduino par Nanikana-Aerospace.
// Sonde stratosphérique avec Arduino Uno R3 et PixHawk + GY80 + Ublox 6 + Relay +DHT11 + DS18B20
// Code basé sur des sketchs sous licence Open Source. TinyGPS++ library by Mikal Hart
// Assemblage et programmation par Josianne Gilbert et JF Nadeau
// Plus d'info sur le site web- More informations on web site. 
//           www.nanikana-aerospace.org
// ***  MPPI CTRL Mission COMP Prog. v.251014-E  ***
//  For informations : info@nanikana-aerospace.org

#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <dht.h>
#include <MemoryFree.h>

//no ; here. Set equal to channel DHT sensor is on
#define dht_dpin 3 
dht DHT;
// Defini la pin pour Code Morse
#define PIN_OUT 9
// Defini la pin pour Relay PTT Radio
#define RELAY1  6 
// Defini la pin pour Relay CUTOFF BALLOON
#define RELAY2  7
//Define the LED Pin
#define UNIT_LENGTH 60
#define BMP085_ADDRESS 0x77  // I2C address of BMP085
// Data wire is plugged into pin 2 on the Arduino
#define ONE_WIRE_BUS 2
// Setup a oneWire instance to communicate with any OneWire devices 
// (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);
// arrays to hold device address
DeviceAddress insideThermometer;

const unsigned char OSS = 0;  // Oversampling Setting
char buffer0_long [50];
char buffer1_lat [10];
// Calibration values
int ac1;
int ac2;
int ac3;
unsigned int ac4;
unsigned int ac5;
unsigned int ac6;
int b1;
int b2;
int mb;
int mc;
int md;
int sequence;

// b5 is calculated in bmp085GetTemperature(...), this variable is also used in bmp085GetPressure(...)
// so ...Temperature(...) must be called before ...Pressure(...).
long b5; 
long lat_morse,lon_morse; // create variable for latitude and longitude object
float lat_morse_float, lon_morse_float;
float vaccgps1; //Acc Vertical GPS
float vaccgps2;
/*
   GPS device hooked up on pins 10(rx) and 11(tx).
*/
static const int RXPin = 10, TXPin = 11;
static const uint32_t GPSBaud = 9600;

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

//Build a struct with the morse code mapping
static const struct {const char letter, *code;} MorseMap[] =
{
{ 'A', ".-" },
{ 'B', "-..." },
{ 'C', "-.-." },
{ 'D', "-.." },
{ 'E', "." },
{ 'F', "..-." },
{ 'G', "--." },
{ 'H', "...." },
{ 'I', ".." },
{ 'J', ".---" },
{ 'K', "-.-" },
{ 'L', ".-.." },
{ 'M', "--" },
{ 'N', "-." },
{ 'O', "---" },
{ 'P', ".--." },
{ 'Q', "--.-" },
{ 'R', ".-." },
{ 'S', "..." },
{ 'T', "-" },
{ 'U', "..-" },
{ 'V', "...-" },
{ 'W', ".--" },
{ 'X', "-..-" },
{ 'Y', "-.--" },
{ 'Z', "--.." },
{ ' ', " " }, //Gap between word, seven units
{ '1', ".----" },
{ '2', "..---" },
{ '3', "...--" },
{ '4', "....-" },
{ '5', "....." },
{ '6', "-...." },
{ '7', "--..." },
{ '8', "---.." },
{ '9', "----." },
{ '0', "-----" },
{ '.', "·–·–·–" },
{ ',', "--..--" },
{ '?', "..--.." },
{ '!', "-.-.--" },
{ ':', "---..." },
{ ';', "-.-.-." },
{ '(', "-.--." },
{ ')', "-.--.-" },
{ '"', ".-..-." },
{ '@', ".--.-." },
{ '&', ".-..." },
};
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
  return result;
}
void printTemperature(DeviceAddress deviceAddress)
{
  
  // method 2 - faster
  float tempC = sensors.getTempC(deviceAddress);
  //Serial.print("Temp C: ");
  Serial.print(tempC);
 
  Serial.println();//line break
  //delay(1200); //wait
}

unsigned long time; // variable pour definir le running time du prog.

void setup()
{
  Serial.print(sensors.getDeviceCount(), DEC);//Jojo DEBUT
         if (sensors.isParasitePowerMode()) Serial.println("ON");
         else Serial.println("OFF");
        if (!sensors.getAddress(insideThermometer, 0)) //Serial.println("Unable to find address for Device 0"); 
        printAddress(insideThermometer);
        sensors.setResolution(insideThermometer, 9);
        //Jojo FIN
   // Initialise the Arduino data pins for RELAY OUTPUT
  pinMode(RELAY1, OUTPUT);
   digitalWrite(RELAY1,HIGH);          // Turns Relay PTT Off  
  pinMode(RELAY2, OUTPUT); 
   digitalWrite(RELAY2,HIGH);          // Turns Relay  Off
  pinMode( PIN_OUT, OUTPUT );
  digitalWrite( PIN_OUT, LOW );
  
  Serial.begin(9600);
  ss.begin(GPSBaud);
  Serial.println(F("Initialisation"));
  Serial.print("*");
  smartDelay(500); // Wait 5 sec before starting
  Serial.print("*");
  smartDelay(500);
  Serial.print("*");
  smartDelay(500); 
  Serial.print("*");
  smartDelay(500);  
  Serial.print("*");
  smartDelay(500); 
  Serial.print("*");
  smartDelay(500); 
  Serial.print("*");
  smartDelay(500); 
  Serial.print("*");
  smartDelay(500); 
  Serial.print("*");
  smartDelay(500); 
  Serial.println("*");
  smartDelay(500); 
  Serial.println(F("MPPI CTRL Mission Comp v.251014E"));
  smartDelay(2000); // Wait 2 sec before starting
  Serial.println(F("Nanikana-Aerospace Project Otachi-Arduino 2014-2015"));
    Serial.println(F(""));
  
  Wire.begin();
  bmp085Calibration();
  pinMode( PIN_OUT, OUTPUT );
  digitalWrite( PIN_OUT, LOW );
  
   // Start up the library
  sensors.begin();


  }
   
void loop()
{
  
  sequence = sequence+1;
  Serial.print("SEQ      : ");
  Serial.println(sequence);
  Serial.print("RUN TIME : ");
  time = millis(); //prints time since program started
  time = time /60000;
  Serial.print(time); Serial.println(" MIN");
  Serial.print("DATE  UTC: ");
  Serial.println(gps.date.value());
  Serial.print("TIME  UTC: ");
  Serial.println(gps.time.value());
  Serial.print("LAT      : ");
  printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
  Serial.println();
  Serial.print("LON      : ");
  printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);
  Serial.println();
  Serial.print("Vacc(GPS): ");
  vaccgps2 = (gps.altitude.meters() - vaccgps1) /7;
  Serial.print (vaccgps2); Serial.println(" m/sec");
  Serial.print("ALT (GPS): ");
  printFloat(gps.altitude.meters(), gps.altitude.isValid(), 7, 2);
  vaccgps1 = gps.altitude.meters();
  Serial.print(" M");
  Serial.println();
  Serial.print("GR SPEED : ");
  printFloat(gps.speed.kmph(), gps.speed.isValid(), 6, 2);
  Serial.println("KML/H");
  Serial.print("HEADING  : ");
  printStr(gps.course.isValid() ? TinyGPSPlus::cardinal(gps.course.value()) : "*** ", 6);
  Serial.println();
  Serial.print("SAT FIX  : ");
  printInt(gps.satellites.value(), gps.satellites.isValid(), 5);
  Serial.println();
  Serial.print("TEMP.TOP.: ");
  float temperature = bmp085GetTemperature(bmp085ReadUT()); //MUST be called first
  float pressure = bmp085GetPressure(bmp085ReadUP());
  float atm = pressure / 101325; // "standard atmosphere"
  float altitude = calcAltitude(pressure); //Uncompensated caculation - in Meters 
  Serial.print(temperature, 2); //display 2 decimal places
  Serial.println();//line break
  sensors.requestTemperatures();//Jojo
  Serial.print("TEMP.EXT.: ");
  printTemperature(insideThermometer);//Jojo
   
  DHT.read11(dht_dpin);

  Serial.print("TEMP BOT : ");
  Serial.print(DHT.temperature);
  Serial.println("C ");
  Serial.print("HUMIDITY : ");
  Serial.print(DHT.humidity);
  Serial.print("% ");
  Serial.println();//line break
  Serial.print("PRESSURE : ");
  Serial.print(pressure, 0); //whole number only.
  Serial.println(" PA");
  Serial.print("STD ATM  : ");
  Serial.println(atm, 4); //display 4 decimal places
  Serial.print("ALT(BARO): ");
  Serial.print(altitude, 2); //display 2 decimal places
  Serial.println(" M");
  Serial.print("CPU POWER: ");
  Serial.print( readVcc(), DEC );
  Serial.println(" mV");
  Serial.print("FREE MEM.: ");
  Serial.print(freeMemory());
  
  Serial.println();
 /* 
  // CUT ROPE SEQUENCE 
    if (time >= 180 && altitude >= 20000)// If more than 3 hours AND more than 20 klm
  
  {
    digitalWrite(RELAY2,LOW);           // Turns ON Relays 2 IGNITION of E-Match
    Serial.println("CUT ROPE : ***IGNITION***"); delay (10000); digitalWrite(RELAY2,HIGH); }//Wait 10 sec then turn relay2 off
    else
   { digitalWrite(RELAY2,HIGH);   delay (10);        // STAY OFF Relays 2 WAIT until 20klm AND 3 hours flight
    Serial.println("CUT ROPE : WAITING");} 
  
  */
   if (sequence == 20)
  { 
  //debut code pour le morse NANIKANA
  digitalWrite(RELAY1,LOW);   delay (1000);        // Turns ON Relays 1 PTT
  Serial.println("PTT TX   : ON"); 
  {
  String morseWord = encode("NANIKANA-AEROSPACE");
  Serial.println("MORSE ID : NANIKANA-AEROSPACE");
  
  for(int i=0; i<=morseWord.length(); i++)
{
switch( morseWord[i] )
{
case '.': //dit
digitalWrite( PIN_OUT, HIGH );
delay( UNIT_LENGTH );
digitalWrite( PIN_OUT, LOW );
delay( UNIT_LENGTH );
break;
 
case '-': //dah
digitalWrite( PIN_OUT, HIGH );
delay( UNIT_LENGTH*3 );
digitalWrite( PIN_OUT, LOW );
delay( UNIT_LENGTH );
break;
 
case ' ': //gap
delay( UNIT_LENGTH );
}
}
// Wait 
  delay(2000);
 } 
}
//fin du code morse

if (sequence == 20)
  { 
  //debut code pour le morse LATITUDE
  
{
//  buffer1_lat =(          );
lat_morse_float = gps.location.lat(), gps.location.isValid(), 11, 6;
lat_morse_float = lat_morse_float *1000000;
dtostrf(lat_morse_float, 2, 0, buffer1_lat);
String morseWord = encode(buffer1_lat);
Serial.print("MORSE LAT: ");
Serial.println(buffer1_lat);
//Serial.println(morseWord); //debug output
for(int i=0; i<=morseWord.length(); i++)
{
switch( morseWord[i] )
{
case '.': //dit
digitalWrite( PIN_OUT, HIGH );
delay( UNIT_LENGTH );
digitalWrite( PIN_OUT, LOW );
delay( UNIT_LENGTH );
break;
 
case '-': //dah
digitalWrite( PIN_OUT, HIGH );
delay( UNIT_LENGTH*3 );
digitalWrite( PIN_OUT, LOW );
delay( UNIT_LENGTH );
break;
 
case ' ': //gap
delay( UNIT_LENGTH );
}
}
// Wait 
 delay(2000);
}
}
//fin du code morse LATITUDE

if (sequence == 20)
  { 
 //debut code pour le morse LONGITUDE

{
lon_morse_float = gps.location.lng(), gps.location.isValid(), 12, 6;
lon_morse_float= -lon_morse_float*1000000;
dtostrf(lon_morse_float, 2, 0, buffer1_lat);
String morseWord = encode(buffer1_lat);

Serial.print("MORSE LON: ");
Serial.println(buffer1_lat);
//Serial.println(morseWord); //debug output
for(int i=0; i<=morseWord.length(); i++)
{
switch( morseWord[i] )
{
case '.': //dit
digitalWrite( PIN_OUT, HIGH );
delay( UNIT_LENGTH );
digitalWrite( PIN_OUT, LOW );
delay( UNIT_LENGTH );
break;
 
case '-': //dah
digitalWrite( PIN_OUT, HIGH );
delay( UNIT_LENGTH*3 );
digitalWrite( PIN_OUT, LOW );
delay( UNIT_LENGTH );
break;
 
case ' ': //gap
delay( UNIT_LENGTH );
}
}
sequence = 0;
// Wait 
 delay(2000);
} 
}
//fin du code morse LONGITUDE

   digitalWrite(RELAY1,HIGH);          // Turns Relay PTT Off
   Serial.print("PTT TX   : OFF");Serial.println();
   Serial.print("WAIT     : 10 SEC ");Serial.println();
  // Wait 
 delay(10000); 
   

  smartDelay(5000);

  if (millis() > 5000 && gps.charsProcessed() < 10)
    Serial.println(F("--WARNING : No GPS data received --"));
  delay(1000); //wait sec  1
  
  Serial.println();
 
  
}
String encode(const char *string)
{
size_t i, j;
String morseWord = "";
for( i = 0; string[i]; ++i )
{
for( j = 0; j < sizeof MorseMap / sizeof *MorseMap; ++j )
{
if( toupper(string[i]) == MorseMap[j].letter )
{
morseWord += MorseMap[j].code;
break;
}
}
morseWord += " "; //Add tailing space to seperate the chars
}
 
return morseWord;
}
 
String decode(String morse)
{
String msg = "";
int lastPos = 0;
int pos = morse.indexOf(' ');
while( lastPos <= morse.lastIndexOf(' ') )
{
for( int i = 0; i < sizeof MorseMap / sizeof *MorseMap; ++i )
{
if( morse.substring(lastPos, pos) == MorseMap[i].code )
{
msg += MorseMap[i].letter;
}
}
 
lastPos = pos+1;
pos = morse.indexOf(' ', lastPos);
// Handle white-spaces between words (7 spaces)
while( morse[lastPos] == ' ' && morse[pos+1] == ' ' )
{
pos ++;
}
}
 
return msg;
}
void bmp085Calibration()
{
  ac1 = bmp085ReadInt(0xAA);
  ac2 = bmp085ReadInt(0xAC);
  ac3 = bmp085ReadInt(0xAE);
  ac4 = bmp085ReadInt(0xB0);
  ac5 = bmp085ReadInt(0xB2);
  ac6 = bmp085ReadInt(0xB4);
  b1 = bmp085ReadInt(0xB6);
  b2 = bmp085ReadInt(0xB8);
  mb = bmp085ReadInt(0xBA);
  mc = bmp085ReadInt(0xBC);
  md = bmp085ReadInt(0xBE);
}

// Calculate temperature in deg C
float bmp085GetTemperature(unsigned int ut){
  long x1, x2;

  x1 = (((long)ut - (long)ac6)*(long)ac5) >> 15;
  x2 = ((long)mc << 11)/(x1 + md);
  b5 = x1 + x2;

  float temp = ((b5 + 8)>>4);
  temp = temp /10;

  return temp;
}

// Calculate pressure given up
// calibration values must be known
// b5 is also required so bmp085GetTemperature(...) must be called first.
// Value returned will be pressure in units of Pa.
long bmp085GetPressure(unsigned long up){
  long x1, x2, x3, b3, b6, p;
  unsigned long b4, b7;

  b6 = b5 - 4000;
  // Calculate B3
  x1 = (b2 * (b6 * b6)>>12)>>11;
  x2 = (ac2 * b6)>>11;
  x3 = x1 + x2;
  b3 = (((((long)ac1)*4 + x3)<<OSS) + 2)>>2;

  // Calculate B4
  x1 = (ac3 * b6)>>13;
  x2 = (b1 * ((b6 * b6)>>12))>>16;
  x3 = ((x1 + x2) + 2)>>2;
  b4 = (ac4 * (unsigned long)(x3 + 32768))>>15;

  b7 = ((unsigned long)(up - b3) * (50000>>OSS));
  if (b7 < 0x80000000)
    p = (b7<<1)/b4;
  else
    p = (b7/b4)<<1;

  x1 = (p>>8) * (p>>8);
  x1 = (x1 * 3038)>>16;
  x2 = (-7357 * p)>>16;
  p += (x1 + x2 + 3791)>>4;

  long temp = p;
  return temp;
}

// Read 1 byte from the BMP085 at 'address'
char bmp085Read(unsigned char address)
{
  unsigned char data;

  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(address);
  Wire.endTransmission();

  Wire.requestFrom(BMP085_ADDRESS, 1);
  while(!Wire.available())
    ;

  return Wire.read();
}

// Read 2 bytes from the BMP085
// First byte will be from 'address'
// Second byte will be from 'address'+1
int bmp085ReadInt(unsigned char address)
{
  unsigned char msb, lsb;

  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(address);
  Wire.endTransmission();

  Wire.requestFrom(BMP085_ADDRESS, 2);
  while(Wire.available()<2)
    ;
  msb = Wire.read();
  lsb = Wire.read();

  return (int) msb<<8 | lsb;
}

// Read the uncompensated temperature value
unsigned int bmp085ReadUT(){
  unsigned int ut;

  // Write 0x2E into Register 0xF4
  // This requests a temperature reading
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(0xF4);
  Wire.write(0x2E);
  Wire.endTransmission();

  // Wait at least 4.5ms
  delay(5);

  // Read two bytes from registers 0xF6 and 0xF7
  ut = bmp085ReadInt(0xF6);
  return ut;
}

// Read the uncompensated pressure value
  unsigned long bmp085ReadUP(){

  unsigned char msb, lsb, xlsb;
  unsigned long up = 0;

  // Write 0x34+(OSS<<6) into register 0xF4
  // Request a pressure reading w/ oversampling setting
  Wire.beginTransmission(BMP085_ADDRESS);
  Wire.write(0xF4);
  Wire.write(0x34 + (OSS<<6));
  Wire.endTransmission();

  // Wait for conversion, delay time dependent on OSS
  delay(2 + (3<<OSS));

  // Read register 0xF6 (MSB), 0xF7 (LSB), and 0xF8 (XLSB)
  msb = bmp085Read(0xF6);
  lsb = bmp085Read(0xF7);
  xlsb = bmp085Read(0xF8);

  up = (((unsigned long) msb << 16) | ((unsigned long) lsb << 8) | (unsigned long) xlsb) >> (8-OSS);

  return up;
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

float calcAltitude(float pressure){

  float A = pressure/101325;
  float B = 1/5.25588;
  float C = pow(A,B);
  C = 1 - C;
  C = C /0.0000225577;

  return C;
}
void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    if (deviceAddress[i] < 16) Serial.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
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
  char sz[32] = "****************";
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
