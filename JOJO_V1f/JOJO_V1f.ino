#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <Wire.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// Defini la pin pour Code Morse
#define PIN_OUT 9
// Defini la pin pour Relay PTT Radio
#define RELAY1  6 
// Defini la pin pour Relay CUTOFF BALLOON
#define RELAY2  7
//Define the LED Pin
#define UNIT_LENGTH 125
#define BMP085_ADDRESS 0x77  // I2C address of BMP085
// Data wire is plugged into pin 2 on the Arduino
#define ONE_WIRE_BUS 2
// Setup a oneWire instance to communicate with any OneWire devices 
// (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);



char buffer0_long [50];
char buffer1_lat [50];
unsigned long fix_age, time, date, speed, course; 
unsigned long chars, ldate; 
unsigned short sentences, failed_checksum; 

const unsigned char OSS = 0;  // Oversampling Setting

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

// b5 is calculated in bmp085GetTemperature(...), this variable is also used in bmp085GetPressure(...)
// so ...Temperature(...) must be called before ...Pressure(...).
long b5; 
long lat_morse,lon_morse; // create variable for latitude and longitude object


SoftwareSerial gpsSerial(10, 11); // create gps sensor connection
TinyGPS gps; // create gps object


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
{ 'K', ".-.-" },
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


void setup(){
  
   // Initialise the Arduino data pins for RELAY OUTPUT
  pinMode(RELAY1, OUTPUT);       
  pinMode(RELAY2, OUTPUT);
  
  Serial.begin(9600); // connect serial
  gpsSerial.begin(9600); // connect gps sensor
  Wire.begin();
  bmp085Calibration();
  pinMode( PIN_OUT, OUTPUT );
  digitalWrite( PIN_OUT, LOW );
   // Start up the library
  sensors.begin();

}

void loop(){
  while(gpsSerial.available()){ // check for gps data
   if(gps.encode(gpsSerial.read())){ // encode gps data
    gps.get_position(&lat_morse,&lon_morse); // get latitude and longitude
    // display position
    //itoa(lat, buffer_morse, 10);
    sprintf(buffer1_lat, "%ld", lat_morse);
    sprintf(buffer0_long, "%ld", lon_morse);
    //Serial.print("Position: ");
    //Serial.print("lat: ");Serial.print(lat);Serial.print(" ");// print latitude
    //Serial.print("lon: ");Serial.println(lon); // print longitude
    digitalWrite(RELAY1,LOW);           // Turns ON Relays 1 PTT
  Serial.print("PTT TX RELAY= ON");Serial.println();
    Serial.print("LAT: "); Serial.println(buffer1_lat);
    Serial.print("LON: ");Serial.println(buffer0_long);
  Serial.print("-----NANIKANA-AEROSPACE-----");Serial.println();
      String morseWord = encode(buffer1_lat);
      for(int i=0; i<=morseWord.length(); i++){
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
   morseWord = encode(buffer0_long);
      for(int i=0; i<=morseWord.length(); i++){
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
   morseWord = encode("NANIKANA");
     //Serial.print("TEST3: ");Serial.println(morseWord);
      for(int i=0; i<=morseWord.length(); i++){
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
  digitalWrite(RELAY1,HIGH);          // Turns Relay PTT Off
   Serial.print("PTT TX RELAY= OFF");Serial.println();
   float temperature = bmp085GetTemperature(bmp085ReadUT()); //MUST be called first
  float pressure = bmp085GetPressure(bmp085ReadUP());
  float atm = pressure / 101325; // "standard atmosphere"
  float altitude = calcAltitude(pressure); //Uncompensated caculation - in Meters 
  float falt = gps.f_altitude(); //altitude en metre
  //Serial.print("Date: ");
  gps.get_datetime(&date, &time, &fix_age);   
  Serial.print("Alt(GPS): "); 
  Serial.println(falt); 
  
  Serial.print("Date: "); 
  Serial.println(date);
  Serial.print("Time: "); 
  Serial.println(time);
  // returns speed in 100ths of a knot 
  speed = gps.speed(); 
  float fkmph = gps.f_speed_kmph (); //vitessse en kmh
 Serial.print("Speed: "); 
 //Serial.println(speed); 
 Serial.print(fkmph); 
 Serial.println("km/h");
  // course in 100ths of a degree 
  course = gps.course(); 
  Serial.print("Head: ");
  Serial.println(course);


  Serial.print("Temp.int.: ");
  Serial.print(temperature, 2); //display 2 decimal places
  Serial.print("Temp.ext.: ");
  Serial.print(sensors.getTempCByIndex(0)); 
  Serial.println("deg C");

  Serial.print("Pressure: ");
  Serial.print(pressure, 0); //whole number only.
  Serial.println(" Pa");

  Serial.print("Standard Atmosphere: ");
  Serial.println(atm, 4); //display 4 decimal places

  Serial.print("Alt(baro): ");
  Serial.print(altitude, 2); //display 2 decimal places
  Serial.println(" M");

  Serial.println();//line break

  delay(1000); //wait a second and get values again.
   }
  }


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
