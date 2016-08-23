
#include <SoftwareSerial.h>

// GPS Setup
#define rxGPS 2
#define txGPS 3
SoftwareSerial serialGPS = SoftwareSerial(rxGPS, txGPS);
String stringGPS = "";

void setup() {
  pinMode(rxGPS, INPUT);
  pinMode(txGPS, OUTPUT);


  Serial.begin(9600);
  Serial.println("Started");

  // GPS Setup
  serialGPS.begin(4800);
  digitalWrite(txGPS,HIGH);

  // Cut first gibberish
  while(serialGPS.available())
    if (serialGPS.read() == '\r')
      break;
}

void loop()
{
  String s = checkGPS();
  if(s && s.substring(0, 6) == "$GPGGA")
  {
    Serial.println(s);
  }
}

// Check GPS and returns string if full line recorded, else false
String checkGPS()
{
  if (serialGPS.available())
  {
    char c = serialGPS.read();
    if (c != '\n' && c != '\r')
    {
    String  stringGPS  = String(c);
    }
    else
    {
      if (stringGPS != "")
      {
        String tmp = stringGPS;
        stringGPS = "";
        return tmp;
      }
    }
  }
  return false;
}
