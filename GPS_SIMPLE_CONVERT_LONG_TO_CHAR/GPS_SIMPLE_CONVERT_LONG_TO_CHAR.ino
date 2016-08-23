#include <SoftwareSerial.h>
#include <TinyGPS.h>
char buffer0_long [50];
char buffer1_lat [50];

long lat_morse,lon_morse; // create variable for latitude and longitude object


SoftwareSerial gpsSerial(10, 11); // create gps sensor connection
TinyGPS gps; // create gps object

void setup(){
  Serial.begin(9600); // connect serial
  gpsSerial.begin(9600); // connect gps sensor
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
    Serial.print("LAT: "); Serial.println(buffer1_lat);
    Serial.print("LON: ");Serial.println(buffer0_long);
     // Wait 
  delay(5000);
   }
  }
}
