#include <max6675.h>


// ThermoCouple
int thermo_gnd_pin = 45;
int thermo_vcc_pin = 47;
int thermo_so_pin  = 49;
int thermo_cs_pin  = 51;
int thermo_sck_pin = 53;
  
MAX6675 thermocouple(thermo_sck_pin, thermo_cs_pin, thermo_so_pin);
  
void setup() {
  Serial.begin(9600);

  pinMode(thermo_vcc_pin, OUTPUT); 
  pinMode(thermo_gnd_pin, OUTPUT); 
  digitalWrite(thermo_vcc_pin, HIGH);
  digitalWrite(thermo_gnd_pin, LOW);
}

void loop() {
  Serial.print("Temp: ");
  Serial.println(thermocouple.readCelsius());
  
  delay(1000);
}
