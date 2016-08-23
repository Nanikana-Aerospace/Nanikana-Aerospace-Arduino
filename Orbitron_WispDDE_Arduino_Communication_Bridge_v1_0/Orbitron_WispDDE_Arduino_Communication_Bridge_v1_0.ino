/*
Orbitron-WispDDE-Arduino Communication Bridge v1.0

JF Nadeau 22 feb 2015

www.nanikana-aerospace.org


*/
void setup() {
Serial.begin(9600);
}
void loop() {

int satazimuth = 0;
int satelevation = 0;
if (Serial.find("Z")) {
satazimuth = Serial.parseInt(); // parses numeric characters before the space
satelevation = Serial.parseInt();// parses numeric characters after the space&EL
// print the results:
Serial.print("SAT AZ: " );
Serial.print(satazimuth);
Serial.println ("  ANT AZ: ");
Serial.print("SAT EL: ");
Serial.print(satelevation);
Serial.println ("  ANT EL: ");
}
}

