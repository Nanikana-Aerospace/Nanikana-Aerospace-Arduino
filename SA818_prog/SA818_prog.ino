void setup() {
  Serial.begin(9600);      // open the serial port at 9600 bps:    
}

void loop() {



Serial.println("AT+DMOCONNECT");

delay(20000);            // delay 200 milliseconds
Serial.println("AT+DMOSETGROUP=1,144.3900,144.3900,0000,0,0000");
delay(20000);
Serial.println("AT+SETFILTER=0,1,0");
}
