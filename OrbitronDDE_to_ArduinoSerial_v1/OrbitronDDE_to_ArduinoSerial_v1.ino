/*

 Basic code created 29 Nov 2010 by Tom Igoe
 Mod for Orbitron-WispDDE-Arduino by J-F Nadeau/Josianne Gilbert 
 21 feb 2015  version 1.0
 More Info : http://www.nanikana-aerospace.org
 
 */

String inString = "";    // string to hold input

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }

  // send an intro:
  Serial.println("\n\nString toInt():");
  Serial.println();
}

void loop() {
  // Read serial input of WispDDE drivers:
  while (Serial.available() > 0) {
    
    int inChar = Serial.read();
    if (isDigit(inChar)) {
      // convert the incoming byte to a char
      // and add it to the string:
     inString += (char)inChar;
    }
    // if you get a newline, print the string,
    // then the string's value:
   if (inChar == '\n') {
     
      Serial.print("Value:");
      Serial.println(inString.toInt());
      Serial.print("String: ");
      Serial.println(inString);
      // clear the string for new input:
      inString = "";
    }
  }
}