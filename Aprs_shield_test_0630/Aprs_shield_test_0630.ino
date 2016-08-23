int counter = 1;

void setup()
{
  Serial.begin(4800);                      //OPEN SERIAL LINE AT 4800
  delay(3);
  Serial.print("MVA2NJF\r\n");               //SET YOUR CALLSIGN HERE, HERE YOU SEE W1AW
  delay(10);                       
  Serial.print("PPWIDE1-1,WIDE2-1\r\n");    //SET DIGIPATH HERE
  delay(10);
  
}

void loop()
{
  //Serial.print("@VA2NJF NANIKANA AEROSPACE");
  //Serial.print("\r\n");
  delay (15);
  //Serial.print("!>NaniKana-AeroSpace  MSG#");     //BEGIN MESSAGE BUT DON'T SEND YET...
  //Serial.print(counter);  //  ...CONCATENATE VALUE OF count TO OUTPUT...
  Serial.print ("!>48.4567/-77.1234 Ag337 Ab337 Ti39 To29 Uv5"); 
  Serial.print("\r\n");                                     //    ...SEND CR/LF TO COMPLETE AND TRANSMIT PACKET.
  counter++;
  delay(10000);                                             //10,000ms = 10sec
}
