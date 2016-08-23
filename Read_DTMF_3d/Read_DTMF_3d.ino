//Nanikana-Aerospace 2016 RX Arduino Module (09 janv 2016)
//by Michel Thibodeau & JF Nadeau

const byte LED = 13; //On board Arduino Uno LED
const byte STQ = 2; //DMTF board connected to interrupt pin 2
const byte Q1 = 4; //LSB of the DMTF board
const byte Q2 = 5; //2nd pin of the DMTF board
const byte Q3 = 6; //3rd pin of the DMTF board
const byte Q4 = 7; //4th pin of the DMTF board
int DigitCounter = 0; //counts the number of digits
int DTMF_Code = 0; //this variable will hold the DTMF received code
//String DTMF_Command; //the 4 digits commad
int DTMF_count; // DTMF Code Counter
int DTMF_command; //DTMF 4 digits  Command
int DTMF_digit1;//  digit 1
int DTMF_digit2;//  digit 2
int DTMF_digit3;//  digit 3
int DTMF_digit4;//  digit 4
unsigned long timer1;// timer for maximum time between rx digit
unsigned long timer2;
int heartbeatpin = 8;

void setup ()
{
     // use a for loop to initialize each pin as an input:
  for (int thisPin = 4; thisPin < 8; thisPin++) 
    pinMode(thisPin, INPUT);
  pinMode (LED, OUTPUT);  // so we can update the LED
  attachInterrupt (0, readDTMF, FALLING);  // attach interrupt handler
  Serial.begin(9600);
  DTMF_count = 0;
  DTMF_command = 0;
  pinMode(3, OUTPUT); // RESET MAIN Arduino via CMD
  pinMode(12, OUTPUT); // To MAIN D6 ( Spare CS on CMD )
  Serial.println ("REBOOT...");
} 

void loop ()
{
  digitalWrite(heartbeatpin, HIGH);// Heartbeat Signal to Main Nano 
  // DTMF_Command = DTMF_Command + DTMF_Code;
//Serial.print ("HB"); 
//Serial.println (DTMF_Code);// Debug mode
  if (DTMF_Code != 0)
  {
   //Serial.print (DTMF_Code);// Debug mode
   DTMF_count = DTMF_count + 1;
   timer1 = millis ();
  // Serial.print (DigitCounter); //Debug mode
  //Serial.print (DTMF_count); // Debug mode
   
   if (DTMF_count == 1) {DTMF_digit1 = DTMF_Code; timer1 = millis ();}
   if (DTMF_count == 2  ) {DTMF_digit2 = DTMF_Code; timer1 = millis(); }
   if (DTMF_count == 3  ) {DTMF_digit3 = DTMF_Code; timer1 = millis();}
   if (DTMF_count == 4  ) {DTMF_digit4 = DTMF_Code; 
   //Serial.print ("C"); 
   if (DTMF_digit1 == 6 && DTMF_digit2 == 6 && DTMF_digit3 == 1 && DTMF_digit4 == 5) {Serial.println ("5"); } // MASTER CODE CMD to MAIN 6615
   if (DTMF_digit1 == 5 && DTMF_digit2 == 3 && DTMF_digit3 == 3 && DTMF_digit4 == 5) {Serial.println ("0"); } // DISABLE UHF TX CMD to MAIN 5335
   if (DTMF_digit1 == 5 && DTMF_digit2 == 1 && DTMF_digit3 == 1 && DTMF_digit4 == 1) {Serial.println ("1"); } // ENABLE UHF TX CMD to MAIN 5111
   if (DTMF_digit1 == 5 && DTMF_digit2 == 2 && DTMF_digit3 == 3 && DTMF_digit4 == 2) {Serial.println ("2"); digitalWrite(12, HIGH);} // EMERGENCY DISABLE UHF TX via CS (D12) 5232
   if (DTMF_digit1 == 5 && DTMF_digit2 == 6 && DTMF_digit3 == 1 && DTMF_digit4 == 1) {Serial.println ("3"); digitalWrite(12, LOW);} // EMERGENCY ENABLE UHF TX via CS (D12) 5611
   if (DTMF_digit1 == 5 && DTMF_digit2 == 3 && DTMF_digit3 == 3 && DTMF_digit4 == 6) {Serial.println ("4"); } // VHF TX INFORMATION 1 time only 5336
   if (DTMF_digit1 == 5 && DTMF_digit2 == 1 && DTMF_digit3 == 6 && DTMF_digit4 == 3) {Serial.println ("R"); digitalWrite(3, HIGH); delay (1000); digitalWrite(3, LOW);} //RESET MAIN Arduino 5163
   
   //Serial.print (DTMF_digit1); Serial.print (DTMF_digit2);Serial.print (DTMF_digit3);Serial.print (DTMF_digit4); Serial.println ("C"); // DEBUG MODE
   digitalWrite(heartbeatpin, LOW);// Heartbeat Signal to Main Nano
   DTMF_count = 0; delay (3000);}
   DTMF_Code = 0;
   }
   else{  
   //DTMF_count = 0;
   DigitCounter = 0;
   DTMF_Code = 0;
   //Serial.println (millis() - timer1); //Debug Mode
   if (millis() - timer1 >10000)DTMF_count = 0; // IF time between 2 digit or all digit more than 10 sec: Flush all
   
   }
   
 }

  
// Interrupt Service Routine (ISR)
void readDTMF ()
{
  if (digitalRead (Q1) == LOW)
    DTMF_Code = 0;
  else
    DTMF_Code = 1;
 if (digitalRead (Q2) == LOW)
    DTMF_Code = DTMF_Code;
  else
    DTMF_Code = DTMF_Code + 2;
 if (digitalRead (Q3) == LOW)
    DTMF_Code = DTMF_Code;
  else
    DTMF_Code = DTMF_Code + 4;
 if (digitalRead (Q4) == LOW)
    DTMF_Code = DTMF_Code;
  else
    DTMF_Code = DTMF_Code + 8;
  if (DTMF_Code==10)
   DTMF_Code = DTMF_Code - 10;
  else 
  DigitCounter = DigitCounter + 1; 
}  // end of readDTMF ISR
