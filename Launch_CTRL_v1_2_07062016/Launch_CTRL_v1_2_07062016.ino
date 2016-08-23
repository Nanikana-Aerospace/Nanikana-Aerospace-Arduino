//Nanikana-Aerospace 2016 LAUNCH CTRL BOX SYSTEM V 1.2
//by Michel Thibodeau (DTMF CODE) & JF Nadeau
// LAST REVISION : June 07 2016
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address
const byte LED = 13; //On board Arduino Uno LED
const byte STQ = 2; //DMTF board connected to interrupt pin 2
const byte Q1 = 4; //LSB of the DMTF board
const byte Q2 = 5; //2nd pin of the DMTF board
const byte Q3 = 6; //3rd pin of the DMTF board
const byte Q4 = 7; //4th pin of the DMTF board
int DigitCounter = 0; //counts the number of digits
int DTMF_Code = 0; //this variable will hold the DTMF received code
//String DTMF_Command; //the 3 digits commad
int DTMF_count; // DTMF Code Counter
int DTMF_command; //DTMF 4 digits  Command
int DTMF_digit1;//  digit 1
int DTMF_digit2;//  digit 2
int DTMF_digit3;//  digit 3
int MasterCode = 0;
int PadActivated1 = 0;
int PadActivated2 = 0;
int PadActivated3 = 0;
int PadActivated4 = 0;
int LaunchMode = 0;
int PadStatus1 = 0;
int PadStatus2 = 0;
int PadStatus3 = 0;
int PadStatus4 = 0;
int BattStatus = 0;
int ArmRocketKey = 0;
int MasterCTRL = 0;
int RelayPin1 = 22; // DIGITAL PIN 22 P1 ACTIVE
int RelayPin2 = 24; // DIGITAL PIN 24 P2 ACTIVE
int RelayPin3 = 26; // DIGITAL PIN 26 P3 ACTIVE
int RelayPin4 = 28; // DIGITAL PIN 28 P4 ACTIVE
int RelayPin5 = 30; // DIGITAL PIN 30 CONNECTIVITY TEST
int RelayPin6 = 32; // DIGITAL PIN 32 ARM ACTIVE ( POWER UP FinalRelay Module )
int RelayPin7 = 34; // DIGITAL PIN 34 NOS FUELING RELAY
int RelayPin8 = 36;  // DIGITAL PIN 36
int FinalRelayPin1 = 23; // DIGITAL PIN 22 LAUNCH ACTIVE
int FinalRelayPin2 = 25; // DIGITAL PIN 25 LAUNCH ACTIVE
int FinalRelayPin3 = 27; // DIGITAL PIN 27 LAUNCH ACTIVE
int FinalRelayPin4 = 29; // DIGITAL PIN 29 LAUNCH ACTIVE
int PinPadTest1 = 4; // ANALOG PIN 4 CONNECTIVITY TEST READING 3.3V = 650 to 800
int PinPadTest2 = 5; // ANALOG PIN 5 CONNECTIVITY TEST READING 3.3V = 650 to 800
int PinPadTest3 = 6; // ANALOG PIN 6 CONNECTIVITY TEST READING 3.3V = 650 to 800
int PinPadTest4 = 7; // ANALOG PIN 7 CONNECTIVITY TEST READING 3.3V = 650 to 800
int PinPadTestVal1 = 0; // VARIABLE TO STORE VALUE READ
int PinPadTestVal2 = 0; // VARIABLE TO STORE VALUE READ
int PinPadTestVal3 = 0; // VARIABLE TO STORE VALUE READ
int PinPadTestVal4 = 0; // VARIABLE TO STORE VALUE READ
int TestResult1 = 0; // VARIABLE TO COMPARE TEST VALUE AND PAD ACTIVATED
int TestResult2 = 0; // VARIABLE TO COMPARE TEST VALUE AND PAD ACTIVATED
int TestResult3 = 0; // VARIABLE TO COMPARE TEST VALUE AND PAD ACTIVATED
int TestResult4 = 0; // VARIABLE TO COMPARE TEST VALUE AND PAD ACTIVATED
int Ready1 = 0; // Ready 2 GO Variable
int Ready2 = 0; // Ready 2 GO Variable
int Ready3 = 0; // Ready 2 GO Variable
int Ready4 = 0; // Ready 2 GO Variable
int SystemArm = 0; // SYSTEM ARM (0 = NO)
int NosStatus = 0; // FUELING ON or OFF (0 = OFF)
int MTestSwitchPin = 50; // Manual Test Swith Pin (D50)
int MTestSwitch = 0; // Manual Test Switch Status
int ByPassSwitchPin = 48; // D48
int ByPassSwitch = 0; // Bypass Status
int MTestLedPin = 51; // D51 Led Manual Test
int ByPassLedPin = 49; // D49 Led Bypass Mode
int AlphaLed = 46; // Station Activeted Led Pin
int BravoLed = 44; // Station Activeted Led Pin
int CharlieLed = 42; // Station Activeted Led Pin
int DeltaLed = 40; // Station Activeted Led Pin
int ByPassCount = 0;

unsigned long timer1;// timer for maximum time between rx digit
unsigned long timer2;
;

void setup ()
{
     // use a for loop to initialize each pin as an input:
  for (int thisPin = 4; thisPin < 8; thisPin++) 
    pinMode(thisPin, INPUT);
  pinMode (LED, OUTPUT);  // so we can update the LED
  attachInterrupt (0, readDTMF, FALLING);  // attach interrupt handler
  Serial.begin(9600);
  Serial1.begin(9600);
  lcd.begin(16,2);   // initialize the lcd for 16 chars 2 lines, turn on backlight

// ------- Quick 3 blinks of backlight  -------------
  for(int i = 0; i< 3; i++)
  {
    lcd.backlight();
    delay(250);
    lcd.noBacklight();
    delay(250);
  }
  lcd.backlight(); // finish with backlight on  

  DTMF_count = 0;
  DTMF_command = 0;
  digitalWrite (RelayPin1, 1); //INACTIVE ALL RELAY
  digitalWrite (RelayPin2, 1); //INACTIVE ALL RELAY
  digitalWrite (RelayPin3, 1); //INACTIVE ALL RELAY
  digitalWrite (RelayPin4, 1); //INACTIVE ALL RELAY
  digitalWrite (RelayPin5, 1); //INACTIVE ALL RELAY
  digitalWrite (RelayPin6, 1); //INACTIVE ALL RELAY
  digitalWrite (RelayPin7, 1); //INACTIVE ALL RELAY
  digitalWrite (RelayPin8, 1); //INACTIVE ALL RELAY
  digitalWrite (RelayPin6, 0); //ARM 
  digitalWrite (ByPassSwitch, 0); //BYPASS VALUE
  digitalWrite (FinalRelayPin1, 1); // INACTIVE ALL RELAY
  digitalWrite (FinalRelayPin2, 1); // INACTIVE ALL RELAY
  digitalWrite (FinalRelayPin3, 1); // INACTIVE ALL RELAY
  digitalWrite (FinalRelayPin4, 1); // INACTIVE ALL RELAY
  digitalWrite (AlphaLed, 0); // INACTIVE ALL LED STATUS AFTER LAUNCH
  digitalWrite (BravoLed, 0); // INACTIVE ALL LED STATUS AFTER LAUNCH
  digitalWrite (CharlieLed, 0); // INACTIVE ALL LED STATUS AFTER LAUNCH
  digitalWrite (DeltaLed, 0); // INACTIVE ALL LED STATUS AFTER LAUNCH
  digitalWrite (ByPassLedPin, 0); // INACTIVE ALL LED STATUS AFTER LAUNCH
  digitalWrite (RelayPin6, 1);
  pinMode (RelayPin1, OUTPUT); // SET PIN AS OUTPUT
  pinMode (RelayPin2, OUTPUT); // SET PIN AS OUTPUT
  pinMode (RelayPin3, OUTPUT); // SET PIN AS OUTPUT
  pinMode (RelayPin4, OUTPUT); // SET PIN AS OUTPUT
  pinMode (RelayPin5, OUTPUT); // SET PIN AS OUTPUT
  pinMode (RelayPin6, OUTPUT); // SET PIN AS OUTPUT
  pinMode (RelayPin7, OUTPUT); // SET PIN AS OUTPUT
  pinMode (RelayPin8, OUTPUT); // SET PIN AS OUTPUT
  pinMode (FinalRelayPin1, OUTPUT); // SET PIN AS OUTPUT
  pinMode (FinalRelayPin2, OUTPUT); // SET PIN AS OUTPUT
  pinMode (FinalRelayPin3, OUTPUT); // SET PIN AS OUTPUT
  pinMode (FinalRelayPin4, OUTPUT); // SET PIN AS OUTPUT
  pinMode (MTestLedPin, OUTPUT); // SET PIN AS OUTPUT
  pinMode (ByPassLedPin, OUTPUT); // SET PIN AS OUTPUT
  pinMode (AlphaLed, OUTPUT); // SET PIN AS OUTPUT
  pinMode (BravoLed, OUTPUT); // SET PIN AS OUTPUT
  pinMode (CharlieLed, OUTPUT); // SET PIN AS OUTPUT
  pinMode (DeltaLed, OUTPUT); // SET PIN AS OUTPUT
  pinMode (MTestSwitchPin, INPUT); // SET PIN as input
  pinMode (ByPassSwitchPin, INPUT); // SET PIN as input
  lcd.setCursor(0,0); //Start at character 4 on line 0
  lcd.print("*NANIKANA-SPACE*");
  lcd.setCursor(0,1); //Start at character 4 on line 0
  lcd.print("*LAUNCH CONTROL*");
  Serial.println (" ALL INTERNAL SYSTEM CHECK");
} 

void loop ()
{
 ByPassSwitch = digitalRead (ByPassSwitchPin); // READ IF BYPASSSWITCH ACTIVATED
 MTestSwitch = digitalRead (MTestSwitchPin); // READ INF MANUAL TEST SWITCH ACTIVATED
 if (ByPassSwitch == HIGH && ByPassCount == 0)
   {
     delay (1200); 
     digitalWrite (RelayPin8, 0);
     digitalWrite (ByPassLedPin, 1); 
     delay (1500); 
     Serial.println ("S WARNING..BYPASS MODE"); 
     lcd.setCursor(0,0); 
     lcd.print("*BYPASS  MODE*"); lcd.print(ByPassSwitch); lcd.print(ByPassCount);
     PadActivated1 = 1; digitalWrite (RelayPin1, 0); digitalWrite (AlphaLed, PadActivated1);
     PadActivated2 = 1; digitalWrite (RelayPin2, 0); digitalWrite (BravoLed, PadActivated2);
     PadActivated3 = 1; digitalWrite (RelayPin3, 0); digitalWrite (CharlieLed, PadActivated3);
     PadActivated4 = 1; digitalWrite (RelayPin4, 0); digitalWrite (DeltaLed, PadActivated4);
     lcd.setCursor(0,1); 
     lcd.print("P A-B-C-D:ACTIVE"); 
     LaunchMode = 1;
     MasterCode = 2;
     SystemArm = 1;
     digitalWrite (RelayPin6, 0); // ARM RELAY ON
     delay (1500);
     digitalWrite (FinalRelayPin1, 0); // Final Relay Stage Activated
     digitalWrite (FinalRelayPin2, 0);
     digitalWrite (FinalRelayPin3, 0);
     digitalWrite (FinalRelayPin4, 0);
     delay (1500); 
     lcd.setCursor(0,1); 
     lcd.print("*FinalRelay: ON*");
     digitalWrite (RelayPin8, 1);
     ByPassCount = 1;
    }
  
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
   if (DTMF_count == 3  ) {DTMF_digit3 = DTMF_Code; ;
   //if (DTMF_count == 4  ) {DTMF_digit4 = DTMF_Code; 
   //Serial.print ("C"); 
   
   if (DTMF_digit1 == 6 && DTMF_digit2 == 6 && DTMF_digit3 == 6 && ByPassSwitch == 0) 
     {
     delay (1200); 
     digitalWrite (RelayPin8, 0); 
     delay (1500); 
     Serial.println ("S SYSTEM STANDBY"); 
     lcd.setCursor(0,0); 
     lcd.clear(); 
     lcd.print("STATUS : STANDBY");
     MasterCode = 1; 
     delay (2200); 
     digitalWrite (RelayPin8, 1);
     } // 668 STBY CODE
     
   if (DTMF_digit1 == 6 && DTMF_digit2 == 6 && DTMF_digit3 == 1 && MasterCode == 1) 
     {
     delay (1200); 
     digitalWrite (RelayPin8, 0); 
     delay (1500); 
     Serial.println ("S ACTIVE MODE"); 
     lcd.setCursor(0,0); 
     lcd.print("**ACTIVE  MODE**");
     PadActivated1 = 1; digitalWrite (RelayPin1, 0); digitalWrite (AlphaLed, PadActivated1);
     PadActivated2 = 1; digitalWrite (RelayPin2, 0); digitalWrite (BravoLed, PadActivated2);
     PadActivated3 = 1; digitalWrite (RelayPin3, 0); digitalWrite (CharlieLed, PadActivated3);
     PadActivated4 = 1; digitalWrite (RelayPin4, 0); digitalWrite (DeltaLed, PadActivated4);
     lcd.setCursor(0,1); 
     lcd.print("P A-B-C-D:ACTIVE"); 
     LaunchMode = 1; 
     delay (1200); 
     digitalWrite (RelayPin8, 1);
     } // MANUAL MODE 661
     
   if (DTMF_digit1 == 6 && DTMF_digit2 == 6 && DTMF_digit3 == 2 && MasterCode == 1) 
     {
     delay (1200); 
     digitalWrite (RelayPin8, 0); 
     delay (1500); 
     Serial.println ("S SELECT MODE"); 
     lcd.setCursor(0,0); 
     lcd.print("***SELECT  MODE***");
     
     lcd.setCursor(0,1); 
     lcd.print("P A-B-C-D:WAIT***"); 
     LaunchMode = 4; 
     delay (1200); 
     digitalWrite (RelayPin8, 1);
     } //  AUTOMATIC MODE 662
     
   if (DTMF_digit1 == 1 && DTMF_digit2 == 1 && DTMF_digit3 == 1 && MasterCode == 1 && LaunchMode == 4) 
     {
     delay (1200); 
     digitalWrite (RelayPin8, 0); 
     delay (1500); 
     Serial.println ("S ALPHA, STATION : . ACTIVATED"); 
     lcd.setCursor(0,1); 
     lcd.print("SA ACTIVATED"); 
     PadActivated1 = 1; digitalWrite (AlphaLed, PadActivated1); 
     digitalWrite (RelayPin1, 0); 
     delay (3200); 
     digitalWrite (RelayPin8, 1);
     }//111
     
   if (DTMF_digit1 == 2 && DTMF_digit2 == 2 && DTMF_digit3 == 1 && MasterCode == 1 && LaunchMode == 4) 
     {
     delay (1200); 
     digitalWrite (RelayPin8, 0); 
     delay (1500); 
     Serial.println ("S  BRAVO, STATION : . ACTIVATED"); 
     lcd.setCursor(0,1); 
     lcd.print("SB ACTIVATED"); 
     PadActivated2 = 1; digitalWrite (BravoLed, PadActivated2);
     digitalWrite (RelayPin2, 0); 
     delay (3200); 
     digitalWrite (RelayPin8, 1);
     }//221
 
   if (DTMF_digit1 == 3 && DTMF_digit2 == 3 && DTMF_digit3 == 1 && MasterCode == 1 && LaunchMode == 4) 
     {
     delay (1200); 
     digitalWrite (RelayPin8, 0); 
     delay (1500); 
     Serial.println ("S  CHARLIE, STATION : . ACTIVATED"); 
     lcd.setCursor(0,1); 
     lcd.print("SC ACTIVATED");
     PadActivated3 = 1; digitalWrite (CharlieLed, PadActivated3);
     digitalWrite (RelayPin3, 0); 
     delay (3200); 
     digitalWrite (RelayPin8, 1);
      } // 331
      
   if (DTMF_digit1 == 4 && DTMF_digit2 == 4 && DTMF_digit3 == 1 && MasterCode == 1 && LaunchMode == 4) 
     {
     delay (1200); 
     digitalWrite (RelayPin8, 0); 
     delay (1500); 
     Serial.println ("S  DELTA, STATION : . ACTIVATED"); 
     lcd.setCursor(0,1); 
     lcd.print("SD ACTIVATED");
     PadActivated4 = 1; digitalWrite (DeltaLed, PadActivated4);
     digitalWrite (RelayPin4, 0); 
     delay (3200); 
     digitalWrite (RelayPin8, 1);
     } //441
     
   if (DTMF_digit1 == 6 && DTMF_digit2 == 6 && DTMF_digit3 == 7 && MasterCode == 1) 
     {
     delay (1200); 
     digitalWrite (RelayPin8, 0); 
     delay (1500); 
     Serial.println ("S SYSTEM READY TO ARM"); 
     lcd.setCursor(0,0); 
     lcd.print("*STATUS: ARM? **");
     MasterCode = 2;
     delay (2200); 
     digitalWrite (RelayPin8, 1);
     } //668 LEVEL 2 SECURITY ACCESS NEEDED TO LAUNCH
     
   if (DTMF_digit1 == 7 && DTMF_digit2 == 7 && DTMF_digit3 == 7 ) 
     {
     delay (1200); 
     digitalWrite (RelayPin8, 0); 
     delay (1500); 
     Serial.println ("S STOP SEQUENCE");  
     lcd.clear(); 
     lcd.setCursor(0,0); 
     lcd.print("*STOP SEQUENCE*"); 
     lcd.setCursor(0,1); 
     lcd.print("****************");
     MasterCode = 0; // MASTER CODE (ALL) DISABLE
     Ready1 = 0;  Ready2 = 0; Ready3 = 0; Ready4 = 0;// ALL READY CODE DISABLE
     SystemArm = 0; // DISARM VALUE
     PadActivated1 = 0; digitalWrite (RelayPin1, 1); digitalWrite (AlphaLed, PadActivated1);// POS DESACTIVATION
     PadActivated2 = 0; digitalWrite (RelayPin2, 1); digitalWrite (BravoLed, PadActivated2);// POS DESACTIVATION
     PadActivated3 = 0; digitalWrite (RelayPin3, 1); digitalWrite (CharlieLed, PadActivated3);// POS DESACTIVATION
     PadActivated4 = 0; digitalWrite (RelayPin4, 1); digitalWrite (DeltaLed, PadActivated4);// POS DESACTIVATION
     digitalWrite (RelayPin5, 1); //DESACTIVATED CONNECTIVITY (3.3V) TEST TO ALL LAUNCH PAD ACTIVE
     digitalWrite (RelayPin6, 1); // DISABLE ARM PAD
     digitalWrite (RelayPin7, 1); // DISABLE FUELING
     digitalWrite (ByPassLedPin, 0); // INACTIVE ALL LED STATUS AFTER LAUNCH
     ByPassCount = 0;
     delay (3200);
     lcd.setCursor(0,1); 
     lcd.print("ALL P DESACTIVED"); 
     digitalWrite (RelayPin8, 1); // STOP PTT
     } //ALL SYSTEMS/VALUES DESACTIVATED
     
   if (DTMF_digit1 == 7 && DTMF_digit2 == 7 && DTMF_digit3 == 1 && MasterCode == 1 && ArmRocketKey == 0) 
     {
     delay (1200); 
     digitalWrite (RelayPin8, 0); 
     delay (1500); 
     Serial.println ("S INITIATE  TEST SEQUENCE"); 
     lcd.clear(); 
     lcd.setCursor(0,0); 
     lcd.print("TEST: ..BEGIN.."); 
     delay (3000); 
     digitalWrite (RelayPin8, 1); // STOP PTT
     digitalWrite (RelayPin5, 0); //ACTIVATED CONNECTIVITY (3.3V)TEST TO ALL LAUNCH PAD ACTIVE
     delay (4000);
     PinPadTestVal1 = analogRead (PinPadTest1); 
     Serial.println (PinPadTestVal1); 
       if (PinPadTestVal1 >450 && PinPadTestVal1 <800 && PadActivated1 == 1)  
           {
           Ready1 = 1; 
           delay (1200); 
           digitalWrite (RelayPin8, 0); 
           delay (1500); 
           Serial.println ("S ALPHA, STATION: READY"); 
           lcd.setCursor(0,1); 
           lcd.print("A:Go");} 
       else if (PadActivated1 == 1)
           {
           delay (1200); 
           digitalWrite (RelayPin8, 0); 
           delay (1500); 
           Serial.println ("S ALPHA, STATION : ERROR"); 
           lcd.setCursor(0,1); 
           lcd.print("A"); lcd.print(PinPadTestVal1); 
           PadActivated1 = 0; digitalWrite (AlphaLed, PadActivated1);
           digitalWrite (RelayPin1, 1);
           
           }
     delay (4000);
     PinPadTestVal2 = analogRead (PinPadTest2);  
     Serial.println (PinPadTestVal2);
       if (PinPadTestVal2 >450 && PinPadTestVal2 <800 && PadActivated2 == 1) 
           {
           Ready2 = 1;
           delay (1200); 
           digitalWrite (RelayPin8, 0); 
           delay (1500);
           Serial.println ("S BRAVO, STATION: READY"); 
           lcd.setCursor(4,1); 
           lcd.print("B:Go");
           } 
       else if (PadActivated2 == 1)
           {
           delay (1200); 
           digitalWrite (RelayPin8, 0); 
           delay (1500); 
           Serial.println ("S BRAVO, STATION : ERROR"); 
           lcd.setCursor(4,1); 
           lcd.print("B"); lcd.print(PinPadTestVal2); 
           PadActivated2 = 0; digitalWrite (BravoLed, PadActivated2);
           digitalWrite (RelayPin2, 1);
           
           }
     delay (4000);
     PinPadTestVal3 = analogRead (PinPadTest3); 
     Serial.println (PinPadTestVal3); 
       if (PinPadTestVal3 >450 && PinPadTestVal3 <800 && PadActivated3 == 1) 
           {
           Ready3 = 1;
           delay (1200); 
           digitalWrite (RelayPin8, 0); 
           delay (1500);
           Serial.println ("S CHARLIE, STATION: READY"); 
           lcd.setCursor(8,1); 
           lcd.print("C:Go");
           } 
       else if (PadActivated3 == 1)
           {
           delay (1200); 
           digitalWrite (RelayPin8, 0); 
           delay (1500); 
           Serial.println ("S CHARLIE, STATION : ERROR"); 
           lcd.setCursor(8,1); digitalWrite (CharlieLed, PadActivated3);
           lcd.print("C"); lcd.print(PinPadTestVal3); 
           PadActivated3 = 0; 
           digitalWrite (RelayPin3, 1);
           
           }
     delay (4000);
     PinPadTestVal4 = analogRead (PinPadTest4); 
     Serial.println (PinPadTestVal4); 
       if (PinPadTestVal4 >450 && PinPadTestVal4 <800 && PadActivated4 == 1) 
           {
           Ready4 = 1;
           delay (1200); 
           digitalWrite (RelayPin8, 0); 
           delay (1500);
           Serial.println ("S DELTA,STATION: READY"); 
           lcd.setCursor(12,1); 
           lcd.print("D:Go");
           }
       else if (PadActivated4 == 1)
           {
           delay (1200); 
           digitalWrite (RelayPin8, 0); 
           delay (1500); 
           Serial.println ("S DELTA,STATION : ERROR"); 
           lcd.setCursor(12,1); 
           lcd.print("D"); lcd.print(PinPadTestVal4); 
           PadActivated4 = 0; digitalWrite (DeltaLed, PadActivated4);
           digitalWrite (RelayPin4, 1);
           }
     delay (3000);
     
     Serial.print (PinPadTestVal1); Serial.print (PinPadTestVal2); Serial.print (PinPadTestVal3); Serial.println (PinPadTestVal4); // DEBUG MODE
     
     digitalWrite (RelayPin5, 1); //DESACTIVATED CONNECTIVITY (3.3V)TEST TO ALL IGNITORS 
      
     digitalWrite (RelayPin8, 1);
     lcd.setCursor(0,0); 
     lcd.print("TEST: .COMPLETE.");
    } // PRE FLIGHT TEST 771
   
   if (DTMF_digit1 == 6 && DTMF_digit2 == 6 && DTMF_digit3 == 4 && MasterCode == 2 && ArmRocketKey == 0 ) 
     {
     delay (1200); 
     digitalWrite (RelayPin8, 0); 
     delay (1500); 
     Serial.println ("S WARNING . SYSTEM ARMED AND READY"); 
     lcd.clear(); 
     lcd.setCursor(0,0); 
     lcd.print("**SYSTEM ARMED**"); 
     lcd.setCursor(0,1);
     lcd.print("P1="); 
     lcd.print(PadActivated1);
     lcd.print("P2="); 
     lcd.print(PadActivated2);
     lcd.print("P3=");
     lcd.print(PadActivated3);
     lcd.print("P4=");
     lcd.print(PadActivated4);
     SystemArm = 1;
     digitalWrite (RelayPin6, 0);
     delay (3500);
     digitalWrite (RelayPin8, 1);
     } //  SYSTEM ARMED 664
     
   if (DTMF_digit1 == 8 && DTMF_digit2 == 8 && DTMF_digit3 == 1 && MasterCode == 1  && SystemArm != 1  && ByPassCount !=1) 
     {
     delay (1200); 
     digitalWrite (RelayPin8, 0); 
     delay (1500); 
     Serial.println ("S WARNING.STARTING FUELING SEQUENCE"); 
     lcd.clear(); 
     lcd.print("*****START*****"); 
     lcd.setCursor(0,1); 
     lcd.print("*****FUELING****"); 
      digitalWrite (RelayPin7, 0); 
      NosStatus = 1; 
      delay (2900);
      digitalWrite (RelayPin8, 1);
      }//881 NOS ON
     
   if (DTMF_digit1 == 8 && DTMF_digit2 == 8 && DTMF_digit3 == 2 && MasterCode == 1) 
     {
     delay (1200); 
     digitalWrite (RelayPin8, 0); 
     delay (1500); 
     Serial.println ("S STOP FUELING SEQUENCE"); 
     lcd.clear(); 
     lcd.print("*****STOP*****"); 
     lcd.setCursor(0,1); 
     lcd.print("*****FUELING****"); 
     digitalWrite (RelayPin7, 1); 
     NosStatus = 0; 
     delay (2900);
     digitalWrite (RelayPin8, 1);
     }//882 NOS OFF
   
   if (DTMF_digit1 == 6 && DTMF_digit2 == 6 && DTMF_digit3 == 8 && MasterCode == 2 && ArmRocketKey == 0  && SystemArm == 1 && NosStatus == 0 && ByPassCount != 1 ) 
     {
     delay (1200); 
     digitalWrite (RelayPin8, 0); 
     delay (1500); 
     Serial.println ("S LAUNCH SEQUENCE INITIATED"); 
     lcd.clear(); 
     lcd.print("*****LAUNCH*****"); 
     lcd.setCursor(0,1); 
     lcd.print("****SEQUENCE****"); 
     delay (5000);
     Serial.println ("S10");
     lcd.setCursor(0,1); lcd.print("****** 10 ******");
     delay (1500);
     Serial.println ("S9");
     lcd.setCursor(0,1); lcd.print(" ***** 09 ***** ");
     delay (1500);
     Serial.println ("S8");
     lcd.setCursor(0,1); lcd.print(" ***** 08 ***** ");
     delay (1500);
     Serial.println ("S7");
     lcd.setCursor(0,1); lcd.print("  **** 07 ****  ");
     delay (1500);
     Serial.println ("S6");
     lcd.setCursor(0,1); lcd.print("  **** 06 ****  ");
     delay (1500);
     Serial.println ("S5");
     lcd.setCursor(0,1); lcd.print("   *** 05 ***   ");
     delay (1500);
     Serial.println ("S4");
     lcd.setCursor(0,1); lcd.print("   *** 04 ***   ");
     delay (1500);
     Serial.println ("S3");
     lcd.setCursor(0,1); lcd.print("    ** 03 **    ");
     delay (1500);
     Serial.println ("S2");
     lcd.setCursor(0,1); lcd.print("    ** 02 **    ");
     delay (1500);
     Serial.println("S1");
     lcd.setCursor(0,1); lcd.print("     * 01 *     ");
     delay (1500);
     lcd.setCursor(0,1); lcd.print("****** 00 ******");
     Serial.println ("S IGNITION");
     
          if (PadActivated1 == 1) 
            digitalWrite (FinalRelayPin1, 0); 
          else 
            digitalWrite (FinalRelayPin1, 1);
          if (PadActivated2 == 1) 
            digitalWrite (FinalRelayPin2, 0); 
          else 
            digitalWrite (FinalRelayPin2, 1);
          if (PadActivated3 == 1) 
            digitalWrite (FinalRelayPin3, 0); 
          else 
            digitalWrite (FinalRelayPin3, 1);
          if (PadActivated4 == 1) 
            digitalWrite (FinalRelayPin4, 0); 
          else 
            digitalWrite (FinalRelayPin4, 1);
     delay (2500);
     digitalWrite (RelayPin8, 1);
     lcd.setCursor(0,1); 
     delay (10000); // DELAY FOR ACTIVATE LAST LAUNCH SEQUENCE : HUMAN ;)
      //Serial.println ("S ALL SYSTEM RESETTING");
      lcd.clear();
      digitalWrite (FinalRelayPin1, 1); 
      digitalWrite (FinalRelayPin2, 1); 
      digitalWrite (FinalRelayPin3, 1); 
      digitalWrite (FinalRelayPin4, 1);
      MasterCode = 0; 
      Ready1 = 0; Ready2 = 0; Ready3 = 0; Ready4 = 0; 
      SystemArm = 0; 
      digitalWrite (RelayPin6, 1); 
      digitalWrite (RelayPin7, 1);
      PadActivated1 = 0; digitalWrite (AlphaLed, PadActivated1);
      digitalWrite (RelayPin1, 1);
      PadActivated2 = 0; digitalWrite (BravoLed, PadActivated1);
      digitalWrite (RelayPin2, 1);
      PadActivated3 = 0; digitalWrite (CharlieLed, PadActivated1);
      digitalWrite (RelayPin3, 1);
      PadActivated4 = 0; digitalWrite (DeltaLed, PadActivated1);
      digitalWrite (RelayPin4, 1);
      lcd.setCursor(0,0); //Start at character 4 on line 0
      lcd.print("*NANIKANA-SPACE*");
      lcd.setCursor(0,1); //Start at character 4 on line 0
      lcd.print("*LAUNCH CONTROL*");
      }//668 LAUNCHING THEN RESET ALL AFTER 10 SEC !
      
      
   Serial.print (DTMF_digit1); Serial.print (DTMF_digit2);Serial.print (DTMF_digit3); Serial.println (""); // DEBUG MODE
   
   DTMF_count = 0; delay (3000);}
   DTMF_Code = 0;
   }
   else{  
   //DTMF_count = 0;
   DigitCounter = 0;
   DTMF_Code = 0;
   //Serial.println (millis() - timer1); //Debug Mode
   if (millis() - timer1 >3000)DTMF_count = 0; // IF time between 2 digit or all digit more than 3 sec: Flush all
   
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

