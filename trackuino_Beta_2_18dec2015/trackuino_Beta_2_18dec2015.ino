/* trackuino copyright (C) 2010  EA5HAV Javi
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

// Mpide 22 fails to compile Arduino code because it stupidly defines ARDUINO 
// as an empty macro (hence the +0 hack). UNO32 builds are fine. Just use the
// real Arduino IDE for Arduino builds. Optionally complain to the Mpide
// authors to fix the broken macro.
#if (ARDUINO + 0) == 0
#error "Oops! We need the real Arduino IDE (version 22 or 23) for Arduino builds."
#error "See trackuino.pde for details on this"

// Refuse to compile on arduino version 21 or lower. 22 includes an 
// optimization of the USART code that is critical for real-time operation
// of the AVR code.
#elif (ARDUINO + 0) < 22
#error "Oops! We need Arduino 22 or 23"
#error "See trackuino.pde for details on this"

#endif


// Trackuino custom libs
#include "config.h"
#include "afsk_avr.h"
#include "afsk_pic32.h"
#include "aprs.h"
#include "buzzer.h"
#include "gps.h"
#include "pin.h"
#include "power.h"
#include "sensors_avr.h"
#include "sensors_pic32.h"

// Arduino/AVR libs
#if (ARDUINO + 1) >= 100
#  include <Arduino.h>
#else
#  include <WProgram.h>
#endif

// Module constants
static const uint32_t VALID_POS_TIMEOUT = 2000;  // ms

const int buzzerPin = AUDIO_PIN;
const int tonefreq = 1020;
// constants for tone and rest durations
const int dotlength = 50;
const int dashlength = dotlength * 3;
// inter-element gap - between each dot or dash of a letter
const int inter = dotlength; 
// letter gap is 3 dots - the inter gap is always added - so this is one fewer
const int lgap = dotlength * 2; // inter-letter gap
// word gap is 7 dots - with letter and inter gap already counted, this is -1
const int wgap = dotlength * 4; //inter-word gap
// Module variables
static int32_t next_aprs = 0;
int ack = 0; // Acknowledge from Main. 

void setup()
{
  pinMode(LED_PIN, OUTPUT);
  pin_write(LED_PIN, LOW);
  pinMode(buzzerPin, OUTPUT);
  pinMode (12, INPUT); // Acknowledge pin from MAIN.  On CMD RX "beep" 250ms HIGHT
  Serial.begin(GPS_BAUDRATE);
//#ifdef DEBUG_RESET
  Serial.println("REBOOT...");
//#endif

  buzzer_setup();
  afsk_setup();
  gps_setup();
  sensors_setup();

//#ifdef DEBUG_SENS
  //Serial.print("Ti=");
  //Serial.print(sensors_int_lm60());
  //Serial.print(", Te=");
  //Serial.print(sensors_ext_lm60());
  //Serial.print(", Vin=");
  //Serial.println(sensors_vin());
//#endif

  // Do not start until we get a valid time reference
  // for slotted transmissions.
  if (APRS_SLOT >= 0) {
    do {
      while (! Serial.available())
        power_save();
    } while (! gps_decode(Serial.read()));
    
    next_aprs = millis() + 1000 *
      (APRS_PERIOD - (gps_seconds + APRS_PERIOD - APRS_SLOT) % APRS_PERIOD);
  }
  else {
    next_aprs = millis();
  }  
  // TODO: beep while we get a fix, maybe indicating the number of
  // visible satellites by a series of short beeps?
}

void get_pos()
{
  // Get a valid position from the GPS
  int valid_pos = 0;
  uint32_t timeout = millis();
  do {
    if (Serial.available())
      valid_pos = gps_decode(Serial.read());
  } while ( (millis() - timeout < VALID_POS_TIMEOUT) && ! valid_pos) ;

  if (valid_pos) {
    if (gps_altitude > BUZZER_ALTITUDE) {
      buzzer_off();   // In space, no one can hear you buzz
    } else {
      buzzer_on();
    }
  }
}

void loop()
{
  
  ack = digitalRead (12);
  if (ack == 1)
     {
  // Send Acknowledge Beeb to TX Radio
    digitalWrite(4, HIGH);// Key the radio pin 4
    delay(250); // Wait for Activated PTT on TX Radio
    // play a dash
  tone(buzzerPin, tonefreq);
  // LED
  digitalWrite(13, HIGH);
  delay(dashlength);
  noTone(buzzerPin);
  // LED
  digitalWrite(13, LOW);
  delay(inter);
    digitalWrite(4, LOW);// Release PTT pin 4
     }
  // Time for another APRS frame
  if ((int32_t) (millis() - next_aprs) >= 0) {
    digitalWrite(8, HIGH);// Heartbeat Signal to Main Nano (int pin 3)
    delay(250); // Wait for Activated PTT 
    digitalWrite(4, HIGH);// Key the radio pin 4
    
    delay(250); // Wait for Activated PTT on TX Radio
    
// Morse TX in ASCII Char : nanikana-aerospace   
    char thischar;
    thischar = 53;
    soundLetter(thischar);
    delay(lgap);
    thischar = 78;
    soundLetter(thischar);
    delay(lgap);
    thischar = 65;
    soundLetter(thischar);
    delay(lgap);
    thischar = 78;
    soundLetter(thischar);
    delay(lgap);
    thischar = 73;
    soundLetter(thischar);
    delay(lgap);
    thischar = 75;
    soundLetter(thischar);
    delay(lgap);
    thischar = 65;
    soundLetter(thischar);
    delay(lgap);
    thischar = 78;
    soundLetter(thischar);
    delay(lgap);
    thischar = 65;
    soundLetter(thischar);
    delay(lgap);
    delay(500);
    thischar = 65;
    soundLetter(thischar);
    delay(lgap);
    thischar = 69;
    soundLetter(thischar);
    delay(lgap);
    thischar = 82;
    soundLetter(thischar);
    delay(lgap);
    thischar = 79;
    soundLetter(thischar);
    delay(lgap);
    thischar = 83;
    soundLetter(thischar);
    delay(lgap);
    thischar = 80;
    soundLetter(thischar);
    delay(lgap);
    thischar = 65;
    soundLetter(thischar);
    delay(lgap);
    thischar = 67;
    soundLetter(thischar);
    delay(lgap);
    thischar = 69;
    soundLetter(thischar);
    
    // End Morse TX
    
    digitalWrite(4, LOW);// Release PTT pin 4
    delay(5000);// Delay before APRS TX
    get_pos();
    aprs_send();
    next_aprs += APRS_PERIOD * 1000L;
    while (afsk_flush()) {
      power_save();
    }
delay(250); // Time to Desactivated PTT on TX Radio
digitalWrite(8, LOW);// End of Heartbeat Signal
#ifdef DEBUG_MODEM
    // Show modem ISR stats from the previous transmission
    afsk_debug();
#endif
  }

  power_save(); // Incoming GPS data or interrupts will wake us up
}




void dot()
{
  // play a dot
  tone(buzzerPin, tonefreq);
  // LED
  digitalWrite(13, HIGH);
  delay(dotlength);
  noTone(buzzerPin);
  // LED
  digitalWrite(13, LOW);
  delay(inter);
}

void dash()
{
  // play a dash
  tone(buzzerPin, tonefreq);
  // LED
  digitalWrite(13, HIGH);
  delay(dashlength);
  noTone(buzzerPin);
  // LED
  digitalWrite(13, LOW);
  delay(inter);
}

void soundLetter(char letter)
{
  // letters are in order of frequency
  switch(letter)
  {
  case 'E':
    dot();
    return; 
  case 'T':
    dash();
    return; 
  case 'A':
    dot();
    dash();
    return;
  case 'O':
    dash();
    dash();
    dash();
    return; 
  case 'I':
    dot();
    dot();
    return;
  case 'N':
    dash();
    dot();
    return;
  case 'S':
    dot();
    dot();
    dot();
    return;
  case 'H':
    dot();
    dot();
    dot();
    dot();
    return;
  case 'R':
    dot();
    dash();
    dot();
    return;
  case 'D':
    dash();
    dot();
    dot();
    return;
  case 'L':
    dot();
    dash();
    dot();
    dot();
    return;
  case 'C':
    dash();
    dot();
    dash();
    dot();
    return;
  case 'U':
    dot();
    dot();
    dash();
    return;
  case 'M':
    dash();
    dash();
    return;
  case 'W':
    dot();
    dash();
    dash();
    return;
  case 'F':
    dot();
    dot();
    dash();
    dot();
    return;
  case 'G':
    dash();
    dash();
    dot();
    return;
  case 'Y':
    dash();
    dot();
    dash();
    dash();
    return;
  case 'P':
    dot();
    dash();
    dash();
    dot();
    return;
  case 'B':
    dash();
    dot();
    dot();
    dot();
    return;
  case 'V':
    dot();
    dot();
    dot();
    dash();
    return;
  case 'K':
    dash();
    dot();
    dash();
    return;
  case 'J':
    dot();
    dash();
    dash();
    dash();
    return;
  case 'X':
    dash();
    dot();
    dot();
    dash();
    return;
  case 'Q':
    dash();
    dash();
    dot();
    dash();
    return;
  case 'Z':
    dash();
    dash();
    dot();
    dot();
    return;
  case ' ':
    delay(wgap);
    return; 
  }
   }
