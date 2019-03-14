#include <EEPROM.h>

void setup() {
  // initialize the LED pin as an output.
  pinMode(13, OUTPUT);
  // turn the LED off before success
  digitalWrite(13, LOW);
  // Mark this as Arduino M for Micro ROV
  EEPROM.write(0, 'M'); 
  // turn the LED on when we're done
  digitalWrite(13, HIGH);
}

void loop() {
  /** Empty loop. **/
}
