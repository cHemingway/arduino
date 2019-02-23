#include <EEPROM.h>

void setup() {
  // initialize the LED pin as an output.
  pinMode(13, OUTPUT);
 // turn the LED off before success
  digitalWrite(13, LOW);

  EEPROM.write(O, 'O'); // Mark this as Arduino O for Output
  EEPROM.write(1, '1');
 
 // turn the LED on when we're done
  digitalWrite(13, HIGH);
}

void loop() {
  /** Empty loop. **/
}
