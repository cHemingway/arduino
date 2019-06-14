/* NURovers 2019
 *  
 * Metal Detector Freq 
 *  
 * Frequency to serial for metal detector
 * Reads frequency from pin D5 and outputs to serial in Hz
 * Runs on Arduino Nano, 
 * 
 * Requires FreqCount library, install through Tools->ManageLibraries 
 * Docs: https://www.pjrc.com/teensy/td_libs_FreqCount.html
 */
 
#include <FreqCount.h>

// Flash LED every reading
const int led_pin = 13;

// Update at 20Hz
const unsigned update_delay_ms = 50;

void setup() {
  Serial.begin(9600);
  pinMode(13, OUTPUT);
  FreqCount.begin(1000);
}

void loop() {
  if (FreqCount.available()) {
    digitalWrite(led_pin, HIGH);
    unsigned long count = FreqCount.read();
    Serial.println(count);
    digitalWrite(led_pin, LOW);
  }
  // Delay updates to not overwhelm reciever
  delay(update_delay_ms);
}
