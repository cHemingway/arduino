/* Program to recieve and parse an incoming JSON string */
/* Main IO code based on https://www.arduino.cc/en/Tutorial/SerialEvent */
/* JSON parser code from https://arduinojson.org/v5/example/parser/ */

/* ============================================================ */
/* ======================Import libraries====================== */
/* ============================================================ */
#include <EEPROM.h> // Library for writing to Arduino's non volatile memory
#include <ArduinoJson.h>

/* ============================================================ */
/* ==================Set up global variables=================== */
/* ============================================================ */
String inputString = "";         // a String to hold incoming data
bool stringComplete = false;  // whether a full JSON string has been received


/* ============================================================ */
/* =======================Setup function======================= */
/* =============Runs once when Arduino is turned on============ */
void setup() {
  // Write to Arduino storage that this is Arduino A (Only need to do once)
  // EEPROM.write(0, 'A');
  
  // initialize serial:
  Serial.begin(9600);
  // reserve 2000 bytes for the inputString:
  inputString.reserve(2000);
}

/* ============================================================ */
/* =======================Loop function======================== */
/* ======Runs continuously after setup function finishes======= */
void loop() {
  // print the string when a newline arrives:
  if (stringComplete) {
    
    // Set up JSON parser
    StaticJsonBuffer<200> jsonBuffer;
    JsonObject& root = jsonBuffer.parseObject(inputString);

    // Test if parsing succeeds.
    if (!root.success()) {
      Serial.println("parseObject() failed"); // Remove/change this line in production code
      return;
    }

    if(root["mType"]=="ping"){
      // If the incoming message is a ping then respond with which arduino this is
      Serial.print("{\"mtype\":\"ping\",\"deviceID\":\"Arduino");
      Serial.print(char(EEPROM.read(0))); // Read arduino ID from memory
      Serial.println("\"}");
    }
    else{
      // Else just respond with the received message to ensure it was received
      Serial.print(inputString);
    }
    // clear the string ready for the next input
    inputString = "";
    stringComplete = false;
  }
}

/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a close brace, set a flag so the main loop will parse the whole string
    if (inChar == '}') {
      stringComplete = true;
    }
  }
}
