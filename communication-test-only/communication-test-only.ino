/* Program to recieve and parse an incoming JSON string */
/* Main IO code based on https://www.arduino.cc/en/Tutorial/SerialEvent */
/* JSON parser code from https://arduinojson.org/v5/example/parser/ */

/* ============================================================ */
/* ======================Import libraries====================== */
/* ============================================================ */
#include <EEPROM.h> // Library for writing to Arduino's non volatile memory
#include <ArduinoJson.h> // JSON encoding and decoding
#include <Servo.h> // For controlling servos and thrusters
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

/* ============================================================ */
/* ==================Set up global variables=================== */
/* ============================================================ */
String inputString = "";         // a String to hold incoming data
bool stringComplete = false;  // whether a full JSON string has been received
String arduinoID = "";

// TODO set up some sort of mapping from the JSON ID to device object

/* ============================================================ */
/* =======================Set up classes======================= */
/* ============================================================ */

/* ==========================Communication========================== */

// Class to handle sending values back up to the surface

class Communication{
  private:
    
  public:
    static void sendValue(String device, String value){
      String resString;
      const int capacity = 100;
      StaticJsonBuffer<capacity> jb;
      JsonObject& res = jb.createObject();
      res["deviceID"] = arduinoID;
      res[device] = value;
      res.printTo(Serial);
      Serial.println();
    }
    static void sendError(String errorMessage){
      String resString;
      const int capacity = 100;
      StaticJsonBuffer<capacity> jb;
      JsonObject& res = jb.createObject();
      res["deviceID"] = arduinoID;
      res["error"] = errorMessage;
      res.printTo(Serial);
      Serial.println();
    }
};

Communication communication;


/* ============================================================ */
/* =======================Setup function======================= */
/* =============Runs once when Arduino is turned on============ */
void setup() {


  // initialize serial:
  Serial.begin(9600);
  // reserve 2000 bytes for the inputString:
  inputString.reserve(200);
  arduinoID = "Ard-" + String(char(EEPROM.read(0)));

}

/* ============================================================ */
/* ====================Message Functions======================= */
/* ===========Run when the Arduino receives a message========== */



/* ============================================================ */
/* =======================Loop function======================== */
/* ======Runs continuously after setup function finishes======= */
void loop() {
  // print the string when a newline arrives:
  if (stringComplete) {

    // Set up JSON parser
    StaticJsonBuffer<100> jsonBuffer;
    JsonObject& root = jsonBuffer.parseObject(inputString);

    // Test if parsing succeeds.
    if (!root.success()) {
      communication.sendError("parseObject() failed");
      inputString = "";
      stringComplete = false;
      return;
    }
    communication.sendValue("example","100");
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
    if (inChar == '\n' || inChar == '\r') {
      stringComplete = true;
      break;
    }
    inputString += inChar;
    // if the incoming character is a close brace, set a flag so the main loop will parse the whole string

  }
}
