
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
String arduinoID = "";
bool sensors = false;


/* ============================================================ */
/* =======================Setup function======================= */
/* =============Runs once when Arduino is turned on============ */
void setup() {
  // initialize serial:
  Serial.begin(9600);
  // reserve 2000 bytes for the inputString:
  inputString.reserve(200);
  arduinoID = "Arduino" + String(char(EEPROM.read(0)));
  if (EEPROM.read(1) == '1') {
//    TODO: Set bool to true here when not testing so value is always set to '1'
    sensors = true;
  }
}

/* ============================================================ */
/* ====================Message Functions======================= */
/* ===========Run when the Arduino receives a message========== */

/*
 * Responds to a ping with the ArduinoID.
 */
void pingResponse() {
  // If the incoming message is a ping then respond with which arduino this is
      String resString;
      const int capacity = 100;
      StaticJsonBuffer<capacity> jb;
      JsonObject& res = jb.createObject();
      res["mType"] = "ping";
      res["deviceID"] = arduinoID;
      res["error"] = 0;
      res.printTo(Serial);
//      Serial.println(resdeString);
}

/*
 * Takes the servoID and pwmValue and sets the output for that 
 * 
 * Returns 0 on success
 * Return -1 on failure
 * 
 * TODO: Remove current code and properly implement setting of values.
 */
void setServo(String servoID, int pwmValue) {
      const int capacity = 100;
      StaticJsonBuffer<capacity> jb;
      JsonObject& res = jb.createObject();
      res["mType"] = "servo";
      res["deviceID"] = arduinoID;
      res["error"] = 0;
      res.printTo(Serial);
      Serial.println();
}

/*
 * Reads a value from the sensor which corresponds to partID. Prints that value to serial.
 * 
 * TODO: Remove current code and implement reading from sensor.
 */
void readSensor(String partID) {
      const int capacity = 100;
      StaticJsonBuffer<capacity> jb;
      JsonObject& res = jb.createObject();
      res["mType"] = "sensor";
      res["deviceID"] = arduinoID;
      res["partID"] = "sensor1";
      res["value"] = 10;
      res["error"] = 0;
      res.printTo(Serial);
      Serial.println();}

void returnError(String type, int code) {
      const int capacity = 100;
      StaticJsonBuffer<capacity> jb;
      JsonObject& res = jb.createObject();
      res["mType"] = type;
      res["deviceID"] = arduinoID;
      res["error"] = code;
      res.printTo(Serial);
      Serial.println();;
}

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
      returnError(root["mType"], 400);
      inputString = "";
      stringComplete = false;
      return;
      }

    if(root["mType"]=="ping"){
        pingResponse();
    }
    else if (root["mType"] == "servo"){
      String part = root["partID"];
      if (part == NULL) {
        returnError(root["mType"], 400);
      } else {
        setServo(root["partID"], root["value"]);   
      }
    } else {
      // Else send error
      returnError(root["mType"], 404);
    }

    // clear the string ready for the next input
    inputString = "";
    stringComplete = false;
  }
  if (sensors) {
    readSensor("test");
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
