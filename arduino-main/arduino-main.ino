/* Program to recieve and parse an incoming JSON string */
/* Main IO code based on https://www.arduino.cc/en/Tutorial/SerialEvent */
/* JSON parser code from https://arduinojson.org/v5/example/parser/ */

/* ============================================================ */
/* ======================Import libraries====================== */
/* ============================================================ */
#include <EEPROM.h> // Library for writing to Arduino's non volatile memory
#include <ArduinoJson.h> // JSON encoding and decoding
#include <Servo.h> // For controlling servos and thrusters
//#include <HashMap.h>

/* ============================================================ */
/* ==================Set up global variables=================== */
/* ============================================================ */
String inputString = "";         // a String to hold incoming data
bool stringComplete = false;  // whether a full JSON string has been received
String arduinoID = "";
bool sensors = false;

// TODO set up some sort of mapping from the JSON ID to actual device

//// Mapping devices to pins
////define the max size of the hashtable
//const byte HASH_SIZE = 50;
////storage
//HashType<char*,int> hashRawArray[HASH_SIZE];
////handles the storage [search,retrieve,insert]
//HashMap<char*,int> deviceMap = HashMap<char*,int>( \hashRawArray , HASH_SIZE );
//
//
//
//
//void mapDevicesToPins(){
//  // Set Values
//    deviceMap[0]("Thr-M",0);
//    deviceMap[1]("Thr-FP",1);
//    deviceMap[2]("Thr-FS",2);
//    deviceMap[3]("Thr-AP",3);
//    deviceMap[4]("Thr-AS",4);
//    deviceMap[5]("Thr-TFP",5);
//    deviceMap[6]("Thr-TFS",6);
//    deviceMap[7]("Thr-TAP",7);
//    deviceMap[8]("Thr-TAS",8);
//    
//
//  // Initialise pins
//}

/* ============================================================ */
/* =======================Set up classes======================= */
/* ============================================================ */

class Thruster {
  
  // Represents a thruster controlled by an ESC
  const int maxThrust = 1900;
  const int minThrust = 1100;
  int currentThrust = 1500;
  int pin; // The physical pin this is associated with
  Servo thruster; // Using the Servo class because it uses the same values as our ESCs
  
  public:
  
    void init(int inputPin){
      // Run this method in setup() to initialise a thruster
      thruster.attach(inputPin); // Associate the thruster with the specified pin
      pin = inputPin; // Record the associated pin
      thruster.writeMicroseconds(1500); // Set value to "stopped"
    }
    
    void setThrust(int inputThrust){
      int thrust = inputThrust;
      // Method to set thrust capped to min and max
      if (thrust > maxThrust){
        thrust = maxThrust;
      }
      else if (thrust < minThrust){
        thrust = minThrust;
      }
      thruster.writeMicroseconds(thrust);
      currentThrust = thrust;
    }

    int getThrust(){
      return currentThrust;
    }
};

/* ============================================================ */
/* =======================Setup function======================= */
/* =============Runs once when Arduino is turned on============ */
void setup() {  
  // initialize serial:
  Serial.begin(9600);
  // reserve 2000 bytes for the inputString:
  inputString.reserve(200);
  arduinoID = "Ard-" + String(char(EEPROM.read(0)));

  if (arduinoID == "Ard-O") {
    // This is an output Arduino

    // TODO: Set up thrusters
  }
  
  if (EEPROM.read(1) == '1') {
//    TODO: Set bool to true here when not testing so value is always set to '1'
    sensors = false;
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
      res.printTo(Serial);
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
      res.printTo(Serial);}

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
      Serial.println("parseObject() failed");// Remove/change this line in production code
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
        Serial.println("Invalid message format");
      } else {
        setServo(root["partID"], root["value"]);   
      }
    } else {
      // Else just respond with the received message to ensure it was received
      Serial.print(inputString);
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
