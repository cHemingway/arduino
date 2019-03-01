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
bool sensors = false;

Adafruit_BNO055 bno = Adafruit_BNO055(55); // IMU device

// TODO set up some sort of mapping from the JSON ID to device object

/* ============================================================ */
/* =======================Set up classes======================= */
/* ============================================================ */



class Input {
    // Designed to be a generic interface for all output devices.

  protected:
    int pin=0; // The physical pin this is associated with
    String partID="part ID not set";

  public:
    virtual void init(int inputPin, String incomingPartID) {
      partID = incomingPartID;
      // Run this method in setup() to initialise a device
    }

    virtual int getValue() {
      // Get the current value of this device (EG: Temperature)
    }
};

class IMU: public Input {
    // Designed to be a generic interface for all output devices.

  protected:
    bool initialised = false;

  public:
    virtual void init(int inputPin, String incomingPartID) {
      // Run parent method
      Input::init(inputPin, incomingPartID);
      if(!bno.begin())
      {
        // Send error message saying the incoming value was out of range
        String resString;
        const int capacity = 100;
        StaticJsonBuffer<capacity> jb;
        JsonObject& res = jb.createObject();
        res["mType"] = "error";
        res["deviceID"] = arduinoID;
        res["partID"] = partID;
        res["value"] = "IMU BNO055 not found. Check wiring.";
        res.printTo(Serial);
      }
      else{
        bno.setExtCrystalUse(true);
        initialised = true;
      }
    }

    virtual int getValue() {
      if(initialised){
        // Send x, y, z data from this sensor
        /* Get a new sensor event */
        sensors_event_t event;
        bno.getEvent(&event);
        /* Output the floating point data */
        // x
        String resString;
        const int capacity = 100;
        StaticJsonBuffer<capacity> jb;
        JsonObject& res = jb.createObject();
        res["mType"] = "sensor";
        res["deviceID"] = arduinoID;
        res["partID"] = partID+'x';
        res["value"] = event.orientation.x;
        res.printTo(Serial);
  
        // y
        res["partID"] = partID+'y';
        res["value"] = event.orientation.y;
        res.printTo(Serial);
  
        // z
        res["partID"] = partID+'z';
        res["value"] = event.orientation.z;
        res.printTo(Serial);
      }
      else{
        // Throw error because this sensor has not yet been initialised properly
        // Send error message saying the incoming value was out of range
        String resString;
        const int capacity = 100;
        StaticJsonBuffer<capacity> jb;
        JsonObject& res = jb.createObject();
        res["mType"] = "error";
        res["deviceID"] = arduinoID;
        res["partID"] = partID;
        res["value"] = "Sensor not initialised.";
        res.printTo(Serial);
      }
      
    }
};

class Output {
    // Designed to be a generic interface for all output devices.

  protected:
    int maxValue=0;
    int minValue=0;
    int currentValue=0;
    int pin=0; // The physical pin this is associated with
    String partID="part ID not set";

  public:
    virtual void init(int inputPin, String incomingPartID) {
      partID = incomingPartID;
      // Run this method in setup() to initialise a device
    }

    virtual int setValue(int inputValue) {
      int value = inputValue;
      // Method to set thrust capped to min and max
      bool outOfRange = false;
      if (value > maxValue) {
        value = maxValue;
        outOfRange = true;
      }
      else if (value < minValue) {
        value = minValue;
        outOfRange = true;
      }

      if (outOfRange) {
        // Send error message saying the incoming value was out of range
        String resString;
        const int capacity = 100;
        StaticJsonBuffer<capacity> jb;
        JsonObject& res = jb.createObject();
        res["mType"] = "error";
        res["deviceID"] = arduinoID;
        res["partID"] = partID;
        res["value"] = "Incoming value out of range.";
        res.printTo(Serial);
      }

      currentValue = value;
      return value;
      // Then set the value on the device
    }

    virtual int getValue() {
      // Get the current value of this device (EG: Servo position)
      return currentValue;
    }
};

class Thruster: public Output {

  protected:
    // Represents a thruster controlled by an ESC
    Servo thruster; // Using the Servo class because it uses the same values as our ESCs

  public:

    void init(int inputPin, String partID) {
      // Run this method in setup() to initialise a thruster

      // Run parent method
      Output::init(inputPin, partID);

      // Set limit and starting values
      maxValue = 1900;
      minValue = 1100;
      currentValue = 1500;

      thruster.attach(inputPin); // Associate the thruster with the specified pin
      pin = inputPin; // Record the associated pin
      thruster.writeMicroseconds(1500); // Set value to "stopped"
    }

    int setValue(int inputValue) {
      // call parent logic (keeps value within preset boundary)
      int value = Output::setValue(inputValue);
      // Actually control the device
      thruster.writeMicroseconds(value);
      // Return the set value
      return value;
    }
};

class ArmMotor: public Output {

  protected:
    // Represents a thruster controlled by an ESC
    Servo servo; // Using the Servo class for servos

  public:

    void init(int inputPin, String partID) {
      // Run this method in setup() to initialise a thruster

      // Run parent method
      Output::init(inputPin, partID);

      // Set limit and starting values
      maxValue = 1900;
      minValue = 1100;
      currentValue = 1500;

      servo.attach(inputPin); // Associate the thruster with the specified pin
      pin = inputPin; // Record the associated pin
      servo.writeMicroseconds(1500); // Set value to "stopped"
    }

    int setValue(int inputValue) {
      // call parent logic (keeps value within preset boundary)
      int value = Output::setValue(inputValue);
      // Actually control the device
      servo.writeMicroseconds(value);
      // Return the set value
      return value;
    }
};

class Mapper {
  protected:
    const static int numberOfOutputs=12;
    static Output OutputObjects[numberOfOutputs];
    static String OutputIDs[numberOfOutputs];

    const static int numberOfInputs=12;
    static Input InputObjects[numberOfInputs];
    static String InputIDs[numberOfInputs];
    
  public:
    static void mapOutputs(){
      
    }
    static void mapInputs(){
      
    }
    static Output* getOutput(String jsonID){
      for(int i = 0; i < numberOfOutputs; i++){
        if(jsonID == OutputIDs[i]){
          return &OutputObjects[i];
        }
      }

      // Send error message saying the device was not found
      String resString;
      const int capacity = 100;
      StaticJsonBuffer<capacity> jb;
      JsonObject& res = jb.createObject();
      res["mType"] = "error";
      res["deviceID"] = arduinoID;
      res["partID"] = jsonID;
      res["value"] = "This partID does not exist.";
      res.printTo(Serial);
    }
    static Input* getInput(String jsonID){
      for(int i = 0; i < numberOfInputs; i++){
        if(jsonID == InputIDs[i]){
          return &InputObjects[i];
        }
      }

      // Send error message saying the device was not found
      String resString;
      const int capacity = 100;
      StaticJsonBuffer<capacity> jb;
      JsonObject& res = jb.createObject();
      res["mType"] = "error";
      res["deviceID"] = arduinoID;
      res["partID"] = jsonID;
      res["value"] = "This partID does not exist.";
      res.printTo(Serial);
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
   Responds to a ping with the ArduinoID.
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

    if (root["mType"] == "ping") {
      pingResponse();
    }
    else if (root["mType"] == "servo") {
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
