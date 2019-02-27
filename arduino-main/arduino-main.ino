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
      res[device] = value;
      res.printTo(Serial);
      Serial.println();
    }
    static void sendError(String errorMessage){
      String resString;
      const int capacity = 100;
      StaticJsonBuffer<capacity> jb;
      JsonObject& res = jb.createObject();
      res["error"] = errorMessage;
      res.printTo(Serial);
      Serial.println();
    }
};

Communication communication;

/* ==========================Abstract========================== */


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

      if (value < minValue || value > maxValue) {
        // Send error message saying the incoming value was out of range
        communication.sendError("Incoming value out of range.");
      }
      else{
        currentValue = value;
      }
      return value;
      // Then set the value on the device
    }

    virtual int getValue() {
      // Get the current value of this device (EG: Servo position)
      return currentValue;
    }
};


/* ===========================Inputs=========================== */


class IMU: public Input {
    // Designed to be a generic interface for all output devices.

  protected:
    bool initialised = false;
    Adafruit_BNO055 bno = Adafruit_BNO055(55); // IMU device

  public:
    virtual void init(int inputPin, String incomingPartID) {
      // Run parent method
      Input::init(inputPin, incomingPartID);
      if(!bno.begin())
      {
        // Send error message
        communication.sendError("IMU BNO055 not found. Check wiring.");
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
        communication.sendValue(partID+'x',String(event.orientation.x));
  
        // y
        communication.sendValue(partID+'y',String(event.orientation.y));
  
        // z
        communication.sendValue(partID+'z',String(event.orientation.z));
      }
      else{
        // Throw error because this sensor has not yet been initialised properly
        communication.sendError("IMU BNO055 not initialised.");
      }
      
    }
};


/* ===========================Outputs=========================== */


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

/* ==========================Mapper========================== */

// Maps device ID strings to object pointers

class Mapper {
  private:
    const static int numberOfOutputs=100;
    static Output* OutputObjects[numberOfOutputs];
    static String OutputIDs[numberOfOutputs];

    const static int numberOfInputs=100;
    static Input* InputObjects[numberOfInputs];
    static String InputIDs[numberOfInputs];
    
  public:
    static void mapOutputs(){
      // Map and initialise outputs
      int i = 0; // Current position
      OutputIDs[i] = "Thr-FP";
      OutputObjects[i] = new Thruster();
      i = i++;
      OutputIDs[i] = "Thr-FS";
      OutputObjects[i] = new Thruster();
      i = i++;
      OutputIDs[i] = "Thr-AP";
      OutputObjects[i] = new Thruster();
      i = i++;
      OutputIDs[i] = "Thr-AS";
      OutputObjects[i] = new Thruster();
      i = i++;
      OutputIDs[i] = "Thr-TFP";
      OutputObjects[i] = new Thruster();
      i = i++;
      OutputIDs[i] = "Thr-TFS";
      OutputObjects[i] = new Thruster();
      i = i++;
      OutputIDs[i] = "Thr-TAP";
      OutputObjects[i] = new Thruster();
      i = i++;
      OutputIDs[i] = "Thr-TAS";
      OutputObjects[i] = new Thruster();
      i = i++;
      OutputIDs[i] = "Thr-M";
      OutputObjects[i] = new Thruster();
    }
    static void mapInputs(){
      // Map and initialise inputs
      
    }
    static Output* getOutput(String jsonID){
      for(int i = 0; i < numberOfOutputs; i++){
        if(jsonID == OutputIDs[i]){
          return OutputObjects[i];
        }
      }

      // Send error message saying the device was not found
      String errorMessage = "Output device ID is not valid: "+jsonID;
      communication.sendError(errorMessage);
    }
    static Input* getInput(String jsonID){
      for(int i = 0; i < numberOfInputs; i++){
        if(jsonID == InputIDs[i]){
          return InputObjects[i];
        }
      }

      // Send error message saying the device was not found
      String errorMessage = "Input device ID is not valid: "+jsonID;
      communication.sendError(errorMessage);
    }
    
};

Mapper mapper; // Declare a new mapper object to map IDs to devices




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
    mapper.mapOutputs;
    // This is an output Arduino
  }
  else if (arduinoID == "Ard-I"){
    mapper.mapInputs;
  }
  mapper.getOutput("Thr-FP")->init(2,"Thr-FP");
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
        //setServo(root["partID"], root["value"]);
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
    //readSensor("test");
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
