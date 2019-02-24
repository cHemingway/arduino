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

// TODO set up some sort of mapping from the JSON ID to device object

/* ============================================================ */
/* =======================Set up classes======================= */
/* ============================================================ */

class Input {
    // Designed to be a generic interface for all output devices.

  protected:
    int pin; // The physical pin this is associated with
    String partID;

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
    int maxValue;
    int minValue;
    int currentValue;
    int pin; // The physical pin this is associated with
    String partID;

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

      if(outOfRange){
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


  //TESTING
   Serial.println("Start Test");
    Output *test = new Thruster();
    test->init(2,"Thr-FP");
    test->setValue(5);
    test->setValue(1500);
    test->setValue(2000);
    Serial.println("End Test");
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

/*
   Takes the servoID and pwmValue and sets the output for that

   Returns 0 on success
   Return -1 on failure

   TODO: Remove current code and properly implement setting of values.
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
   Reads a value from the sensor which corresponds to partID. Prints that value to serial.

   TODO: Remove current code and implement reading from sensor.
*/
void readSensor(String partID) {
  const int capacity = 100;
  StaticJsonBuffer<capacity> jb;
  JsonObject& res = jb.createObject();
  res["mType"] = "sensor";
  res["deviceID"] = arduinoID;
  res["partID"] = "sensor1";
  res["value"] = 10;
  res.printTo(Serial);
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
