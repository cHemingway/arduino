/* CUT DOWN APPLICATION DESIGNED FOR MICRO ROV */


/* Main IO code based on https://www.arduino.cc/en/Tutorial/SerialEvent */
/* JSON parser code from https://arduinojson.org/v5/example/parser/ */

/* ============================================================ */
/* ======================Import libraries====================== */
/* ============================================================ */
#include <EEPROM.h> // Library for writing to Arduino's non volatile memory
#include <ArduinoJson.h> // JSON encoding and decoding
#include <Servo.h> // For controlling servos and thrusters


/* ============================================================ */
/* ==================Set up global variables=================== */
/* ============================================================ */
String inputString = "";         // a String to hold incoming data
bool stringComplete = false;  // whether a full JSON string has been received
String arduinoID = "";
unsigned long lastMessage;
bool safetyActive = false;

/* ============================================================ */
/* =======================Set up classes======================= */
/* ============================================================ */

/* ==========================Communication========================== */

// Class to handle sending values back up to the surface

class Communication{
  private:
    String messageContents = "";
    
  public:
    void bufferValue(String device, String incomingValue){
      // buffer a key value pair to be sent with next load
      messageContents+=",\""+device;
      messageContents+="\":\"";
      messageContents+=incomingValue+"\"";
    }
    void bufferError(String errorMessage){
      messageContents+=",\"error_"+String(char(EEPROM.read(0)));
      messageContents+="\":\"";
      messageContents+=errorMessage+"\"";
    }
    void sendStatus(String status){
      //Hardcoded JSON
      Serial.print("{\"deviceID\":\"");
      Serial.print(arduinoID);
      Serial.print("\",\"status_");
      Serial.print(String(char(EEPROM.read(0))));
      Serial.print("\":\"");
      Serial.print(status);
      Serial.println("\"}");
    }
    void sendAll(){
      Serial.print("{\"deviceID\":\"");
      Serial.print(arduinoID);
      Serial.print("\"");
      Serial.print(messageContents);
      Serial.println("}");
      messageContents="";
    }
};

Communication communication;

class Thruster {

  protected:
    int maxValue=0;
    int minValue=0;
    int currentValue=0;
    int pin=0; // The physical pin this is associated with
    String partID="part ID not set";

    // Represents a thruster controlled by an ESC
    Servo thruster; // Using the Servo class because it uses the same values as our ESCs
    const int stoppedValue=1500;

  public:

    Thruster (int inputPin, String incomingPartID) {
      partID = incomingPartID;

      // Set limit and starting values
      maxValue = 1900;
      minValue = 1100;
      currentValue = stoppedValue;
      thruster.attach(inputPin); // Associate the thruster with the specified pin
      pin = inputPin; // Record the associated pin
      thruster.writeMicroseconds(stoppedValue); // Set value to "stopped"
    }


    int setValue(int inputValue) {
      if (inputValue < minValue || inputValue > maxValue) {
        communication.bufferError("Incoming value out of range.");
        return currentValue; // Keep output at same value
      }
      else{
        currentValue = inputValue;
        // Actually control the device
        thruster.writeMicroseconds(inputValue);
        // Return the set value
        return inputValue;
      }
      
    }

    void turnOff(){
      // Switch off in case of emergency
      currentValue = stoppedValue;
      thruster.writeMicroseconds(stoppedValue);
    }
};

Thruster* thr;
    

/* ============================================================ */
/* =======================Setup function======================= */
/* =============Runs once when Arduino is turned on============ */

void setup() {
  arduinoID = "Ard_" + String(char(EEPROM.read(0)));
  
  // initialize serial:
  Serial.begin(9600);
  communication.sendStatus("Arduino Booting.");
  inputString.reserve(30);
  if(arduinoID!="Ard_M"){
    communication.bufferError("You are attempting to run Arduino M code on a different Arduino");
    while(true){}
  }
  thr = new Thruster(3,"Thr_M");
  
  communication.sendAll();
  communication.sendStatus("Arduino Active.");
}

void loop() {
  // parse the string when a newline arrives:
  if (stringComplete) {
    // Set up JSON parser
    StaticJsonBuffer<50> jsonBuffer;
    JsonObject& root = jsonBuffer.parseObject(inputString);
    // Test if parsing succeeds.
    if (!root.success()) {
      communication.bufferError("JSON parsing failed.");
      communication.sendAll();
      inputString = "";
      stringComplete = false;
      return;
    }
    safetyActive = false; // Switch off auto-off because valid message received
    if(arduinoID=="Ard_M"){
      thr->setValue(root["Thr_M"]);
    }
    else{
      communication.bufferError("You are attempting to run Arduino M code on a different Arduino");
    }

    // Finish by sending all the values
    communication.sendAll();
    // clear the string ready for the next input
    inputString = "";
    stringComplete = false;

    // Update time last message received
    lastMessage = millis();
    
  }
    // Check if it's been too long since last message - bad sign
    // Turn everything off
    if(millis() - lastMessage > 1000 && !safetyActive){ // 1 second limit
      safetyActive = true; //activate safety
      communication.bufferError("No incoming data received for more than 1 second. Switching all devices off");
      communication.sendAll();
      thr->turnOff();
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
  }
}
