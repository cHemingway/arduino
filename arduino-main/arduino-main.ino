/* Program to recieve and parse an incoming JSON string */
/* Main IO code based on https://www.arduino.cc/en/Tutorial/SerialEvent */
/* JSON parser code from https://arduinojson.org/v5/example/parser/ */

/* ============================================================ */
/* ======================Import libraries====================== */
/* ============================================================ */
#include <EEPROM.h> // Library for writing to Arduino's non volatile memory
#include <ArduinoJson.h> // JSON encoding and decoding
#include <Servo.h> // For controlling servos and thrusters
//IMU
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
//Depth
#include <Wire.h>
#include "MS5837.h"

/* ============================================================ */
/* ==================Set up global variables=================== */
/* ============================================================ */
String inputString = "";         // a String to hold incoming data
bool stringComplete = false;  // whether a full JSON string has been received
String arduinoID = "";
bool sensors = false;

unsigned long lastMessage;
bool safetyActive = false;




// TODO set up some sort of mapping from the JSON ID to device object

/* ============================================================ */
/* =======================Set up classes======================= */
/* ============================================================ */

/* ==========================Communication========================== */

// Class to handle sending values back up to the surface

class Communication{
  private:
    static const int elementCount = 20;
    String key[elementCount];
    String value[elementCount];
    int currentPosition = 0; // value of next free space
    
  public:
    void incrementPosition(){
      // increment currentValue and send if over limit
      currentPosition++;
      if(currentPosition>=elementCount){
        sendAll();
        currentPosition = 0;
      }
    }
    void bufferValue(String device, String incomingValue){
      // buffer a key value pair to be sent with next load
      key[currentPosition] = device;
      value[currentPosition] = incomingValue;
      incrementPosition();
    }
    void bufferError(String errorMessage){
      // buffer an error message to be sent with next load
      String tempKey = "error_" + String(char(EEPROM.read(0)));
      key[currentPosition] = tempKey;
      value[currentPosition] = errorMessage;
      incrementPosition();
    }
    void sendStatus(String status){
      // immediately sends current status to pi
      String resString;
      const int capacity = 100;
      StaticJsonBuffer<capacity> jb;
      JsonObject& res = jb.createObject();
      res["deviceID"] = arduinoID; // add Arduino ID to every message
      String tempKey = "status_" + String(char(EEPROM.read(0)));
      res[tempKey] = status;
      res.printTo(Serial);
      Serial.println();
    }
    void sendAll(){
      String resString;
      const int capacity = 1000; // Not sure about this size - probably needs calculating
      StaticJsonBuffer<capacity> jb;
      JsonObject& res = jb.createObject();
      res["deviceID"] = arduinoID; // add Arduino ID to every message
      for(int i = 0; i < currentPosition; i++){
        // prepare all buffered values
        res[key[i]] = value[i];
      }
      res.printTo(Serial);
      Serial.println();
      currentPosition = 0;
    }
};

Communication communication;

/* ==========================Abstract========================== */


class Input {
    // Designed to be a generic interface for all output devices.

  protected:
    int pin=0; // The physical pin this is associated with
    String partID="Part ID not set.";

  public:
    Input() {
      //Empty constructor to create harmless input
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
    int stoppedValue=0;
    int pin=0; // The physical pin this is associated with
    String partID="part ID not set";

  public:
    Output() {
      //Empty constructor to create harmless output
    }

    virtual int setValue(int inputValue) {
      int value = inputValue;
      // Method to set thrust capped to min and max

      if (value < minValue || value > maxValue) {
        // Send error message saying the incoming value was out of range
        communication.bufferError("Incoming value out of range.");
        return currentValue; // Keep output at same value
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

    virtual void constantTask(){
      // Something which needs to be run all the time
    }

    String getID (){
      return partID;
    }

    virtual void turnOff(){
      // Switch device off - for safety
    }
};


/* ===========================Inputs=========================== */


class IMU: public Input { 
    // Designed to be a generic interface for all output devices.

  protected:
    bool initialised = false;
    Adafruit_BNO055 imu = Adafruit_BNO055(55); // IMU device

  public:
    IMU(int inputPin, String incomingPartID){
      // Run parent method
      partID = incomingPartID;
      if(!imu.begin())
      {
        // Send error message
        communication.bufferError("IMU BNO055 not found. Check wiring.");
      }
      else{
        imu.setExtCrystalUse(true);
        initialised = true;
      }
    }

    int getValue() {
      if(initialised){
        // Send x, y, z data from this sensor
        /* Get a new sensor event */
        sensors_event_t event;
        imu.getEvent(&event);
        /* Output the floating point data */
        // x
        communication.bufferValue(this->partID+"_X",String(event.orientation.x));
  
        // y
        communication.bufferValue(this->partID+"_Y",String(event.orientation.y));
  
        // z
        communication.bufferValue(this->partID+"_Z",String(event.orientation.z));

        // Get temperature recorded by IMU
        int8_t temp = imu.getTemp();
        communication.bufferValue(this->partID+"_Temp",String(temp));

        // Get acceleration data
        imu::Vector<3> euler = imu.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);

        communication.bufferValue(this->partID+"_AccX",String(euler.x()));
        communication.bufferValue(this->partID+"_AccY",String(euler.y()));
        communication.bufferValue(this->partID+"_AccZ",String(euler.z()));

      }
      else{
        // Throw error because this sensor has not yet been initialised properly
        communication.bufferError("IMU BNO055 not initialised.");
      }
      
    }
};

class Depth: public Input {
    // Designed to be a generic interface for all output devices.

  protected:
    bool initialised = false;
    MS5837 depthSensor;

  public:
    Depth(int inputPin, String incomingPartID){
      Wire.begin();
      // Run parent method
      partID = incomingPartID;
      if(!depthSensor.init())
      {
        // Send error message
        communication.bufferError("Depth Sensor not found. Check wiring.");
      }
      else{
        depthSensor.setModel(MS5837::MS5837_30BA);
        depthSensor.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)
        initialised = true;
      }
    }

    int getValue() {
      if(initialised){
        depthSensor.read(); // Read current values
        communication.bufferValue(this->partID+"_Pres",String(depthSensor.pressure()));
        communication.bufferValue(this->partID+"_Temp",String(depthSensor.temperature()));
        communication.bufferValue(this->partID+"_Dep",String(depthSensor.depth()));
        communication.bufferValue(this->partID+"_Alt",String(depthSensor.altitude()));
        
      }
      else{
        // Throw error because this sensor has not yet been initialised properly
        communication.bufferError("Depth sensor not initialised.");
      }
      
    }
};

class PHSensor: public Input {
    // Designed to be a generic interface for all output devices.

  protected:
    int buf[10], temp; // Store 10 samples from the sensor for an accurate average
    unsigned long int avgValue;  //Store the average value of the sensor feedback
  public:
    PHSensor(int inputPin, String incomingPartID){
      partID = incomingPartID;
      pin = inputPin;
      
    }

    int getValue() {
      // This might need rethinking since it looks a bit s l o w
      
      for(int i=0;i<10;i++)       //Get 10 sample values from the sensor to smooth the result
      { 
        buf[i]=analogRead(pin);
        delay(1); // This delay might be too short
      }
      for(int i=0;i<9;i++)        //sort the analog from small to large
      {
        for(int j=i+1;j<10;j++)
        {
          if(buf[i]>buf[j])
          {
            temp=buf[i];
            buf[i]=buf[j];
            buf[j]=temp;
          }
        }
      }
      avgValue=0;
      for(int i=2;i<8;i++){                      //take the average value of 6 center sample
        avgValue+=buf[i];
      }
      float phValue=(float)avgValue*5.0/1024/6; //convert the analog into millivolt
      phValue=3.5*phValue;                      //convert the millivolt into pH value
      communication.bufferValue(this->partID,String(phValue)); // Send averaged sensor value
    }
};


/* ===========================Outputs=========================== */


class Thruster: public Output {

  protected:
    // Represents a thruster controlled by an ESC
    Servo thruster; // Using the Servo class because it uses the same values as our ESCs
    const int stoppedValue=1500;

  public:

    Thruster (int inputPin, String partID) {
      this->partID = partID;

      // Set limit and starting values
      maxValue = 1900;
      minValue = 1100;
      currentValue = stoppedValue;
      thruster.attach(inputPin); // Associate the thruster with the specified pin
      pin = inputPin; // Record the associated pin
      thruster.writeMicroseconds(stoppedValue); // Set value to "stopped"
    }


    int setValue(int inputValue) {
      // call parent logic (keeps value within preset boundary)
      int value = Output::setValue(inputValue);
      // Actually control the device
      thruster.writeMicroseconds(value);
      // Return the set value
      return value;
    }

    void turnOff(){
      //Serial.println(partID);
      // Switch off in case of emergency
      currentValue = stoppedValue;
      thruster.writeMicroseconds(stoppedValue);
    }
};


class ArmGripper: public Output {

  protected:
    // Represents a thruster motor opening and closing gripper
    Servo thruster;
    int leftLimit, rightLimit;
    const int stoppedValue=1500;

 public:

    ArmGripper(int inputPin, String partID, int limitPinLeft, int limitPinRight ) {
      this->partID = partID;

      // Set limit and starting values
      maxValue = 1900;
      minValue = 1100;
      currentValue = stoppedValue;
      
      thruster.attach(inputPin); // Associate the motor with the specified pin
      pin = inputPin; // Record the associated pin
      thruster.writeMicroseconds(stoppedValue); // Set value to "stopped"

      // Set limit switches
      leftLimit = limitPinLeft;
      rightLimit = limitPinRight;
      pinMode(limitPinLeft,INPUT_PULLUP);
      pinMode(limitPinRight,INPUT_PULLUP);
    }

    int setValue(int inputValue) {//                                                    TODO: in main loop check if hit limit all the time (update: done - please test)

        // call parent logic (keeps value within preset boundary)
        int value = Output::setValue(inputValue); // set currentValue accordingly
        if(hitLeftLimit() || hitRightLimit()){ // Checks if limit switch pressed and acts accordingly
          return currentValue;
        }
        // Actually control the device
        thruster.writeMicroseconds(value);
        // Return the set value
        return value;
    }

    bool hitLeftLimit(){ // check if a limit switch was hit
      if(digitalRead(leftLimit)==LOW && currentValue<stoppedValue){ // Low = pressed
        communication.bufferError("Left gripper limit hit. Motor stopped.");
        currentValue = stoppedValue;
        thruster.writeMicroseconds(currentValue);
        return true;
      }
      return false;
    }
    bool hitRightLimit(){ // check if a limit switch was hit
      //Serial.println("Pin is");
      //Serial.println(rightLimit);
      if(digitalRead(rightLimit)==LOW && currentValue>stoppedValue){ // Low = pressed
        communication.bufferError("Right gripper limit hit. Motor stopped.");
        currentValue = stoppedValue;
        thruster.writeMicroseconds(currentValue);
        return true;
      }
      return false;
    }

    void constantTask(){ // run in main loop: limit checking
      hitLeftLimit(); hitRightLimit();
    }

    void turnOff(){
      //Serial.println(partID);
      // Switch off in case of emergency
      currentValue = stoppedValue;
      thruster.writeMicroseconds(stoppedValue);
    }
};

class ArmRotation: public Output {

  protected:
    // Represents a servo controlling arm rotation
    Servo servo;
    const int stoppedValue=1500;
    
 public:

    ArmRotation (int inputPin, String partID) {
      this->partID = partID;

      // Set limit and starting values
      maxValue = 1650;
      minValue = 1350;
      currentValue = stoppedValue;
      servo.attach(inputPin); // Associate the servo with the specified pin
      pin = inputPin; // Record the associated pin
      servo.writeMicroseconds(stoppedValue); // Set value to "stopped"
    }


    int setValue(int inputValue) {
      // call parent logic (keeps value within preset boundary)
      int value = Output::setValue(inputValue);
      // Actually control the device
      servo.writeMicroseconds(value);
      // Return the set value
      return value;
    }

    void turnOff(){
      // Switch off in case of emergency
      currentValue = stoppedValue;
      servo.writeMicroseconds(stoppedValue);
    }
};

class Lamp: public Output { //todo

  protected:
    // Represents a dimmable light
    Servo led;
    const int stoppedValue=1100;
    
 public:

    Lamp (int inputPin, String partID) {
      this->partID = partID;

      // Set limit and starting values
      maxValue = 1900;
      minValue = 1100;
      currentValue = stoppedValue;
      led.attach(inputPin); // Associate with the specified pin
      pin = inputPin; // Record the associated pin
      led.writeMicroseconds(stoppedValue); // Set value to "stopped"
    }


    int setValue(int inputValue) {
      // call parent logic (keeps value within preset boundary)
      int value = Output::setValue(inputValue);
      // Actually control the device
      led.writeMicroseconds(value);
      // Return the set value
      return value;
    }

    void turnOff(){
      // Switch off in case of emergency
      //(Do nothing - it's just an LED)
    }
};

/* ==========================Mapper========================== */

// Maps device ID strings to object pointers

class Mapper {
  private:
    // t for Ard_T (Thrusters)
    const static int tCount=11;
    //const static int tCount=8;
    Output* tObjects[tCount];
    //String tIDs[tCount] = {"Thr_FP", "Thr_FS", "Thr_AP", "Thr_AS", "Thr_TFP", "Thr_TFS", "Thr_TAP", "Thr_TAS"};
    // Aberdeen change
    String tIDs[tCount] = {"Thr_FP", "Thr_FS", "Thr_AP", "Thr_AS", "Thr_TFP", "Thr_TFS", "Thr_TAP", "Thr_TAS", "Mot_R", "Mot_G", "Mot_F"};

    // i for Ard_I (Input)
    const static int iCount=3;
    Input* iObjects[iCount];
    String iIDs[iCount] = {"Sen_IMU", "Sen_Dep", "Sen_PH"};

    // a for Ard_A (Arm)
    const static int aCount=4;
    Output* aObjects[aCount];
    String aIDs[aCount] = {"Mot_R", "Mot_G", "Mot_F", "LED_M"};

    // m for Ard_M (Micro ROV)
    const static int mCount=1;
    Output* mObjects[mCount];
    String mIDs[mCount] = {"Thr_M"};

    
  public:
    void mapT(){
      // Map and initialise thrusters
      for ( int i = 0; i < tCount-3; i++) {
        tObjects[i] = new Thruster(2+i, tIDs[i]);
      }
      delay(2000);
      tObjects[8] = new ArmRotation(10, tIDs[8]);
      delay(2000);
      tObjects[9] = new ArmGripper(11, tIDs[9],26,27);
      delay(2000);
      tObjects[10] = new ArmGripper(12, tIDs[10],28,29); // Fish box
    }
    
    void mapI(){
      // Map and initialise inputs
      iObjects[0] = new IMU(0,iIDs[0]);
      iObjects[1] = new Depth(0,iIDs[1]);
      iObjects[2] = new PHSensor(56,iIDs[2]);
    }

    void mapA(){
      aObjects[0] = new ArmRotation(2, aIDs[0]);
      aObjects[1] = new ArmGripper(3, aIDs[1],10,11);
      aObjects[2] = new ArmGripper(4, aIDs[2],12,13); // Fish box
      aObjects[3] = new Lamp(5,aIDs[3]);
    }

    void mapM(){
      mObjects[0] = new Thruster(3,mIDs[0]);
      
    }
    
    Output* getOutput(String jsonID){
      if(arduinoID=="Ard_T"){
        for(int i = 0; i < tCount; i++){
          if(jsonID == tIDs[i]){
            return tObjects[i];
          }
        }
      }
      else if(arduinoID=="Ard_A"){
        for(int i = 0; i < aCount; i++){
          if(jsonID == aIDs[i]){
            return aObjects[i];
          }
        }
      }
      else if(arduinoID=="Ard_M"){
        for(int i = 0; i < mCount; i++){
          if(jsonID == mIDs[i]){
            return mObjects[i];
          }
        }
      }
      else{
        // Send error message saying the Arduino was not found
        String errorMessage = "getOutput method doesn't have an option for "+arduinoID;
        communication.bufferError(errorMessage);
        return new Output();
      }
      // Send error message saying the device was not found
      String errorMessage = "Output device ID is not valid: "+jsonID;
      communication.bufferError(errorMessage);
      return new Output();
    }
    
    Input* getInput(String jsonID){
      if(arduinoID=="Ard_I"){
        for(int i = 0; i < iCount; i++){
          if(jsonID == iIDs[i]){
            return iObjects[i];
          }
        }
      }
      else{
        // Send error message saying the Arduino was not found
        String errorMessage = "getInput method doesn't have an option for "+arduinoID;
        communication.bufferError(errorMessage);
        return new Input();
      }
      // Send error message saying the device was not found
      String errorMessage = "Input device ID is not valid: "+jsonID;
      communication.bufferError(errorMessage);
    }

    int getNumberOfInputs(){
      return iCount;
    }

    int getNumberOfOutputs(){
      if(arduinoID == "Ard_T"){
        return tCount;
      }
      else if(arduinoID == "Ard_A"){
        return aCount;
      }
      else if(arduinoID == "Ard_M"){
        return mCount;
      }
      return 0;
    }

    void sendAllSensors(){
      for(int i = 0; i < iCount; i++){
        iObjects[i]->getValue();
      }
      communication.sendAll();
    }

    void stopOutputs(){ // safety function to turn everything off
      if(arduinoID == "Ard_T"){
        for(int i = 0; i < tCount; i++){
          tObjects[i]->turnOff();
          delay(125); // delay 125ms between each thruster to avoid sudden power halt
        }
      }
      else if(arduinoID == "Ard_A"){
        for(int i = 0; i < aCount; i++){
          aObjects[i]->turnOff();
        }
      }
      else if(arduinoID == "Ard_M"){
        for(int i = 0; i < mCount; i++){
          mObjects[i]->turnOff();
        }
      }
      else{
        // Send error message saying the Arduino was not found
        communication.bufferError("Can't call stopOutputs from a non-output Arduino.");
      }
      communication.sendStatus("Outputs halted.");
    }
    
};

Mapper mapper; // Declare a new mapper object to map IDs to devices




/* ============================================================ */
/* =======================Setup function======================= */
/* =============Runs once when Arduino is turned on============ */
void setup() {
  arduinoID = "Ard_" + String(char(EEPROM.read(0)));
  
  // initialize serial:
  Serial.begin(9600);
  communication.sendStatus("Arduino Booting.");
  // reserve 2000 bytes for the inputString:
  inputString.reserve(200);


  // Map inputs and outputs based on which Arduino this is
  if (arduinoID == "Ard_T") {
    mapper.mapT();
  }
  else if (arduinoID == "Ard_I"){
    mapper.mapI();
  }
  else if (arduinoID == "Ard_A") {
    mapper.mapA();
  }
  else if (arduinoID == "Ard_M"){
    mapper.mapM();
  }
  communication.sendAll();
  communication.sendStatus("Arduino Active.");
}

/* ============================================================ */
/* =======================Loop function======================== */
/* ======Runs continuously after setup function finishes======= */
void loop() {  
  // parse the string when a newline arrives:
  if (stringComplete) {
    

    // DEBUG line for Kacper - prints the received string
    Serial.println(inputString);

    
    // Set up JSON parser
    StaticJsonBuffer<1000> jsonBuffer;
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

// START DIRTY HACK
// Return the received message back with BACK_ prepended to each key.

      String resString;
      const int capacity = 1000;
      StaticJsonBuffer<capacity> jb;
      JsonObject& res = jb.createObject();
      res["deviceID"] = arduinoID; // add Arduino ID to every message

     for(const auto& current: root){
        // For each incoming value
        String tempStuff = "BACK_"+String(current.key);
        res[tempStuff] = current.value;
      }
      res.printTo(Serial);
      Serial.println();

// END DIRTY HACK

    
    // Act on incoming message accordingly
    if(arduinoID=="Ard_T" || arduinoID=="Ard_M" || arduinoID=="Ard_A"){
      // This Arduino is for outputting
      for(const auto& current: root){
        // For each incoming value
        mapper.getOutput(current.key)->setValue(current.value);
      }
    }
    else if (arduinoID=="Ard_I"){
      
    }
    else{
      communication.bufferError("Arduino ID not set up. This Arduino will not function");
    }

    // Finish by sending all the values
    communication.sendAll();
    // clear the string ready for the next input
    inputString = "";
    stringComplete = false;
    

    // Update time last message received
    lastMessage = millis();
    
  }

  // Code to run all the time goes here:

  if(arduinoID=="Ard_A"){
    //mapper.getOutput("Mot_G")->constantTask(); // Keep checking if gripper limit hit (TODO: automatically run all constant tasks)
    //mapper.getOutput("Mot_F")->constantTask(); 
  }

  
  if(arduinoID=="Ard_T" || arduinoID=="Ard_M" || arduinoID=="Ard_A"){
    // This Arduino is for outputting
    // Check if it's been too long since last message - bad sign
    // Turn everything off
    if(millis() - lastMessage > 1000 && !safetyActive){ // 1 second limit
      safetyActive = true; //activate safety
      communication.bufferError("No incoming data received for more than 1 second. Switching all devices off");
      communication.sendAll();
      mapper.stopOutputs();
    }
  }
  else if(arduinoID=="Ard_I"){
    // Output all sensor data
      mapper.sendAllSensors();
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
//      Serial.println(inputString);
      stringComplete = true;
      break;
    }
    inputString += inChar;
   
  }
}
