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
//Sonar
#include "ping1d.h"

/* ============================================================ */
/* ==================Set up global variables=================== */
/* ============================================================ */
String inputString = "";         // a String to hold incoming data
bool stringComplete = false;  // whether a full JSON string has been received
String arduinoID = "";  // JSON ID representing this Arduino (read from long-term memory)
unsigned long lastMessage; // The timestamp of when the last message was received
bool safetyActive = false; // Whether output devices are stopped because no data was received

/* ============================================================ */
/* =======================Set up classes======================= */
/* ============================================================ */

/* ==========================Communication========================== */

/*
  The communication class is the implementation of the method of communicating with the Raspberry Pi.
  External methods can buffer data to be sent to the Pi, and this class will handle conversion to JSON and other formatting.
*/
class Communication{
  private:
    static const int elementCount = 20;
    String key[elementCount];
    String value[elementCount];
    int currentPosition = 0; // value of next free space
    
  public:
    /*
      Increment currentValue and send all values if buffer is full
    */
    void incrementPosition(){

      currentPosition++;
      if(currentPosition>=elementCount){
        sendAll();
        currentPosition = 0;
      }
    }

    /*
      Buffer a key:value pair to be sent to the Pi
    */
    void bufferValue(String device, String incomingValue){
      // buffer a key value pair to be sent with next load
      key[currentPosition] = device;
      value[currentPosition] = incomingValue;
      incrementPosition();
    }

    /*
      Buffer an error message to be sent to the PI
    */
    void bufferError(String errorMessage){
      // buffer an error message to be sent with next load
      String tempKey = "error_" + String(char(EEPROM.read(0)));
      key[currentPosition] = tempKey;
      value[currentPosition] = errorMessage;
      incrementPosition();
    }

    /*
      Send the current status of this Arduino (e.g. booting)
    */
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

    /*
      Send all buffered values to the Pi
    */
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

Communication communication; // Object to handle communication between Arduino and Pi

/* ==========================Abstract========================== */

/*
  An abstract Input which has generic fields for the associated physical pin and JSON ID.
  Also has generic method for reading the values from a sensor.
*/
class Input {

  protected:
    int pin=0; // The physical pin this is associated with
    String partID="Part ID not set."; // The JSON Part ID of this device

  public:
    Input() {
      //Empty constructor to create harmless input
    }

    // Get the current value of this device (EG: Temperature)
    virtual int getValue() {
      
    }
};

/*
  An abstract Output which has generic fields for the associated physical pin, min value, max value, stopped value, current value, and JSON ID.
  Also has generic methods for setting a new value (e.g. motor speed) with validation, getting the current value, getting the JSON ID, and switching this device off for safety.
*/
class Output {
    // Designed to be a generic interface for all output devices.

  protected:
    int maxValue=0; // The maximum value accepted to control this device
    int minValue=0; // The minimum value accepted to control this device
    int currentValue=0; // The current value of this device (e.g. PWM value representing speed)
    int stoppedValue=0; // The control value (e.g. PWM value representing speed) which means this device is stopped/not moving
    int pin=0; // The physical pin this is associated with
    String partID="part ID not set"; // The JSON Part ID of this device

  public:
    Output() {
      //Empty constructor to create harmless output
    }

    /*
      Control this device. This method will take e.g. a pwm thruster value representing speed and set the thruster to this value.
    */
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

    /*
      Get the current value of this device (EG: Servo position)
    */
    virtual int getValue() {
      return currentValue;
    }

    /*
      Something which needs to be run all the time
    */
    virtual void constantTask(){
    }

    /*
      Get the JSON ID for this device
    */
    String getID (){
      return partID;
    }

    /*
      Switch device off - for safety
    */
    virtual void turnOff(){
    }
};


/* ===========================Inputs=========================== */

/*
  The IMU class reads data from the IMU's sensors (position, acceleration, temperature) and sends this to the Pi using the communication class.
*/
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

/*
  The Depth class represents the BlueRobotics pressure sensor (measuring depth and external temperature) and sends this to the Pi using the communication class.
*/
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

/*
  The PHSensor class represents a PH sensor which measures the water outside the ROV and sends the data to the Pi using the communication class.
*/
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

/*
  The Sonar class represents the BlueRobotics Sonar (measuring distance) and sends this to the Pi using the communication class.
*/
class Sonar: public Input {
    // Designed to be a generic interface for all output devices.

  protected:
    bool initialised = false;
    Ping1D sonar { Serial1 }; // sonar object
    

  public:
    Sonar(String incomingPartID){
      partID = incomingPartID;
      Serial1.begin(115200); // sonar io
      if(!sonar.initialize())
      {
        // Send error message
        communication.bufferError("Sonar not found. Check wiring.");
      }
      else{
        initialised = true;
      }
    }

    int getValue() {
      if(initialised){
        if(sonar.update()){
          communication.bufferValue(this->partID+"_Dist",String(sonar.distance()));
          communication.bufferValue(this->partID+"_Conf",String(sonar.confidence()));
        }
        else{
          communication.bufferError("Could not update sonar readings.");
        }
      }
      else{
        // Throw error because this sensor has not yet been initialised properly
        communication.bufferError("Sonar not initialised.");
      }
      
    }
};



/* ===========================Outputs=========================== */

/*
  The Thruster class represents a BlueRobotics T100 or T200 PWM Thruster.
  It takes values between 1100 and 1900 to control the rotation speed and direction of the thruster.
*/
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

/*
  The ArmGripper class represents a BlueRobotics M100 or M200 PWM motor.
  It takes values between 1100 and 1900 to control the rotation speed and direction of the motor.
  It also has the option of limit switches to stop movement in one direction if pressed.
*/
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

/*
  The ArmRotation class represents a BlueRobotics M100 or M200 PWM motor.
  It takes values between 1100 and 1900 to control the rotation speed and direction of the motor.
  This is similar to ArmGripper except it does not have the option for limit switches.
*/
class ArmRotation: public Output {

  protected:
    // Represents a motor controlling arm rotation
    Servo servo;
    const int stoppedValue=1500;
    
 public:

    ArmRotation (int inputPin, String partID) {
      this->partID = partID;

      // Set limit and starting values
      maxValue = 1650;
      minValue = 1350;
      currentValue = stoppedValue;
      servo.attach(inputPin); // Associate the motor with the specified pin
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

/* ==========================Mapper========================== */
/*
  The Mapper class is a lightweight replacement for a map/dictionary structure.
  Each Arduino ID has its own array of Strings for JSON IDs, which correspond to Input or Output objects to control devices.
*/
class Mapper {
  private:
    // t for Ard_T (Thrusters)
    const static int tCount=11; // Number of devices attached to Arduino T
    Output* tObjects[tCount];  // Devices attached to Arduino T
    String tIDs[tCount] = {"Thr_FP", "Thr_FS", "Thr_AP", "Thr_AS", "Thr_TFP", "Thr_TFS", "Thr_TAP", "Thr_TAS", "Mot_R", "Mot_G", "Mot_F"}; // Device IDs of those attached to Arduino T

    // i for Ard_I (Input)
    const static int iCount=4; // Number of devices attached to Arduino I
    Input* iObjects[iCount]; // Devices attached to Arduino I
    String iIDs[iCount] = {"Sen_IMU", "Sen_Dep", "Sen_PH", "Sen_Sonar"}; // Device IDs of those attached to Arduino I

    // m for Ard_M (Micro ROV)
    const static int mCount=1; // Number of devices attached to Arduino M
    Output* mObjects[mCount]; // Devices attached to Arduino M
    String mIDs[mCount] = {"Thr_M"}; // Device IDs of those attached to Arduino M

    
  public:
    /*
      Assign JSON IDs to devices on this Arduino
    */
    void mapT(){
      int numberOfThrusters = 8;
      for ( int i = 0; i < numberOfThrusters; i++) {
        tObjects[i] = new Thruster(2+i, tIDs[i]); // The 8 movement Thrusters
      }
      // Delays between each device so they initialise separately. This helps to give an auditory signal that everything is connected properly.
      delay(2000);
      tObjects[8] = new ArmRotation(10, tIDs[8]); // Rotation motor for the arm
      delay(2000);
      tObjects[9] = new ArmGripper(11, tIDs[9],26,27); // Gripper motor for the arm
      delay(2000);
      tObjects[10] = new ArmGripper(12, tIDs[10],28,29); // Fish box opening
    }
    
    /*
      Assign JSON IDs to sensors on this Arduino
    */
    void mapI(){
      // Map and initialise sensors
      iObjects[0] = new IMU(0,iIDs[0]);
      iObjects[1] = new Depth(0,iIDs[1]);
      iObjects[2] = new PHSensor(56,iIDs[2]);
      iObjects[3] = new Sonar(iIDs[3]);
    }

    /*
      Assign JSON IDs to devices on this Arduino
    */
    void mapM(){
      mObjects[0] = new Thruster(3,mIDs[0]); // Micro ROV Thruster
    }
    
    /*
      Get the object representing an output device connected to this Arduino with the specified JSON ID
     */
    Output* getOutput(String jsonID){
      if(arduinoID=="Ard_T"){
        for(int i = 0; i < tCount; i++){
          if(jsonID == tIDs[i]){
            return tObjects[i];
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
    
    /*
      Get the object representing a senor connected to this Arduino with the specified JSON ID
     */
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

    /*
      Get the number of sensors
     */
    int getNumberOfInputs(){
      return iCount;
    }

    /*
      Get the number of output devices connected to this Arduino
     */
    int getNumberOfOutputs(){
      if(arduinoID == "Ard_T"){
        return tCount;
      }
      else if(arduinoID == "Ard_M"){
        return mCount;
      }
      return 0;
    }

    /*
      Read data from all sensors and send this to the Pi
     */
    void sendAllSensors(){
      for(int i = 0; i < iCount; i++){
        iObjects[i]->getValue();
      }
      communication.sendAll();
    }

    /*
      Switch off all devices attached to this Arduino.
      This is primarily a safety feature to be used if no control signals are being received.
     */
    void stopOutputs(){
      if(arduinoID == "Ard_T"){
        for(int i = 0; i < tCount; i++){
          tObjects[i]->turnOff();
          delay(125); // delay 125ms between each thruster to avoid sudden power halt
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

Mapper mapper; // Lightweight replacement for a map/dictionary structure to map JSON IDs to objects representing devices.

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
    
    // Act on incoming message accordingly
    if(arduinoID=="Ard_T" || arduinoID=="Ard_M"){
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
  
  if(arduinoID=="Ard_T" || arduinoID=="Ard_M"){
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
      stringComplete = true;
      break;
    }
    inputString += inChar;
  }
}
