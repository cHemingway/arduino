#include <Wire.h>
bool stringComplete = false;
String inputString;
// Address of power converter as per manual. See manual for resistor values
int addr = 0x57;

void setup() {
  // Begin serial for debugging
  Serial.begin(9600);
  Serial.println("Power Management Arduino Booted");
  scanForAddresses();
  showMenu();
  
}

void loop() {
  // put your main code here, to run repeatedly:
  if (stringComplete) {
    // Run menu
    if(inputString == "0"){
      testSignal();
    }
    else if(inputString == "1"){
      writeSignal(0x03);
    }
    else if(inputString == "2"){
      readSignal(0x88, 2);
    }
    else if(inputString == "3"){
      scanForAddresses();
    }
    else if(inputString == "4"){
      monitorStatus();
    }
    else if(inputString == "5"){
      readSignal(0x8B, 2);
    }
    else if(inputString == "6"){
      readSignal(0x99, 2);
    }
    else if(inputString == "7"){
      writeSignal(0x01,0x80);
    }
    else if(inputString == "8"){
      readSignal(0x01, 1);
    }
    else if(inputString == "9"){
      readSignal(0x79, 2);
    }
    else if(inputString == "10"){
      readSignal(0x78, 1);
    }
    else if(inputString == "11"){
      readSignal(0x7B, 1);
    }
    else if(inputString == "12"){
      readSignal(0x7C, 1);
    }
    else if(inputString == "13"){
      readSignal(0x7D, 1);
    }
    else if(inputString == "14"){
      readSignal(0x7E, 1);
    }
    else{
      Serial.println("Unknown option");
      
    }
    stringComplete = false;
    inputString = "";
    showMenu();
  }
}

void showMenu(){
  Serial.println();
  Serial.println();
  Serial.print("Menu for device 0x");
  Serial.println(addr, HEX);
  Serial.println("0. Test connection");
  Serial.println("1. Clear faults");
  Serial.println("2. Get input voltage");
  Serial.println("3. Scan for addresses");
  Serial.println("4. Realtime monitoring");
  Serial.println("5. Get output voltage");
  Serial.println("6. Get controller ID");
  Serial.println("7. TURN DEVICE ON");
  Serial.println("8. Check if on");
  Serial.println("9. Check device status word");
  Serial.println("10. Check device status byte");
  Serial.println("11. Check for overcurrent fault");
  Serial.println("12. Check for voltage fault");
  Serial.println("13. Check for temperature fault");
  Serial.println("14. Check for communication fault");
}

void testSignal(){
  Serial.println("Sending test message.");
  Wire.beginTransmission (addr);
  Serial.println("Waiting for response.");
  if (Wire.endTransmission () == 0){
    Serial.println("Success.");
  }
  else{
    Serial.println("Error.");
  }
    
}

void writeSignal(int command){
  
  int ack = 0;
  
  Serial.println("Begin Transmission");
  Wire.beginTransmission(addr);
  Serial.println("Sending Command Code");
  Wire.write(command); // Write command code
  Serial.println("Ending Transmission. Waiting for status...");
  ack = Wire.endTransmission();
  if(ack==0){
    Serial.println("Transmission success.");
  }
  else{
    Serial.print("ACK was not received. Error code: ");
    Serial.println(ack);
  }
}

void writeSignal(int command, int data){
  
  int ack = 0;
  
  Serial.println("Begin Transmission (with data)");
  Wire.beginTransmission(addr);
  Serial.println("Sending Command Code");
  Wire.write(command); // Write command code
  Wire.write(data); // Write data
  Serial.println("Ending Transmission. Waiting for status...");
  ack = Wire.endTransmission();
  if(ack==0){
    Serial.println("Transmission success.");
  }
  else{
    Serial.print("ACK was not received. Error code: ");
    Serial.println(ack);
  }
}

void readSignal(int command, int byteCount){
  int ack = 0;

  Serial.println("Begin Transmission");
  Wire.beginTransmission(addr);
  Serial.println("Sending Command Code");
  Wire.write(command); // Write command code
  Serial.println("Ending Transmission of command. Waiting for status...");
  ack = Wire.endTransmission(false);
  if(ack==0){
    Serial.println("Transmission success.");
  }
  else{
    Serial.print("ACK was not received. Error code: ");
    Serial.println(ack);
  }

  Serial.println("Requesting data of command...");
  Wire.requestFrom(addr, byteCount,(uint8_t) true );    // request n bytes from slave device

  // receive reading from sensor

  //String partCode = "";
  while (Wire.available()) { // if bytes were received
    byte reading;
    reading = Wire.read();  // receive high byte (overwrites previous reading)
    Serial.println("Received response: ");
    //char letter = (char) reading;
    //partCode += String(reading);
    //Serial.print("0x");
    Serial.println(reading, BIN);   // print the reading
  }
  //Serial.println(partCode);
  
  
}
void readSignal(int command, int byteCount, int data){
  int ack = 0;

  Serial.println("Begin Transmission");
  Wire.beginTransmission(addr);
  Serial.println("Sending Command Code");
  Wire.write(command); // Write command code
  Wire.write(data); // Write data
  Serial.println("Ending Transmission of command. Waiting for status...");
  ack = Wire.endTransmission(false);
  if(ack==0){
    Serial.println("Transmission success.");
  }
  else{
    Serial.print("ACK was not received. Error code: ");
    Serial.println(ack);
  }

  Serial.println("Requesting data of command...");
  Wire.requestFrom(addr, byteCount,(uint8_t) true );    // request n bytes from slave device

  // receive reading from sensor

  //String partCode = "";
  while (Wire.available()) { // if bytes were received
    byte reading;
    reading = Wire.read();  // receive high byte (overwrites previous reading)
    Serial.println("Received response: ");
    //char letter = (char) reading;
    //partCode += String(reading);
    //Serial.print("0x");
    Serial.println(reading, BIN);   // print the reading
  }
  //Serial.println(partCode);
  
  
}

void scanForAddresses(){
  Serial.println ("I2C scanner. Scanning ...");
  byte count = 0;
  
  Wire.begin();
  for (byte i = 8; i < 120; i++)
  {
    Wire.beginTransmission (i);
    if (Wire.endTransmission () == 0)
      {
      Serial.print ("Found address: ");
      Serial.print (i, DEC);
      Serial.print (" (0x");
      Serial.print (i, HEX);
      Serial.println (")");
      count++;
      delay (1);  // maybe unneeded?
      } // end of good response
  } // end of for loop
  Serial.println ("Done.");
  Serial.print ("Found ");
  Serial.print (count, DEC);
  Serial.println (" device(s).");
}

void monitorStatus(){
  while(true){
    Serial.print("Status: ");
    Serial.print(getValue(0x78,1),BIN);
    /*Serial.print(" | ");
    Serial.print("Temperature status: ");
    Serial.print(getValue(0x7D,1));*/
    Serial.print(" | ");
    Serial.print("Vin: ");
    Serial.print(getValue(0x88,2,1,1,0,true));
    Serial.print(" | ");
    Serial.print("IIn: ");
    Serial.print(getValue(0x89,2,1,2,0,true));
    Serial.print(" | ");
    Serial.print("Vout: ");
    Serial.print(getValue(0x8B,2,1,1,0,true));
    Serial.print(" | ");
    Serial.print("IOut: ");
    Serial.print(getValue(0x8C,2,1,2,0,true));
    Serial.print(" | ");
    Serial.print("Temperature: ");
    Serial.print(getValue(0x8D,2,1,0,0,true));
    Serial.print(" | ");
    Serial.print("Load resistance: ");
    Serial.print(getValue(0xD4,2,1,5,0,true));
    Serial.print(" | ");
    Serial.print("Overtemp fault: 0x");
    Serial.print(getValue(0x4F,2),HEX);
    Serial.print(" | ");
    Serial.print("Overtemp warn: 0x");
    Serial.print(getValue(0x51,2),HEX);
    Serial.print(" | ");
    Serial.print("  ");
    Serial.println();
    delay(1);
  }
}

int getValue(int command, int numberOfBytes){
  // Method for if no conversion is needed
  return getValue(command,numberOfBytes,0,0,0,false);
}


int getValue(int command, int numberOfBytes, int m, int R, int b, bool needsConverting){ // Quietly get a certain value
  Wire.beginTransmission(addr);
  Wire.write(command); // Write command code
  if(Wire.endTransmission(false)==0){
    Wire.requestFrom(addr, numberOfBytes, (uint8_t) true );    // request n bytes from slave device
    // step 5: receive reading from sensor
    if (numberOfBytes <= Wire.available()) { // if two bytes were received
      int reading = 0;
      for(int i=0; i<numberOfBytes; i++){
        reading |= Wire.read();
        reading = reading << (8*(((numberOfBytes-1)-i)));    // shift high byte to be high 8 bits
      }
      // Convert into correct units
      if(needsConverting){
        reading = (1/m)*(reading*(10^(-R))-b);
      }
      return reading;
    }
    Serial.print("(err response)");
    return 0;
  }
  else{
    Serial.print("(err command)");
    return 0;
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
