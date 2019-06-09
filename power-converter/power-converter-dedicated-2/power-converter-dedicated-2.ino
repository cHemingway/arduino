#include <Wire.h>
// Address of power converter as per manual. See manual for resistor values
int addr = 0x57;


void setup() {
  // Begin serial for debugging
  Serial.begin(9600);
  Serial.println("Power Management Arduino Booted");
  int ack;


  // Check it's actually present
  scanForAddresses();

  // Clear faults
  clearFault();


  Serial.print("Temperature: ");
  Wire.beginTransmission(addr);
  Wire.write(0x8D); // Write command code
  //Wire.write(0x01); // Write data
  ack = Wire.endTransmission(false); // Send repeated start
  if(ack!=0){
    Serial.print("(Error: ");
    Serial.print(ack);
    Serial.print(")");
  }
  Wire.requestFrom(addr, 2,(uint8_t) true);    // request bytes from slave device
  if(Wire.available()==2 ){
   
   int result, fullResult;
   result = Wire.read();
   Serial.print(" 0b");
   Serial.print(result,BIN);//Show result in binary
   fullResult = result;
   result = Wire.read();
   Serial.print(" 0b");
   Serial.print(result,BIN);//Show result in binary
   fullResult += (result << 8);
   
   Serial.print(" Which is exactly: ");
   Serial.print(fullResult);
   Serial.print(" Which should be: ");
   Serial.print(convertToUnits(1,fullResult,0,0));
   Serial.print("C.");
  }
  else{
    Serial.print("No response");
  }
   Serial.println(";");

  commFault();

  // Switch pages???
  switchToPage(0x01);

  
  /*// Check if something has caused a communication fault
  commFault();
  
  // Check current status
  showStatus();
  
  // Check if something has caused a communication fault
  commFault();

  
  // Show mfr status
  Serial.print("MFR Status: ");
  Wire.beginTransmission(addr);
  Wire.write(0x80); // Write command code
  int ack = Wire.endTransmission(false); // Send repeated start
  if(ack!=0){
    Serial.print("(Error: ");
    Serial.print(ack);
    Serial.print(")");
  }
   Wire.requestFrom(addr, 1,(uint8_t) true);    // request bytes from slave device
   while (Wire.available()) { // if bytes were received
    Serial.print(" 0b");
    Serial.print(Wire.read(),BIN);//Show result in binary
   }
   Serial.println();

   // Show page
  Serial.print("Read Page: ");
  Wire.beginTransmission(addr);
  Wire.write(0x00); // Write command code
  ack = Wire.endTransmission(false); // Send repeated start
  if(ack!=0){
    Serial.print("(Error: ");
    Serial.print(ack);
    Serial.print(")");
  }
   Wire.requestFrom(addr, 1,(uint8_t) true);    // request bytes from slave device
   while (Wire.available()) { // if bytes were received
    Serial.print(" 0b");
    Serial.print(Wire.read(),BIN);//Show result in binary
   }
   Serial.println();

  
  // Check if something has caused a communication fault
  commFault();
*/
  // Check current status
  showStatus();
  //switchToPage(1);
  Serial.print("Turn on");
  Wire.beginTransmission(addr);
  Wire.write(0x01); // Write command code
  Wire.write(0x80); // Write data
  ack = Wire.endTransmission();
  if(ack!=0){
    Serial.print("(Error: ");
    Serial.print(ack);
    Serial.print(")");
  }
   Serial.println(";");


// Show out voltage
  Serial.print("Vout: ");
  Wire.beginTransmission(addr);
  Wire.write(0x8B); // Write command code
  //Wire.write(0x01); // Write data
  ack = Wire.endTransmission(false); // Send repeated start
  if(ack!=0){
    Serial.print("(Error: ");
    Serial.print(ack);
    Serial.print(")");
  }
  Wire.requestFrom(addr, 2,(uint8_t) true);    // request bytes from slave device
  if(Wire.available()==2 ){
   
   int result, fullResult;
   result = Wire.read();
   Serial.print(" 0b");
   Serial.print(result,BIN);//Show result in binary
   fullResult = result;
   result = Wire.read();
   Serial.print(" 0b");
   Serial.print(result,BIN);//Show result in binary
   fullResult += (result << 8);
   
   Serial.print(" Which is exactly: ");
   Serial.print(fullResult);
   Serial.print(" Which should be: ");
   Serial.print(convertToUnits(1,fullResult,1,0));
   Serial.print("V.");
  }
  else{
    Serial.print("No response");
  }
   Serial.println(";");




   Serial.print("Status: ");
  Wire.beginTransmission(addr);
  Wire.write(0x78); // Write command code
  ack = Wire.endTransmission(false); // Send repeated start
  if(ack!=0){
    Serial.print("(Error: ");
    Serial.print(ack);
    Serial.print(")");
  }
   Wire.requestFrom(addr, 1,(uint8_t) true);    // request bytes from slave device
   while (Wire.available()) { // if bytes were received
    Serial.print(" 0b");
    Serial.print(Wire.read(),BIN);//Show result in binary
   }
   Serial.println();

   delay(3000);

/*
Serial.print("Turn off");
  Wire.beginTransmission(addr);
  Wire.write(0x01); // Write command code
  Wire.write(0x00); // Write data
  ack = Wire.endTransmission();
  if(ack!=0){
    Serial.print("(Error: ");
    Serial.print(ack);
    Serial.print(")");
  }
   Serial.println(";");


// Show out voltage
  Serial.print("Vout: ");
  Wire.beginTransmission(addr);
  Wire.write(0x8B); // Write command code
  //Wire.write(0x01); // Write data
  ack = Wire.endTransmission(false); // Send repeated start
  if(ack!=0){
    Serial.print("(Error: ");
    Serial.print(ack);
    Serial.print(")");
  }
  Wire.requestFrom(addr, 2,(uint8_t) true);    // request bytes from slave device
  if(Wire.available()==2 ){
   
   int result, fullResult;
   result = Wire.read();
   Serial.print(" 0b");
   Serial.print(result,BIN);//Show result in binary
   fullResult = result;
   result = Wire.read();
   Serial.print(" 0b");
   Serial.print(result,BIN);//Show result in binary
   fullResult += (result << 8);
   
   Serial.print(" Which is exactly: ");
   Serial.print(fullResult);
   Serial.print(" Which should be: ");
   Serial.print(convertToUnits(1,fullResult,1,0));
   Serial.print("V.");
  }
  else{
    Serial.print("No response");
  }
   Serial.println(";");




   Serial.print("Status: ");
  Wire.beginTransmission(addr);
  Wire.write(0x78); // Write command code
  ack = Wire.endTransmission(false); // Send repeated start
  if(ack!=0){
    Serial.print("(Error: ");
    Serial.print(ack);
    Serial.print(")");
  }
   Wire.requestFrom(addr, 1,(uint8_t) true);    // request bytes from slave device
   while (Wire.available()) { // if bytes were received
    Serial.print(" 0b");
    Serial.print(Wire.read(),BIN);//Show result in binary
   }
   Serial.println();


/*
   // Show out voltage
  Serial.print("Vout: ");
  Wire.beginTransmission(addr);
  Wire.write(0x8B); // Write command code
  //Wire.write(0x01); // Write data
  ack = Wire.endTransmission(false); // Send repeated start
  if(ack!=0){
    Serial.print("(Error: ");
    Serial.print(ack);
    Serial.print(")");
  }
  Wire.requestFrom(addr, 2,(uint8_t) true);    // request bytes from slave device
  if(Wire.available()==2 ){
   
   int result, fullResult;
   result = Wire.read();
   Serial.print(" 0b");
   Serial.print(result,BIN);//Show result in binary
   fullResult = result;
   result = Wire.read();
   Serial.print(" 0b");
   Serial.print(result,BIN);//Show result in binary
   fullResult += (result << 8);
   
   Serial.print(" Which is exactly: ");
   Serial.print(fullResult);
   Serial.print(" Which should be: ");
   Serial.print(convertToUnits(1,fullResult,1,0));
   Serial.print("V.");
  }
  else{
    Serial.print("No response");
  }
   Serial.println(";");

*/

//
//   Serial.print("Turn on");
//  Wire.beginTransmission(addr);
//  Wire.write(0x01); // Write command code
//  Wire.write(0x80); // Write data
//  ack = Wire.endTransmission();
//  if(ack!=0){
//    Serial.print("(Error: ");
//    Serial.print(ack);
//    Serial.print(")");
//  }
//   Serial.println(";");





   Serial.print("Status: ");
  Wire.beginTransmission(addr);
  Wire.write(0x78); // Write command code
  ack = Wire.endTransmission(false); // Send repeated start
  if(ack!=0){
    Serial.print("(Error: ");
    Serial.print(ack);
    Serial.print(")");
  }
   Wire.requestFrom(addr, 1,(uint8_t) true);    // request bytes from slave device
   while (Wire.available()) { // if bytes were received
    Serial.print(" 0b");
    Serial.print(Wire.read(),BIN);//Show result in binary
   }
   Serial.println();
   
   
  //switchToPage(0x0);
  //switchToPage(0);
  
  // Check if something has caused a communication fault
  switchToPage(0x00);
  commFault();
  switchToPage(0x01);
/*
  // Check current status
  showStatus();*/

  // Show in voltage
  Serial.print("Vin: ");
  Wire.beginTransmission(addr);
  Wire.write(0x88); // Write command code
  //Wire.write(0x01); // Write data
  ack = Wire.endTransmission(false); // Send repeated start
  if(ack!=0){
    Serial.print("(Error: ");
    Serial.print(ack);
    Serial.print(")");
  }
  Wire.requestFrom(addr, 2,(uint8_t) true);    // request bytes from slave device
  if(Wire.available()==2 ){
   
   int result, fullResult;
   result = Wire.read();
   Serial.print(" 0b");
   Serial.print(result,BIN);//Show result in binary
   fullResult = result;
   result = Wire.read();
   Serial.print(" 0b");
   Serial.print(result,BIN);//Show result in binary
   fullResult += (result << 8);
   
   Serial.print(" Which is exactly: ");
   Serial.print(fullResult);
   Serial.print(" Which should be: ");
   Serial.print(convertToUnits(1,fullResult,1,0));
   Serial.print("V.");
  }
  else{
    Serial.print("No response");
  }
   Serial.println(";");

switchToPage(0x00);
  commFault();
  switchToPage(0x01);

// Show out voltage
  Serial.print("Vout: ");
  Wire.beginTransmission(addr);
  Wire.write(0x8B); // Write command code
  //Wire.write(0x01); // Write data
  ack = Wire.endTransmission(false); // Send repeated start
  if(ack!=0){
    Serial.print("(Error: ");
    Serial.print(ack);
    Serial.print(")");
  }
  Wire.requestFrom(addr, 2,(uint8_t) true);    // request bytes from slave device
  if(Wire.available()==2 ){
   
   int result, fullResult;
   result = Wire.read();
   Serial.print(" 0b");
   Serial.print(result,BIN);//Show result in binary
   fullResult = result;
   result = Wire.read();
   Serial.print(" 0b");
   Serial.print(result,BIN);//Show result in binary
   fullResult += (result << 8);
   
   Serial.print(" Which is exactly: ");
   Serial.print(fullResult);
   Serial.print(" Which should be: ");
   Serial.print(convertToUnits(1,fullResult,1,0));
   Serial.print("V.");
  }
  else{
    Serial.print("No response");
  }
   Serial.println(";");



Serial.print("High side Current: ");
  Wire.beginTransmission(addr);
  Wire.write(0x89); // Write command code
  ack = Wire.endTransmission(false); // Send repeated start
  if(ack!=0){
    Serial.print("(Error: ");
    Serial.print(ack);
    Serial.print(")");
  }
  Wire.requestFrom(addr, 2,(uint8_t) true);    // request bytes from slave device
  if(Wire.available()==2 ){
   
   int result, fullResult;
   result = Wire.read();
   Serial.print(" 0b");
   Serial.print(result,BIN);//Show result in binary
   fullResult = result;
   result = Wire.read();
   Serial.print(" 0b");
   Serial.print(result,BIN);//Show result in binary
   fullResult += (result << 8);
   
   Serial.print(" Which is exactly: ");
   Serial.print(fullResult);
   Serial.print(" Which should be: ");
   Serial.print(convertToUnits(1,fullResult,2,0));
   Serial.print("A");
  }
  else{
    Serial.print("No response");
  }
   Serial.println(";");


Serial.print("Low side Current: ");
  Wire.beginTransmission(addr);
  Wire.write(0x8C); // Write command code
  ack = Wire.endTransmission(false); // Send repeated start
  if(ack!=0){
    Serial.print("(Error: ");
    Serial.print(ack);
    Serial.print(")");
  }
  Wire.requestFrom(addr, 2,(uint8_t) true);    // request bytes from slave device
  if(Wire.available()==2 ){
   
   int result, fullResult;
   result = Wire.read();
   Serial.print(" 0b");
   Serial.print(result,BIN);//Show result in binary
   fullResult = result;
   result = Wire.read();
   Serial.print(" 0b");
   Serial.print(result,BIN);//Show result in binary
   fullResult += (result << 8);
   
   Serial.print(" Which is exactly: ");
   Serial.print(fullResult);
   Serial.print(" Which should be: ");
   Serial.print(convertToUnits(1,fullResult,2,0));
   Serial.print("A");
  }
  else{
    Serial.print("No response");
  }
   Serial.println(";");


Serial.print("Low side resistance: ");
  Wire.beginTransmission(addr);
  Wire.write(0xD4); // Write command code
  ack = Wire.endTransmission(false); // Send repeated start
  if(ack!=0){
    Serial.print("(Error: ");
    Serial.print(ack);
    Serial.print(")");
  }
  Wire.requestFrom(addr, 2,(uint8_t) true);    // request bytes from slave device
  if(Wire.available()==2 ){
   
   int result, fullResult;
   result = Wire.read();
   Serial.print(" 0b");
   Serial.print(result,BIN);//Show result in binary
   fullResult = result;
   result = Wire.read();
   Serial.print(" 0b");
   Serial.print(result,BIN);//Show result in binary
   fullResult += (result << 8);
   
   Serial.print(" Which is exactly: ");
   Serial.print(fullResult);
   Serial.print(" Which should be: ");
   Serial.print(convertToUnits(1,fullResult,5,0));
   Serial.print("Ohms");
  }
  else{
    Serial.print("No response");
  }
   Serial.println(";");


  // Check if something has caused a communication fault


  switchToPage(0x00);

  commFault();

  // Check current status
  showStatus();
  // Check if something has caused a communication fault
  //commFault();
  
}

void loop() {
  // put your main code here, to run repeatedly:

}
void commFault(){
  
  Wire.beginTransmission(addr);
  Wire.write(0x7E); // Write command code
  int ack = Wire.endTransmission(false); // Send repeated start
  if(ack!=0){
    Serial.print("(Error: ");
    Serial.print(ack);
    Serial.print(")");
  }
   Wire.requestFrom(addr, 1,(uint8_t) true);    // request bytes from slave device
   int result = 0;
   while (Wire.available()) { // if bytes were received
    result = (int) Wire.read();
    if(result>0){
      Serial.print(">>>Communication Fault: ");
      Serial.print("0b");
      Serial.println(result,BIN);//Show result in binary
    }
    
   }
}

void clearFault(){
  Serial.println("=Clearing fault=");
  Wire.beginTransmission(addr);
  Wire.write(0x03); // Write command code
  int ack = Wire.endTransmission(); // Send repeated start
  if(ack!=0){
    Serial.print("(Error: ");
    Serial.print(ack);
    Serial.print(")");
  }
}

void showStatus(){
  Serial.print("Status: ");
  Wire.beginTransmission(addr);
  Wire.write(0x78); // Write command code
  int ack = Wire.endTransmission(false); // Send repeated start
  if(ack!=0){
    Serial.print("(Error: ");
    Serial.print(ack);
    Serial.print(")");
  }
   Wire.requestFrom(addr, 1,(uint8_t) true);    // request bytes from slave device
   while (Wire.available()) { // if bytes were received
    Serial.print("0b");
    Serial.println(Wire.read(),BIN);//Show status register in binary
   }
}

void switchToPage(int pageNum){
  // Change page
  Serial.print("Switching to page ");
  Serial.print(pageNum);
  Wire.beginTransmission(addr);
  Wire.write(0x00); // Write command code
  Wire.write(pageNum); // Write data
  int ack = Wire.endTransmission(); // Send repeated start
  if(ack!=0){
    Serial.print("(Error: ");
    Serial.print(ack);
    Serial.print(")");
  }
   Serial.println();

}

double convertToUnits(int m, int y, int r, int b){
  return (1/m)*((y*(pow(10,-r)))-b);
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
