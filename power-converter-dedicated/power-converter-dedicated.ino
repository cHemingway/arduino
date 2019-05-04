#include <Wire.h>
void setup() {
  // Begin serial for debugging
  Serial.begin(9600);
  Serial.println("Power Management Arduino Booted");
  
  // Address of power converter as per manual. See manual for resistor values
  int addr = 1, command = 0x03, ack = 0;
  
  Serial.println("Begin Transmission");
  Wire.beginTransmission(addr);
  Serial.println("Sending Command Code");
  Wire.write(command); // Write command code
  Serial.println("Ending Transmission");
  ack = Wire.endTransmission();
  if(ack==0){
    Serial.println("Transmission success.");
  }
  else{
    Serial.print("ACK was not received. Error code: ");
    Serial.println(ack);
  }

//  
//
//  
//
//  Serial.println("Waiting for ACK of Address");
//  byte n = Wire.requestFrom(addr, 1); // Request data
//  while (Wire.available()) // Read all incoming data
//  {
//    Serial.print("Received: ");
//    char c = Wire.read();    // receive a byte as character
//    Serial.println(c);         // print the character
//  }
//
//  Serial.println("Sending Command Code");
//  Wire.beginTransmission(addr);
//  Wire.write(command); // Write command code
//  Wire.endTransmission();
//
//  Serial.println("Waiting for ACK of Command Code");
//  byte n = Wire.requestFrom(addr, 1); // Request data
//  while (Wire.available()) // Read all incoming data
//  {
//    Serial.print("Received: ");
//    char c = Wire.read();    // receive a byte as character
//    Serial.println(c);         // print the character
//  }
//
//  


}

void loop() {
  // put your main code here, to run repeatedly:

}
