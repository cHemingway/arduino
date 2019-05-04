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
}

void loop() {
  // put your main code here, to run repeatedly:

}
