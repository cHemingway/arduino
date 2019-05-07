#include <Wire.h>
// Address of power converter as per manual. See manual for resistor values
int addr = 0x57;

void setup() {
  // Begin serial for debugging
  Serial.begin(9600);
  Serial.println("Power Management Arduino Booted");
  scanForAddresses();

  // Check current status
  showStatus();
  // Check if something has caused a communication fault
  commFault();

  // Show in voltage
  Serial.print("Vin: ");
  Wire.beginTransmission(addr);
  Wire.write(0x88); // Write command code
  int ack = Wire.endTransmission(false); // Send repeated start
  if(ack!=0){
    Serial.print("(Error: ");
    Serial.print(ack);
    Serial.print(")");
  }
   Wire.requestFrom(addr, byteCount,(uint8_t) true);    // request bytes from slave device
   while (Wire.available()) { // if bytes were received
    Serial.print(" 0b");
    Serial.println(Wire.read(),BIN);//Show result in binary
   }

  // Check current status
  showStatus();
  // Check if something has caused a communication fault
  commFault();
  
}

void loop() {
  // put your main code here, to run repeatedly:

}
void commFault(){
  Serial.print("Communication Fault: ");
  Wire.beginTransmission(addr);
  Wire.write(0x7E); // Write command code
  int ack = Wire.endTransmission(false); // Send repeated start
  if(ack!=0){
    Serial.print("(Error: ");
    Serial.print(ack);
    Serial.print(")");
  }
   Wire.requestFrom(addr, 1,(uint8_t) true);    // request bytes from slave device
   while (Wire.available()) { // if bytes were received
    Serial.print("0b");
    Serial.println(Wire.read(),BIN);//Show result in binary
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

int convertToUnits(int m, int y, int r, int b){
  return (1/m)*(y*(10^(-r))-b);
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
