#include <Wire.h>
// Address of power converter as per manual. See manual for resistor values
int addr = 0x57;

void setup() {
  // Begin serial for debugging
  Serial.begin(9600);
  Serial.println("Power Management Arduino Booted");
  scanForAddresses();

  // Show status

  // Check for comms fault

  // Show in voltage

  // Show status

  // check for comms fault
  
}

void loop() {
  // put your main code here, to run repeatedly:

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

void writeSignal(int command){
  
  int ack = 0;
  
  Serial.print("Transmit Command: 0x");
  Serial.print(command,HEX);
  Wire.beginTransmission(addr);
  Wire.write(command); // Write command code
  ack = Wire.endTransmission();
  if(ack==0){
    Serial.println(" Success.");
  }
  else{
    Serial.print("ACK was not received. Error code: ");
    Serial.println(ack);
  }
}
