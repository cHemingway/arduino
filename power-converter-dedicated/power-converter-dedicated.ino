#include <Wire.h>
bool stringComplete = false;
String inputString;
int addr = 0x58;

void setup() {
  // Begin serial for debugging
  Serial.begin(9600);
  Serial.println("Power Management Arduino Booted");
  showMenu();
  
}

void loop() {
  // put your main code here, to run repeatedly:
  if (stringComplete) {
    // Run menu
    
    if(inputString == "1"){
      sendSignal(0x03);
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
  Serial.println("Menu:");
  Serial.println("1. Clear faults");
}

void sendSignal(int command){
  // Address of power converter as per manual. See manual for resistor values
  int ack = 0;
  
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
