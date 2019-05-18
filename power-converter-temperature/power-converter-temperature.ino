#include <Wire.h>
// Address of power converter as per manual. See manual for resistor values
int addr = 0x57;
int ack;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Power Management Arduino Booted");

  


  // Check it's actually present
  scanForAddresses();
  getTemp();
}

void loop() {
  // put your main code here, to run repeatedly:
  getTemp();

}

void getTemp(){
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


double convertToUnits(int m, int y, int r, int b){
  return (1/m)*((y*(pow(10,-r)))-b);
}
