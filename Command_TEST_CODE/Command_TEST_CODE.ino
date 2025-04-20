#include "BluetoothSerial.h"
#include <Arduino.h>
BluetoothSerial serialBT;

//Bluetooth signal Store in this variable
char btSignal;

void setup() {
  Serial.begin(115200);

  //Bluetooth Name
  serialBT.begin("ASLAM YT");
}

void loop() {

  while (serialBT.available()) {
    btSignal = serialBT.read();
    Serial.print("Incoming Signal: ");
    Serial.println(btSignal);

    
  }
}