#include <Arduino.h>
#include <Wire.h>
#include "sensor_fusion.h"

float ypr[3];
float altitude;

void setup(){
    Serial.begin(115200);
    Wire.begin(SDA, SCL);
    initSensors();
}

void loop(){
    //getYPR(ypr);
    //Serial.print("Yaw: ");
    //Serial.print(ypr[0]*180/3.1416);
    //Serial.print(" Pitch: ");
    //Serial.print(ypr[1]*180/3.1416);
    //Serial.print(" Roll: ");
    //Serial.println(ypr[2]*180/3.1416);

    //Serial.print("Altitude: ");
    //Serial.print(getFilteredAltitude());
    //Serial.println(" cm");
    //delay(50);

    
}