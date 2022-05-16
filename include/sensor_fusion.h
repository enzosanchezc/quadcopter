#ifndef SENSOR_FUSION
#define SENSOR_FUSION

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 2 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
#define SDA 21
#define SCL 22

#define TRIGGER 17
#define ECHO 16

#define WINDOW_SIZE 5

void initSensors();
void dmpDataReady();
void getYPR(float *ypr);
float getUltrasonicAltitude();
float getBMPRelAltitude();
float getFilteredAltitude();

#endif