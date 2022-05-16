#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps612.h"
#include <HCSR04.h>
#include <Adafruit_BMP085.h>
#include "sensor_fusion.h"

MPU6050 mpu;
Adafruit_BMP085 bmp;

bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 gy;         // [x, y, z]            gyro sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr_old[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// HC-SR04 vars
unsigned long pulse_raw;
float pulse_length;
float dist;

// BMP180 vars
float baseline;

// MA5 Filter vars
int idx = 0;
float sum = 0;
float readings[WINDOW_SIZE];
float averaged = 0;

// ================================================================
// ===           MPU6050 INTERRUPT DETECTION ROUTINE            ===
// ================================================================
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}


void initSensors(){
    if(!Serial){
        Serial.begin(115200);
    }
    Wire.begin(SDA, SCL);

    /* MPU6050 initialization */
    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(17);
    mpu.setYGyroOffset(18);
    mpu.setZGyroOffset(-28);
    mpu.setXAccelOffset(-1265);
    mpu.setYAccelOffset(2135);
    mpu.setZAccelOffset(1536);
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration Time: generate offsets and calibrate our MPU6050
        //mpu.CalibrateAccel(6);
        //mpu.CalibrateGyro(6);
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection"));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);

    /* HC-SR04 Initialization */
    pinMode(TRIGGER, OUTPUT);
    pinMode(ECHO, INPUT);

    /* BMP180 Initialization */
    if (!bmp.begin()) {
        Serial.println("BMP180 init failed");
    }
    baseline = bmp.readAltitude();
}

void getYPR(float *ypr){
    // if programming failed, don't try to do anything
    if (!dmpReady) return;
    // read a packet from FIFO
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet
        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        ypr_old[0] = ypr[0];
        ypr_old[1] = ypr[1];
        ypr_old[2] = ypr[2];
        /*
        Serial.print("ypr\t");
        Serial.print(ypr[0] * 180 / 3.1416);
        Serial.print("\t");
        Serial.print(ypr[1] * 180 / 3.1416);
        Serial.print("\t");
        Serial.println(ypr[2] * 180 / 3.1416);
        */
        // blink LED to indicate activity
        //blinkState = !blinkState;
        //digitalWrite(LED_PIN, blinkState);
    }
}

float getUltrasonicAltitude(){
    float dist_old = dist;
    digitalWrite(TRIGGER, HIGH);
    delay(0.01);
    digitalWrite(TRIGGER, LOW);
    pulse_raw = pulseIn(ECHO,HIGH);
    if(pulse_raw == 0) return dist_old;
    pulse_length = (float) pulse_raw/10000;
    dist = (pulse_length)*(343/2);
    return dist;
}

float getBMPRelAltitude(){
    return (bmp.readAltitude() - baseline);
}

float getFilteredAltitude(){
    float value = getUltrasonicAltitude();
    sum = sum - readings[idx];
    readings[idx] = value;
    sum = sum + value;
    idx = (idx + 1) % WINDOW_SIZE;

    averaged = sum / WINDOW_SIZE;

    return averaged;
}