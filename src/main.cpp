
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "control/sensor.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#include <Arduino.h>
#include <Streaming.h>

#define MAX_RETRIES        20

#define LED_PIN 0
bool blinkState = false;

unsigned long lastMillis;

IMU6D imu;

void initAttitude() {
    // initialize device
    Serial << "Initializing IMU devices: ";
    uint8_t retryCount = 0;
    for (uint8_t c = 0; c < MAX_RETRIES; c++) {
        Serial << ".";
        if (imu.initialize()) {
            break;
        }
        delay(50);
    }
    if (imu.isInitialized()) {
        Serial << endl << "Done! " << endl;

        Serial << "Calibrating sensors..." << endl;
        imu.calibrate();
        Serial << "Done! " << endl;
    } else {
        Serial << endl << "Can't initialize IMU after " << MAX_RETRIES << endl;
    }
}

void setup() {
    // configure Arduino LED pin for output
    pinMode(LED_PIN, OUTPUT);

    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(115200);
    initAttitude();
}

void loop() {
    /*
    if (!attitude.isInitialized()) {
        delay(5000);
        initAttitude();
        return;
    }
    */
    float_t accel[3];
    float_t rot[3];

    unsigned long currentMillis = millis();
    if (lastMillis == 0L || (currentMillis - lastMillis) >= 500L) {
        lastMillis = currentMillis;

        imu.getMotion(accel, rot);

        // display comma-separated accel/gyro x/y/z values
        Serial << "a/g: " << accel[0] << ", " << accel[1] << ", " << accel[2] << "; " <<
                            rot[0] << ", " << rot[1] << ", " << rot[2] << endl;
        // Serial << "t: " << temp << endl;
        
        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
}
