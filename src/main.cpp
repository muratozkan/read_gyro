
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

Attitude6D attitude;

void initAttitude() {
    // initialize device
    Serial << "Initializing I2C devices: ";
    uint8_t retryCount = 0;
    for (uint8_t c = 0; c < MAX_RETRIES; c++) {
        Serial << ".";
        if (attitude.initialize()) {
            break;
        }
        delay(50);
    }
    if (attitude.isInitialized()) {
        Serial << endl << "Done! " << endl;

        Serial << "Calibrating sensors..." << endl;
        attitude.calibrate();
        Serial << "Done! " << endl;
    } else {
        Serial << endl << "Can't initialize after " << MAX_RETRIES << endl;
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
    if (!attitude.isInitialized()) {
        delay(5000);
        initAttitude();
        return;
    }
    
    float_t accel[3];
    float_t rot[3];
    float_t temp;

    if (Serial.available()) {
        String inStr = Serial.readString();
        String cmdStr = inStr.substring(0, 3);
        cmdStr.trim();
        String paramStr = inStr.substring(3, 8);
        paramStr.trim();
        int16_t parameter = paramStr.toInt();
        Serial << "applying: " << cmdStr << ": " << parameter << endl;
        Serial.flush();

        if (cmdStr == "ax") {
            attitude.getMPU().setXAccelOffset(parameter);
        } else if (cmdStr == "ay") {
            attitude.getMPU().setYAccelOffset(parameter);
        } else if (cmdStr == "az") {
            attitude.getMPU().setZAccelOffset(parameter);
        } else if (cmdStr == "gx") {
            attitude.getMPU().setXGyroOffset(parameter);
        } else if (cmdStr == "gy") {
            attitude.getMPU().setYGyroOffset(parameter);
        } else if (cmdStr == "gz") {
            attitude.getMPU().setZGyroOffset(parameter);
        } else if (cmdStr == "a") {
            attitude.getMPU().setFullScaleAccelRange(parameter);
        } else if (cmdStr == "g") {
            attitude.getMPU().setFullScaleGyroRange(parameter);
        }
    }

    unsigned long currentMillis = millis();
    if (lastMillis == 0L || (currentMillis - lastMillis) >= 500L) {
        lastMillis = currentMillis;

        attitude.getMotion(accel, rot);
        temp = attitude.getTemperatureCelcius();

        // display comma-separated accel/gyro x/y/z values
        Serial << "a/g: " << accel[0] << ", " << accel[1] << ", " << accel[2] << "; " <<
                            rot[0] << ", " << rot[1] << ", " << rot[2] << endl;
        // Serial << "t: " << temp << endl;

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
}


// Arduino sketch that returns calibration offsets for MPU6050 //   Version 1.1  (31th January 2014)
// Done by Luis Ródenas <luisrodenaslorda@gmail.com>
// Based on the I2Cdev library and previous work by Jeff Rowberg <jeff@rowberg.net>
// Updates (of the library) should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib

// These offsets were meant to calibrate MPU6050's internal DMP, but can be also useful for reading sensors. 
// The effect of temperature has not been taken into account so I can't promise that it will work if you 
// calibrate indoors and then use it outdoors. Best is to calibrate and use at the same room temperature.
/*
// I2Cdev and MPU6050 must be installed as libraries
#include <Arduino.h>

#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

///////////////////////////////////   CONFIGURATION   /////////////////////////////
//Change this 3 variables if you want to fine tune the skecth to your needs.
int BUFFER_SIZE=1000;     //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
int ACCEL_DEADZONE=8;     //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
int giro_deadzone=1;     //Giro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)

// default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
//MPU6050 accelgyro;
MPU6050 accelgyro; // <-- use for AD0 high

int16_t ax, ay, az,gx, gy, gz;

int mean_ax,mean_ay,mean_az,mean_gx,mean_gy,mean_gz,state=0;
int ax_offset,ay_offset,az_offset,gx_offset,gy_offset,gz_offset;

///////////////////////////////////   SETUP   ////////////////////////////////////
void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();

  // initialize serial communication
  Serial.begin(115200);

  // initialize device
  accelgyro.initialize();

  // wait for ready
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available()){
    Serial.println(F("Send any character to start sketch.\n"));
    delay(1500);
  }                
  while (Serial.available() && Serial.read()); // empty buffer again

  // start message
  Serial.println("\nMPU6050 Calibration Sketch");
  delay(2000);
  Serial.println("\nYour MPU6050 should be placed in horizontal position, with package letters facing up. \nDon't touch it until you see a finish message.\n");
  delay(3000);
  // verify connection
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  delay(1000);
  // reset offsets
  accelgyro.setXAccelOffset(0);
  accelgyro.setYAccelOffset(0);
  accelgyro.setZAccelOffset(0);
  accelgyro.setXGyroOffset(0);
  accelgyro.setYGyroOffset(0);
  accelgyro.setZGyroOffset(0);
}

///////////////////////////////////   FUNCTIONS   ////////////////////////////////////
void meansensors(){
  long i=0,buff_ax=0,buff_ay=0,buff_az=0,buff_gx=0,buff_gy=0,buff_gz=0;

  while (i<(BUFFER_SIZE+101)){
    // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    if (i>100 && i<=(BUFFER_SIZE+100)){ //First 100 measures are discarded
      buff_ax=buff_ax+ax;
      buff_ay=buff_ay+ay;
      buff_az=buff_az+az;
      buff_gx=buff_gx+gx;
      buff_gy=buff_gy+gy;
      buff_gz=buff_gz+gz;
    }
    if (i==(BUFFER_SIZE+100)){
      mean_ax=buff_ax/BUFFER_SIZE;
      mean_ay=buff_ay/BUFFER_SIZE;
      mean_az=buff_az/BUFFER_SIZE;
      mean_gx=buff_gx/BUFFER_SIZE;
      mean_gy=buff_gy/BUFFER_SIZE;
      mean_gz=buff_gz/BUFFER_SIZE;
    }
    i++;
    delay(2); //Needed so we don't get repeated measures
  }
}

void calibration(){
  ax_offset=-mean_ax/8;
  ay_offset=-mean_ay/8;
  az_offset=(16384-mean_az)/8;

  gx_offset=-mean_gx/4;
  gy_offset=-mean_gy/4;
  gz_offset=-mean_gz/4;
  while (1){
    int ready=0;
    accelgyro.setXAccelOffset(ax_offset);
    accelgyro.setYAccelOffset(ay_offset);
    accelgyro.setZAccelOffset(az_offset);

    accelgyro.setXGyroOffset(gx_offset);
    accelgyro.setYGyroOffset(gy_offset);
    accelgyro.setZGyroOffset(gz_offset);

    meansensors();
    Serial.println("...");

    if (abs(mean_ax)<=ACCEL_DEADZONE) ready++;
    else ax_offset=ax_offset-mean_ax/ACCEL_DEADZONE;

    if (abs(mean_ay)<=ACCEL_DEADZONE) ready++;
    else ay_offset=ay_offset-mean_ay/ACCEL_DEADZONE;

    if (abs(16384-mean_az)<=ACCEL_DEADZONE) ready++;
    else az_offset=az_offset+(16384-mean_az)/ACCEL_DEADZONE;

    if (abs(mean_gx)<=giro_deadzone) ready++;
    else gx_offset=gx_offset-mean_gx/(giro_deadzone+1);

    if (abs(mean_gy)<=giro_deadzone) ready++;
    else gy_offset=gy_offset-mean_gy/(giro_deadzone+1);

    if (abs(mean_gz)<=giro_deadzone) ready++;
    else gz_offset=gz_offset-mean_gz/(giro_deadzone+1);

    if (ready==6) break;
  }
}

///////////////////////////////////   LOOP   ////////////////////////////////////
void loop() {
  if (state==0){
    Serial.println("\nReading sensors for first time...");
    meansensors();
    state++;
    delay(1000);
  }

  if (state==1) {
    Serial.println("\nCalculating offsets...");
    calibration();
    state++;
    delay(1000);
  }

  if (state==2) {
    meansensors();
    Serial.println("\nFINISHED!");
    Serial.print("\nSensor readings with offsets:\t");
    Serial.print(mean_ax); 
    Serial.print("\t");
    Serial.print(mean_ay); 
    Serial.print("\t");
    Serial.print(mean_az); 
    Serial.print("\t");
    Serial.print(mean_gx); 
    Serial.print("\t");
    Serial.print(mean_gy); 
    Serial.print("\t");
    Serial.println(mean_gz);
    Serial.print("Your offsets:\t");
    Serial.print(ax_offset); 
    Serial.print("\t");
    Serial.print(ay_offset); 
    Serial.print("\t");
    Serial.print(az_offset); 
    Serial.print("\t");
    Serial.print(gx_offset); 
    Serial.print("\t");
    Serial.print(gy_offset); 
    Serial.print("\t");
    Serial.println(gz_offset); 
    Serial.println("\nData is printed as: acelX acelY acelZ giroX giroY giroZ");
    Serial.println("Check that your sensor readings are close to 0 0 16384 0 0 0");
    Serial.println("If calibration was succesful write down your offsets so you can set them in your projects using something similar to mpu.setXAccelOffset(youroffset)");
    while (1);
  }
}
*/
