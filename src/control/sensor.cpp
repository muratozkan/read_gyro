#include "sensor.h"

Attitude6D::Attitude6D() {
}

bool Attitude6D::initialize() {
    mpu.initialize();

    mpu.setFullScaleAccelRange(ACCEL_FS_RANGE);

    initialized = mpu.testConnection();
    return initialized;
}

bool Attitude6D::isInitialized() {
    return initialized;
}

void Attitude6D::setOffsets(int16_t offsets[]) {
    mpu.setXAccelOffset(offsets[0]);
    mpu.setYAccelOffset(offsets[1]);
    mpu.setZAccelOffset(offsets[2]);

    mpu.setXGyroOffset(offsets[3]);
    mpu.setYGyroOffset(offsets[4]);
    mpu.setZGyroOffset(offsets[5]);
}

void Attitude6D::calibrate() {
    #ifdef ATTITUDE_SERIAL_DEBUG
    Serial << "ao:" << mpu.getXAccelOffset() << ", " << mpu.getYAccelOffset() << ", " << mpu.getZAccelOffset() << endl;
    Serial << "go:" << mpu.getXGyroOffset() << ", " << mpu.getYGyroOffset() << ", " << mpu.getZGyroOffset() << endl;  
    #endif

    int16_t offsets[] = {0, 0, 0, 0, 0, 0};
    int16_t targets[] = {0, 0, ACCEL_SENSITIVITY, 0, 0, 0};
    
    setOffsets(offsets);

    int16_t meansFirst[] = {0, 0, 0, 0, 0, 0};
    sampleAverage(meansFirst, CALIBRATION_SAMPLES);

    // increase offsets by a specific value
    for (uint8_t s = 0; s < 6; s++) {
        offsets[s] += CALIBRATION_STEP;
    }
    setOffsets(offsets);

    int16_t meansSecond[] = {0, 0, 0, 0, 0, 0};
    sampleAverage(meansSecond, CALIBRATION_SAMPLES);

    for (uint8_t s = 0; s < 6; s++) {
        offsets[s] -= (meansSecond[s] - targets[s]) * 1024 / (meansSecond[s] - meansFirst[s]);
    }

    setOffsets(offsets);

    #ifdef ATTITUDE_SERIAL_DEBUG
    Serial << "ao:" << mpu.getXAccelOffset() << ", " << mpu.getYAccelOffset() << ", " << mpu.getZAccelOffset() << endl;
    Serial << "go:" << mpu.getXGyroOffset() << ", " << mpu.getYGyroOffset() << ", " << mpu.getZGyroOffset() << endl;  
    #endif
}

MPU6050 Attitude6D::getMPU() {
    return mpu;
}

void Attitude6D::getMotion(float_t rot[], float_t rot_d[]) {
    int16_t acc[] = {0, 0, 0};
    int16_t gyro[] = {0, 0, 0};

    mpu.getMotion6(&acc[0], &acc[1], &acc[2], &gyro[0], &gyro[1], &gyro[2]);

    rot[0] = (float_t) acc[0] / ACCEL_SENSITIVITY;
    rot[1] = (float_t) acc[1] / ACCEL_SENSITIVITY;
    rot[2] = (float_t) acc[2] / ACCEL_SENSITIVITY;

    rot_d[0] = (float_t) gyro[0] / GYRO_SENSITIVITY;
    rot_d[1] = (float_t) gyro[1] / GYRO_SENSITIVITY;
    rot_d[2] = (float_t) gyro[2] / GYRO_SENSITIVITY;
}

float_t Attitude6D::getTemperatureCelcius() {
    return (float) mpu.getTemperature() / TEMP_CONSTANT + TEMP_BIAS;
}

void Attitude6D::sampleAverage(int16_t means[], uint16_t samples) {
    long totals[] = {0L, 0L, 0L, 0L, 0L, 0L}; 
    int16_t vals[] = {0, 0, 0, 0, 0, 0};
    for (uint16_t n = 0; n < samples; n++) {
        mpu.getMotion6(&vals[0], &vals[1], &vals[2], &vals[3], &vals[4], &vals[5]);

        for (uint8_t s = 0; s < 6; s ++) {
            totals[s] += vals[s]; 
        }
        delay(5);
    }

    for (uint8_t s = 0; s < 6; s ++) {
        means[s] = totals[s] / samples; 
    }
}
