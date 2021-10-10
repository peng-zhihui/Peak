#include <EEPROM.h>
#include "MPU9250.h"

const uint8_t EEPROM_SIZE = 1 + sizeof(float) * 3 * 4;
extern MPU9250 mpu;

enum EEP_ADDR {
    EEP_CALIB_FLAG = 0x00,
    EEP_ACC_BIAS = 0x01,
    EEP_GYRO_BIAS = 0x0D,
    EEP_MAG_BIAS = 0x19,
    EEP_MAG_SCALE = 0x25
};

void writeByte(int address, byte value) {
    EEPROM.put(address, value);
}

void writeFloat(int address, float value) {
    EEPROM.put(address, value);
}

byte readByte(int address) {
    byte valueIn = 0;
    EEPROM.get(address, valueIn);
    return valueIn;
}

float readFloat(int address) {
    float valueIn = 0;
    EEPROM.get(address, valueIn);
    return valueIn;
}

void clearCalibration() {
    writeByte(EEP_CALIB_FLAG, 0);
}

bool isCalibrated() {
    return (readByte(EEP_CALIB_FLAG) == 0x01);
}

void saveCalibration() {
    Serial.println("Write calibrated parameters to EEPROM");
    writeByte(EEP_CALIB_FLAG, 1);
    writeFloat(EEP_ACC_BIAS + 0, mpu.getAccBias(0));
    writeFloat(EEP_ACC_BIAS + 4, mpu.getAccBias(1));
    writeFloat(EEP_ACC_BIAS + 8, mpu.getAccBias(2));
    writeFloat(EEP_GYRO_BIAS + 0, mpu.getGyroBias(0));
    writeFloat(EEP_GYRO_BIAS + 4, mpu.getGyroBias(1));
    writeFloat(EEP_GYRO_BIAS + 8, mpu.getGyroBias(2));
    writeFloat(EEP_MAG_BIAS + 0, mpu.getMagBias(0));
    writeFloat(EEP_MAG_BIAS + 4, mpu.getMagBias(1));
    writeFloat(EEP_MAG_BIAS + 8, mpu.getMagBias(2));
    writeFloat(EEP_MAG_SCALE + 0, mpu.getMagScale(0));
    writeFloat(EEP_MAG_SCALE + 4, mpu.getMagScale(1));
    writeFloat(EEP_MAG_SCALE + 8, mpu.getMagScale(2));
#if defined(ESP_PLATFORM) || defined(ESP8266)
    EEPROM.commit();
#endif
}

void loadCalibration() {
    Serial.println("Load calibrated parameters from EEPROM");
    if (isCalibrated()) {
        Serial.println("calibrated? : YES");
        Serial.println("load calibrated values");
        mpu.setAccBias(
            readFloat(EEP_ACC_BIAS + 0),
            readFloat(EEP_ACC_BIAS + 4),
            readFloat(EEP_ACC_BIAS + 8));
        mpu.setGyroBias(
            readFloat(EEP_GYRO_BIAS + 0),
            readFloat(EEP_GYRO_BIAS + 4),
            readFloat(EEP_GYRO_BIAS + 8));
        mpu.setMagBias(
            readFloat(EEP_MAG_BIAS + 0),
            readFloat(EEP_MAG_BIAS + 4),
            readFloat(EEP_MAG_BIAS + 8));
        mpu.setMagScale(
            readFloat(EEP_MAG_SCALE + 0),
            readFloat(EEP_MAG_SCALE + 4),
            readFloat(EEP_MAG_SCALE + 8));
    } else {
        Serial.println("calibrated? : NO");
        Serial.println("load default values");
        mpu.setAccBias(0., 0., 0.);
        mpu.setGyroBias(0., 0., 0.);
        mpu.setMagBias(0., 0., 0.);
        mpu.setMagScale(1., 1., 1.);
    }
}

void printCalibration() {
    Serial.println("< calibration parameters >");
    Serial.print("calibrated? : ");
    Serial.println(readByte(EEP_CALIB_FLAG) ? "YES" : "NO");
    Serial.print("acc bias x  : ");
    Serial.println(readFloat(EEP_ACC_BIAS + 0) * 1000.f / MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print("acc bias y  : ");
    Serial.println(readFloat(EEP_ACC_BIAS + 4) * 1000.f / MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print("acc bias z  : ");
    Serial.println(readFloat(EEP_ACC_BIAS + 8) * 1000.f / MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print("gyro bias x : ");
    Serial.println(readFloat(EEP_GYRO_BIAS + 0) / MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print("gyro bias y : ");
    Serial.println(readFloat(EEP_GYRO_BIAS + 4) / MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print("gyro bias z : ");
    Serial.println(readFloat(EEP_GYRO_BIAS + 8) / MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print("mag bias x  : ");
    Serial.println(readFloat(EEP_MAG_BIAS + 0));
    Serial.print("mag bias y  : ");
    Serial.println(readFloat(EEP_MAG_BIAS + 4));
    Serial.print("mag bias z  : ");
    Serial.println(readFloat(EEP_MAG_BIAS + 8));
    Serial.print("mag scale x : ");
    Serial.println(readFloat(EEP_MAG_SCALE + 0));
    Serial.print("mag scale y : ");
    Serial.println(readFloat(EEP_MAG_SCALE + 4));
    Serial.print("mag scale z : ");
    Serial.println(readFloat(EEP_MAG_SCALE + 8));
}

void printBytes() {
    for (size_t i = 0; i < EEPROM_SIZE; ++i)
        Serial.println(readByte(i), HEX);
}

void setupEEPROM() {
    Serial.println("EEPROM start");

    if (!isCalibrated()) {
        Serial.println("Need Calibration!!");
    }
    Serial.println("EEPROM calibration value is : ");
    printCalibration();
    Serial.println("Loaded calibration value is : ");
    loadCalibration();
}
