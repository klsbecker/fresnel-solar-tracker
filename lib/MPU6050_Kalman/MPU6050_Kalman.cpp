#include "MPU6050_Kalman.h"

MPU6050_Kalman::MPU6050_Kalman()
    : _rateRoll(0), _ratePitch(0), _rateYaw(0),
      _rateCalibrationRoll(0), _kalmanAngleRoll(0), _kalmanUncertaintyAngleRoll(4) {}

void MPU6050_Kalman::begin(uint8_t addr) {
    _addr = addr;
    Wire.begin();

    // Desativa o modo sleep
    Wire.beginTransmission(_addr);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission();

    delay(100);
}

void MPU6050_Kalman::runRollCalibration(uint16_t samples) {
    for (uint16_t i = 0; i < samples; i++) {
        readSensors();
        _rateCalibrationRoll += _kalmanAngleRoll;
        delay(1);
    }
    _rateCalibrationRoll /= samples;
}

void MPU6050_Kalman::setRollCalibration(float calibration) {
    _rateCalibrationRoll = calibration;
}

float MPU6050_Kalman::getRollCalibration() {
    return _rateCalibrationRoll;
}

void MPU6050_Kalman::readSensors() {
    Wire.beginTransmission(_addr);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)_addr, (uint8_t)6, (bool)true);

    int16_t accXLSB = Wire.read() << 8 | Wire.read();
    int16_t accYLSB = Wire.read() << 8 | Wire.read();
    int16_t accZLSB = Wire.read() << 8 | Wire.read();

    _accX = (float)accXLSB / 4096.0 - 0.1;
    _accY = (float)accYLSB / 4096.0 + 0.078;
    _accZ = (float)accZLSB / 4096.0 - 0.1;

    _angleRoll = atan(_accX / sqrt(_accY * _accY + _accZ * _accZ)) * RAD_TO_DEG;

    Wire.beginTransmission(_addr);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)_addr, (uint8_t)6, true);

    int16_t gyroX = Wire.read() << 8 | Wire.read();
    int16_t gyroY = Wire.read() << 8 | Wire.read();
    int16_t gyroZ = Wire.read() << 8 | Wire.read();

    _rateRoll = (float)gyroX / 65.5;
    _ratePitch = (float)gyroY / 65.5;
    _rateYaw = (float)gyroZ / 65.5;

    kalman1D(_kalmanAngleRoll, _kalmanUncertaintyAngleRoll, _rateRoll, _angleRoll);
}

float MPU6050_Kalman::getRollAngle() {
    return _kalmanAngleRoll - _rateCalibrationRoll;
}

void MPU6050_Kalman::kalman1D(float& kalmanState, float& kalmanUncertainty, float kalmanInput, float kalmanMeasurement) {
    kalmanState += 0.004 * kalmanInput;
    kalmanUncertainty += 0.004 * 0.004 * 4 * 4;

    float kalmanGain = kalmanUncertainty / (kalmanUncertainty + 3 * 3);
    kalmanState += kalmanGain * (kalmanMeasurement - kalmanState);
    kalmanUncertainty *= (1 - kalmanGain);
}
