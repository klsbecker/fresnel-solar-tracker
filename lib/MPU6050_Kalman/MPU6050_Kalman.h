#ifndef MPU6050_KALMAN_H
#define MPU6050_KALMAN_H

#include <Wire.h>
#include <Arduino.h>

class MPU6050_Kalman {
public:
    MPU6050_Kalman();

    void begin(uint8_t addr = 0x68);
    void readSensors();
    void runRollCalibration(uint16_t samples = 2000);
    void setRollCalibration(float calibration);
    float getRollCalibration();
    float getRollAngle();

private:
    void kalman1D(float& kalmanState, float& kalmanUncertainty, float kalmanInput, float kalmanMeasurement);

    uint8_t _addr;
    float _rateRoll, _ratePitch, _rateYaw;
    float _rateCalibrationRoll;
    float _accX, _accY, _accZ;
    float _angleRoll;

    float _kalmanAngleRoll, _kalmanUncertaintyAngleRoll;
};

#endif
