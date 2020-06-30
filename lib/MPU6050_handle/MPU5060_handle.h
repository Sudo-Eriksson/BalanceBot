#ifndef MPU6050_HANDLE_H
#define MPU6050_HANDLE_H

#include <Arduino.h>
#include <Wire.h>

class MPU5060_handle{
private:
    #define MPU_POWER_REG                   0x6B
    #define MPU_POWER_CYCLE                 0b00000000
    #define MPU_READ_TIMEOUT                2000
    #define MPU_SAMP_FREQ                   250

    #define MPU_GYRO_CFG_REG                0x1B
    #define MPU_GYRO_READ_REG               0x43
    #define MPU_GYRO_READ_REG_SIZE          6
    #define MPU_GYRO_CFG_500DEG             0b00001000
    #define MPU_GYRO_READINGSCALE_500DEG    65.5
    #define MPU_CALIBRATE_READING_NUM       2000

    #define MPU_ACCEL_CFG_REG               0x1C
    #define MPU_ACCEL_READ_REG              0x3B
    #define MPU_ACCEL_READ_REG_SIZE         6
    #define MPU_ACCEL_CFG_2G                0b00000000
    #define MPU_ACCEL_READINGSCALE_2G       16384.0

    #define MPU1_I2C_ADDRESS                0b1101000
public:
    // Constructor
    MPU5060_handle();

    // Methods
    bool MPUReadAccel();
    bool MPUReadGyro();
    void calibrateGyro();
    bool SetupMPU();

    // Variables
    float gForceX, gForceY, gForceZ;  // Accelerometer sensor values
    float rotX, rotY, rotZ;           // Gyroscope sensor values
    float calibX, calibY, calibZ;     // Gyroscope calibration values
};
#endif 