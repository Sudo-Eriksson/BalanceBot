#include <MPU5060_handle.h>

//TODO: Write documentation in these methods

MPU5060_handle::MPU5060_handle()
{
    Wire.begin();
}

bool MPU5060_handle::MPUReadAccel(){
    Wire.beginTransmission(MPU1_I2C_ADDRESS);
    Wire.write(MPU_ACCEL_READ_REG);
    Wire.endTransmission();
    Wire.requestFrom(MPU1_I2C_ADDRESS, MPU_ACCEL_READ_REG_SIZE);

    unsigned long timeout = millis() + MPU_READ_TIMEOUT;

    while(Wire.available() < MPU_ACCEL_READ_REG_SIZE && timeout < millis());

    if (timeout <= millis()) return false;
        gForceX = (long)(Wire.read() << 8 | Wire.read()) * MPU_ACCEL_READINGSCALE_2G;
        gForceY = (long)(Wire.read() << 8 | Wire.read()) * MPU_ACCEL_READINGSCALE_2G;
        gForceZ = (long)(Wire.read() << 8 | Wire.read()) * MPU_ACCEL_READINGSCALE_2G;
    return true;
}

bool MPU5060_handle::MPUReadGyro(){
    Wire.beginTransmission(MPU1_I2C_ADDRESS);
    Wire.write(MPU_GYRO_READ_REG);
    Wire.endTransmission();
    Wire.requestFrom(MPU1_I2C_ADDRESS, MPU_GYRO_READ_REG_SIZE);

    unsigned long timeout = millis() + MPU_READ_TIMEOUT;

    while(Wire.available() < MPU_ACCEL_READ_REG_SIZE && timeout < millis());

    if (timeout <= millis()) return false;
        rotX = (long)(Wire.read() << 8 | Wire.read()) * MPU_GYRO_READINGSCALE_500DEG;
        //rotY = (long)(Wire.read() << 8 | Wire.read()) * MPU_GYRO_READINGSCALE_500DEG;
        //rotZ = (long)(Wire.read() << 8 | Wire.read()) * MPU_GYRO_READINGSCALE_500DEG;
    return true;
}

void MPU5060_handle::calibrateGyro(){
    int loopTimer = 0;

    //digitalWrite(STATUS_PIN, HIGH);
    Serial.println("Calibrating Gyro");

    calibX = 0;
    //calibY = 0;
    //calibZ = 0;

    for(int i=0; i<MPU_CALIBRATE_READING_NUM;i++){
        if(MPUReadGyro()){
            calibX += rotX;
            //calibY += rotY;
            //calibZ += rotZ;

            //wait for the next sample cycle
            while(micros() - loopTimer < 4000);
            loopTimer = micros();
        }
        else{
            i--;
        }
    }
    calibX = calibX / MPU_CALIBRATE_READING_NUM;
    //calibY = calibY / MPU_CALIBRATE_READING_NUM;
    //calibZ = calibZ / MPU_CALIBRATE_READING_NUM;

    Serial.print("x: ");
    Serial.print(calibX);
    //Serial.print("y: ");
    //Serial.print(calibY);
    //Serial.print("z: ");
    //Serial.println(calibZ);

    Serial.println("Calibration of Gyro Done.");

    //digitalWrite(STATUS_PIN, LOW);
    }

bool MPU5060_handle::SetupMPU(){
    Serial.println("Seting Up MPU6050");

    int res = 0;

    Wire.beginTransmission(MPU1_I2C_ADDRESS);
    Wire.write(MPU_POWER_REG);
    Wire.write(MPU_POWER_CYCLE);
    res += Wire.endTransmission();

    Wire.beginTransmission(MPU1_I2C_ADDRESS);
    Wire.write(MPU_GYRO_CFG_REG);
    Wire.write(MPU_GYRO_CFG_500DEG);
    res += Wire.endTransmission();

    Wire.beginTransmission(MPU1_I2C_ADDRESS);
    Wire.write(MPU_ACCEL_CFG_REG);
    Wire.write(MPU_ACCEL_CFG_2G);
    res += Wire.endTransmission();

    // Check the result of the setup
    if(res != 0){
        Serial.print("MPU setup failed. Return:"); Serial.println(res);
        return false;
    }
    else{
        return true;
    }

}