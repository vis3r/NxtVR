#include "mpu6050.h"

void MPU6050::writeI2C2(uint8_t reg_addres, uint16_t data)
{
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(reg_addres);
    Wire.write((uint8_t)(data >> 8));
    Wire.write((uint8_t)data);
    Wire.endTransmission(true);
}

void MPU6050::writeI2C(uint8_t reg_addres, int16_t writing)
{
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(reg_addres);
    Wire.write(writing);
    Wire.endTransmission(true);
}

void MPU6050::begin()
{
    Wire.begin();
    writeI2C(MPU6050_POWER, 0);
    delay(20);
}

void MPU6050::configure()
{	
    /*
     
     * Offset correction directly to the buffer of 
     * the IMU. Deprecated as planning to implement
     * a calibration programm in the driver side of 
     * things. Feel free to use it
     
    writeI2C2(MPU6050_OFFSET_AX, OFFSET_AX);
    writeI2C2(MPU6050_OFFSET_AY, OFFSET_AY);
    writeI2C2(MPU6050_OFFSET_AZ, OFFSET_AZ);
    writeI2C2(MPU6050_OFFSET_GX, OFFSET_GX);
    writeI2C2(MPU6050_OFFSET_GY, OFFSET_GY);
    writeI2C2(MPU6050_OFFSET_GZ, OFFSET_GZ);
    */
}

void MPU6050::setReadRegister(uint8_t reg_addres, int16_t read_amount)
{
    Wire.beginTransmission(MPU6050_ADDRESS);
    Wire.write(reg_addres);

    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDRESS, read_amount);
}

void MPU6050::readAccelData(int16_t *destination)
{
    //begin with X reg, but read thru up to 6 bytes for Y,Z
    setReadRegister(MPU6050_ACCE_X, 6);
    destination[0] = (int16_t)Wire.read() << 8 | Wire.read();
    destination[1] = (int16_t)Wire.read() << 8 | Wire.read();
    destination[2] = (int16_t)Wire.read() << 8 | Wire.read();
}

void MPU6050::readGyroData(int16_t *destination)
{
    //begin with X reg, but read thru up to 6 bytes for Y,Z
    setReadRegister(MPU6050_GYRO_X, 6);
    destination[0] = (int16_t)Wire.read() << 8 | Wire.read();
    destination[1] = (int16_t)Wire.read() << 8 | Wire.read();
    destination[2] = (int16_t)Wire.read() << 8 | Wire.read();
}

void MPU6050::readTemp()
{
    setReadRegister(MPU6050_TEMP, 2);
    temp = Wire.read() << 8 | Wire.read();
    temp = temp / 340.00 + 36.53;    //equation for temperature in degrees C
}
