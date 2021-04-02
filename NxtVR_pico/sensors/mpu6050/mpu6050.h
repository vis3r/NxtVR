#include <cstdint>
#define MPU6050_ADDRESS   0x68
#define MPU6050_POWER     0x6B
#define MPU6050_ACCEL_X   0x3B
#define MPU6050_ACCEL_Y   0x3D
#define MPU6050_ACCEL_Z   0x3F
#define MPU6050_GYRO_X    0x43
#define MPU6050_GYRO_Y    0x45
#define MPU6050_GYRO_Z    0x47
#define MPU6050_TEMP      0x41

#define MPU6050_OFFSET_AX 0x06
#define MPU6050_OFFSET_AY 0x08
#define MPU6050_OFFSET_AZ 0x0A
#define MPU6050_OFFSET_GX 0x13
#define MPU6050_OFFSET_GY 0x15
#define MPU6050_OFFSET_GZ 0x17


#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define I2C_PORT i2c0

#include "hardware/i2c.h"

class MPU6050
{
   public:
      int16_t temp;
      int16_t accel[3];
      int16_t gyro[3];
      uint16_t bias[6];

      void begin();
      void configure(uint16_t *bias);
      void readAccel(int16_t *accel);
      void readGyro(int16_t *gyro);
};
