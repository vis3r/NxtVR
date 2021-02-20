#define MPU6050_ADDRESS   0x68
#define MPU6050_POWER     0x6B
#define MPU6050_ACCE_X    0x3B
#define MPU6050_ACCE_Y    0x3D
#define MPU6050_ACCE_Z    0x3F
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

#define OFFSET_AX 0 
#define OFFSET_AY 0 
#define OFFSET_AZ 0
#define OFFSET_GX 0
#define OFFSET_GY 0 
#define OFFSET_GZ 0 

#define MPU6050_ACCEL_FS_2 0x00
#define MPU6050_ACCEL_FS_4 0x01
#define MPU6050_ACCEL_FS_8 0x02
#define MPU6050_ACCEL_FS_16 0x03

#define MPU6050_GYRO_FS_250 0x00
#define MPU6050_GYRO_FS_500 0x01
#define MPU6050_GYRO_FS_1000 0x02
#define MPU6050_GYRO_FS_2000 0x03

#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_ACCEL_CONFIG 0x1C

#include "Wire.h"
#include "../../hid_hmd/hid_utils.h"

class MPU6050
{

protected:
    void setReadRegister(uint8_t reg_addres, int16_t read_amount);

public:
    
    MotionReport_t motion;

    int16_t temp;
    void writeI2C(uint8_t reg_addres, int16_t writing);
    void writeI2C2(uint8_t reg_addres, uint16_t data);
    void begin();
    void configure();
    void readAccelData(int16_t *destination);
    void readGyroData(int16_t *destination);
    void getOffset();
    void readTemp();
};
