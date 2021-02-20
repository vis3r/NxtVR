#define SENSOR_MPU6050
#if defined(SENSOR_MPU6050)
#include "../base_sensor.h"
#include "mpu6050.h"

class SensorBoard : public BaseSensorBoard
{
protected:
    MPU6050 mpu6050;
    int16_t Accel_buff[3];
    int16_t Gyro_buff[3];
public:

    void init(void)
    {
        mpu6050.begin();
    };

    void configure(void)
    {
        mpu6050.configure();
    };

    void refresh(void)
    {
        for (int i = 0; i < 3; i++)
        {
            //Data is sent as the mpu6050 works by default. This means that we are not
            //doing any kind of processing of data in the arduino side. All of this
            //processing of data is made, either on  the Monado driver, or the OpenHMD
            //one

            //TODO: Test this nxtvr code with the experimental Monado driver

            mpu6050.readAccelData(Accel_buff);
            mpu6050.readGyroData(Gyro_buff);

            motion.AccelReport.data[i] = Accel_buff[i];
            motion.GyroReport.data[i] = Gyro_buff[i];
        }
    };
};

#endif
