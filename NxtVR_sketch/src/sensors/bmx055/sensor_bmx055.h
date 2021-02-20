#ifdef SENSOR_BMX055
#include "../base_sensor.h"
#include "bmx055.h"

class SensorBoard : BaseSensorBoard
{
protected:
    BMX055 bmx055;
    int16_t Accel_buff[3];
    int16_t Gyro_buff[3];
    int16_t Mag_buff[3];

public:
    void init(void)
    {
        bmx055.begin();
    };
    void configure(void)
    {
        bmx055.init();
    };
    void refresh(void)
    {
        bmx055.readAccelData(Accel_buff);
        bmx055.readGyroData(Gyro_buff);
        bmx055.readMagData(Mag_buff);

        for (int i = 0; i < 3; i++)
        {
            motion.AccelReport.data[i] = Accel_buff[i];
            motion.GyroReport.data[i] = Gyro_buff[i];
            motion.MagReport.data[i] = Mag_buff[i];
        }
    };
};

#endif
