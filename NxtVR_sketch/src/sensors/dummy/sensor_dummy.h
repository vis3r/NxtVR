#ifdef SENSOR_DUMMY
#include "../base_sensor.h"
#include "../../hid_hmd/hid_utils.h"
#include <Wire.h>

class SensorBoard : public BaseSensorBoard
{
protected:
    raw3L Accel_buff = {};
    raw3L Gyro_buff = {};
    raw3L Mag_buff = {};

    raw3L *buffers[3] = {&Accel_buff, &Gyro_buff, &Mag_buff};

public:
    void init(void)
    {
        Wire.begin();

        //Init values with some dummy ones
        // In range of -1000 <> 1000

        Accel_buff.x = 900;
        Accel_buff.y = -450;
        Accel_buff.z = -1;

        Gyro_buff.x = -5;
        Gyro_buff.y = -123;
        Gyro_buff.z = 423;

        memset(&Mag_buff.data, 0, sizeof(Mag_buff.data));
    };
    void configure(void)
    {
        memset(&confres.data, 0, sizeof(confres.data));
    };

    void refresh(void)
    {

        // Dummy values for testing
        for (uint i = 0; i < 3; i++)
        {
            raw3L *buff_p = buffers[i];

            for (uint n = 0; n < 3; n++)
            {
                // loop thru -1000 -> 1000
                if (buff_p->data[n] >= 1000)
                {
                    buff_p->data[n] = -1000;
                }
                else
                {
                    buff_p->data[n]++;
                };
            };
        };

        for (int i = 0; i < 3; i++)
        {
            motion.AccelReport.data[i] = Accel_buff.data[i];
            motion.GyroReport.data[i] = Gyro_buff.data[i];
            motion.MagReport.data[i] = Mag_buff.data[i];
        };
    };
};

#endif
