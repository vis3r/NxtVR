/*
* sensor_board.h
* This file acts as a include proxy
* for sensor board selection
* 
*/
#define SENSOR_DUMMY


#if defined(SENSOR_BMX055)
    #include "sensors/bmx055/sensor_bmx055.h"

#elif defined(SENSOR_MPU6050)
    #include "sensors/mpu6050/sensor_mpu6050.h"

#elif defined(SENSOR_DUMMY)
    #include "sensors/dummy/sensor_dummy.h"

#else
    #error Sensor board not defined
#endif
