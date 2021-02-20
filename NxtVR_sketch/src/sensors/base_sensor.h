#ifndef BASE_SENSOR_H
#define BASE_SENSOR_H

#include "../hid_hmd/hid_utils.h"

class BaseSensorBoard
{

public:
    MotionReport_t motion;
//    ConfigResolutionReport_t confres;
    /*
    *   Inits communication with sensor board
    */
    void init(void);

    /*
    * Place for sensor specific configuration
    */
    void configure(void);

    /* 
    * Refreshes data in BaseSensorBoard::motion
    */
    void refresh(void);
};
#endif
