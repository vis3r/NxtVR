#include "bmx055.h"

//################# Accelerometer #####################
int BMX055::getAccX()
{

    readAccelData(accData);

    return (int)accData[0];
}

int BMX055::getAccY()
{

    readAccelData(accData);

    return (int)accData[1];
}

int BMX055::getAccZ()
{

    readAccelData(accData);

    return (int)accData[2];
}
//#################################################
//################# Gyrometer #####################
int BMX055::getGyroX()
{

    readGyroData(gyroData);

    return (int)gyroData[0];
}

int BMX055::getGyroY()
{

    readGyroData(gyroData);

    return (int)gyroData[1];
}

int BMX055::getGyroZ()
{

    readGyroData(gyroData);

    return (int)gyroData[2];
}
//######################################
//################# Magnetmeter #####################
int BMX055::getMagX()
{

    readGyroData(magData);

    return (int)magData[0];
}

int BMX055::getMagY()
{

    readGyroData(magData);

    return (int)magData[1];
}

int BMX055::getMagZ()
{

    readMagData(magData);

    return (int)magData[2];
}
//######################################

float BMX055::getAres(void)
{
    switch (gAscale)
    {
    // Possible accelerometer scales (and their register bit settings) are:
    // 2 Gs (0011), 4 Gs (0101), 8 Gs (1000), and 16 Gs  (1100).
    // BMX055 ACC data is signed 12 bit
    case AFS_2G:
        aRes = 2.0 / 2048.0;
        break;
    case AFS_4G:
        aRes = 4.0 / 2048.0;
        break;
    case AFS_8G:
        aRes = 8.0 / 2048.0;
        break;
    case AFS_16G:
        aRes = 16.0 / 2048.0;
        break;
    }

    return aRes;
}

float BMX055::getGres(void)
{
    switch (gGscale)
    {
    // Possible gyro scales (and their register bit settings) are:
    // 125 DPS (100), 250 DPS (011), 500 DPS (010), 1000 DPS (001), and 2000 DPS
    // (000).
    case GFS_125DPS:
        gRes = 124.87 / 32768.0; // per data sheet, not exactly 125!?
        break;
    case GFS_250DPS:
        gRes = 249.75 / 32768.0;
        break;
    case GFS_500DPS:
        gRes = 499.5 / 32768.0;
        break;
    case GFS_1000DPS:
        gRes = 999.0 / 32768.0;
        break;
    case GFS_2000DPS:
        gRes = 1998.0 / 32768.0;
        break;
    }

    return gRes;
}

float BMX055::getPitch(void) { return pitch; }

float BMX055::getRoll(void) { return roll; }

float BMX055::getYaw(void) { return yaw; }

void BMX055::readAccelData(int16_t *destination)
{
    uint8_t rawData[6]; // x/y/z accel register data stored here

    // Read the six raw data registers into data array
    readBytes(BMX055_ACC_ADDRESS, BMX055_ACC_D_X_LSB, 6, &rawData[0]);
    if ((rawData[0] & 0x01) && (rawData[2] & 0x01) &&
        (rawData[4] & 0x01))
    { // Check that all 3 axes have new data
        // Turn the MSB and LSB into a signed 12-bit value
        destination[0] =
            (int16_t)(((int16_t)rawData[1] << 8) | rawData[0]) >> 4;
        destination[1] =
            (int16_t)(((int16_t)rawData[3] << 8) | rawData[2]) >> 4;
        destination[2] =
            (int16_t)(((int16_t)rawData[5] << 8) | rawData[4]) >> 4;
    }
}

void BMX055::readGyroData(int16_t *destination)
{
    uint8_t rawData[6]; // x/y/z gyro register data stored here

    // Read the six raw data registers sequentially into data array
    readBytes(BMX055_GYRO_ADDRESS, BMX055_GYRO_RATE_X_LSB, 6, &rawData[0]);
    // Turn the MSB and LSB into a signed 16-bit value
    destination[0] = (int16_t)(((int16_t)rawData[1] << 8) | rawData[0]);
    destination[1] = (int16_t)(((int16_t)rawData[3] << 8) | rawData[2]);
    destination[2] = (int16_t)(((int16_t)rawData[5] << 8) | rawData[4]);
}

void BMX055::readMagData(int16_t *magData)
{
    int16_t mdata_x = 0, mdata_y = 0, mdata_z = 0, temp = 0;
    uint16_t data_r = 0;
    uint8_t
        rawData[8]; // x/y/z hall magnetic field data, and Hall resistance data
    readBytes(BMX055_MAG_ADDRESS, BMX055_MAG_XOUT_LSB, 8,
              &rawData[0]); // Read the eight raw data registers sequentially
                            // into data array
    if (rawData[6] & 0x01)
    { // Check if data ready status bit is set
        mdata_x = (int16_t)(((int16_t)rawData[1] << 8) | rawData[0]) >>
                  3; // 13-bit signed integer for x-axis field
        mdata_y = (int16_t)(((int16_t)rawData[3] << 8) | rawData[2]) >>
                  3; // 13-bit signed integer for y-axis field
        mdata_z = (int16_t)(((int16_t)rawData[5] << 8) | rawData[4]) >>
                  1; // 15-bit signed integer for z-axis field
        data_r = (uint16_t)(((uint16_t)rawData[7] << 8) | rawData[6]) >>
                 2; // 14-bit unsigned integer for Hall resistance

        // calculate temperature compensated 16-bit magnetic fields
        temp = ((int16_t)(((uint16_t)((((int32_t)dig_xyz1) << 14) /
                                      (data_r != 0 ? data_r : dig_xyz1))) -
                          ((uint16_t)0x4000)));
        magData[0] =
            ((int16_t)((((int32_t)mdata_x) *
                        ((((((((int32_t)dig_xy2) *
                              ((((int32_t)temp) * ((int32_t)temp)) >> 7)) +
                             (((int32_t)temp) *
                              ((int32_t)(((int16_t)dig_xy1) << 7)))) >>
                            9) +
                           ((int32_t)0x100000)) *
                          ((int32_t)(((int16_t)dig_x2) + ((int16_t)0xA0)))) >>
                         12)) >>
                       13)) +
            (((int16_t)dig_x1) << 3);

        temp = ((int16_t)(((uint16_t)((((int32_t)dig_xyz1) << 14) /
                                      (data_r != 0 ? data_r : dig_xyz1))) -
                          ((uint16_t)0x4000)));
        magData[1] =
            ((int16_t)((((int32_t)mdata_y) *
                        ((((((((int32_t)dig_xy2) *
                              ((((int32_t)temp) * ((int32_t)temp)) >> 7)) +
                             (((int32_t)temp) *
                              ((int32_t)(((int16_t)dig_xy1) << 7)))) >>
                            9) +
                           ((int32_t)0x100000)) *
                          ((int32_t)(((int16_t)dig_y2) + ((int16_t)0xA0)))) >>
                         12)) >>
                       13)) +
            (((int16_t)dig_y1) << 3);
        magData[2] =
            (((((int32_t)(mdata_z - dig_z4)) << 15) -
              ((((int32_t)dig_z3) *
                ((int32_t)(((int16_t)data_r) - ((int16_t)dig_xyz1)))) >>
               2)) /
             (dig_z2 +
              ((int16_t)(((((int32_t)dig_z1) * ((((int16_t)data_r) << 1))) +
                          (1 << 15)) >>
                         16))));
    }
}

float BMX055::getTemperature()
{
    uint8_t c = readByte(BMX055_ACC_ADDRESS,
                         BMX055_ACC_D_TEMP); // Read the raw data register
    int16_t tempCount = ((int16_t)((int16_t)c << 8)) >>
                        8; // Turn the byte into a signed 8-bit integer

    return ((((float)tempCount) * 0.5f) +
            23.0f); // temperature in degrees Centigrade
}

void BMX055::fastcompaccelBMX055(float *dest1)
{
    writeByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_CTRL,
              0x80); // set all accel offset compensation registers to zero
    writeByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_SETTING,
              0x20); // set offset targets to 0, 0, and +1 g for x, y, z axes
    writeByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_CTRL,
              0x20); // calculate x-axis offset

    uint8_t c = readByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_CTRL);
    while (!(c & 0x10))
    { // check if fast calibration complete
        c = readByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_CTRL);
        delay(10);
    }
    writeByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_CTRL,
              0x40); // calculate y-axis offset

    c = readByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_CTRL);
    while (!(c & 0x10))
    { // check if fast calibration complete
        c = readByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_CTRL);
        delay(10);
    }
    writeByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_CTRL,
              0x60); // calculate z-axis offset

    c = readByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_CTRL);
    while (!(c & 0x10))
    { // check if fast calibration complete
        c = readByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_CTRL);
        delay(10);
    }

    int8_t compx = readByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_OFFSET_X);
    int8_t compy = readByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_OFFSET_Y);
    int8_t compz = readByte(BMX055_ACC_ADDRESS, BMX055_ACC_OFC_OFFSET_Z);

    dest1[0] = (float)compx / 128.0f; // accleration bias in g
    dest1[1] = (float)compy / 128.0f; // accleration bias in g
    dest1[2] = (float)compz / 128.0f; // accleration bias in g
}

void BMX055::magcalBMX055(float *dest1)
{
    uint16_t ii = 0, sample_count = 0;
    int32_t mag_bias[3] = {0, 0, 0};
    int16_t mag_max[3] = {0, 0, 0}, mag_min[3] = {0, 0, 0};

    // ## "Mag Calibration: Wave device in a figure eight until done!" ##
    // wait(4);

    sample_count = 48;
    for (ii = 0; ii < sample_count; ii++)
    {
        int16_t mag_temp[3] = {0, 0, 0};
        readMagData(mag_temp);
        for (int jj = 0; jj < 3; jj++)
        {
            if (mag_temp[jj] > mag_max[jj])
                mag_max[jj] = mag_temp[jj];
            if (mag_temp[jj] < mag_min[jj])
                mag_min[jj] = mag_temp[jj];
        }
        delay(105); // at 10 Hz ODR, new mag data is available every 100 ms
    }

    mag_bias[0] =
        (mag_max[0] + mag_min[0]) / 2; // get average x mag bias in counts
    mag_bias[1] =
        (mag_max[1] + mag_min[1]) / 2; // get average y mag bias in counts
    mag_bias[2] =
        (mag_max[2] + mag_min[2]) / 2; // get average z mag bias in counts

    // save mag biases in G for main program
    dest1[0] = (float)mag_bias[0] * mRes;
    dest1[1] = (float)mag_bias[1] * mRes;
    dest1[2] = (float)mag_bias[2] * mRes;

    // ## "Mag Calibration done!" ##
}

// Get trim values for magnetometer sensitivity
void BMX055::trim(void)
{
    uint8_t rawData[2]; // placeholder for 2-byte trim data

    dig_x1 = readByte(BMX055_ACC_ADDRESS, BMM050_DIG_X1);
    dig_x2 = readByte(BMX055_ACC_ADDRESS, BMM050_DIG_X2);
    dig_y1 = readByte(BMX055_ACC_ADDRESS, BMM050_DIG_Y1);
    dig_y2 = readByte(BMX055_ACC_ADDRESS, BMM050_DIG_Y2);
    dig_xy1 = readByte(BMX055_ACC_ADDRESS, BMM050_DIG_XY1);
    dig_xy2 = readByte(BMX055_ACC_ADDRESS, BMM050_DIG_XY2);
    readBytes(BMX055_MAG_ADDRESS, BMM050_DIG_Z1_LSB, 2, &rawData[0]);
    dig_z1 = (uint16_t)(((uint16_t)rawData[1] << 8) | rawData[0]);
    readBytes(BMX055_MAG_ADDRESS, BMM050_DIG_Z2_LSB, 2, &rawData[0]);
    dig_z2 = (int16_t)(((int16_t)rawData[1] << 8) | rawData[0]);
    readBytes(BMX055_MAG_ADDRESS, BMM050_DIG_Z3_LSB, 2, &rawData[0]);
    dig_z3 = (int16_t)(((int16_t)rawData[1] << 8) | rawData[0]);
    readBytes(BMX055_MAG_ADDRESS, BMM050_DIG_Z4_LSB, 2, &rawData[0]);
    dig_z4 = (int16_t)(((int16_t)rawData[1] << 8) | rawData[0]);
    readBytes(BMX055_MAG_ADDRESS, BMM050_DIG_XYZ1_LSB, 2, &rawData[0]);
    dig_xyz1 = (uint16_t)(((uint16_t)rawData[1] << 8) | rawData[0]);
}

/** Initialize device for active mode
 * @param Ascale set accel full scale
 * @param ACCBW set bandwidth for accelerometer
 * @param Gscale set gyro full scale
 * @param GODRBW set gyro ODR and bandwidth
 * @param Mmode set magnetometer operation mode
 * @param MODR set magnetometer data rate
 * @see ACCBW, GODRBW and MODR enums
 */
void BMX055::init(uint8_t mAscale, uint8_t mACCBW, uint8_t mGscale,
                  uint8_t mGODRBW, uint8_t mMmode, uint8_t mMODR)
{
    gAscale = mAscale;
    gGscale = mGscale;

    // Configure accelerometer
    writeByte(BMX055_ACC_ADDRESS, BMX055_ACC_PMU_RANGE,
              mAscale & 0x0F); // Set accelerometer full scale
    writeByte(BMX055_ACC_ADDRESS, BMX055_ACC_PMU_BW,
              (0x08 | mACCBW) & 0x0F);                     // Set accelerometer bandwidth
    writeByte(BMX055_ACC_ADDRESS, BMX055_ACC_D_HBW, 0x00); // Use filtered data

    // Configure Gyro
    writeByte(BMX055_GYRO_ADDRESS, BMX055_GYRO_RANGE,
              mGscale); // set GYRO FS range
    writeByte(BMX055_GYRO_ADDRESS, BMX055_GYRO_BW,
              mGODRBW); // set GYRO ODR and Bandwidth

    // Configure magnetometer
    writeByte(BMX055_MAG_ADDRESS, BMX055_MAG_PWR_CNTL1,
              0x82); // Softreset magnetometer, ends up in sleep mode
    delay(100);
    writeByte(BMX055_MAG_ADDRESS, BMX055_MAG_PWR_CNTL1,
              0x01); // Wake up magnetometer
    delay(100);
    writeByte(BMX055_MAG_ADDRESS, BMX055_MAG_PWR_CNTL2,
              mMODR << 3); // Normal mode

    // Set up four standard configurations for the magnetometer
    switch (mMmode)
    {
    case lowPower:
        // Low-power
        writeByte(BMX055_MAG_ADDRESS, BMX055_MAG_REP_XY,
                  0x01); // 3 repetitions (oversampling)
        writeByte(BMX055_MAG_ADDRESS, BMX055_MAG_REP_Z,
                  0x02); // 3 repetitions (oversampling)
        break;
    case Regular:
        // Regular
        writeByte(BMX055_MAG_ADDRESS, BMX055_MAG_REP_XY,
                  0x04); //  9 repetitions (oversampling)
        writeByte(BMX055_MAG_ADDRESS, BMX055_MAG_REP_Z,
                  0x16); // 15 repetitions (oversampling)
        break;
    case enhancedRegular:
        // Enhanced Regular
        writeByte(BMX055_MAG_ADDRESS, BMX055_MAG_REP_XY,
                  0x07); // 15 repetitions (oversampling)
        writeByte(BMX055_MAG_ADDRESS, BMX055_MAG_REP_Z,
                  0x22); // 27 repetitions (oversampling)
        break;
    case highAccuracy:
        // High Accuracy
        writeByte(BMX055_MAG_ADDRESS, BMX055_MAG_REP_XY,
                  0x17); // 47 repetitions (oversampling)
        writeByte(BMX055_MAG_ADDRESS, BMX055_MAG_REP_Z,
                  0x51); // 83 repetitions (oversampling)
        break;
    }

    // get sensor resolutions, only need to do this once
    getAres();
    getGres();
    // magnetometer resolution is 1 microTesla/16 counts or 1/1.6
    // milliGauss/count
    mRes = 1. / 1.6;

    trim(); // read the magnetometer calibration data

    fastcompaccelBMX055(accelBias);
    magcalBMX055(magBias);
    // TODO: see magcalBMX055(): 128 samples * 105ms = 13.44s
    // So far, magnetometer bias is calculated and subtracted here manually,
    // should construct an algorithm to do it automatically like the gyro and
    // accelerometer biases magBias[0] = -5.;   // User environmental x-axis
    // correction in milliGauss magBias[1] = -95.;  // User environmental y-axis
    // correction in milliGauss magBias[2] = -260.; // User environmental z-axis
    // correction in milliGauss
}

// Implementation of Sebastian Madgwick's "...efficient orientation filter
// for... inertial/magnetic sensor arrays" (see
// http://www.x-io.co.uk/category/open-source/ for examples and more details)
// which fuses acceleration, rotation rate, and magnetic moments to produce a
// quaternion-based estimate of absolute device orientation -- which can be
// converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
// The performance of the orientation filter is at least as good as conventional
// Kalman-based filtering algorithms but is much less computationally intensive
void BMX055::MadgwickQuaternionUpdate(float ax, float ay, float az, float gx,
                                      float gy, float gz, float mx, float my,
                                      float mz)
{
    float q1 = q[0], q2 = q[1], q3 = q[2],
          q4 = q[3]; // short name local variable for readability
    float norm;
    float hx, hy, _2bx, _2bz;
    float s1, s2, s3, s4;
    float qDot1, qDot2, qDot3, qDot4;

    // Auxiliary variables to avoid repeated arithmetic
    float _2q1mx;
    float _2q1my;
    float _2q1mz;
    float _2q2mx;
    float _4bx;
    float _4bz;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _2q4 = 2.0f * q4;
    float _2q1q3 = 2.0f * q1 * q3;
    float _2q3q4 = 2.0f * q3 * q4;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q1q4 = q1 * q4;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q2q4 = q2 * q4;
    float q3q3 = q3 * q3;
    float q3q4 = q3 * q4;
    float q4q4 = q4 * q4;

    // Normalise accelerometer measurement
    norm = sqrt(ax * ax + ay * ay + az * az);
    if (norm == 0.0f)
        return; // handle NaN
    norm = 1.0f / norm;
    ax *= norm;
    ay *= norm;
    az *= norm;

    // Normalise magnetometer measurement
    norm = sqrt(mx * mx + my * my + mz * mz);
    if (norm == 0.0f)
        return; // handle NaN
    norm = 1.0f / norm;
    mx *= norm;
    my *= norm;
    mz *= norm;

    // Reference direction of Earth's magnetic field
    _2q1mx = 2.0f * q1 * mx;
    _2q1my = 2.0f * q1 * my;
    _2q1mz = 2.0f * q1 * mz;
    _2q2mx = 2.0f * q2 * mx;
    hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 +
         _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
    hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 +
         my * q3q3 + _2q3 * mz * q4 - my * q4q4;
    _2bx = sqrt(hx * hx + hy * hy);
    _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 +
           _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    // Gradient decent algorithm corrective step
    s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) +
         _2q2 * (2.0f * q1q2 + _2q3q4 - ay) -
         _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) +
         (-_2bx * q4 + _2bz * q2) *
             (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) +
         _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) +
         _2q1 * (2.0f * q1q2 + _2q3q4 - ay) -
         4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) +
         _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) +
         (_2bx * q3 + _2bz * q1) *
             (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) +
         (_2bx * q4 - _4bz * q2) *
             (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) +
         _2q4 * (2.0f * q1q2 + _2q3q4 - ay) -
         4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) +
         (-_4bx * q3 - _2bz * q1) *
             (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) +
         (_2bx * q2 + _2bz * q4) *
             (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) +
         (_2bx * q1 - _4bz * q3) *
             (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) +
         _2q3 * (2.0f * q1q2 + _2q3q4 - ay) +
         (-_4bx * q4 + _2bz * q2) *
             (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) +
         (-_2bx * q1 + _2bz * q3) *
             (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) +
         _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    norm =
        sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4); // normalise step magnitude
    norm = 1.0f / norm;
    s1 *= norm;
    s2 *= norm;
    s3 *= norm;
    s4 *= norm;

    // Compute rate of change of quaternion
    qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - beta * s1;
    qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - beta * s2;
    qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - beta * s3;
    qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - beta * s4;

    // Integrate to yield quaternion
    q1 += qDot1 * deltat;
    q2 += qDot2 * deltat;
    q3 += qDot3 * deltat;
    q4 += qDot4 * deltat;
    norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4); // normalise quaternion
    norm = 1.0f / norm;
    q[0] = q1 * norm;
    q[1] = q2 * norm;
    q[2] = q3 * norm;
    q[3] = q4 * norm;
}

/** Get raw 9-axis motion sensor readings (accel/gyro/compass).
 * @param ax 12-bit signed integer container for accelerometer X-axis value
 * @param ay 12-bit signed integer container for accelerometer Y-axis value
 * @param az 12-bit signed integer container for accelerometer Z-axis value
 * @param gx 16-bit signed integer container for gyroscope X-axis value
 * @param gy 16-bit signed integer container for gyroscope Y-axis value
 * @param gz 16-bit signed integer container for gyroscope Z-axis value
 * @param mx 13-bit signed integer container for magnetometer X-axis value
 * @param my 13-bit signed integer container for magnetometer Y-axis value
 * @param mz 15-bit signed integer container for magnetometer Z-axis value
 * @see getAcceleration()
 * @see getRotation()
 * @see getMag()
 */
void BMX055::getRaw9(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx,
                     int16_t *gy, int16_t *gz, int16_t *mx, int16_t *my,
                     int16_t *mz)
{
    uint8_t rawData[8]; // x/y/z MSB and LSB registers raw data stored here

    // Read the six raw data registers into data array
    // Turn the MSB and LSB into a signed 12-bit value
    readBytes(BMX055_ACC_ADDRESS, BMX055_ACC_D_X_LSB, 6, rawData);
    *ax = (int16_t)(((int16_t)rawData[1] << 8) | rawData[0]) >> 4;
    *ay = (int16_t)(((int16_t)rawData[3] << 8) | rawData[2]) >> 4;
    *az = (int16_t)(((int16_t)rawData[5] << 8) | rawData[4]) >> 4;

    // Read the six raw data registers sequentially into data array
    // Turn the MSB and LSB into a signed 16-bit value
    readBytes(BMX055_GYRO_ADDRESS, BMX055_GYRO_RATE_X_LSB, 6, rawData);
    *gx = (int16_t)(((int16_t)rawData[1] << 8) | rawData[0]);
    *gy = (int16_t)(((int16_t)rawData[3] << 8) | rawData[2]);
    *gz = (int16_t)(((int16_t)rawData[5] << 8) | rawData[4]);

    // Read the six raw data registers into data array
    // 13-bit signed integer for x-axis and y-axis field
    // 15-bit signed integer for z-axis field
    readBytes(BMX055_MAG_ADDRESS, BMX055_MAG_XOUT_LSB, 8, rawData);
    if (rawData[6] & 0x01) // Check if data ready status bit is set
    {
        *mx = (int16_t)(((int16_t)rawData[1] << 8) | rawData[0]) >> 3;
        *my = (int16_t)(((int16_t)rawData[3] << 8) | rawData[2]) >> 3;
        *mz = (int16_t)(((int16_t)rawData[5] << 8) | rawData[4]) >> 1;
    }
}

/** Get raw 9-axis motion sensor readings (accel/gyro/compass).
 * @param ax accelerometer X-axis value (g's)
 * @param ay accelerometer Y-axis value (g's)
 * @param az accelerometer Z-axis value (g's)
 * @param gx gyroscope X-axis value (degrees per second)
 * @param gy gyroscope Y-axis value (degrees per second)
 * @param gz gyroscope Z-axis value (degrees per second)
 * @param mx magnetometer X-axis value (milliGauss)
 * @param my magnetometer Y-axis value (milliGauss)
 * @param mz magnetometer Z-axis value (milliGauss)
 * @see getAcceleration()
 * @see getRotation()
 * @see getMag()
 */
void BMX055::getMotion9vec3f(vec3f *accel, vec3f *gyro, vec3f *mag)
{
    // int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor
    // output int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor
    // output int16_t magCount[3];    // Stores the 13/15-bit signed magnetometer
    // sensor output

    // Read the x/y/z raw values
    readAccelData(accelCount);

    // Calculate the accleration value into actual g's
    // get actual g value, this depends on scale being set
    accel->x = (float)accelCount[0] * aRes; // + accelBias[0];
    accel->y = (float)accelCount[1] * aRes; // + accelBias[1];
    accel->z = (float)accelCount[2] * aRes; // + accelBias[2];

    // Read the x/y/z raw values
    readGyroData(gyroCount);

    // Calculate the gyro value into actual degrees per second
    // get actual gyro value, this depends on scale being set
    gyro->x = (float)gyroCount[0] * gRes;
    gyro->y = (float)gyroCount[1] * gRes;
    gyro->z = (float)gyroCount[2] * gRes;

    // Read the x/y/z raw values
    readMagData(magCount);

    // Calculate the magnetometer values in milliGauss
    // Temperature-compensated magnetic field is in 16 LSB/microTesla
    // get actual magnetometer value, this depends on scale being set
    mag->x = (float)magCount[0] * mRes - magBias[0];
    mag->y = (float)magCount[1] * mRes - magBias[1];
    mag->z = (float)magCount[2] * mRes - magBias[2];
}

void BMX055::getMotion9(float *ax, float *ay, float *az, float *gx, float *gy, float *gz, float *mx, float *my, float *mz)
{
    // int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor
    // output int16_t gyroCount[3];   // Stores the 16-bit signed gyro sensor
    // output int16_t magCount[3];    // Stores the 13/15-bit signed magnetometer
    // sensor output

    // Read the x/y/z raw values
    readAccelData(accelCount);

    // Calculate the accleration value into actual g's
    // get actual g value, this depends on scale being set
    *ax = (float)accelCount[0] * aRes; // + accelBias[0];
    *ay = (float)accelCount[1] * aRes; // + accelBias[1];
    *az = (float)accelCount[2] * aRes; // + accelBias[2];

    // Read the x/y/z raw values
    readGyroData(gyroCount);

    // Calculate the gyro value into actual degrees per second
    // get actual gyro value, this depends on scale being set
    *gx = (float)gyroCount[0] * gRes;
    *gy = (float)gyroCount[1] * gRes;
    *gz = (float)gyroCount[2] * gRes;

    // Read the x/y/z raw values
    readMagData(magCount);

    // Calculate the magnetometer values in milliGauss
    // Temperature-compensated magnetic field is in 16 LSB/microTesla
    // get actual magnetometer value, this depends on scale being set
    *mx = (float)magCount[0] * mRes - magBias[0];
    *my = (float)magCount[1] * mRes - magBias[1];
    *mz = (float)magCount[2] * mRes - magBias[2];
}

void BMX055::runAHRS(float mdeltat, float local_declination)
{
    float ax, ay, az, gx, gy, gz, mx, my, mz;

    getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
    deltat = mdeltat;

    // Sensors x (y)-axis of the accelerometer is aligned with the -y (x)-axis
    // of the magnetometer; the magnetometer z-axis (+ up) is aligned with
    // z-axis (+ up) of accelerometer and gyro! We have to make some allowance
    // for this orientation mismatch in feeding the output to the quaternion
    // filter. For the BMX-055, we have chosen a magnetic rotation that keeps
    // the sensor forward along the x-axis just like in the MPU9250 sensor. This
    // rotation can be modified to allow any convenient orientation convention.
    // This is ok by aircraft orientation standards!
    // Pass gyro rate as rad/s
    // MadgwickQuaternionUpdate(ax, ay, az, gx*M_PI/180.0f, gy*M_PI/180.0f,
    // gz*M_PI/180.0f, -my, mx, mz);
    MadgwickQuaternionUpdate(-ay, ax, az, -gy * M_PI / 180.0f,
                             gx * M_PI / 180.0f, gz * M_PI / 180.0f, mx, my,
                             mz);

    // Define output variables from updated quaternion---these are Tait-Bryan
    // angles, commonly used in aircraft orientation. In this coordinate system,
    // the positive z-axis is down toward Earth. Yaw is the angle between Sensor
    // x-axis and Earth magnetic North (or true North if corrected for local
    // declination, looking down on the sensor positive yaw is counterclockwise.
    // Pitch is angle between sensor x-axis and Earth ground plane, toward the
    // Earth is positive, up toward the sky is negative. Roll is angle between
    // sensor y-axis and Earth ground plane, y-axis up is positive roll. These
    // arise from the definition of the homogeneous rotation matrix constructed
    // from quaternions. Tait-Bryan angles as well as Euler angles are
    // non-commutative; that is, the get the correct orientation the rotations
    // must be applied in the correct order which for this configuration is yaw,
    // pitch, and then roll. For more see
    // http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    // which has additional links.
    yaw = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]),
                q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
    pitch = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
    roll = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]),
                 q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
    pitch *= 180.0f / M_PI;
    yaw *= 180.0f / M_PI;
    yaw -= local_declination;
    roll *= 180.0f / M_PI;
}

void BMX055::idleTick()
{
    readAccelData(accData);
    readGyroData(gyroData);
    readMagData(magData);
}
