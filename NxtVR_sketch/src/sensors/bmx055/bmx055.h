/* IMU chipset BMX055 Library
 * Copyright (c) 2016 Ioton Technology
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef _BMX055_H_
#define _BMX055_H_

/* Includes ------------------------------------------------------------------*/
#include <Wire.h>
#include "../../hid_hmd/hid_utils.h"

#ifndef M_PI
#define M_PI 3.1415927f
#endif

// Accelerometer registers
#define BMX055_ACC_WHOAMI 0x00 // should return 0xFA
//#define BMX055_ACC_Reserved    0x01
#define BMX055_ACC_D_X_LSB 0x02
#define BMX055_ACC_D_X_MSB 0x03
#define BMX055_ACC_D_Y_LSB 0x04
#define BMX055_ACC_D_Y_MSB 0x05
#define BMX055_ACC_D_Z_LSB 0x06
#define BMX055_ACC_D_Z_MSB 0x07
#define BMX055_ACC_D_TEMP 0x08
#define BMX055_ACC_INT_STATUS_0 0x09
#define BMX055_ACC_INT_STATUS_1 0x0A
#define BMX055_ACC_INT_STATUS_2 0x0B
#define BMX055_ACC_INT_STATUS_3 0x0C
//#define BMX055_ACC_Reserved    0x0D
#define BMX055_ACC_FIFO_STATUS 0x0E
#define BMX055_ACC_PMU_RANGE 0x0F
#define BMX055_ACC_PMU_BW 0x10
#define BMX055_ACC_PMU_LPW 0x11
#define BMX055_ACC_PMU_LOW_POWER 0x12
#define BMX055_ACC_D_HBW 0x13
#define BMX055_ACC_BGW_SOFTRESET 0x14
//#define BMX055_ACC_Reserved    0x15
#define BMX055_ACC_INT_EN_0 0x16
#define BMX055_ACC_INT_EN_1 0x17
#define BMX055_ACC_INT_EN_2 0x18
#define BMX055_ACC_INT_MAP_0 0x19
#define BMX055_ACC_INT_MAP_1 0x1A
#define BMX055_ACC_INT_MAP_2 0x1B
//#define BMX055_ACC_Reserved    0x1C
//#define BMX055_ACC_Reserved    0x1D
#define BMX055_ACC_INT_SRC 0x1E
//#define BMX055_ACC_Reserved    0x1F
#define BMX055_ACC_INT_OUT_CTRL 0x20
#define BMX055_ACC_INT_RST_LATCH 0x21
#define BMX055_ACC_INT_0 0x22
#define BMX055_ACC_INT_1 0x23
#define BMX055_ACC_INT_2 0x24
#define BMX055_ACC_INT_3 0x25
#define BMX055_ACC_INT_4 0x26
#define BMX055_ACC_INT_5 0x27
#define BMX055_ACC_INT_6 0x28
#define BMX055_ACC_INT_7 0x29
#define BMX055_ACC_INT_8 0x2A
#define BMX055_ACC_INT_9 0x2B
#define BMX055_ACC_INT_A 0x2C
#define BMX055_ACC_INT_B 0x2D
#define BMX055_ACC_INT_C 0x2E
#define BMX055_ACC_INT_D 0x2F
#define BMX055_ACC_FIFO_CONFIG_0 0x30
//#define BMX055_ACC_Reserved    0x31
#define BMX055_ACC_PMU_SELF_TEST 0x32
#define BMX055_ACC_TRIM_NVM_CTRL 0x33
#define BMX055_ACC_BGW_SPI3_WDT 0x34
//#define BMX055_ACC_Reserved    0x35
#define BMX055_ACC_OFC_CTRL 0x36
#define BMX055_ACC_OFC_SETTING 0x37
#define BMX055_ACC_OFC_OFFSET_X 0x38
#define BMX055_ACC_OFC_OFFSET_Y 0x39
#define BMX055_ACC_OFC_OFFSET_Z 0x3A
#define BMX055_ACC_TRIM_GPO 0x3B
#define BMX055_ACC_TRIM_GP1 0x3C
//#define BMX055_ACC_Reserved    0x3D
#define BMX055_ACC_FIFO_CONFIG_1 0x3E
#define BMX055_ACC_FIFO_DATA 0x3F

// BMX055 Gyroscope Registers
#define BMX055_GYRO_WHOAMI 0x00 // should return 0x0F
//#define BMX055_GYRO_Reserved       0x01
#define BMX055_GYRO_RATE_X_LSB 0x02
#define BMX055_GYRO_RATE_X_MSB 0x03
#define BMX055_GYRO_RATE_Y_LSB 0x04
#define BMX055_GYRO_RATE_Y_MSB 0x05
#define BMX055_GYRO_RATE_Z_LSB 0x06
#define BMX055_GYRO_RATE_Z_MSB 0x07
//#define BMX055_GYRO_Reserved       0x08
#define BMX055_GYRO_INT_STATUS_0 0x09
#define BMX055_GYRO_INT_STATUS_1 0x0A
#define BMX055_GYRO_INT_STATUS_2 0x0B
#define BMX055_GYRO_INT_STATUS_3 0x0C
//#define BMX055_GYRO_Reserved    0x0D
#define BMX055_GYRO_FIFO_STATUS 0x0E
#define BMX055_GYRO_RANGE 0x0F
#define BMX055_GYRO_BW 0x10
#define BMX055_GYRO_LPM1 0x11
#define BMX055_GYRO_LPM2 0x12
#define BMX055_GYRO_RATE_HBW 0x13
#define BMX055_GYRO_BGW_SOFTRESET 0x14
#define BMX055_GYRO_INT_EN_0 0x15
#define BMX055_GYRO_INT_EN_1 0x16
#define BMX055_GYRO_INT_MAP_0 0x17
#define BMX055_GYRO_INT_MAP_1 0x18
#define BMX055_GYRO_INT_MAP_2 0x19
#define BMX055_GYRO_INT_SRC_1 0x1A
#define BMX055_GYRO_INT_SRC_2 0x1B
#define BMX055_GYRO_INT_SRC_3 0x1C
//#define BMX055_GYRO_Reserved    0x1D
#define BMX055_GYRO_FIFO_EN 0x1E
//#define BMX055_GYRO_Reserved    0x1F
//#define BMX055_GYRO_Reserved    0x20
#define BMX055_GYRO_INT_RST_LATCH 0x21
#define BMX055_GYRO_HIGH_TH_X 0x22
#define BMX055_GYRO_HIGH_DUR_X 0x23
#define BMX055_GYRO_HIGH_TH_Y 0x24
#define BMX055_GYRO_HIGH_DUR_Y 0x25
#define BMX055_GYRO_HIGH_TH_Z 0x26
#define BMX055_GYRO_HIGH_DUR_Z 0x27
//#define BMX055_GYRO_Reserved    0x28
//#define BMX055_GYRO_Reserved    0x29
//#define BMX055_GYRO_Reserved    0x2A
#define BMX055_GYRO_SOC 0x31
#define BMX055_GYRO_A_FOC 0x32
#define BMX055_GYRO_TRIM_NVM_CTRL 0x33
#define BMX055_GYRO_BGW_SPI3_WDT 0x34
//#define BMX055_GYRO_Reserved    0x35
#define BMX055_GYRO_OFC1 0x36
#define BMX055_GYRO_OFC2 0x37
#define BMX055_GYRO_OFC3 0x38
#define BMX055_GYRO_OFC4 0x39
#define BMX055_GYRO_TRIM_GP0 0x3A
#define BMX055_GYRO_TRIM_GP1 0x3B
#define BMX055_GYRO_BIST 0x3C
#define BMX055_GYRO_FIFO_CONFIG_0 0x3D
#define BMX055_GYRO_FIFO_CONFIG_1 0x3E

// BMX055 magnetometer registers
#define BMX055_MAG_WHOAMI 0x40 // should return 0x32
#define BMX055_MAG_Reserved 0x41
#define BMX055_MAG_XOUT_LSB 0x42
#define BMX055_MAG_XOUT_MSB 0x43
#define BMX055_MAG_YOUT_LSB 0x44
#define BMX055_MAG_YOUT_MSB 0x45
#define BMX055_MAG_ZOUT_LSB 0x46
#define BMX055_MAG_ZOUT_MSB 0x47
#define BMX055_MAG_ROUT_LSB 0x48
#define BMX055_MAG_ROUT_MSB 0x49
#define BMX055_MAG_INT_STATUS 0x4A
#define BMX055_MAG_PWR_CNTL1 0x4B
#define BMX055_MAG_PWR_CNTL2 0x4C
#define BMX055_MAG_INT_EN_1 0x4D
#define BMX055_MAG_INT_EN_2 0x4E
#define BMX055_MAG_LOW_THS 0x4F
#define BMX055_MAG_HIGH_THS 0x50
#define BMX055_MAG_REP_XY 0x51
#define BMX055_MAG_REP_Z 0x52
/* Trim Extended Registers */
#define BMM050_DIG_X1 0x5D // needed for magnetic field calculation
#define BMM050_DIG_Y1 0x5E
#define BMM050_DIG_Z4_LSB 0x62
#define BMM050_DIG_Z4_MSB 0x63
#define BMM050_DIG_X2 0x64
#define BMM050_DIG_Y2 0x65
#define BMM050_DIG_Z2_LSB 0x68
#define BMM050_DIG_Z2_MSB 0x69
#define BMM050_DIG_Z1_LSB 0x6A
#define BMM050_DIG_Z1_MSB 0x6B
#define BMM050_DIG_XYZ1_LSB 0x6C
#define BMM050_DIG_XYZ1_MSB 0x6D
#define BMM050_DIG_Z3_LSB 0x6E
#define BMM050_DIG_Z3_MSB 0x6F
#define BMM050_DIG_XY2 0x70
#define BMM050_DIG_XY1 0x71

// Using SDO1 = SDO2 = CSB3 = 3V3|VCC
// Seven-bit device addresses are ACC = 0x19, GYRO = 0x69, MAG = 0x13
#define BMX055_ACC_ADDRESS 0x19  // Address of BMX055 accelerometer
#define BMX055_GYRO_ADDRESS 0x69 // Address of BMX055 gyroscope
#define BMX055_MAG_ADDRESS 0x13  // Address of BMX055 magnetometer

// Set initial input parameters
// define BMX055 ACC full scale options
#define AFS_2G 0x03
#define AFS_4G 0x05
#define AFS_8G 0x08
#define AFS_16G 0x0C

enum ACCBW
{              // define BMX055 accelerometer bandwidths
    ABW_8Hz,   // 7.81 Hz,  64 ms update time
    ABW_16Hz,  // 15.63 Hz, 32 ms update time
    ABW_31Hz,  // 31.25 Hz, 16 ms update time
    ABW_63Hz,  // 62.5  Hz,  8 ms update time
    ABW_125Hz, // 125   Hz,  4 ms update time
    ABW_250Hz, // 250   Hz,  2 ms update time
    ABW_500Hz, // 500   Hz,  1 ms update time
    ABW_1000Hz // 1000  Hz,  0.5 ms update time
};

enum Gscale
{
    GFS_2000DPS = 0,
    GFS_1000DPS,
    GFS_500DPS,
    GFS_250DPS,
    GFS_125DPS
};

enum GODRBW
{
    G_2000Hz523Hz = 0, // 2000 Hz ODR and unfiltered (bandwidth 523Hz)
    G_2000Hz230Hz,
    G_1000Hz116Hz,
    G_400Hz47Hz,
    G_200Hz23Hz,
    G_100Hz12Hz,
    G_200Hz64Hz,
    G_100Hz32Hz // 100 Hz ODR and 32 Hz bandwidth
};

enum MODR
{
    MODR_10Hz = 0, // 10 Hz ODR
    MODR_2Hz,      // 2 Hz ODR
    MODR_6Hz,      // 6 Hz ODR
    MODR_8Hz,      // 8 Hz ODR
    MODR_15Hz,     // 15 Hz ODR
    MODR_20Hz,     // 20 Hz ODR
    MODR_25Hz,     // 25 Hz ODR
    MODR_30Hz      // 30 Hz ODR
};

enum Mmode
{
    lowPower = 0,    // rms noise ~1.0 microTesla, 0.17 mA power
    Regular,         // rms noise ~0.6 microTesla, 0.5 mA power
    enhancedRegular, // rms noise ~0.5 microTesla, 0.8 mA power
    highAccuracy     // rms noise ~0.3 microTesla, 4.9 mA power
};

#define Kp 2.0f * 5.0f // these are the free parameters in the Mahony filter and fusion scheme, Kp for proportional feedback, Ki for integral
#define Ki 0.0f

// Declination at Sao Paulo, Brazil is -21 degrees 7 minutes on 2016-03-27
#define LOCAL_DECLINATION -21.1f

class BMX055
{
private:
    bool wire_on = false;
    float pitch, yaw, roll;
    int16_t accData[6];
    int16_t gyroData[6];
    int16_t magData[6];

    // Specify sensor full scale
    uint8_t gAscale = AFS_2G;
    uint8_t gGscale = GFS_125DPS;
    float aRes, gRes, mRes; // scale resolutions per LSB for the sensors

    // Parameters to hold BMX055 trim values
    int8_t dig_x1;
    int8_t dig_y1;
    int8_t dig_x2;
    int8_t dig_y2;
    uint16_t dig_z1;
    int16_t dig_z2;
    int16_t dig_z3;
    int16_t dig_z4;
    uint8_t dig_xy1;
    int8_t dig_xy2;
    uint16_t dig_xyz1;

    // BMX055 variables
    int16_t accelCount[3]; // Stores the 16-bit signed accelerometer sensor output
    int16_t gyroCount[3];  // Stores the 16-bit signed gyro sensor output
    int16_t magCount[3];   // Stores the 13/15-bit signed magnetometer sensor output

    float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0}, magBias[3] = {0, 0, 0}; // Bias corrections for gyro, accelerometer, mag
    float SelfTest[6];                                                               // holds results of gyro and accelerometer self test

    // global constants for 9 DoF fusion and AHRS (Attitude and Heading Reference System)
    float GyroMeasError = M_PI * (40.0f / 180.0f); // gyroscope measurement error in rads/s (start at 40 deg/s)
    float GyroMeasDrift = M_PI * (0.0f / 180.0f);  // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
    // There is a tradeoff in the beta parameter between accuracy and response speed.
    // In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
    // However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
    // Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
    // By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
    // I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense;
    // the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy.
    // In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
    float beta = sqrt(3.0f / 4.0f) * GyroMeasError; // compute beta
    float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift; // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value

    float deltat = 0.0f; // integration interval for both filter schemes

    float q[4] = {1.0f, 0.0f, 0.0f, 0.0f}; // vector to hold quaternion
    float eInt[3] = {0.0f, 0.0f, 0.0f};    // vector to hold integral error for Mahony method

    void writeByte(uint8_t address, uint8_t subAddress, uint8_t data)
    {
        Wire.beginTransmission(address);
        Wire.write(subAddress);
        Wire.write(data);
        Wire.endTransmission();
    }

    char readByte(uint8_t address, uint8_t subAddress)
    {
        uint8_t data = 0;

        readBytes(address, subAddress, 1, &data);

        return (char)data;
    }

    void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t *dest)
    {
        Wire.beginTransmission(address);
        Wire.write(subAddress);
        Wire.endTransmission();

        Wire.requestFrom((int)address, count);

        int i = 0;
        while (Wire.available())
        {
            dest[i] = Wire.read();
            i++;
        }
    }

public:
    void begin(void)
    {
        if (!wire_on)
        {
            Wire.begin();
            wire_on = true;
        };
    };

    // BMX055();

    //######################################
    int getAccX();
    int getAccY();
    int getAccZ();

    int getGyroX();
    int getGyroY();
    int getGyroZ();

    int getMagX();
    int getMagY();
    int getMagZ();
    //######################################

    float getAres(void);
    float getGres(void);
    float getPitch(void);
    float getRoll(void);
    float getYaw(void);
    void readAccelData(int16_t *destination);
    void readGyroData(int16_t *destination);
    void readMagData(int16_t *magData);
    float getTemperature();
    void fastcompaccelBMX055(float *dest1);
    void magcalBMX055(float *dest1);
    // Get trim values for magnetometer sensitivity
    void trim(void);
    /** Initialize device for active mode
     * @param Ascale set accel full scale
     * @param ACCBW set bandwidth for accelerometer
     * @param Gscale set gyro full scale
     * @param GODRBW set gyro ODR and bandwidth
     * @param Mmode set magnetometer operation mode
     * @param MODR set magnetometer data rate
     * @see ACCBW, GODRBW and MODR enums
     */
    void init(uint8_t mAscale = AFS_2G, uint8_t mACCBW = ABW_16Hz,
              uint8_t mGscale = GFS_125DPS, uint8_t mGODRBW = G_200Hz23Hz,
              uint8_t mMmode = Regular, uint8_t mMODR = MODR_10Hz);

    // Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
    // (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
    // which fuses acceleration, rotation rate, and magnetic moments to produce a quaternion-based estimate of absolute
    // device orientation -- which can be converted to yaw, pitch, and roll. Useful for stabilizing quadcopters, etc.
    // The performance of the orientation filter is at least as good as conventional Kalman-based filtering algorithms
    // but is much less computationally intensive
    void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
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
    void getRaw9(int16_t *ax, int16_t *ay, int16_t *az,
                 int16_t *gx, int16_t *gy, int16_t *gz,
                 int16_t *mx, int16_t *my, int16_t *mz);

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
    void getMotion9vec3f(vec3f *accel, vec3f *gyro, vec3f *mag);
    void getMotion9(float* ax, float* ay, float* az,
                        float* gx, float* gy, float* gz,
                        float* mx, float* my, float* mz);
    void runAHRS(float mdeltat, float local_declination = LOCAL_DECLINATION);
    void idleTick();
};

#endif /* BMX055_H_ */
