////////////////////////////////////////////////////////////////////////////
//
//  This file is part of RTIMULib-Arduino
//
//  Copyright (c) 2014-2015, richards-tech
//
//  Permission is hereby granted, free of charge, to any person obtaining a copy of 
//  this software and associated documentation files (the "Software"), to deal in 
//  the Software without restriction, including without limitation the rights to use, 
//  copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the 
//  Software, and to permit persons to whom the Software is furnished to do so, 
//  subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all 
//  copies or substantial portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
//  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
//  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT 
//  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION 
//  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
//  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#ifndef _RTIMUMPU9150_H
#define	_RTIMUMPU9150_H

#include "RTIMU.h"

//  MPU9150 I2C Slave Addresses

#define MPU9150_ADDRESS0            0x68
#define MPU9150_ADDRESS1            0x69
#define MPU9150_ID                  0x68

//  these magnetometers are on aux bus

#define AK8975_ADDRESS              0x0c
#define HMC5883_ADDRESS             0x1e

//  Register map

#define MPU9150_YG_OFFS_TC          0x01
#define MPU9150_SMPRT_DIV           0x19
#define MPU9150_LPF_CONFIG          0x1a
#define MPU9150_GYRO_CONFIG         0x1b
#define MPU9150_ACCEL_CONFIG        0x1c
#define MPU9150_FIFO_EN             0x23
#define MPU9150_I2C_MST_CTRL        0x24
#define MPU9150_I2C_SLV0_ADDR       0x25
#define MPU9150_I2C_SLV0_REG        0x26
#define MPU9150_I2C_SLV0_CTRL       0x27
#define MPU9150_I2C_SLV1_ADDR       0x28
#define MPU9150_I2C_SLV1_REG        0x29
#define MPU9150_I2C_SLV1_CTRL       0x2a
#define MPU9150_I2C_SLV4_CTRL       0x34
#define MPU9150_INT_PIN_CFG         0x37
#define MPU9150_INT_ENABLE          0x38
#define MPU9150_INT_STATUS          0x3a
#define MPU9150_ACCEL_XOUT_H        0x3b
#define MPU9150_GYRO_XOUT_H         0x43
#define MPU9150_EXT_SENS_DATA_00    0x49
#define MPU9150_I2C_SLV1_DO         0x64
#define MPU9150_I2C_MST_DELAY_CTRL  0x67
#define MPU9150_USER_CTRL           0x6a
#define MPU9150_PWR_MGMT_1          0x6b
#define MPU9150_PWR_MGMT_2          0x6c
#define MPU9150_FIFO_COUNT_H        0x72
#define MPU9150_FIFO_R_W            0x74
#define MPU9150_WHO_AM_I            0x75

//  sample rate defines (applies to gyros and accels, not mags)

#define MPU9150_SAMPLERATE_MIN      5                      // 5 samples per second is the lowest
#define MPU9150_SAMPLERATE_MAX      1000                   // 1000 samples per second is the absolute maximum

//  compass rate defines

#define MPU9150_COMPASSRATE_MIN     1                      // 1 samples per second is the lowest
#define MPU9150_COMPASSRATE_MAX     100                    // 100 samples per second is maximum

//  LPF options (gyros and accels)

#define MPU9150_LPF_256             0                       // gyro: 256Hz, accel: 260Hz
#define MPU9150_LPF_188             1                       // gyro: 188Hz, accel: 184Hz
#define MPU9150_LPF_98              2                       // gyro: 98Hz, accel: 98Hz
#define MPU9150_LPF_42              3                       // gyro: 42Hz, accel: 44Hz
#define MPU9150_LPF_20              4                       // gyro: 20Hz, accel: 21Hz
#define MPU9150_LPF_10              5                       // gyro: 10Hz, accel: 10Hz
#define MPU9150_LPF_5               6                       // gyro: 5Hz, accel: 5Hz

//  Gyro FSR options

#define MPU9150_GYROFSR_250         0                       // +/- 250 degrees per second
#define MPU9150_GYROFSR_500         8                       // +/- 500 degrees per second
#define MPU9150_GYROFSR_1000        0x10                    // +/- 1000 degrees per second
#define MPU9150_GYROFSR_2000        0x18                    // +/- 2000 degrees per second

//  Accel FSR options

#define MPU9150_ACCELFSR_2          0                       // +/- 2g
#define MPU9150_ACCELFSR_4          8                       // +/- 4g
#define MPU9150_ACCELFSR_8          0x10                    // +/- 8g
#define MPU9150_ACCELFSR_16         0x18                    // +/- 16g


//  AK8975 compass registers

#define AK8975_DEVICEID             0x0                     // the device ID
#define AK8975_ST1                  0x02                    // status 1
#define AK8975_CNTL                 0x0a                    // control reg
#define AK8975_ASAX                 0x10                    // start of the fuse ROM data

//  HMC5883 compass registers

#define HMC5883_CONFIG_A            0x00                    // configuration A
#define HMC5883_CONFIG_B            0x01                    // configuration B
#define HMC5883_MODE                0x02                    // mode
#define HMC5883_DATA_X_HI           0x03                    // data x msb
#define HMC5883_STATUS              0x09                    // status
#define HMC5883_ID                  0x0a                    // id

//  FIFO transfer size

#define MPU9150_FIFO_CHUNK_SIZE     12                      // gyro and accels take 12 bytes

class RTIMUMPU9150 : public RTIMU
{
public:
    RTIMUMPU9150(RTIMUSettings *settings);
    ~RTIMUMPU9150();

    bool setLpf(unsigned char lpf);
    bool setSampleRate(int rate);
    bool setCompassRate(int rate);
    bool setGyroFsr(unsigned char fsr);
    bool setAccelFsr(unsigned char fsr);

    virtual const char *IMUName() { return "MPU-9150"; }
    virtual int IMUType() { return RTIMU_TYPE_MPU9150; }
    virtual int IMUInit();
    virtual bool IMURead();
    virtual int IMUGetPollInterval();

private:
    bool configureCompass();                                // configure the compass
    bool bypassOn();                                        // talk to compass
    bool bypassOff();                                       // talk to MPU9150
    bool setSampleRate();
    bool setCompassRate();
    bool resetFifo();

    bool m_firstTime;                                       // if first sample

    unsigned char m_slaveAddr;                              // I2C address of MPU9150
    unsigned char m_bus;                                    // I2C bus (usually 1 for Raspberry Pi for example)

    unsigned char m_lpf;                                    // low pass filter setting
    int m_compassRate;                                      // compass sample rate in Hz
    unsigned char m_gyroFsr;
    unsigned char m_accelFsr;

    RTFLOAT m_gyroScale;
    RTFLOAT m_accelScale;

    RTFLOAT m_compassAdjust[3];                             // the compass fuse ROM values converted for use
    bool m_compassPresent;                                  // false for MPU-6050
    bool m_compassIs5883;                                   // if it is an MPU-6050/HMC5883 combo
    int m_compassDataLength;                                // 8 for MPU-9150, 6 for HMC5883
};

#endif // _RTIMUMPU9150_H
