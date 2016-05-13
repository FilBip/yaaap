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

#include "RTPressureMS5611.h"

RTPressureMS5611::RTPressureMS5611(RTIMUSettings *settings) : RTPressure(settings)
{
    m_validReadings = false;
}

RTPressureMS5611::~RTPressureMS5611()
{
}

bool RTPressureMS5611::pressureInit()
{
    unsigned char cmd = MS5611_CMD_PROM + 2;
    unsigned char data[2];

    m_pressureAddr = m_settings->m_I2CPressureAddress;

    // get calibration data

    for (int i = 0; i < 6; i++) {
        if (!I2Cdev::readBytes(m_pressureAddr, cmd, 2, data))
            return false;
        m_calData[i] = (((uint16_t)data[0]) << 8) + (uint16_t)data[1];
        // printf("Cal index: %d, data: %d\n", i, m_calData[i]);
        cmd += 2;
    }

    m_state = MS5611_STATE_IDLE;
    return true;
}

bool RTPressureMS5611::pressureRead(float &latestPressure, float &latestTemperature)
{
    if (m_state == MS5611_STATE_IDLE) {
        // start pressure conversion
        if (!I2Cdev::writeBytes(m_pressureAddr, MS5611_CMD_CONV_D1, 0, 0)) {
            return false;
        } else {
            m_state = MS5611_STATE_PRESSURE;
            m_timer = millis();
        }
    }

    pressureBackground();

    if (m_validReadings) {
        latestTemperature = m_temperature;
        latestPressure = m_pressure;
    }
    return true;
}


void RTPressureMS5611::pressureBackground()
{
    uint8_t data[3];

    switch (m_state) {
        case MS5611_STATE_IDLE:
        break;

        case MS5611_STATE_PRESSURE:
        if ((millis() - m_timer) < 10)
            break;                                          // not time yet
        if (!I2Cdev::readBytes(m_pressureAddr, MS5611_CMD_ADC, 3, data)) {
            break;
        }
        m_D1 = (((uint32_t)data[0]) << 16) + (((uint32_t)data[1]) << 8) + (uint32_t)data[2];

        // start temperature conversion

        if (!I2Cdev::writeBytes(m_pressureAddr, MS5611_CMD_CONV_D2, 0, 0)) {
            break;
        } else {
            m_state = MS5611_STATE_TEMPERATURE;
            m_timer = millis();
        }
        break;

        case MS5611_STATE_TEMPERATURE:
        if ((millis() - m_timer) < 10)
            break;                                          // not time yet
        if (!I2Cdev::readBytes(m_pressureAddr, MS5611_CMD_ADC, 3, data)) {
            break;
        }
        m_D2 = (((uint32_t)data[0]) << 16) + (((uint32_t)data[1]) << 8) + (uint32_t)data[2];

        //  call this function for testing only
        //  should give T = 2007 (20.07C) and pressure 100009 (1000.09hPa)

        // setTestData();

        //  now calculate the real values

        int64_t deltaT = (int32_t)m_D2 - (((int32_t)m_calData[4]) << 8);

        int32_t temperature = 2000 + ((deltaT * (int64_t)m_calData[5]) >> 23); // note - this needs to be divided by 100

        int64_t offset = ((int64_t)m_calData[1] << 16) + (((int64_t)m_calData[3] * deltaT) >> 7);
        int64_t sens = ((int64_t)m_calData[0] << 15) + (((int64_t)m_calData[2] * deltaT) >> 8);

        //  do second order temperature compensation

        if (temperature < 2000) {
            int64_t T2 = (deltaT * deltaT) >> 31;
            int64_t offset2 = 5 * ((temperature - 2000) * (temperature - 2000)) / 2;
            int64_t sens2 = offset2 / 2;
            if (temperature < -1500) {
                offset2 += 7 * (temperature + 1500) * (temperature + 1500);
                sens2 += 11 * ((temperature + 1500) * (temperature + 1500)) / 2;
            }
            temperature -= T2;
            offset -= offset2;
            sens -= sens2;
        }

        m_pressure = (RTFLOAT)(((((int64_t)m_D1 * sens) >> 21) - offset) >> 15) / (RTFLOAT)100.0;
        m_temperature = (RTFLOAT)temperature / (RTFLOAT)100;

        // printf("Temp: %f, pressure: %f\n", m_temperature, m_pressure);

        m_validReadings = true;
        m_state = MS5611_STATE_IDLE;
        break;
    }
}

void RTPressureMS5611::setTestData()
{
    m_calData[0] = 40127;
    m_calData[1] = 36924;
    m_calData[2] = 23317;
    m_calData[3] = 23282;
    m_calData[4] = 33464;
    m_calData[5] = 28312;

    m_D1 = 9085466;
    m_D2 = 8569150;
}
