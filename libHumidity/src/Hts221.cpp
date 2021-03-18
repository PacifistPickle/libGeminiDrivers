#include <stdio.h>
#include "Hts221.h"

namespace gemini  {
namespace humidity {

Hts221::Hts221(interfaces::I2cInterface & i2c) : _i2c(i2c)
, _T0_DEGC_X8(0)
, _T1_DEGC_X8(0)
, _T0_out(0)
, _T1_out(0)
, _H0_RH_x2(0)
, _H1_RH_x2(0)
, _H0_T0_OUT(0)
, _H1_T0_OUT(0)
{

}

bool Hts221::initialize(bool activeMode, ODR odr)
{
    if(!verifyWhoAmI())
    {
        return false;
    }

    if(!loadTempCalibrationRegs())
    {
        return false;
    }

    if(!loadAllHumidityCalibrationRegs())
    {
        return false;
    }

    if(!setPowerDownControlMode(true))
    {
        return false;
    }

    if(!setOutputDataRate(ODR::HZ_1))
    {
        return false;
    }

    return true;
}

bool Hts221::setPowerDownControlMode(bool activeMode)
{
    //Read register first
    uint8_t ctrlReg1 = 0xFF;
    if(!_i2c.read(REG_ADDR::CTRL_REG1, ctrlReg1))
    {
        return false;
    }

    if(activeMode)
    {
        ctrlReg1 |= (1 << CTRL_REG1_BITS::PD);
    }
    else
    {
        ctrlReg1 &= ~(1 << CTRL_REG1_BITS::PD);
    }

    //Write new register values
    if(!_i2c.write(CTRL_REG1, ctrlReg1))
    {
        return false;
    }

    return true;
}

bool Hts221::setBlockDataUpdate(bool continuousMode)
{
    //Read register first
    uint8_t ctrlReg1 = 0xFF;
    if(!_i2c.read(REG_ADDR::CTRL_REG1, ctrlReg1))
    {
        return false;
    }

    if(continuousMode)
    {
        ctrlReg1 &= ~(1 << CTRL_REG1_BITS::BDU);
    }
    else
    {
        ctrlReg1 |= (1 << CTRL_REG1_BITS::BDU);
    }

    //Write new register values
    if(!_i2c.write(CTRL_REG1, ctrlReg1))
    {
        return false;
    }

    return true;
}

bool Hts221::setOutputDataRate(ODR odr)
{
    //Read register first
    uint8_t ctrlReg1 = 0xFF;
    if(!_i2c.read(REG_ADDR::CTRL_REG1, ctrlReg1))
    {
        return false;
    }

    switch(odr)
    {
        case ODR::ONE_SHOT:
            ctrlReg1 &= ~(1 << CTRL_REG1_BITS::ODR0);
            ctrlReg1 &= ~(1 << CTRL_REG1_BITS::ODR1);
            break;

        case ODR::HZ_1:
            ctrlReg1 |=  (1 << CTRL_REG1_BITS::ODR0);
            ctrlReg1 &= ~(1 << CTRL_REG1_BITS::ODR1);
            break;

        case ODR::HZ_7:
            ctrlReg1 &= ~(1 << CTRL_REG1_BITS::ODR0);
            ctrlReg1 |=  (1 << CTRL_REG1_BITS::ODR1);
            break;

        case ODR::HZ_12_5:
            ctrlReg1 |=  (1 << CTRL_REG1_BITS::ODR0);
            ctrlReg1 |=  (1 << CTRL_REG1_BITS::ODR1);
            break;
    }

    //Write new register values
    if(!_i2c.write(CTRL_REG1, ctrlReg1))
    {
        return false;
    }

    return true;
}


bool Hts221::verifyWhoAmI()
{
    uint8_t regVal = 0x00;
    if(!_i2c.read(REG_ADDR::WHO_AM_I, regVal))
    {
        return false;
    }

    if(regVal != WHO_AM_I_VAL)
    {
        return false;
    }

    return true;
}

bool Hts221::readTempRegs(float & degC)
{
    uint8_t tempOutL = 0x00;
    uint8_t tempOutH = 0x00;

    if(!_i2c.read(REG_ADDR::TEMP_OUT_H, tempOutH))
    {
        return false;
    }

    if(!_i2c.read(REG_ADDR::TEMP_OUT_L, tempOutL))
    {
        return false;
    }

    int16_t T_OUT = ((tempOutH << 8) | tempOutL);

    degC = (float)_T0_DEGC_X8 / 8.0 + (T_OUT - _T0_out) * (_T1_DEGC_X8 - _T0_DEGC_X8) / 8.0 / (_T1_out - _T0_out);
    return true;
}

bool Hts221::readHumidityRegs(float & humidityPcnt)
{
    uint8_t humidityOutL = 0x00;
    uint8_t humidityOutH = 0x00;

    if(!_i2c.read(REG_ADDR::H_OUT_H, humidityOutH))
    {
        return false;
    }


    if(!_i2c.read(REG_ADDR::H_OUT_L, humidityOutL))
    {
        return false;
    }


    int16_t H_OUT = ((humidityOutH << 8) | humidityOutL);

    // if(H_OUT > 32768)
    // {
    //     H_OUT -= 65536;
    // }

    humidityPcnt = (float)_H0_RH_x2 / 2.0 + (H_OUT - _H0_T0_OUT) * (_H1_RH_x2 - _H0_RH_x2) / 2.0 / (_H1_T0_OUT - _H0_T0_OUT);
    return true;
}

bool Hts221::loadAllHumidityCalibrationRegs()
{
    if(!_i2c.read(REG_ADDR::H0_RH_x2, _H0_RH_x2))
    {
        return false;
    }

    if(!_i2c.read(REG_ADDR::H1_RH_x2, _H1_RH_x2))
    {
        return false;
    }

    uint8_t h0t0outlsb = 0x00;
    if(!_i2c.read(REG_ADDR::H0_T0_OUT_LSB, h0t0outlsb))
    {
        return false;
    }

    uint8_t h0t0outmsb = 0x00;
    if(!_i2c.read(REG_ADDR::H0_T0_OUT_MSB, h0t0outmsb))
    {
        return false;
    }
    _H0_T0_OUT = (h0t0outmsb << 8) | (h0t0outlsb);

    uint8_t h1t0outlsb = 0x00;
    if(!_i2c.read(REG_ADDR::H1_T0_OUT_LSB, h1t0outlsb))
    {
        return false;
    }

    uint8_t h1t0outmsb = 0x00;
    if(!_i2c.read(REG_ADDR::H1_T0_OUT_MSB, h1t0outmsb))
    {
        return false;
    }
    _H1_T0_OUT = (h1t0outmsb << 8) | (h1t0outlsb);

    return true;
}

bool Hts221::loadTempCalibrationRegs()
{
    //Load first half of temperature calibration regs
    {
        uint8_t t0degc  = 0x00;
        uint8_t t1degc  = 0x00;
        uint8_t t0t1msb = 0x00;

        if(!_i2c.read(REG_ADDR::T0_DEGC_X8, t0degc))
        {
            return false;
        }

        if(!_i2c.read(REG_ADDR::T1_DEGC_X8, t1degc))
        {
            return false;
        }

        if(!_i2c.read(REG_ADDR::T1_T0_msb, t0t1msb))
        {
            return false;
        }

        _T0_DEGC_X8 = (((0b0011 & t0t1msb) >> 0) << 8) | t0degc;
        _T1_DEGC_X8 = (((0b1100 & t0t1msb) >> 2) << 8) | t1degc;
    }

    //load second half of temperature calibration reg
    {
        uint8_t t0outlsb = 0x00;
        uint8_t t0outmsb = 0x00;
        uint8_t t1outlsb = 0x00;
        uint8_t t1outmsb = 0x00;

        if(!_i2c.read(REG_ADDR::T0_OUT_LSB, t0outlsb))
        {
            return false;
        }
        if(!_i2c.read(REG_ADDR::T0_OUT_MSB, t0outmsb))
        {
            return false;
        }
        if(!_i2c.read(REG_ADDR::T1_OUT_LSB, t1outlsb))
        {
            return false;
        }
        if(!_i2c.read(REG_ADDR::T1_OUT_MSB, t1outmsb))
        {
            return false;
        }

        _T0_out = (t0outmsb << 8) | t0outlsb;
        _T1_out = (t1outmsb << 8) | t1outlsb;

        return true;
    }
}

}
}
