#ifndef LIBGEMINIDRIVERS_LIBHUMIDITY_HTS221_H
#define LIBGEMINIDRIVERS_LIBHUMIDITY_HTS221_H

#include "I2cInterface.h"

namespace gemini {
namespace humidity {

/**
 * @brief This class represents a Hts221 temp/humidity sensor
 * @details To use this class, you must pass in an I2C driver conforming to
 *          the provided I2CInterface class.
 *
 *          Basic Usage Example:
 *          yourPlatformI2cDriver::i2cDrier i2c("/dev/i2c-1", HTS221_I2C_ADDR);
 *          adapters::I2cAdapter i2cAdapter(i2c); //conforms to I2cInterface
 *          Hts221 hts221(i2cAdapter);
 *          hts221.initialize();
 *          float degC = hts221.getTempDegC()
 */
class Hts221
{
public:
    static constexpr uint8_t HTS221_I2C_ADDR = 0x5F; // I2C address of the IC

    /**
     * Output Data Rate Options
     */
    enum class ODR
    {
        ONE_SHOT,
        HZ_1,
        HZ_7,
        HZ_12_5
    };

    /**
     * @brief Constructor - Creates an instance of Hts221 driver
     * @param i2c An i2c driver that conforms to the I2cInterface
     */
    explicit Hts221(interfaces::I2cInterface & i2c);

    /**
     * @brief Must be called first!
     * @details Verifies Hts221 is present, loads temp calibration regs,
     *          loads humidity calibration regs, enabled device, and sets
     *           output data rate (ODR).
     * @param activeMode False for power down mode, true for active mode
     * @param odr [description]
     *
     * @return [description]
     */
    bool initialize(bool activeMode=true, ODR odr=ODR::HZ_1);

    /**
     * @brief Reads temperature registers and compuets temperature
     * @param degC Degrees Celcius is returned
     * @return True if successful, false otherwise
     */
    bool readTempRegs(float & degC);
    float getTempDegC();

    /**
     * @brief Reads humidity registers and computes humidity
     * @param humidityPcnt Humidty percent is returned
     * @return True if succefful, false otherwise
     */
    bool readHumidityRegs(float & humidityPcnt);
    float getHumidtyPcnt();

    /**
     * @brief Enables/Disables the IC
     * @details The IC is in disabled (0) state at startup.
     *          This is called by the init function
     * @param activeMode False for power down mode, true for active mode
     * @return True if succeddful, false otherwise
     */
    bool setPowerDownControlMode(bool activeMode);

    /**
     * @brief Sets the Block Data Update (BDU)
     * @details Defaults to continuous mode.
     * @param continuousMode True to update ouptut regisers continuously, to
     *        ensure upper and lower registers are part of the same sample
     *        set this to false.
     * @return True if successful, false otherwise
     */
    bool setBlockDataUpdate(bool continuousMode);


    /**
     * @brief Sets the Ouptut Data Rate (ODR) for both temp and humidity
     * @param odr Output Data Rate for samplicing frequency
     * @return True if successful, false otherwise
     */
    bool setOutputDataRate(ODR odr);

private:
    /**
     * @brief Disable copy constructor
     */
    Hts221(const Hts221&);

    /**
     * @brief Disable assignment operator
     */
    Hts221& operator=(const Hts221&);

    /**
     * @brief Verifies that the WHO_AM_I register matches the expected value
     * @details This is called and verified in the initailze function
     * @return True if successful, false otherwise
     */
    bool verifyWhoAmI();

    /**
     * @brief Loads all temperature calibration registers into private members
     * @details Loads calibration registers for T0_degC and T1_degC
     *          Loads calibration registers for T0_OUT and T1_OUT
     * @return [description]
     */
    bool loadTempCalibrationRegs();

    /**
     * @brief Loads all humidity calibration registers into private members
     * @details Loads calibration registers for H0_RH_x2 and H1_RH_x2
     *          Loads calibration registers for H0_T0_OUT and H1_T0_OUT
     * @return True if successful, false otherwise
     */
    bool loadAllHumidityCalibrationRegs();

    static const uint8_t WHO_AM_I_VAL = 0xBC;     //expected register value

    /**
     * All register addresses
     */
    enum REG_ADDR
    {
        WHO_AM_I = 0x0F,
        // Control Registers
        CTRL_REG1 = 0x20,
        CTRL_REG2 = 0x21,
        CTRL_REG3 = 0x22,
        // Data Registers
        H_OUT_L = 0x28,
        H_OUT_H = 0x29,
        TEMP_OUT_L = 0x2A,
        TEMP_OUT_H = 0x2B,
        // Calibration Registers
        H0_RH_x2 = 0x30,
        H1_RH_x2 = 0x31,
        T0_DEGC_X8 = 0x32,
        T1_DEGC_X8 = 0x33,
        RESERVED0 = 0x34,
        T1_T0_msb = 0x35,
        H0_T0_OUT_LSB = 0x36,
        H0_T0_OUT_MSB = 0x37,
        RESERVED1 = 0x38,
        RESERVED2 = 0x39,
        H1_T0_OUT_LSB = 0x3A,
        H1_T0_OUT_MSB = 0x3B,
        T0_OUT_LSB = 0x3C,
        T0_OUT_MSB = 0x3D,
        T1_OUT_LSB = 0x3E,
        T1_OUT_MSB = 0x3F,

    };

    /**
     * Contains all bit offsets for CTRL_REG1
     */
    enum CTRL_REG1_BITS
    {
        ODR0 = 0, // output data rate selection
        ODR1 = 1, // output data rate selection
        BDU  = 2, // block data update, continuous or both MSB/LSB read
        PD   = 7  // power down control
    };

    /**
     * Contains all bit offsets for CTRL_REG2
     */
    enum CTRL_REG2_BITS
    {
        ONE_SHOT_EN = 0, // 0 waiting for start of conversion, 1 start
        HEATER = 1,      // 0 disable, 1 enable
        BOOT = 7,
    };

    /**
     * Contains all bit offsets for CTRL_REG3
     */
    enum CTRL_REG3_BITS
    {
        DRDY_EN = 2, // default is diable(0), enable(1) data ready output pin3
        PP_OD = 6,   // Pin 3 drain select, push-pull(0), open-drain(1)
        DRDY_H_L = 7 // Data ready output signal active high(0), active low (1)
    };

    //Calibration Values: Temperature
    uint16_t _T0_DEGC_X8;    //10 bits
    uint16_t _T1_DEGC_X8;    //10 bits
    int16_t _T0_out;
    int16_t _T1_out;

    //Calibaration Values: Humidity
    uint8_t _H0_RH_x2;
    uint8_t _H1_RH_x2;
    int16_t _H0_T0_OUT;
    int16_t _H1_T0_OUT;

    interfaces::I2cInterface & _i2c;
};

}
}

#endif