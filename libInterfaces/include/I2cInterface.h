#ifndef LIBGEMINIDRIVERS_LIBINTERFACES_I2CINTERFACE_H
#define LIBGEMINIDRIVERS_LIBINTERFACES_I2CINTERFACE_H

#include <stdint.h>
#include <string>

namespace gemini {
namespace interfaces {

/**
 * @brief Pure virtual class to provide an interface for I2C
 */
class I2cInterface
{
public:
    /**
     * @brief Pure virtual function to provide a read template
     * @param Addr I2C address to read from
     * @param Value Value read from addr, pending a successful read
     * @return True if read was successful, false otherwise
     */
    virtual bool read(uint8_t addr, uint8_t & value) = 0;

    /**
     * @brief Pure virtual function to provide a write template
     * @param Addr I2C address to write to
     * @param Value Value written to addr, pending successful write
     * @return True if read was successful, false otherwise
     */
    virtual bool write(uint8_t addr, uint8_t value) = 0;
};

}
}

#endif
