/**
 *   @file     PCAL6524.cpp
 *   @author   Mingyu Kim
 * 
 *  This library is based on jrj1259a's PCAL6524 library
 *  check it out on https://github.com/jrj12591/PCAL6524
 *  Most function names are changeed so two libraries are not compatible
 * 
 *	This is a driver for NXP's PCAL6254 I2C GPIO
 *	This chip has 24 Individual GPIO that can be independently set as Input or Ouput
 *  Datasheet : https://www.nxp.com/docs/en/data-sheet/PCAL6524.pdf
 *  Define DEBUG_PCAL to enable debug outputs 
 * 
 *  Version 1.0.0 - 04/FEB/2021 All the functions including interrupts are now functional
 *
 */

// Define DEBUG_PCAL to print debug outputs
//#define DEBUG_PCAL
#define PCAL_DEBUG_STREAM Serial

#include "Arduino.h"
#include "PCAL6254.h"

/**
 * @brief prints error value and returns true on error
 * @param error pcal_data_t or pcal_err_t
 * @return returns true on error
 */
bool PCAL6524::onError(pcal_data_t error)
{
    return onError(error.error);
}

/**
 * @brief prints error value and returns true on error
 * @param error pcal_data_t or pcal_err_t
 * @return returns true on error
 */
bool PCAL6524::onError(pcal_err_t error)
{
    switch (error) {
    case PCAL_ERROR_OK:
        return false;
    case PCAL_ERROR_I2C_DEV:
        PCAL_PRINT("PCAL_ERROR_I2C_DEV");
        return true;
    case PCAL_ERROR_I2C_ACK:
        PCAL_PRINT("PCAL_ERROR_I2C_ACK");
        return true;
    case PCAL_ERROR_I2C_TIMEOUT:
        PCAL_PRINT("PCAL_ERROR_I2C_TIMEOUT");
        return true;
    case PCAL_ERROR_I2C_BUS:
        PCAL_PRINT("PCAL_ERROR_I2C_BUS");
        return true;
    case PCAL_ERROR_I2C_BUSY:
        PCAL_PRINT("PCAL_ERROR_I2C_BUSY");
        return true;
    case PCAL_ERROR_I2C_MEMORY:
        PCAL_PRINT("PCAL_ERROR_I2C_MEMORY");
        return true;
    case PCAL_ERROR_I2C_CONTINUE:
        PCAL_PRINT("PCAL_ERROR_I2C_CONTINUE");
        return true;
    case PCAL_ERROR_I2C_NO_BEGIN:
        PCAL_PRINT("PCAL_ERROR_I2C_NO_BEGIN");
        return true;
    case PCAL_ERROR_I2C_DATA_MISMATCH:
        PCAL_PRINT("PCAL_ERROR_I2C_DATA_MISMATCH");
        return true;
    case PCAL_ERROR_OUT_OF_RANGE:
        PCAL_PRINT("PCAL_ERROR_OUT_OF_RANGE");
        return true;
    case PCAL_ERROR_UNKNOWN:
        PCAL_PRINT("PCAL_ERROR_UNKNOWN");
        return true;
    default:
        PCAL_PRINT("PCAL_ERROR_UNKNOWN");
        return true;
    }
    return true;
}

/**
 * @brief  Writes 8-bits to the specified destination register
 * @param i2cAddress address to send data to
 * @param reg register to send data to
 * @param value data to send
 * Do onError after
 */
pcal_err_t PCAL6524::writeRegister(pcal_address_t reg, pcal_data_t value)
{
    m_wire->beginTransmission(m_i2cAddress);
    m_wire->write(reg);
    for (uint8_t i = 0; i < value.size; i++) {
        m_wire->write(value.value[i]);
        PCAL_PRINT_BASE_TWO("Transmitting", value.value[i], HEX);
    }
    return (pcal_err_t)m_wire->endTransmission();
}

/**
 * @brief  Reads 8-bits from the specified destination register
 * @param i2cAddress address to send data to
 * @param reg register to send data to
 * @param size number of bytes to request
 * @return pcal_data_t data read
 */
PCAL6524::pcal_data_t PCAL6524::readRegister(pcal_address_t reg, uint8_t size)
{
    pcal_data_t returnData;
    m_wire->beginTransmission(m_i2cAddress);
    m_wire->write(reg);
    pcal_err_t i2cErr = (pcal_err_t)m_wire->endTransmission();
    if (i2cErr != PCAL_ERROR_OK) {
        returnData.error = i2cErr;
        return returnData;
    } else {
        returnData.size = m_wire->requestFrom((uint8_t)m_i2cAddress, (uint8_t)size, (uint8_t)true);
    }
    if (returnData.size != size) {
        m_wire->flush();
        returnData.error = PCAL_ERROR_I2C_DATA_MISMATCH;
        return returnData;
    } else {
        returnData.error = PCAL_ERROR_OK;
        for (uint8_t i = 0; i < size; i++) {
            returnData.value[i] = m_wire->read();
            PCAL_PRINT_BASE_TWO("Received", returnData.value[i], HEX);
        }
    }
    return returnData;
}

/**
 *    @brief  Instantiates a new PCAL6524 class w/appropriate properties
 *    @param i2cAddress address of the PCAL6524
 */
PCAL6524::PCAL6524(pcal_address_t i2cAddress)
{
    m_i2cAddress = i2cAddress;
}

/**
 * @brief  Resets the CHIP To its defualt state clearing all registers using software method
 * Do onError after
 */
pcal_err_t PCAL6524::reset()
{
    pcal_err_t error = PCAL_ERROR_UNKNOWN;
    PCAL_PRINT("Performing software reset");
    m_wire->beginTransmission(0x00);
    m_wire->write((uint8_t)0x06);
    error = (pcal_err_t)m_wire->endTransmission();
    onError(error);
    return error;
}

/**
 *    @brief Begin I2c comm and perform software reset
 *    @param theWire If you want to use some other Wire instance other than the default one
 *    @param sda specify custom SDA pin
 *    @param scl specify custom SCL pin
 *    @param frequency specify custom I2c frequency
 */
pcal_err_t PCAL6524::begin(int sda, int scl, uint32_t frequency, TwoWire* theWire)
{
    pcal_err_t returnError = PCAL_ERROR_UNKNOWN;
    m_sda = sda;
    m_scl = scl;
    m_freq = frequency;
    m_wire = theWire;
    if (m_wire->begin(m_sda, m_scl, m_freq)) {
        return reset();
    } else {
        return PCAL_ERROR_I2C_NO_BEGIN;
    }
    return returnError;
}

/**
 *    @brief Allows for induvidaul Pins to setup as input or output and select if input has Pull Up or pull down internal resistors
 *    @param pin Px[y] x=port y=pin#
 *    @param mode INPUT, OUTPUT, INPUT_PULLDOWN, INPUT_PULLUP
 */
pcal_err_t PCAL6524::pinMode(pcal_pin_t pin, uint8_t mode)
{
    pcal_data_t config_data;
    pcal_data_t pullup_config_data;
    pcal_data_t pullup_value_data;
    pcal_address_t PCAL6524_CONFIGURATION;
    pcal_address_t PCAL6524_RESISTOR_PULL_ENABLE;
    pcal_address_t PCAL6524_RESISTOR_PULL_SELECTION;
    pcal_err_t writeError = PCAL_ERROR_UNKNOWN;

    //Determine wich bank of pins the requested pin is in
    switch (pin.port) {
    case PORT0:
        PCAL6524_CONFIGURATION = PCAL6524_CONFIGURATION_PORT_0;
        PCAL6524_RESISTOR_PULL_ENABLE = PCAL6524_RESISTOR_PULL_ENABLE_PORT_0;
        PCAL6524_RESISTOR_PULL_SELECTION = PCAL6524_RESISTOR_PULL_SELECTION_PORT_0;
        break;
    case PORT1:
        PCAL6524_CONFIGURATION = PCAL6524_CONFIGURATION_PORT_1;
        PCAL6524_RESISTOR_PULL_ENABLE = PCAL6524_RESISTOR_PULL_ENABLE_PORT_1;
        PCAL6524_RESISTOR_PULL_SELECTION = PCAL6524_RESISTOR_PULL_SELECTION_PORT_1;
        break;
    case PORT2:
        PCAL6524_CONFIGURATION = PCAL6524_CONFIGURATION_PORT_2;
        PCAL6524_RESISTOR_PULL_ENABLE = PCAL6524_RESISTOR_PULL_ENABLE_PORT_2;
        PCAL6524_RESISTOR_PULL_SELECTION = PCAL6524_RESISTOR_PULL_SELECTION_PORT_2;
        break;
    default:
        return PCAL_ERROR_OUT_OF_RANGE;
    }

    //read the current Input/output configuration settings for the given bank of pins
    config_data = readRegister(PCAL6524_CONFIGURATION);
    if (onError(config_data)) {
        return config_data.error;
    }
    pullup_config_data = readRegister(PCAL6524_RESISTOR_PULL_ENABLE);
    if (onError(pullup_config_data)) {
        return pullup_config_data.error;
    }
    pullup_value_data = readRegister(PCAL6524_RESISTOR_PULL_SELECTION);
    if (onError(pullup_value_data)) {
        return pullup_value_data.error;
    }

    //Combine the current configuration with the request pin to ensure that only that pin has changed
    switch (mode) {
    case INPUT:
        config_data.value[0] = config_data.value[0] | pin.pinMask;
        pullup_config_data.value[0] = pullup_config_data.value[0] & ~(pin.pinMask);
        pullup_value_data.value[0] = pullup_value_data.value[0] | pin.pinMask;
        break;
    case OUTPUT:
        config_data.value[0] = config_data.value[0] & ~(pin.pinMask);
        pullup_config_data.value[0] = pullup_config_data.value[0] & ~(pin.pinMask);
        pullup_value_data.value[0] = pullup_value_data.value[0] | pin.pinMask;
        break;
    case INPUT_PULLUP:
        config_data.value[0] = config_data.value[0] | pin.pinMask;
        pullup_config_data.value[0] = pullup_config_data.value[0] | pin.pinMask;
        pullup_value_data.value[0] = pullup_value_data.value[0] | pin.pinMask;
        break;
    case INPUT_PULLDOWN:
        config_data.value[0] = config_data.value[0] | pin.pinMask;
        pullup_config_data.value[0] = pullup_config_data.value[0] | pin.pinMask;
        pullup_value_data.value[0] = pullup_value_data.value[0] & ~(pin.pinMask);
        break;
    default:
        return PCAL_ERROR_OUT_OF_RANGE;
    }

    //Write the new configuration back to the register
    writeError = writeRegister(PCAL6524_CONFIGURATION, config_data);
    if (onError(writeError))
        return writeError;
    writeError = writeRegister(PCAL6524_RESISTOR_PULL_ENABLE, pullup_config_data);
    if (onError(writeError))
        return writeError;
    writeError = writeRegister(PCAL6524_RESISTOR_PULL_SELECTION, pullup_value_data);
    if (onError(writeError))
        return writeError;

    return writeError;
}

/**
 *    @brief Allows for induvidaul Pins to have inversions or latch function
 *    @param pin Px[y] x=port y=pin#
 *    @param inverted if true return value of the pin will be
 *    @param latched if true that pin will be latched until read
 */
pcal_err_t PCAL6524::inputPinMode(pcal_pin_t pin, bool inverted, bool latched)
{
    pcal_data_t inversion_config_data;
    pcal_data_t latch_config_data;
    pcal_address_t PCAL6524_INPUT_LATCH;
    pcal_address_t PCAL6524_POLARITY_INVERSION;
    pcal_err_t writeError = PCAL_ERROR_UNKNOWN;

    //Determine wich bank of pins the requested pin is in
    switch (pin.port) {
    case PORT0:
        PCAL6524_INPUT_LATCH = PCAL6524_INPUT_LATCH_PORT_0;
        PCAL6524_POLARITY_INVERSION = PCAL6524_POLARITY_INVERSION_PORT_0;
        break;
    case PORT1:
        PCAL6524_INPUT_LATCH = PCAL6524_INPUT_LATCH_PORT_1;
        PCAL6524_POLARITY_INVERSION = PCAL6524_POLARITY_INVERSION_PORT_1;
        break;
    case PORT2:
        PCAL6524_INPUT_LATCH = PCAL6524_INPUT_LATCH_PORT_2;
        PCAL6524_POLARITY_INVERSION = PCAL6524_POLARITY_INVERSION_PORT_2;
        break;
    default:
        return PCAL_ERROR_OUT_OF_RANGE;
    }

    //read the current configuration settings for the given bank of pins
    inversion_config_data = readRegister(PCAL6524_POLARITY_INVERSION);
    if (onError(inversion_config_data))
        return inversion_config_data.error;
    latch_config_data = readRegister(PCAL6524_INPUT_LATCH);
    if (onError(latch_config_data))
        return latch_config_data.error;

    //Combine the current configuration with the request pin to ensure that only that pin has changed
    if (inverted) {
        inversion_config_data.value[0] = inversion_config_data.value[0] | pin.pinMask;
    } else {
        inversion_config_data.value[0] = inversion_config_data.value[0] & ~(pin.pinMask);
    }
    if (latched) {
        latch_config_data.value[0] = latch_config_data.value[0] | pin.pinMask;
    } else {
        latch_config_data.value[0] = latch_config_data.value[0] & ~(pin.pinMask);
    }
    writeError = writeRegister(PCAL6524_POLARITY_INVERSION, inversion_config_data);
    if (onError(writeError))
        return writeError;
    writeError = writeRegister(PCAL6524_INPUT_LATCH, latch_config_data);
    if (onError(writeError))
        return writeError;

    return writeError;
}

/**
 *    @brief Allows for induvidaul Pins to have strength or pushpull/opendrain function
 *    @param pin Px[y] x=port y=pin#
 *    @param strength strength of the pin (PCAL6524_STRONGEST/STRONG/WEAK/WEAKEST)
 *    @param isOpenDrain if true that pin will be in open drain mode
 */
pcal_err_t PCAL6524::outputPinMode(pcal_pin_t pin, pcal_strength_t strength, bool isOpenDrain)
{
    pcal_data_t strength_config_data;
    pcal_data_t output_config_data;
    pcal_data_t output_port_data;
    pcal_address_t PCAL6524_OUTPUT_STRENGTH;
    pcal_address_t PCAL6524_PIN_OUTPUT_CONFIG;
    pcal_err_t writeError = PCAL_ERROR_UNKNOWN;

    //Determine wich bank of pins the requested pin is in
    switch (pin.port) {
    case PORT0:
        if (isUpperPin(pin)) //If pin is upper 4 pins (4,5,6,7)
        {
            PCAL6524_OUTPUT_STRENGTH = PCAL6524_OUTPUT_STRENGTH_0B;
        } else {
            PCAL6524_OUTPUT_STRENGTH = PCAL6524_OUTPUT_STRENGTH_0A;
        }
        PCAL6524_PIN_OUTPUT_CONFIG = PCAL6524_PIN_OUTPUT_CONFIG_PORT_0;
        break;
    case PORT1:
        if (isUpperPin(pin)) //If pin is upper 4 pins (4,5,6,7)
        {
            PCAL6524_OUTPUT_STRENGTH = PCAL6524_OUTPUT_STRENGTH_1B;
        } else {
            PCAL6524_OUTPUT_STRENGTH = PCAL6524_OUTPUT_STRENGTH_1A;
        }
        PCAL6524_PIN_OUTPUT_CONFIG = PCAL6524_PIN_OUTPUT_CONFIG_PORT_1;
        break;
    case PORT2:
        if (isUpperPin(pin)) //If pin is upper 4 pins (4,5,6,7)
        {
            PCAL6524_OUTPUT_STRENGTH = PCAL6524_OUTPUT_STRENGTH_2B;
        } else {
            PCAL6524_OUTPUT_STRENGTH = PCAL6524_OUTPUT_STRENGTH_2A;
        }
        PCAL6524_PIN_OUTPUT_CONFIG = PCAL6524_PIN_OUTPUT_CONFIG_PORT_2;
        break;
    default:
        onError(PCAL_ERROR_OUT_OF_RANGE);
        return PCAL_ERROR_OUT_OF_RANGE;
    }

    //read the current configuration settings for the given bank of pins
    strength_config_data = readRegister(PCAL6524_OUTPUT_STRENGTH);
    if (onError(strength_config_data))
        return strength_config_data.error;
    output_config_data = readRegister(PCAL6524_PIN_OUTPUT_CONFIG);
    if (onError(output_config_data))
        return output_config_data.error;
    output_port_data = readRegister(PCAL6524_OUTPUT_PORT_CONFIG);
    if (onError(output_port_data))
        return output_port_data.error;

    //Combine the current configuration with the request pin to ensure that only that pin has changed
    switch (pin.pinMask) {
    case ((uint8_t)bit(0)): //For Pins Px_0
        strength = strength << 0;
        strength_config_data.value[0] = strength_config_data.value[0] | strength;
        break;
    case ((uint8_t)bit(1)): //For Pins Px_1
        strength = strength << 2;
        strength_config_data.value[0] = strength_config_data.value[0] | strength;
        break;
    case ((uint8_t)bit(2)): //For Pins Px_2
        strength = strength << 4;
        strength_config_data.value[0] = strength_config_data.value[0] | strength;
        break;
    case ((uint8_t)bit(3)): //For Pins Px_3
        strength = strength << 6;
        strength_config_data.value[0] = strength_config_data.value[0] | strength;
        break;
    case ((uint8_t)bit(4)): //For Pins Px_4
        strength = strength << 0;
        strength_config_data.value[0] = strength_config_data.value[0] | strength;
        break;
    case ((uint8_t)bit(5)): //For Pins Px_5
        strength = strength << 2;
        strength_config_data.value[0] = strength_config_data.value[0] | strength;
        break;
    case ((uint8_t)bit(6)): //For Pins Px_6
        strength = strength << 4;
        strength_config_data.value[0] = strength_config_data.value[0] | strength;
        break;
    case ((uint8_t)bit(7)): //For Pins Px_7
        strength = strength << 6;
        strength_config_data.value[0] = strength_config_data.value[0] | strength;
        break;
    }
    if (isOpenDrain) {
        output_config_data.value[0] = output_config_data.value[0] | pin.pinMask;
    } else {
        output_config_data.value[0] = output_config_data.value[0] & ~(pin.pinMask);
    }
    output_port_data.value[0] = output_port_data.value[0] & ~(0x07);

    writeError = writeRegister(PCAL6524_OUTPUT_STRENGTH, strength_config_data);
    if (onError(writeError))
        return writeError;
    writeError = writeRegister(PCAL6524_OUTPUT_PORT_CONFIG, output_port_data);
    if (onError(writeError))
        return writeError;
    writeError = writeRegister(PCAL6524_PIN_OUTPUT_CONFIG, output_config_data);
    if (onError(writeError))
        return writeError;

    return writeError;
}

/**
 *    @brief Allows for whole port to have inversions or latch function
 *    @param port Px x=port
 *    @param inverted if true return value of the pin will be
 *    @param latched if true that pin will be latched until read
 */
pcal_err_t PCAL6524::inputPortMode(pcal_port_t port, bool inverted, bool latched)
{
    pcal_data_t inversion_config_data;
    pcal_data_t latch_config_data;
    pcal_address_t PCAL6524_INPUT_LATCH;
    pcal_address_t PCAL6524_POLARITY_INVERSION;
    pcal_err_t writeError = PCAL_ERROR_UNKNOWN;

    //Determine wich bank of pins the requested pin is in
    switch (port) {
    case PORT0:
        PCAL6524_INPUT_LATCH = PCAL6524_INPUT_LATCH_PORT_0;
        PCAL6524_POLARITY_INVERSION = PCAL6524_POLARITY_INVERSION_PORT_0;
        break;
    case PORT1:
        PCAL6524_INPUT_LATCH = PCAL6524_INPUT_LATCH_PORT_1;
        PCAL6524_POLARITY_INVERSION = PCAL6524_POLARITY_INVERSION_PORT_1;
        break;
    case PORT2:
        PCAL6524_INPUT_LATCH = PCAL6524_INPUT_LATCH_PORT_2;
        PCAL6524_POLARITY_INVERSION = PCAL6524_POLARITY_INVERSION_PORT_2;
        break;
    default:
        onError(PCAL_ERROR_OUT_OF_RANGE);
        return PCAL_ERROR_OUT_OF_RANGE;
    }

    //Combine the current configuration with the request pin to ensure that only that pin has changed
    if (inverted) {
        inversion_config_data.value[0] = 0xFF;
    } else {
        inversion_config_data.value[0] = 0x00;
    }
    if (latched) {
        latch_config_data.value[0] = 0xFF;
    } else {
        latch_config_data.value[0] = 0x00;
    }
    writeError = writeRegister(PCAL6524_INPUT_LATCH, latch_config_data);
    if (onError(writeError))
        return writeError;
    writeError = writeRegister(PCAL6524_POLARITY_INVERSION, inversion_config_data);
    if (onError(writeError))
        return writeError;

    return writeError;
}

/**
 *    @brief Allows for whole port to have strength or pushpull/opendrain function
 *    @param port Px x=port
 *    @param strength strength of the pin (PCAL6524_STRONGEST/STRONG/WEAK/WEAKEST)
 *    @param opendrain if true that pin will be in open drain mode
 */
pcal_err_t PCAL6524::outputPortMode(pcal_port_t port, pcal_strength_t strength, bool isOpenDrain)
{
    pcal_data_t strength_config_data;
    pcal_data_t output_config_data;
    pcal_data_t output_port_data;
    pcal_address_t PCAL6524_OUTPUT_STRENGTH;
    pcal_address_t PCAL6524_PIN_OUTPUT_CONFIG;
    pcal_err_t writeError = PCAL_ERROR_UNKNOWN;

    //Determine wich bank of pins the requested pin is in
    switch (port) {
    case PORT0:
        PCAL6524_OUTPUT_STRENGTH = PCAL6524_OUTPUT_STRENGTH_0A;
        PCAL6524_PIN_OUTPUT_CONFIG = PCAL6524_PIN_OUTPUT_CONFIG_PORT_0;
        break;
    case PORT1:
        PCAL6524_OUTPUT_STRENGTH = PCAL6524_OUTPUT_STRENGTH_1A;
        PCAL6524_PIN_OUTPUT_CONFIG = PCAL6524_PIN_OUTPUT_CONFIG_PORT_1;
        break;
    case PORT2:
        PCAL6524_OUTPUT_STRENGTH = PCAL6524_OUTPUT_STRENGTH_2A;
        PCAL6524_PIN_OUTPUT_CONFIG = PCAL6524_PIN_OUTPUT_CONFIG_PORT_2;
        break;
    default:
        onError(PCAL_ERROR_OUT_OF_RANGE);
        return PCAL_ERROR_OUT_OF_RANGE;
    }

    //read the current configuration settings for the given bank of pins
    output_config_data = readRegister(PCAL6524_OUTPUT_PORT_CONFIG);
    if (onError(output_config_data))
        return output_config_data.error;

    //Combine the current configuration with the request pin to ensure that only that pin has changed
    strength_config_data.value[0] = (strength_config_data.value[0] << 2) + strength;
    strength_config_data.value[0] = (strength_config_data.value[0] << 2) + strength;
    strength_config_data.value[0] = (strength_config_data.value[0] << 2) + strength;
    strength_config_data.value[1] = strength_config_data.value[0];
    strength_config_data.size = 2;
    if (isOpenDrain) {
        output_config_data.value[0] = 0xFF;
    } else {
        output_config_data.value[0] = 0x00;
    };
    output_port_data.value[0] = output_port_data.value[0] & ~(0x07);

    //Write the new configuration back to the register
    writeError = writeRegister(PCAL6524_OUTPUT_PORT_CONFIG, output_port_data);
    if (onError(writeError))
        return writeError;
    writeError = writeRegister(PCAL6524_OUTPUT_STRENGTH, strength_config_data);
    if (onError(writeError))
        return writeError;
    writeError = writeRegister(PCAL6524_PIN_OUTPUT_CONFIG, output_config_data);
    if (onError(writeError))
        return writeError;

    return writeError;
}

/**
 *    @brief Allows for induvidaul Pins to setup as input or output and select if input has Pull Up or pull down internal resistors
 *    @param port Px x=port
 *    @param mode INPUT, OUTPUT, INPUT_PULLDOWN, INPUT_PULLUP
 */
pcal_err_t PCAL6524::portMode(pcal_port_t port, uint8_t mode)
{
    pcal_data_t config_data;
    pcal_data_t pullup_config_data;
    pcal_data_t pullup_value_data;
    pcal_address_t PCAL6524_CONFIGURATION;
    pcal_address_t PCAL6524_RESISTOR_PULL_ENABLE;
    pcal_address_t PCAL6524_RESISTOR_PULL_SELECTION;
    pcal_err_t writeError = PCAL_ERROR_UNKNOWN;

    //Determine wich bank of pins the requested pin is in
    switch (port) {
    case PORT0:
        PCAL6524_CONFIGURATION = PCAL6524_CONFIGURATION_PORT_0;
        PCAL6524_RESISTOR_PULL_ENABLE = PCAL6524_RESISTOR_PULL_ENABLE_PORT_0;
        PCAL6524_RESISTOR_PULL_SELECTION = PCAL6524_RESISTOR_PULL_SELECTION_PORT_0;
        break;
    case PORT1:
        PCAL6524_CONFIGURATION = PCAL6524_CONFIGURATION_PORT_1;
        PCAL6524_RESISTOR_PULL_ENABLE = PCAL6524_RESISTOR_PULL_ENABLE_PORT_1;
        PCAL6524_RESISTOR_PULL_SELECTION = PCAL6524_RESISTOR_PULL_SELECTION_PORT_1;
        break;
    case PORT2:
        PCAL6524_CONFIGURATION = PCAL6524_CONFIGURATION_PORT_2;
        PCAL6524_RESISTOR_PULL_ENABLE = PCAL6524_RESISTOR_PULL_ENABLE_PORT_2;
        PCAL6524_RESISTOR_PULL_SELECTION = PCAL6524_RESISTOR_PULL_SELECTION_PORT_2;
        break;
    default:
        onError(PCAL_ERROR_OUT_OF_RANGE);
        return PCAL_ERROR_OUT_OF_RANGE;
    }

    //Combine the current configuration with the request pin to ensure that only that pin has changed
    switch (mode) {
    case INPUT:
        config_data.value[0] = 0xFF;
        pullup_config_data.value[0] = 0x00;
        pullup_value_data.value[0] = 0xFF;
        break;
    case OUTPUT:
        config_data.value[0] = 0x00;
        pullup_config_data.value[0] = 0x00;
        pullup_value_data.value[0] = 0xFF;
        break;
    case INPUT_PULLUP:
        config_data.value[0] = 0xFF;
        pullup_config_data.value[0] = 0xFF;
        pullup_value_data.value[0] = 0xFF;
        break;
    case INPUT_PULLDOWN:
        config_data.value[0] = 0xFF;
        pullup_config_data.value[0] = 0xFF;
        pullup_value_data.value[0] = 0x00;
        break;
    default:
        return PCAL_ERROR_OUT_OF_RANGE;
    }

    //Write the new configuration back to the register
    writeError = writeRegister(PCAL6524_CONFIGURATION, config_data);
    if (onError(writeError))
        return writeError;
    writeError = writeRegister(PCAL6524_RESISTOR_PULL_ENABLE, pullup_config_data);
    if (onError(writeError))
        return writeError;
    writeError = writeRegister(PCAL6524_RESISTOR_PULL_SELECTION, pullup_value_data);
    if (onError(writeError))
        return writeError;

    return writeError;
}

/**
 *    @brief en/disables interrupts on certain pin
 *    @param pin Px_y x=port y=pin#
 *    @param type RISING, FALLING, CHANGE, DISABLED
 */
pcal_err_t PCAL6524::pinInterrupt(pcal_pin_t pin, uint8_t type)
{
    pcal_data_t mask_config_data;
    pcal_data_t type_config_data;
    pcal_address_t PCAL6524_INTERRUPT_EDGE;
    pcal_address_t PCAL6524_INTERRUPT_MASK;
    pcal_err_t writeError = PCAL_ERROR_UNKNOWN;

    //Determine wich bank of pins the requested pin is in
    switch (pin.port) {
    case PORT0:
        if (isUpperPin(pin)) //If pin is upper 4 pins (4,5,6,7)
        {
            PCAL6524_INTERRUPT_EDGE = PCAL6524_INTERRUPT_EDGE_PORT_0B;
        } else {
            PCAL6524_INTERRUPT_EDGE = PCAL6524_INTERRUPT_EDGE_PORT_0A;
        }
        PCAL6524_INTERRUPT_MASK = PCAL6524_INTERRUPT_MASK_PORT_0;
        break;
    case PORT1:
        if (isUpperPin(pin)) //If pin is upper 4 pins (4,5,6,7)
        {
            PCAL6524_INTERRUPT_EDGE = PCAL6524_INTERRUPT_EDGE_PORT_1B;
        } else {
            PCAL6524_INTERRUPT_EDGE = PCAL6524_INTERRUPT_EDGE_PORT_1A;
        }
        PCAL6524_INTERRUPT_MASK = PCAL6524_INTERRUPT_MASK_PORT_1;
        break;
    case PORT2:
        if (isUpperPin(pin)) //If pin is upper 4 pins (4,5,6,7)
        {
            PCAL6524_INTERRUPT_EDGE = PCAL6524_INTERRUPT_EDGE_PORT_2B;
        } else {
            PCAL6524_INTERRUPT_EDGE = PCAL6524_INTERRUPT_EDGE_PORT_2A;
        }
        PCAL6524_INTERRUPT_MASK = PCAL6524_INTERRUPT_MASK_PORT_2;
        break;
    default:
        onError(PCAL_ERROR_OUT_OF_RANGE);
        return PCAL_ERROR_OUT_OF_RANGE;
    }

    //read the current interrupt configuration settings for the given bank of pins
    mask_config_data = readRegister(PCAL6524_INTERRUPT_MASK);
    if (onError(mask_config_data))
        return mask_config_data.error;
    type_config_data = readRegister(PCAL6524_INTERRUPT_EDGE);
    if (onError(type_config_data))
        return type_config_data.error;

    uint8_t intType;
    switch (type) {
    case RISING:
        intType = 0b01010101;
        break;
    case FALLING:
        intType = 0b10101010;
        break;
    case CHANGE:
        intType = 0b11111111;
        break;
    case DISABLED:
        intType = 0x00;
        break;
    default:
        onError(PCAL_ERROR_OUT_OF_RANGE);
        return PCAL_ERROR_OUT_OF_RANGE;
    }
    // Set interrupt Type register
    switch (pin.pinMask) {
    case ((uint8_t)bit(0)): //For Pins Px_0
        intType = intType << 0;
        type_config_data.value[0] = type_config_data.value[0] | intType;
        break;
    case ((uint8_t)bit(1)): //For Pins Px_1
        intType = intType << 2;
        type_config_data.value[0] = type_config_data.value[0] | intType;
        break;
    case ((uint8_t)bit(2)): //For Pins Px_2
        intType = intType << 4;
        type_config_data.value[0] = type_config_data.value[0] | intType;
        break;
    case ((uint8_t)bit(3)): //For Pins Px_3
        intType = intType << 6;
        type_config_data.value[0] = type_config_data.value[0] | intType;
        break;
    case ((uint8_t)bit(4)): //For Pins Px_4
        intType = intType << 0;
        type_config_data.value[0] = type_config_data.value[0] | intType;
        break;
    case ((uint8_t)bit(5)): //For Pins Px_5
        intType = intType << 2;
        type_config_data.value[0] = type_config_data.value[0] | intType;
        break;
    case ((uint8_t)bit(6)): //For Pins Px_6
        intType = intType << 4;
        type_config_data.value[0] = type_config_data.value[0] | intType;
        break;
    case ((uint8_t)bit(7)): //For Pins Px_7
        intType = intType << 6;
        type_config_data.value[0] = type_config_data.value[0] | intType;
        break;
    }
    if (type == DISABLED) {
        mask_config_data.value[0] = mask_config_data.value[0] | pin.pinMask;
    } else {
        mask_config_data.value[0] = mask_config_data.value[0] & ~(pin.pinMask);
    }
    // Set Interupt configuration register
    writeError = writeRegister(PCAL6524_INTERRUPT_EDGE, type_config_data);
    if (onError(writeError))
        return writeError;
    writeError = writeRegister(PCAL6524_INTERRUPT_MASK, mask_config_data);
    if (onError(writeError))
        return writeError;

    return writeError;
}

/**
 *    @brief en/disables interrupts on certain port
 *    @param port Px x=port
 *    @param type RISING, FALLING, CHANGE, DISABLED
 */
pcal_err_t PCAL6524::portInterrupt(pcal_port_t port, uint8_t type)
{
    pcal_address_t PCAL6524_INTERRUPT_EDGE_A;
    pcal_address_t PCAL6524_INTERRUPT_EDGE_B;
    pcal_address_t PCAL6524_INTERRUPT_MASK;
    pcal_data_t intType;
    pcal_data_t intMask;
    pcal_err_t writeError = PCAL_ERROR_UNKNOWN;

    switch (type) {
    case RISING:
        intType.value[0] = 0b01010101;
        break;
    case FALLING:
        intType.value[0] = 0b10101010;
        break;
    case CHANGE:
        intType.value[0] = 0b11111111;
        break;
    case DISABLED:
        intType.value[0] = 0x00;
        break;
    default:
        onError(PCAL_ERROR_OUT_OF_RANGE);
        return PCAL_ERROR_OUT_OF_RANGE;
    }

    if (type == DISABLED) {
        intMask.value[0] = 0xFF;
    } else {
        intMask.value[0] = 0x00;
    }

    //Determine wich bank of pins the requested pin is in
    switch (port) {
    case PORT0:
        PCAL6524_INTERRUPT_EDGE_A = PCAL6524_INTERRUPT_EDGE_PORT_0A;
        PCAL6524_INTERRUPT_EDGE_B = PCAL6524_INTERRUPT_EDGE_PORT_0B;
        PCAL6524_INTERRUPT_MASK = PCAL6524_INTERRUPT_MASK_PORT_0;
        break;
    case PORT1:
        PCAL6524_INTERRUPT_EDGE_A = PCAL6524_INTERRUPT_EDGE_PORT_1A;
        PCAL6524_INTERRUPT_EDGE_B = PCAL6524_INTERRUPT_EDGE_PORT_1B;
        PCAL6524_INTERRUPT_MASK = PCAL6524_INTERRUPT_MASK_PORT_1;
        break;
    case PORT2:
        PCAL6524_INTERRUPT_EDGE_A = PCAL6524_INTERRUPT_EDGE_PORT_2A;
        PCAL6524_INTERRUPT_EDGE_B = PCAL6524_INTERRUPT_EDGE_PORT_2B;
        PCAL6524_INTERRUPT_MASK = PCAL6524_INTERRUPT_MASK_PORT_2;
        break;
    default:
        onError(PCAL_ERROR_OUT_OF_RANGE);
        return PCAL_ERROR_OUT_OF_RANGE;
    }
    writeError = writeRegister(PCAL6524_INTERRUPT_EDGE_A, intType);
    if (onError(writeError))
        return writeError;
    writeError = writeRegister(PCAL6524_INTERRUPT_EDGE_B, intType);
    if (onError(writeError))
        return writeError;
    writeError = writeRegister(PCAL6524_INTERRUPT_MASK, intMask);
    if (onError(writeError))
        return writeError;

    return writeError;
}

/**
 *    @brief Handles interrupt on PCAL6524, this just sets the intflag to true. Call this function inside of the ISR of the interrupt.
 */
void IRAM_ATTR PCAL6524::interruptHandler()
{
    intFlag = true;
}

/**
 *    @brief reads interrupt related registers and clears just the flag
 */
pcal_err_t PCAL6524::readInterrupt()
{
    interruptData = readRegister(PCAL6524_INTERRUPT_STATUS_PORT_0, 3);
    if (onError(interruptData))
        return interruptData.error;
    intFlag = false;
    return PCAL_ERROR_OK;
}

/**
 *    @brief  Tells you if that pin has been the source of interrupt
 *    @param pin Px_y x=port y=pin#
 *    @return uint8_t returns true if that pin triggered an interrupt
 */
bool PCAL6524::interruptSourcePin(pcal_pin_t pin)
{
    if (intFlag)
        readInterrupt();
    bool match = false;
    switch (pin.pinMask) {
    case ((uint8_t)bit(0)): //For Pins Px_0
        if (bitRead(interruptData.value[pin.port], 0))
            match = true;
        break;
    case ((uint8_t)bit(1)): //For Pins Px_1
        if (bitRead(interruptData.value[pin.port], 1))
            match = true;
        break;
    case ((uint8_t)bit(2)): //For Pins Px_2
        if (bitRead(interruptData.value[pin.port], 2))
            match = true;
        break;
    case ((uint8_t)bit(3)): //For Pins Px_3
        if (bitRead(interruptData.value[pin.port], 3))
            match = true;
        break;
    case ((uint8_t)bit(4)): //For Pins Px_4
        if (bitRead(interruptData.value[pin.port], 4))
            match = true;
        break;
    case ((uint8_t)bit(5)): //For Pins Px_5
        if (bitRead(interruptData.value[pin.port], 5))
            match = true;
        break;
    case ((uint8_t)bit(6)): //For Pins Px_6
        if (bitRead(interruptData.value[pin.port], 6))
            match = true;
        break;
    case ((uint8_t)bit(7)): //For Pins Px_7
        if (bitRead(interruptData.value[pin.port], 7))
            match = true;
        break;
    }
    return match;
}

/**
 *    @brief  Tells you if that port has been the source of interrupt
 *    @param port Px x=port
 *    @return uint8_t returns values of that port
 */
PCAL6524::pcal_data_t PCAL6524::interruptSource()
{
    if (intFlag)
        readInterrupt();
    return interruptData;
}

/**
 *    @brief  Reads pin value from specific pin, this will clear all latch, interrupt from that pin's port
 *    @param pin Px_y x=port y=pin#
 *    @return value of that pin
 */
uint8_t PCAL6524::digitalRead(pcal_pin_t pin)
{
    pcal_data_t input_reg_data;
    pcal_address_t PCAL6524_INPUT;

    //Determine wich bank of pins the requested pin is in
    switch (pin.port) {
    case PORT0:
        PCAL6524_INPUT = PCAL6524_INPUT_PORT_0;
        break;
    case PORT1:
        PCAL6524_INPUT = PCAL6524_INPUT_PORT_1;
        break;
    case PORT2:
        PCAL6524_INPUT = PCAL6524_INPUT_PORT_2;
        break;
    default:
        onError(PCAL_ERROR_OUT_OF_RANGE);
        PCAL6524_INPUT = PCAL6524_INPUT_PORT_0;
    }

    //read the input register data
    input_reg_data = readRegister(PCAL6524_INPUT);
    onError(input_reg_data);

    //Isolate the reqested pin value from all other values
    input_reg_data.value[0] = input_reg_data.value[0] & pin.pinMask;

    //Bit Shift the resulting data over so the pin's requested value becomes the LSB
    switch (pin.pinMask) {
    case (0x01): //For Pins Px_0
        return input_reg_data.value[0];
    case (0x02): //For Pins Px_1
        return input_reg_data.value[0] >> 1;
    case (0x04): //For Pins Px_2
        return input_reg_data.value[0] >> 2;
    case (0x08): //For Pins Px_3
        return input_reg_data.value[0] >> 3;
    case (0x10): //For Pins Px_4
        return input_reg_data.value[0] >> 4;
    case (0x20): //For Pins Px_5
        return input_reg_data.value[0] >> 5;
    case (0x40): //For Pins Px_6
        return input_reg_data.value[0] >> 6;
    case (0x80): //For Pins Px_7
        return input_reg_data.value[0] >> 7;
    default:
        return input_reg_data.value[0];
    }
    return 0;
}

/**
 *    @brief  Reads pin values from specific port, this will clear all latch, interrupt from that port
 *    @param port Px x=port
 *    @return values of that port
 */
uint8_t PCAL6524::digitalReadPort(pcal_port_t port)
{
    pcal_address_t PCAL6524_INPUT;
    pcal_data_t input_reg_data;

    //Determine wich bank of pins the requested pin is in
    switch (port) {
    case PORT0:
        PCAL6524_INPUT = PCAL6524_INPUT_PORT_0;
        break;
    case PORT1:
        PCAL6524_INPUT = PCAL6524_INPUT_PORT_1;
        break;
    case PORT2:
        PCAL6524_INPUT = PCAL6524_INPUT_PORT_2;
        break;
    default:
        PCAL6524_INPUT = PCAL6524_INPUT_PORT_0;
        onError(PCAL_ERROR_OUT_OF_RANGE);
    }
    //read the input register data
    input_reg_data = readRegister(PCAL6524_INPUT);
    onError(input_reg_data);
    //Isolate the reqested pin value from all other values
    return input_reg_data.value[0];
}

/**
 *    @brief  Reads pin value from specific pin, this will clear all latch, interrupt from that pin's port
 *    @param pin Px_y x=port y=pin#
 */
pcal_err_t PCAL6524::digitalWrite(pcal_pin_t pin, uint8_t mode)
{
    pcal_data_t output_reg;
    pcal_address_t PCAL6524_OUTPUT;
    pcal_err_t writeError = PCAL_ERROR_UNKNOWN;

    //Determine wich bank of pins the requested pin is in
    switch (pin.port) {
    case PORT0:
        PCAL6524_OUTPUT = PCAL6524_OUTPUT_PORT_0;
        break;
    case PORT1:
        PCAL6524_OUTPUT = PCAL6524_OUTPUT_PORT_1;
        break;
    case PORT2:
        PCAL6524_OUTPUT = PCAL6524_OUTPUT_PORT_2;
        break;
    default:
        onError(PCAL_ERROR_OUT_OF_RANGE);
        return PCAL_ERROR_OUT_OF_RANGE;
    }
    // Read the current Value of out the ouput register
    output_reg = readRegister(PCAL6524_OUTPUT);
    if (onError(output_reg))
        return output_reg.error;

    //Deterime if Pin is being asked to go hi or to go low and set only that pins value;
    if (mode == HIGH) {
        output_reg.value[0] = output_reg.value[0] | pin.pinMask;

    } else if (mode == LOW) {
        output_reg.value[0] = output_reg.value[0] & ~(pin.pinMask);
    }
    writeError = writeRegister(PCAL6524_OUTPUT, output_reg);
    if (onError(writeError))
        return writeError;

    return writeError;
}

/**
 *    @brief  Reads pin value from specific pin, this will clear all latch, interrupt from that pin's port
 *    @param pin Px_y x=port y=pin#
 */
pcal_err_t PCAL6524::digitalWritePort(pcal_port_t port, uint8_t mode)
{
    pcal_data_t output_reg;
    pcal_address_t PCAL6524_OUTPUT;
    pcal_err_t writeError = PCAL_ERROR_UNKNOWN;

    //Determine wich bank of pins the requested pin is in
    switch (port) {
    case PORT0:
        PCAL6524_OUTPUT = PCAL6524_OUTPUT_PORT_0;
        break;
    case PORT1:
        PCAL6524_OUTPUT = PCAL6524_OUTPUT_PORT_1;
        break;
    case PORT2:
        PCAL6524_OUTPUT = PCAL6524_OUTPUT_PORT_2;
        break;
    default:
        onError(PCAL_ERROR_OUT_OF_RANGE);
        return PCAL_ERROR_OUT_OF_RANGE;
    }
    // Read the current Value of out the ouput register
    output_reg = readRegister(PCAL6524_OUTPUT);
    if (onError(output_reg))
        return output_reg.error;

    //Deterime if Pin is being asked to go hi or to go low and set only that pins value;
    if (mode == HIGH) {
        output_reg.value[0] = 0xFF;

    } else if (mode == LOW) {
        output_reg.value[0] = 0x00;
    }
    writeError = writeRegister(PCAL6524_OUTPUT, output_reg);
    if (onError(writeError))
        return writeError;

    return writeError;
}