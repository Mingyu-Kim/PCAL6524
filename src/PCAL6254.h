/**
 *   @file     PCAL6524.h
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

#ifndef PCAL6524_h
#define PCAL6524_h

#include "Arduino.h"
#include <Wire.h>

// Debug output containing lot's of info use it by defining DEBUG_PCAL
//If you want to change the default debug output stream, define PCAL_DEBUG_STREAM to something else
#ifdef DEBUG_PCAL
/**
 * @brief Prints some string in debug mode
 * @param str Something that Arduino Stream.print can print
 */
#define PCAL_PRINT(str)                                                                                 \
    PCAL_DEBUG_STREAM.printf("%d-%s:%d-%s \r\n  -", millis(), __FILE__, __LINE__, __PRETTY_FUNCTION__); \
    PCAL_DEBUG_STREAM.println(str)
/**
 * @brief Prints two strings with colon in between in debug mode
 * @param str1 Something that Arduino Stream.print can print
 * @param str2 Something that Arduino Stream.print can print
 */
#define PCAL_PRINT_TWO(str1, str2)                                                                      \
    PCAL_DEBUG_STREAM.printf("%d-%s:%d-%s \r\n  -", millis(), __FILE__, __LINE__, __PRETTY_FUNCTION__); \
    PCAL_DEBUG_STREAM.print(str1);                                                                      \
    PCAL_DEBUG_STREAM.print(':');                                                                       \
    PCAL_DEBUG_STREAM.println(str2)
/**
 * @brief Prints some value in some base in debug mode
 * @param val Some value
 * @param base Some base
 */
#define PCAL_PRINT_BASE(val, base)                                                                      \
    PCAL_DEBUG_STREAM.printf("%d-%s:%d-%s \r\n  -", millis(), __FILE__, __LINE__, __PRETTY_FUNCTION__); \
    PCAL_DEBUG_STREAM.println(val, base)
/**
 * @brief Prints some value with string in front in some base in debug mode
 * @param str Some string
 * @param val Some value
 * @param base Some base
 */
#define PCAL_PRINT_BASE_TWO(str, val, base)                                                             \
    PCAL_DEBUG_STREAM.printf("%d-%s:%d-%s \r\n  -", millis(), __FILE__, __LINE__, __PRETTY_FUNCTION__); \
    PCAL_DEBUG_STREAM.print(str);                                                                       \
    PCAL_DEBUG_STREAM.print(':');                                                                       \
    PCAL_DEBUG_STREAM.println(val, base)
#else
#define PCAL_PRINT(str)
#define PCAL_PRINT_TWO(str1, str2)
#define PCAL_PRINT_BASE(val, base)
#define PCAL_PRINT_BASE_TWO(str, val, base)
#endif

// Port number of the PCAL6524
typedef uint8_t pcal_port_t;
const pcal_port_t PORT0 = 0; // Port 0
const pcal_port_t PORT1 = 1; // Port 1
const pcal_port_t PORT2 = 2; // Port 2

typedef enum {
    PCAL_ERROR_OK = 0,
    PCAL_ERROR_I2C_DEV,
    PCAL_ERROR_I2C_ACK,
    PCAL_ERROR_I2C_TIMEOUT,
    PCAL_ERROR_I2C_BUS,
    PCAL_ERROR_I2C_BUSY,
    PCAL_ERROR_I2C_MEMORY,
    PCAL_ERROR_I2C_CONTINUE,
    PCAL_ERROR_I2C_NO_BEGIN,
    PCAL_ERROR_I2C_DATA_MISMATCH,
    PCAL_ERROR_OUT_OF_RANGE,
    PCAL_ERROR_WRITE_TO_INPUT,
    PCAL_ERROR_UNKNOWN = 0xff
} pcal_err_t;

/** 
 * @brief This structure has port and mask value
 */
typedef struct {
    pcal_port_t port; // Pin's port Number
    uint8_t pinMask; // Pin's bit mask
} pcal_pin_t;
const pcal_pin_t P0[8] = {
    { PORT0, (uint8_t)bit(0) }, { PORT0, (uint8_t)bit(1) },
    { PORT0, (uint8_t)bit(2) }, { PORT0, (uint8_t)bit(3) },
    { PORT0, (uint8_t)bit(4) }, { PORT0, (uint8_t)bit(5) },
    { PORT0, (uint8_t)bit(6) }, { PORT0, (uint8_t)bit(7) }
}; // Port 0 Pins
const pcal_pin_t P1[8] = {
    { PORT1, (uint8_t)bit(0) }, { PORT1, (uint8_t)bit(1) },
    { PORT1, (uint8_t)bit(2) }, { PORT1, (uint8_t)bit(3) },
    { PORT1, (uint8_t)bit(4) }, { PORT1, (uint8_t)bit(5) },
    { PORT1, (uint8_t)bit(6) }, { PORT1, (uint8_t)bit(7) }
}; // Port 1 Pins
const pcal_pin_t P2[8] = {
    { PORT2, (uint8_t)bit(0) }, { PORT2, (uint8_t)bit(1) },
    { PORT2, (uint8_t)bit(2) }, { PORT2, (uint8_t)bit(3) },
    { PORT2, (uint8_t)bit(4) }, { PORT2, (uint8_t)bit(5) },
    { PORT2, (uint8_t)bit(6) }, { PORT2, (uint8_t)bit(7) }
}; // Port 2 Pins

/** 
 * @brief This defines pin's drive strength
 */
typedef uint8_t pcal_strength_t;
const pcal_strength_t PCAL6524_STRONGEST = 0x11; //1x drive capablity
const pcal_strength_t PCAL6524_STRONG = 0x10; //0.75x drive capablity
const pcal_strength_t PCAL6524_WEAK = 0x01; //0.5x drive capablity
const pcal_strength_t PCAL6524_WEAKEST = 0x00; //0.25x drive capablity

// I2C ADDRESS/BITS
typedef uint8_t pcal_address_t;
const pcal_address_t PCAL6524_ADDRESS_0 = 0x20; //ADDR connected to SCL
const pcal_address_t PCAL6524_ADDRESS_1 = 0x21; //ADDR connected to SDA
const pcal_address_t PCAL6524_ADDRESS_2 = 0x22; //ADDR connected to GND
const pcal_address_t PCAL6524_ADDRESS_3 = 0x23; //ADDR connected to VDD

// Additinal Value
#ifndef INPUT_PULLDOWN
#define INPUT_PULLDOWN 0x03
#endif

class PCAL6524 {
protected:
    // Addresses
    const pcal_address_t PCAL6524_INPUT_PORT_0 = 0x00; //INPUT PORT 0 Register  - Read Only
    const pcal_address_t PCAL6524_INPUT_PORT_1 = 0x01; //INPUT PORT 1 Register  - Read Only
    const pcal_address_t PCAL6524_INPUT_PORT_2 = 0x02; //INPUT PORT 2 Register  - Read Only
    const pcal_address_t PCAL6524_OUTPUT_PORT_0 = 0x04; //OUTPUT PORT 0 Register  - Read & Write
    const pcal_address_t PCAL6524_OUTPUT_PORT_1 = 0x05; //OUTPUT PORT 1 Register  - Read & Write
    const pcal_address_t PCAL6524_OUTPUT_PORT_2 = 0x06; //OUTPUT PORT 2 Register  - Read & Write
    const pcal_address_t PCAL6524_POLARITY_INVERSION_PORT_0 = 0x08; //Read & Write
    const pcal_address_t PCAL6524_POLARITY_INVERSION_PORT_1 = 0x09; //Read & Write
    const pcal_address_t PCAL6524_POLARITY_INVERSION_PORT_2 = 0x0A; //Read & Write
    const pcal_address_t PCAL6524_CONFIGURATION_PORT_0 = 0x0C; //Read & Write
    const pcal_address_t PCAL6524_CONFIGURATION_PORT_1 = 0x0D; //Read & Write
    const pcal_address_t PCAL6524_CONFIGURATION_PORT_2 = 0x0E; //Read & Write
    const pcal_address_t PCAL6524_OUTPUT_STRENGTH_0A = 0x40;
    const pcal_address_t PCAL6524_OUTPUT_STRENGTH_0B = 0x41;
    const pcal_address_t PCAL6524_OUTPUT_STRENGTH_1A = 0x42;
    const pcal_address_t PCAL6524_OUTPUT_STRENGTH_1B = 0x43;
    const pcal_address_t PCAL6524_OUTPUT_STRENGTH_2A = 0x44;
    const pcal_address_t PCAL6524_OUTPUT_STRENGTH_2B = 0x45;
    const pcal_address_t PCAL6524_INPUT_LATCH_PORT_0 = 0X48;
    const pcal_address_t PCAL6524_INPUT_LATCH_PORT_1 = 0X49;
    const pcal_address_t PCAL6524_INPUT_LATCH_PORT_2 = 0X4A;
    const pcal_address_t PCAL6524_RESISTOR_PULL_ENABLE_PORT_0 = 0x4C;
    const pcal_address_t PCAL6524_RESISTOR_PULL_ENABLE_PORT_1 = 0x4D;
    const pcal_address_t PCAL6524_RESISTOR_PULL_ENABLE_PORT_2 = 0x4E;
    const pcal_address_t PCAL6524_RESISTOR_PULL_SELECTION_PORT_0 = 0x50;
    const pcal_address_t PCAL6524_RESISTOR_PULL_SELECTION_PORT_1 = 0x51;
    const pcal_address_t PCAL6524_RESISTOR_PULL_SELECTION_PORT_2 = 0x52;
    const pcal_address_t PCAL6524_INTERRUPT_MASK_PORT_0 = 0x54;
    const pcal_address_t PCAL6524_INTERRUPT_MASK_PORT_1 = 0x55;
    const pcal_address_t PCAL6524_INTERRUPT_MASK_PORT_2 = 0x56;
    const pcal_address_t PCAL6524_INTERRUPT_STATUS_PORT_0 = 0x58;
    const pcal_address_t PCAL6524_INTERRUPT_STATUS_PORT_1 = 0x59;
    const pcal_address_t PCAL6524_INTERRUPT_STATUS_PORT_2 = 0x5A;
    const pcal_address_t PCAL6524_OUTPUT_PORT_CONFIG = 0x5C;
    const pcal_address_t PCAL6524_INTERRUPT_EDGE_PORT_0A = 0x60;
    const pcal_address_t PCAL6524_INTERRUPT_EDGE_PORT_0B = 0x61;
    const pcal_address_t PCAL6524_INTERRUPT_EDGE_PORT_1A = 0x62;
    const pcal_address_t PCAL6524_INTERRUPT_EDGE_PORT_1B = 0x63;
    const pcal_address_t PCAL6524_INTERRUPT_EDGE_PORT_2A = 0x64;
    const pcal_address_t PCAL6524_INTERRUPT_EDGE_PORT_2B = 0x65;
    const pcal_address_t PCAL6524_INTERRUPT_CLEAR_PORT_0 = 0x68;
    const pcal_address_t PCAL6524_INTERRUPT_CLEAR_PORT_1 = 0x69;
    const pcal_address_t PCAL6524_INTERRUPT_CLEAR_PORT_2 = 0x6A;
    const pcal_address_t PCAL6524_INPUT_STATUS_PORT_0 = 0x6C;
    const pcal_address_t PCAL6524_INPUT_STATUS_PORT_1 = 0x6D;
    const pcal_address_t PCAL6524_INPUT_STATUS_PORT_2 = 0x6E;
    const pcal_address_t PCAL6524_PIN_OUTPUT_CONFIG_PORT_0 = 0x70;
    const pcal_address_t PCAL6524_PIN_OUTPUT_CONFIG_PORT_1 = 0x71;
    const pcal_address_t PCAL6524_PIN_OUTPUT_CONFIG_PORT_2 = 0x72;
    const pcal_address_t PCAL6524_SWITCH_DEBOUCE_ENABLE_PORT_0 = 0x74;
    const pcal_address_t PCAL6524_SWITCH_DEBOUCE_ENABLE_PORT_1 = 0x75;
    const pcal_address_t PCAL6524_SWITCH_DEBOUCE_COUNT = 0x76;

    pcal_address_t m_i2cAddress;
    TwoWire* m_wire;
    int m_sda;
    int m_scl;
    int m_freq;
    typedef struct {
        uint8_t size = 1;
        uint8_t value[6] = { 0 };
        pcal_err_t error = PCAL_ERROR_UNKNOWN;
    } pcal_data_t;
    pcal_data_t interruptData;
    pcal_err_t writeRegister(pcal_address_t reg, pcal_data_t value);
    pcal_data_t readRegister(pcal_address_t reg, uint8_t size = 1);
    pcal_err_t readInterrupt();
    bool isUpperPin(pcal_pin_t pin) { return (pin.pinMask > 0x08) ? true : false; };

public:
    PCAL6524(pcal_address_t i2cAddress = PCAL6524_ADDRESS_2); //Default address = PCAL6524_ADDRESS_2
    bool onError(pcal_data_t error);
    bool onError(pcal_err_t error);
    pcal_err_t begin(int sda = -1, int scl = -1, uint32_t frequency = 0, TwoWire* theWire = &Wire);
    pcal_err_t reset();
    pcal_err_t pinMode(pcal_pin_t pin, uint8_t mode);
    pcal_err_t portMode(pcal_port_t port, uint8_t mode);
    pcal_err_t inputPinMode(pcal_pin_t pin, bool inverted = false, bool latched = false);
    pcal_err_t outputPinMode(pcal_pin_t pin, pcal_strength_t strength = PCAL6524_STRONGEST, bool isOpenDrain = false);
    pcal_err_t inputPortMode(pcal_port_t port, bool inverted = false, bool latched = false);
    pcal_err_t outputPortMode(pcal_port_t port, pcal_strength_t strength = PCAL6524_STRONGEST, bool isOpenDrain = false);
    pcal_err_t pinInterrupt(pcal_pin_t pin, uint8_t type = DISABLED);
    pcal_err_t portInterrupt(pcal_port_t port, uint8_t type = DISABLED);
    void IRAM_ATTR interruptHandler();
    bool getIntFlag() { return intFlag; };
    bool interruptSourcePin(pcal_pin_t pin);
    pcal_data_t interruptSource();
    uint8_t digitalRead(pcal_pin_t pin);
    uint8_t digitalReadPort(pcal_port_t port);
    pcal_err_t digitalWrite(pcal_pin_t pin, uint8_t mode);
    pcal_err_t digitalWritePort(pcal_port_t port, uint8_t mode);

private:
    volatile bool intFlag = false;
};

#endif