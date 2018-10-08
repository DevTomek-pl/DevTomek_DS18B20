// Based on DallasTemperature library
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.

#include "DevTomek_DS18B20.h"

#if ARDUINO >= 100
#include "Arduino.h"
#else
extern "C" {
#include "WConstants.h"
}
#endif

// OneWire commands
#define STARTCONVO 0x44       // tells device to take a temperature reading
#define COPYSCRATCH 0x48      // copy EEPROM
#define READSCRATCH 0xBE      // read EEPROM
#define WRITESCRATCH 0x4E     // write to EEPROM
#define RECALLSCRATCH 0xB8    // reload from last known
#define READPOWERSUPPLY 0xB4  // determine if device needs parasite power
#define ALARMSEARCH 0xEC      // query bus for devices with an alarm condition

// scratchpad locations
#define TEMP_LSB 0
#define TEMP_MSB 1
#define HIGH_ALARM_TEMP 2
#define LOW_ALARM_TEMP 3
#define CONFIGURATION 4
#define INTERNAL_BYTE 5
#define COUNT_REMAIN 6
#define COUNT_PER_C 7
#define SCRATCHPAD_CRC 8

// device resolution
#define TEMP_9_BIT 0x1F   //  9 bit
#define TEMP_10_BIT 0x3F  // 10 bit
#define TEMP_11_BIT 0x5F  // 11 bit
#define TEMP_12_BIT 0x7F  // 12 bit

DevTomek_DS18B20::DevTomek_DS18B20(uint8_t pin) {
    oneWire.begin(pin);
    oneWirePtr = &oneWire;
    devices = 0;
    parasite = DEFAULT_PARASITE_MODE;
    bitResolution = DEFAULT_BIT_RESOLUTION;
}

// initialise the bus
void DevTomek_DS18B20::begin(void) {
    DeviceAddress deviceAddress;

    oneWirePtr->reset_search();
    devices = 0;  // reset the number of devices when we enumerate wire devices

    while (oneWirePtr->search(deviceAddress)) {
        if (validAddress(deviceAddress)) {
            if (!parasite && readPowerSupply(deviceAddress))
                parasite = true;

            bitResolution = max(bitResolution, getResolution(deviceAddress));

            devices++;
        }
    }
}

// returns the number of devices found on the bus
uint8_t DevTomek_DS18B20::getDeviceCount(void) {
    return devices;
}

// returns true if address is valid
bool DevTomek_DS18B20::validAddress(const uint8_t* deviceAddress) {
    return (oneWirePtr->crc8(deviceAddress, 7) == deviceAddress[7]);
}

// finds an address at a given index on the bus
// returns true if the device was found
bool DevTomek_DS18B20::getAddress(uint8_t* deviceAddress, uint8_t index) {
    uint8_t depth = 0;

    oneWirePtr->reset_search();

    while (depth <= index && oneWirePtr->search(deviceAddress)) {
        if (depth == index && validAddress(deviceAddress))
            return true;
        depth++;
    }

    return false;
}

// attempt to determine if the device at the given address
// is connected to the bus
bool DevTomek_DS18B20::isConnected(const uint8_t* deviceAddress) {
    ScratchPad scratchPad;
    return isConnected(deviceAddress, scratchPad);
}

// attempt to determine if the device at the given address
// is connected to the bus also allows for updating the read scratchpad
bool DevTomek_DS18B20::isConnected(const uint8_t* deviceAddress,
                                   uint8_t* scratchPad) {
    bool b = readScratchPad(deviceAddress, scratchPad);

    return b && (oneWirePtr->crc8(scratchPad, 8) == scratchPad[SCRATCHPAD_CRC]);
}

bool DevTomek_DS18B20::readScratchPad(const uint8_t* deviceAddress,
                                      uint8_t* scratchPad) {
    // send the reset command and fail fast
    int b = oneWirePtr->reset();
    if (b == 0)
        return false;

    oneWirePtr->select(deviceAddress);
    oneWirePtr->write(READSCRATCH);

    // Read all registers in a simple loop
    // byte 0: temperature LSB
    // byte 1: temperature MSB
    // byte 2: high alarm temp
    // byte 3: low alarm temp
    // byte 4: DS18S20: store for crc
    //         DS18B20 & DS1822: configuration register
    // byte 5: internal use & crc
    // byte 6: DS18S20: COUNT_REMAIN
    //         DS18B20 & DS1822: store for crc
    // byte 7: DS18S20: COUNT_PER_C
    //         DS18B20 & DS1822: store for crc
    // byte 8: SCRATCHPAD_CRC
    for (uint8_t i = 0; i < 9; i++) {
        scratchPad[i] = oneWirePtr->read();
    }

    b = oneWirePtr->reset();
    return (b == 1);
}

void DevTomek_DS18B20::writeScratchPad(const uint8_t* deviceAddress,
                                       const uint8_t* scratchPad) {
    oneWirePtr->reset();
    oneWirePtr->select(deviceAddress);
    oneWirePtr->write(WRITESCRATCH);
    oneWirePtr->write(scratchPad[HIGH_ALARM_TEMP]);  // high alarm temp
    oneWirePtr->write(scratchPad[LOW_ALARM_TEMP]);   // low alarm temp

    // DS1820 and DS18S20 have no configuration register
    if (deviceAddress[0] != DS18S20MODEL)
        oneWirePtr->write(scratchPad[CONFIGURATION]);

    oneWirePtr->reset();

    // save the newly written values to eeprom
    oneWirePtr->select(deviceAddress);
    oneWirePtr->write(COPYSCRATCH, parasite);
    delay(20);  // <--- added 20ms delay to allow 10ms long EEPROM write
                // operation (as specified by datasheet)

    if (parasite)
        delay(10);  // 10ms delay
    oneWirePtr->reset();
}

bool DevTomek_DS18B20::readPowerSupply(const uint8_t* deviceAddress) {
    bool ret = false;
    oneWirePtr->reset();
    oneWirePtr->select(deviceAddress);
    oneWirePtr->write(READPOWERSUPPLY);
    if (oneWirePtr->read_bit() == 0)
        ret = true;
    oneWirePtr->reset();
    return ret;
}

// set resolution of all devices to 9, 10, 11, or 12 bits
// if new resolution is out of range, it is constrained.
void DevTomek_DS18B20::setResolution(uint8_t newResolution) {
    bitResolution = constrain(newResolution, 9, 12);
    DeviceAddress deviceAddress;
    for (int i = 0; i < devices; i++) {
        getAddress(deviceAddress, i);
        setResolution(deviceAddress, bitResolution, true);
    }
}

// set resolution of a device to 9, 10, 11, or 12 bits
// if new resolution is out of range, 9 bits is used.
bool DevTomek_DS18B20::setResolution(const uint8_t* deviceAddress,
                                     uint8_t newResolution,
                                     bool skipGlobalBitResolutionCalculation) {
    // ensure same behavior as setResolution(uint8_t newResolution)
    newResolution = constrain(newResolution, 9, 12);

    // return when stored value == new value
    if (getResolution(deviceAddress) == newResolution)
        return true;

    ScratchPad scratchPad;
    if (isConnected(deviceAddress, scratchPad)) {
        // DS1820 and DS18S20 have no resolution configuration register
        if (deviceAddress[0] != DS18S20MODEL) {
            switch (newResolution) {
                case 12:
                    scratchPad[CONFIGURATION] = TEMP_12_BIT;
                    break;
                case 11:
                    scratchPad[CONFIGURATION] = TEMP_11_BIT;
                    break;
                case 10:
                    scratchPad[CONFIGURATION] = TEMP_10_BIT;
                    break;
                case 9:
                default:
                    scratchPad[CONFIGURATION] = TEMP_9_BIT;
                    break;
            }
            writeScratchPad(deviceAddress, scratchPad);

            // without calculation we can always set it to max
            bitResolution = max(bitResolution, newResolution);

            if (!skipGlobalBitResolutionCalculation &&
                (bitResolution > newResolution)) {
                bitResolution = newResolution;
                DeviceAddress deviceAddr;
                for (int i = 0; i < devices; i++) {
                    getAddress(deviceAddr, i);
                    bitResolution =
                        max(bitResolution, getResolution(deviceAddr));
                }
            }
        }
        return true;  // new value set
    }

    return false;
}

// returns the global resolution
uint8_t DevTomek_DS18B20::getResolution() {
    return bitResolution;
}

// returns the current resolution of the device, 9-12
// returns 0 if device not found
uint8_t DevTomek_DS18B20::getResolution(const uint8_t* deviceAddress) {
    // DS1820 and DS18S20 have no resolution configuration register
    if (deviceAddress[0] == DS18S20MODEL)
        return 12;

    ScratchPad scratchPad;
    if (isConnected(deviceAddress, scratchPad)) {
        switch (scratchPad[CONFIGURATION]) {
            case TEMP_12_BIT:
                return 12;
            case TEMP_11_BIT:
                return 11;
            case TEMP_10_BIT:
                return 10;
            case TEMP_9_BIT:
                return 9;
        }
    }
    return 0;
}

bool DevTomek_DS18B20::isConversionComplete() {
    uint8_t b = oneWirePtr->read_bit();
    return (b == 1);
}

// sends command for all devices on the bus to perform a temperature conversion
void DevTomek_DS18B20::requestTemperatures() {
    oneWirePtr->reset();
    oneWirePtr->skip();
    oneWirePtr->write(STARTCONVO, parasite);

    blockTillConversionComplete(bitResolution);
}

// continue to check if the IC has responded with a temperature
void DevTomek_DS18B20::blockTillConversionComplete(uint8_t bitResolution) {
    int delms = millisToWaitForConversion(bitResolution);
    if (!parasite) {
        unsigned long now = millis();
        while (!isConversionComplete() && (millis() - delms < now))
            ;
    } else {
        delay(delms);
    }
}

// returns number of milliseconds to wait till conversion is complete
// (based on IC datasheet)
int16_t DevTomek_DS18B20::millisToWaitForConversion(uint8_t bitResolution) {
    switch (bitResolution) {
        case 9:
            return 94;
        case 10:
            return 188;
        case 11:
            return 375;
        default:
            return 750;
    }
}

// fetch temperature for device index
float DevTomek_DS18B20::getTemp(uint8_t deviceIndex) {
    DeviceAddress deviceAddress;
    if (!getAddress(deviceAddress, deviceIndex)) {
        return DEVICE_DISCONNECTED_C;
    }

    int16_t raw = getRaw(deviceAddress);

    return rawToCelsius(raw);
}

// returns temperature in 1/128 degrees C or DEVICE_DISCONNECTED_RAW if the
// device's scratch pad cannot be read successfully.
// the numeric value of DEVICE_DISCONNECTED_RAW is defined in
// DevTomek_DS18B20.h. It is a large negative number outside the
// operating range of the device
int16_t DevTomek_DS18B20::getRaw(const uint8_t* deviceAddress) {
    ScratchPad scratchPad;
    if (isConnected(deviceAddress, scratchPad))
        return calculateTemperature(deviceAddress, scratchPad);
    return DEVICE_DISCONNECTED_RAW;
}

// convert from raw to Celsius
float DevTomek_DS18B20::rawToCelsius(int16_t raw) {
    if (raw <= DEVICE_DISCONNECTED_RAW)
        return DEVICE_DISCONNECTED_C;
    // C = RAW/128
    return (float)raw * 0.0078125;
}

// reads scratchpad and returns fixed-point temperature, scaling factor 2^-7
int16_t DevTomek_DS18B20::calculateTemperature(const uint8_t* deviceAddress,
                                               uint8_t* scratchPad) {
    int16_t fpTemperature = (((int16_t)scratchPad[TEMP_MSB]) << 11) |
                            (((int16_t)scratchPad[TEMP_LSB]) << 3);

    /*
      DS1820 and DS18S20 have a 9-bit temperature register.

      Resolutions greater than 9-bit can be calculated using the data from
      the temperature, and COUNT REMAIN and COUNT PER °C registers in the
      scratchpad.  The resolution of the calculation depends on the model.

      While the COUNT PER °C register is hard-wired to 16 (10h) in a
      DS18S20, it changes with temperature in DS1820.

      After reading the scratchpad, the TEMP_READ value is obtained by
      truncating the 0.5°C bit (bit 0) from the temperature data. The
      extended resolution temperature can then be calculated using the
      following equation:

      COUNT_PER_C - COUNT_REMAIN
      TEMPERATURE = TEMP_READ - 0.25 + --------------------------
      COUNT_PER_C

      Hagai Shatz simplified this to integer arithmetic for a 12 bits
      value for a DS18S20, and James Cameron added legacy DS1820 support.

      See -
      http://myarduinotoy.blogspot.co.uk/2013/02/12bit-result-from-ds18s20.html
      */

    if (deviceAddress[0] == DS18S20MODEL) {
        fpTemperature =
            ((fpTemperature & 0xfff0) << 3) - 16 +
            (((scratchPad[COUNT_PER_C] - scratchPad[COUNT_REMAIN]) << 7) /
             scratchPad[COUNT_PER_C]);
    }

    return fpTemperature;
}
