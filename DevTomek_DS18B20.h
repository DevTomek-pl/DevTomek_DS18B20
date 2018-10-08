#ifndef DevTomek_DS18B20_h
#define DevTomek_DS18B20_h

// Based on DallasTemperature library
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.

#include <OneWire.h>
#include <inttypes.h>

// Default configuration
#define DEFAULT_PARASITE_MODE false
#define DEFAULT_BIT_RESOLUTION 9

// Model IDs
#define DS18S20MODEL 0x10  // also DS1820
#define DS18B20MODEL 0x28
#define DS1822MODEL 0x22
#define DS1825MODEL 0x3B
#define DS28EA00MODEL 0x42

// Error Codes
#define DEVICE_DISCONNECTED_C -127
#define DEVICE_DISCONNECTED_RAW -7040

typedef uint8_t DeviceAddress[8];

class DevTomek_DS18B20 {
   public:
    DevTomek_DS18B20(uint8_t pin);

    // initialise bus
    void begin(void);

    // returns the number of devices found on the bus
    uint8_t getDeviceCount(void);

    // returns true if address is valid
    bool validAddress(const uint8_t* deviceAddress);

    // finds an address at a given index on the bus
    bool getAddress(uint8_t* deviceAddress, uint8_t index);

    // attempt to determine if the device at the given address
    // is connected to the bus
    bool isConnected(const uint8_t* deviceAddress);

    // attempt to determine if the device at the given address
    // is connected tothe bus also allows for updating the read scratchpad
    bool isConnected(const uint8_t* deviceAddress, uint8_t* scratchPad);

    // read device's scratchpad
    bool readScratchPad(const uint8_t* deviceAddress, uint8_t* scratchPad);

    // write device's scratchpad
    void writeScratchPad(const uint8_t* deviceAddress,
                         const uint8_t* scratchPad);

    // read device's power requirements
    bool readPowerSupply(const uint8_t* deviceAddress);

    // get global resolution
    uint8_t getResolution();

    // set global resolution to 9, 10, 11, or 12 bits
    void setResolution(uint8_t newResolution);

    // returns the device resolution: 9, 10, 11, or 12 bits
    uint8_t getResolution(const uint8_t* deviceAddress);

    // set resolution of a device to 9, 10, 11, or 12 bits
    bool setResolution(const uint8_t* deviceAddress,
                       uint8_t newResolution,
                       bool skipGlobalBitResolutionCalculation = false);

    // sends command for all devices on the bus to perform a temperature
    // conversion
    void requestTemperatures(void);

    // Get temperature for device index (slow)
    float getTemp(uint8_t deviceIndex);

    // returns temperature raw value (12 bit integer of 1/128 degrees C)
    int16_t getRaw(const uint8_t* deviceAddress);

    // Is a conversion complete on the wire? Only applies to the first sensor on
    // the wire.
    bool isConversionComplete(void);

    int16_t millisToWaitForConversion(uint8_t bitResolution);

    // convert from raw to Celsius
    float rawToCelsius(int16_t raw);

   private:
    typedef uint8_t ScratchPad[9];

    // parasite power on or off
    bool parasite;

    // used to determine the delay amount needed to allow for the
    // temperature conversion to take place
    uint8_t bitResolution;

    // count of devices on the bus
    uint8_t devices;

    // OneWire instance
    OneWire oneWire;

    // take a pointer to one wire instance
    OneWire* oneWirePtr;

    // reads scratchpad and returns the raw temperature
    int16_t calculateTemperature(const uint8_t* deviceAddress,
                                 uint8_t* scratchPad);

    void blockTillConversionComplete(uint8_t);
};
#endif
