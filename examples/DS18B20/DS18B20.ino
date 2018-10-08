#include <DevTomek_DS18B20.h>

#define DS18B20_PIN 5

// set up DevTomek_DS18B20 variable and initialise 1-wire bus
DevTomek_DS18B20 ds18b20(DS18B20_PIN);

void setup() {
  Serial.begin(9600);   // initialise serial bus
  ds18b20.begin();       // search devices on 1-wire bus
}

void loop() {
  delay(1000);

  // sends command for all devices on the bus to perform a temperature conversion
  ds18b20.requestTemperatures();

  // get temperature for first device
  float tempValue = ds18b20.getTemp(0);

  // print temperature in degrees Celsius
  Serial.println(tempValue);
}
