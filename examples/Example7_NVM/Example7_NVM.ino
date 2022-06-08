#include <Wire.h>
#include "SparkFun_BMP581_Arduino_Library.h"

// Create a new sensor object
BMP581 pressureSensor;

// I2C address selection
uint8_t i2cAddress = BMP581_I2C_ADDRESS_DEFAULT; // 0x47
//uint8_t i2cAddress = BMP384_I2C_ADDRESS_SECONDARY; // 0x46

// Data to write into the NVM. In this case we're going to store some
// characters, but any 6 bytes of data can be stored
char dataToWrite[] = "Hello!";

void setup()
{
    // Start serial
    Serial.begin(115200);
    Serial.println("BMP581 Example7 begin!");

    // Initialize the I2C library
    Wire.begin();

    // Check if sensor is connected and initialize
    // Address is optional (defaults to 0x47)
    while(pressureSensor.beginI2C(i2cAddress) != BMP5_OK)
    {
        // Not connected, inform user
        Serial.println("Error: BMP581 not connected, check wiring and I2C address!");

        // Wait a bit to see if connection is established
        delay(1000);
    }

    Serial.println("BMP581 connected!");

    Serial.println("Writing data to NVM: ");
    Serial.println(dataToWrite);

    // The BMP581 contains non-volatile memory (NVM) that is primarily used for
    // calibration data internally by the sensor. However 6 bytes are user programmable,
    // stored in 3 2-byte locations (0x20 - 0x22).
    uint16_t dataIndex = 0;
    for(uint8_t addr = BMP5_NVM_START_ADDR; addr <= BMP5_NVM_END_ADDR; addr++)
    {
        uint16_t data = dataToWrite[dataIndex] | (dataToWrite[dataIndex+1] << 8);
        dataIndex += 2;

        pressureSensor.writeNVM(addr, data);
    }
    
    Serial.println("Data read back from NVM: ");

    // Now we can read back the data and display it
    for(uint8_t addr = BMP5_NVM_START_ADDR; addr <= BMP5_NVM_END_ADDR; addr++)
    {
        uint16_t data = 0;
        pressureSensor.readNVM(addr, &data);
        char first = data & 0xFF;
        char second = (data >> 8) & 0xFF;
        Serial.print(first);
        Serial.print(second);
    }
}

void loop()
{

}