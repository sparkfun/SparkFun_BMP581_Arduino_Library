#include <Wire.h>
#include "SparkFunBMP581.h"

// Create a new sensor object
BMP581 pressureSensor;

// I2C address selection
uint8_t i2cAddress = BMP581_I2C_ADDRESS_DEFAULT; // 0x47
//uint8_t i2cAddress = BMP384_I2C_ADDRESS_SECONDARY; // 0x46

void setup()
{
    // Start serial
    Serial.begin(115200);
    Serial.println("BMP581 Example4 begin!");

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
    
    // Variable to track errors returned by API calls
    int8_t err = BMP5_OK;

    // The BMP581 can filter both the temperature and pressure measurements individually.
    // By default, both filter coefficients are set to 0 (no filtering). We can smooth
    // out the measurements by increasing the coefficients with this function
    err = pressureSensor.setFilterCoefficients(BMP5_IIR_FILTER_COEFF_127, BMP5_IIR_FILTER_COEFF_127);
    if(err)
    {
        // Setting coefficient failed, most likely an invalid coefficient (code -12)
        Serial.print("Error setting filter coefficient! Error code: ");
        Serial.println(err);
    }
}

void loop()
{
    // Get measurements from the sensor
    bmp5_sensor_data data = {0};
    int8_t err = pressureSensor.getSensorData(&data);

    // Check whether data was acquired successfully
    if(err == BMP5_OK)
    {
        // Acquisistion succeeded, print temperature and pressure
        Serial.print("Temperature (C): ");
        Serial.print(data.temperature);
        Serial.print("\t\t");
        Serial.print("Pressure (Pa): ");
        Serial.println(data.pressure);
    }
    else
    {
        // Acquisition failed, most likely a communication error (code -2)
        Serial.print("Error getting data from sensor! Error code: ");
        Serial.println(err);
    }

    // Print every 100ms to see effects of filter
    delay(100);
}