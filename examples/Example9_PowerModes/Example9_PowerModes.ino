#include <Wire.h>
#include "SparkFun_BMP581_Arduino_Library.h"

// Create a new sensor object
BMP581 pressureSensor;

// I2C address selection
uint8_t i2cAddress = BMP581_I2C_ADDRESS_DEFAULT; // 0x47
//uint8_t i2cAddress = BMP384_I2C_ADDRESS_SECONDARY; // 0x46

// Pin used for interrupt detection
int interruptPin = 5;

// Flag to know when interrupts occur
volatile bool interruptOccurred = false;

// OOR range specification
uint32_t oorCenter = 84000;
uint8_t oorWindow = 50;

void setup()
{
    // Start serial
    Serial.begin(115200);
    Serial.println("BMP581 Example3 begin!");

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

    pressureSensor.setMode(BMP5_POWERMODE_CONTINOUS);
    if(err != BMP5_OK)
    {
        // Interrupt settings failed, most likely a communication error (code -2)
        Serial.print("Interrupt settings failed! Error code: ");
        Serial.println(err);
    }
}

void loop()
{
    // Wait for interrupt to occur
    if(interruptOccurred)
    {
        // Reset flag for next interrupt
        interruptOccurred = false;

        // Serial.print("Interrupt occurred!\t\t");

        // Variable to track errors returned by API calls
        int8_t err = BMP5_OK;

        // Get the interrupt status to know which condition triggered
        uint8_t interruptStatus = 0;
        err = pressureSensor.getInterruptStatus(&interruptStatus);
        if(err != BMP5_OK)
        {
            // Status get failed, most likely a communication error (code -2)
            Serial.print("Get interrupt status failed! Error code: ");
            Serial.println(err);
            return;
        }

        // Check if this is the "data ready" interrupt condition
        if(interruptStatus & BMP5_INT_ASSERTED_DRDY)
        {
            // Get measurements from the sensor
            bmp5_sensor_data data = {0};
            int8_t err = pressureSensor.getSensorData(&data);

            // Check whether data was acquired successfully
            if(err == BMP5_OK)
            {
                // Acquisistion succeeded, print temperature and pressure
                // Serial.print("Temperature (C): ");
                // Serial.print(data.temperature);
                // Serial.print("\t\t");
                // Serial.print("Pressure (Pa): ");
                Serial.println(data.pressure);
            }
            else
            {
                // Acquisition failed, most likely a communication error (code -2)
                Serial.print("Error getting data from sensor! Error code: ");
                Serial.println(err);
            }
        }

        // // Check if this is the "out-of-range" interrupt condition
        // if(interruptStatus & BMP5_INT_ASSERTED_PRESSURE_OOR)
        // {
        //     Serial.println("Pressure went out of range!");
        // }

        // // Check if neither interrupt occurred
        // if(!(interruptStatus & (BMP5_INT_ASSERTED_PRESSURE_OOR | BMP5_INT_ASSERTED_DRDY)))
        // {
        //     Serial.println("Wrong interrupt condition!");
        // }
    }
}

void bmp581InterruptHandler()
{
    interruptOccurred = true;
}