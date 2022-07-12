#include <Wire.h>
#include "SparkFun_BMP581_Arduino_Library.h"

// Create a new sensor object
BMP581 pressureSensor;

// I2C address selection
uint8_t i2cAddress = BMP581_I2C_ADDRESS_DEFAULT; // 0x47
//uint8_t i2cAddress = BMP581_I2C_ADDRESS_SECONDARY; // 0x46

// Pin used for interrupt detection
int interruptPin = 2;

// Flag to know when interrupts occur
volatile bool interruptOccurred = false;

// OOR range specification. The center pressure is 1 atm (101325 Pa), and the
// window is the max value supported by the sensor (255)
uint32_t oorCenter = 101325;
uint8_t oorWindow = 255;

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

    // The BMP581 can sample up to 240Hz in normal mode. However we don't want
    // interrupts to trigger that fast in this example, so we can lower the output
    // data rate (ODR) with this function. In this case, we're setting it to 1Hz.
    // For all possible ODR settings, see bmp5_defs.h (line 282-313)
    err = pressureSensor.setODRFrequency(BMP5_ODR_01_HZ);
    if(err != BMP5_OK)
    {
        // Setting ODR failed, most likely an invalid frequncy (code -12)
        Serial.print("ODR setting failed! Error code: ");
        Serial.println(err);
    }

    // The BMP581 has multiple possible interrupt conditions, one of which is
    // out-of-range (OOR). This will trigger once the measured pressure goes above
    // or below some customizable range. This range is defined by a center value
    // +/- a window value. The center value can be any 17-bit number in Pa
    // (up to 131071 Pa), and the window can be any 8-bit number in Pa (up to 255)
    bmp5_oor_press_configuration oorConfig
    {
        .oor_thr_p     = oorCenter, // Center value, up to 131071 Pa
        .oor_range_p   = oorWindow, // Window value, up to 255 Pa
        .cnt_lim       = BMP5_OOR_COUNT_LIMIT_3, // Number of measurements that need to be out of range before interrupt occurs
        .oor_sel_iir_p = BMP5_DISABLE // Whether to check filtered or unfiltered measurements
    };
    pressureSensor.setOORConfig(&oorConfig);
    if(err != BMP5_OK)
    {
        // Interrupt settings failed, most likely a communication error (code -2)
        Serial.print("Interrupt settings failed! Error code: ");
        Serial.println(err);
    }

    // Configure the BMP581 to trigger interrupts whenever a measurement is performed
    BMP581_InterruptConfig interruptConfig =
    {
        .enable   = BMP5_INTR_ENABLE,    // Enable interrupts
        .drive    = BMP5_INTR_PUSH_PULL, // Push-pull or open-drain
        .polarity = BMP5_ACTIVE_HIGH,    // Active low or high
        .mode     = BMP5_PULSED,         // Latch or pulse signal
        .sources  =
        {
            .drdy_en = BMP5_ENABLE,        // Trigger interrupts when data is ready
            .fifo_full_en = BMP5_DISABLE,  // Trigger interrupts when FIFO is full
            .fifo_thres_en = BMP5_DISABLE, // Trigger interrupts when FIFO threshold is reached
            .oor_press_en = BMP5_ENABLE    // Trigger interrupts when pressure goes out of range
        }
    };
    err = pressureSensor.setInterruptConfig(&interruptConfig);
    if(err != BMP5_OK)
    {
        // Interrupt settings failed, most likely a communication error (code -2)
        Serial.print("Interrupt settings failed! Error code: ");
        Serial.println(err);
    }

    // Setup interrupt handler
    attachInterrupt(digitalPinToInterrupt(interruptPin), bmp581InterruptHandler, RISING);
}

void loop()
{
    // Wait for interrupt to occur
    if(interruptOccurred)
    {
        // Reset flag for next interrupt
        interruptOccurred = false;

        Serial.print("Interrupt occurred!\t\t");

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
        }

        // Check if this is the "out-of-range" interrupt condition
        if(interruptStatus & BMP5_INT_ASSERTED_PRESSURE_OOR)
        {
            Serial.println("Out of range condition triggered!");
        }

        // Check if neither interrupt occurred
        if(!(interruptStatus & (BMP5_INT_ASSERTED_PRESSURE_OOR | BMP5_INT_ASSERTED_DRDY)))
        {
            Serial.println("Wrong interrupt condition!");
        }
    }
}

void bmp581InterruptHandler()
{
    interruptOccurred = true;
}