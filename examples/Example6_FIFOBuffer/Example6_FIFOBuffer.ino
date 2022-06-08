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

// Create a buffer for FIFO data
const uint8_t numSamples = 5;
bmp5_sensor_data fifoData[numSamples];

// Track FIFO length to give progress updates
uint8_t previousFIFOLength = 0;

void setup()
{
    // Start serial
    Serial.begin(115200);
    Serial.println("BMP581 Example6 begin!");

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
    // For all possible ODR settings, see 5_defs.h (line 282-313)
    err = pressureSensor.setODRFrequency(BMP5_ODR_01_HZ);
    if(err != BMP5_OK)
    {
        // Setting ODR failed, most likely an invalid frequncy (code -12)
        Serial.print("ODR setting failed! Error code: ");
        Serial.println(err);
    }

    // Configure the FIFO buffer to store pressure and temperature data
    // Note - this is also where the FIFO interrupt conditions are set
    // Note - this must be called before any other FIFO functions
    bmp5_fifo fifoConfig = {0};
    fifoConfig.frame_sel      = BMP5_FIFO_PRESS_TEMP_DATA; // Sensor data to store in FIFO buffer
    fifoConfig.dec_sel        = BMP5_FIFO_NO_DOWNSAMPLING; // Downsampling
    fifoConfig.mode           = BMP5_FIFO_MODE_STREAMING;  // Overwrite or stop writing once FIFO is full
    fifoConfig.threshold      = numSamples;                // Number of frames for threshold interrupt to occur
    fifoConfig.set_fifo_iir_t = BMP5_DISABLE;              // Filter temperature data
    fifoConfig.set_fifo_iir_p = BMP5_DISABLE;              // Filter pressure data
    err = pressureSensor.setFIFOConfig(&fifoConfig);
    if(err != BMP5_OK)
    {
        // FIFO settings failed, most likely a communication error (code -2)
        Serial.print("FIFO settings failed! Error code: ");
        Serial.println(err);
    }

    // Configure the BMP581 to trigger interrupts when the threshold number of samples is reached
    BMP581_InterruptConfig interruptConfig =
    {
        .enable   = BMP5_INTR_ENABLE,    // Enable interrupts
        .drive    = BMP5_INTR_PUSH_PULL, // Push-pull or open-drain
        .polarity = BMP5_ACTIVE_HIGH,    // Active low or high
        .mode     = BMP5_PULSED,         // Latch or pulse signal
        .sources  =
        {
            .drdy_en = BMP5_DISABLE,      // Trigger interrupts when data is ready
            .fifo_full_en = BMP5_DISABLE, // Trigger interrupts when FIFO is full
            .fifo_thres_en = BMP5_ENABLE, // Trigger interrupts when FIFO threshold is reached
            .oor_press_en = BMP5_DISABLE, // Trigger interrupts when pressure goes out of range
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
    // Variable to track errors returned by API calls
    int8_t err = BMP5_OK;

    // Get number of data samples currently stored in FIFO buffer
    uint8_t currentFIFOLength = 0;
    err = pressureSensor.getFIFOLength(&currentFIFOLength);
    if(err != BMP5_OK)
    {
        // FIFO length failed, most likely a communication error (code -2)
        Serial.print("FIFO length failed! Error code: ");
        Serial.println(err);

        // If getFIFOLength() failed this time, it will most likely fail next time. So
        // let's wait a bit before trying again
        delay(1000);
    }

    // Check whether number of samples in FIFO buffer has changed
    if(previousFIFOLength != currentFIFOLength)
    {
        // Update FIFO length
        previousFIFOLength = currentFIFOLength;

        // Print current FIFO length
        Serial.print("FIFO Length: ");
        Serial.print(currentFIFOLength);
        Serial.print("/");
        Serial.println(numSamples);

        // If the buffer length goes beyond the watermark level, then an
        // interrupt was missed. This example will likely run into issues,
        // so we'll just clear the FIFO buffer
        if(currentFIFOLength > numSamples)
        {
            Serial.println("Too many samples in FIFO buffer, flushing...");
            
            err = pressureSensor.flushFIFO();
            if(err != BMP5_OK)
            {
                // FIFO flush failed, most likely a communication error (code -2)
                Serial.print("FIFO flush failed! Error code: ");
                Serial.println(err);
            }
        }
    }

    // Wait for interrupt to occur
    if(interruptOccurred)
    {
        // Reset flag for next interrupt
        interruptOccurred = false;

        Serial.println("Interrupt occurred!");

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

        // Make sure this is the "FIFO watermerk" interrupt condition
        if(interruptStatus & BMP5_INT_ASSERTED_FIFO_THRES)
        {
            // Get FIFO data from the sensor
            err = pressureSensor.getFIFOData(fifoData, numSamples);
            if(err < BMP5_OK)
            {
                // FIFO data get failed, most likely a communication error (code -2)
                Serial.print("Get FIFO data failed! Error code: ");
                Serial.println(err);
                return;
            }
            if(err > BMP5_OK)
            {
                // FIFO data get warning, most likely min/max pressure/temperature (codes 3/4/5/6)
                // This is likely to occur on some systems (eg. Arduino Uno)
                // when numSamples is large (eg. >= 5)
                Serial.print("Get FIFO data warning! Error code: ");
                Serial.println(err);
            }

            // Data was acquired successfully, print it all out
            for(uint8_t i = 0; i < numSamples; i++)
            {
                Serial.print("Sample number: ");
                Serial.print(i);
                Serial.print("\t\t");
                Serial.print("Temperature (C): ");
                Serial.print(fifoData[i].temperature);
                Serial.print("\t\t");
                Serial.print("Pressure (Pa): ");
                Serial.println(fifoData[i].pressure);
            }
        }
        else
        {
            Serial.println("Wrong interrupt condition!");
        }
    }
}

void bmp581InterruptHandler()
{
    interruptOccurred = true;
}