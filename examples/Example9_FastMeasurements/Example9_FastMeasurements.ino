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

// Create a timer to print number of measurements every second
uint32_t printPeriod = 1000;
uint32_t lastPrintTime = 0;

// Track number of measurements
uint16_t measurementsThisPeriod = 0;

void setup()
{
    // Start serial
    Serial.begin(115200);
    Serial.println("BMP581 Example9 begin!");

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

    // Calling beginI2C() puts the sensor into normal mode, where the sensor makes
    // regular measurements defined by the ODR. However the BMP581 also supports
    // continuous mode, where measurements are performed as soon as the previous
    // measurement finishes. With only 1x oversampling, the sensor can preform
    // measurements up to 500Hz
    err = pressureSensor.setMode(BMP5_POWERMODE_CONTINOUS);
    if(err != BMP5_OK)
    {
        // Interrupt settings failed, most likely a communication error (code -2)
        Serial.print("Set mode failed! Error code: ");
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
            .oor_press_en = BMP5_DISABLE   // Trigger interrupts when pressure goes out of range
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

    // Start print timer
    lastPrintTime = millis();
}

void loop()
{
    // Check whether it's time to print
    if(millis() > lastPrintTime + printPeriod)
    {
        // Print out number of measurements this period
        Serial.print("Number of measurements in ");
        Serial.print(printPeriod);
        Serial.print("ms: ");
        Serial.println(measurementsThisPeriod);

        // Reset number of measurements for next period
        measurementsThisPeriod = 0;

        // Increment last print time
        lastPrintTime += printPeriod;
    }

    // Wait for interrupt to occur
    if(interruptOccurred)
    {
        // Reset flag for next interrupt
        interruptOccurred = false;

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
            // Increment number of measurements this period
            measurementsThisPeriod++;
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