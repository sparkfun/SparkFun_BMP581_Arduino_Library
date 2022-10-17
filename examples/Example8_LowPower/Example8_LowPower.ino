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

void setup()
{
    // Start serial
    Serial.begin(115200);
    Serial.println("BMP581 Example8 begin!");

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

    // Calling beginI2C() puts the sensor into normal mode, but we want the sensor to
    // sleep for the majority of the time to minimize power consumption. The BMP581
    // support 2 sleep modes, namely standby (1 uA) and deep standby (0.55 uA).
    // Note - setting deep standby will affect the sensor configuration, see the
    // datasheet for more information
    err = pressureSensor.setMode(BMP5_POWERMODE_DEEP_STANDBY);
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
}

void loop()
{
    // Variable to track errors returned by API calls
    int8_t err = BMP5_OK;

    // Wait until next measurement. For low power applications, this could be
    // replaced by setting the microcontroller into a sleep state
    delay(1000);

    // Transition from sleep mode into forced mode. This will trigger a single
    // measurement, after which the sensor automatically returns to sleep mode
    // Note - the sensor can only enter forced mode from sleep mode. Transitions
    // between forced and normal modes are ignored
    pressureSensor.setMode(BMP5_POWERMODE_FORCED);
    if(err != BMP5_OK)
    {
        // Set mode failed, most likely a communication error (code -2)
        Serial.print("Set mode failed! Error code: ");
        Serial.println(err);
    }

    // Wait for measurement, with a timeout period just in case
    uint32_t t0 = millis();
    uint32_t timeout = 1000;
    while(!interruptOccurred)
    {
        // Check whether timeout has occurred
        if((millis() - t0) > timeout)
        {
            // Measurement has taken too long to finish. There's likely a
            // problem somewhere, so we'll just abort this measurement
            Serial.println("Measurement timeout occurred!");
            return;            
        }
    }

    // Measurement has finished, reset flag for next interrupt
    interruptOccurred = false;

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
        bmp5_sensor_data data = {0,0};
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
    else
    {
        Serial.println("Wrong interrupt condition!");
    }
}

void bmp581InterruptHandler()
{
    interruptOccurred = true;
}