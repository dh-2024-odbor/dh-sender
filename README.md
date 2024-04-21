# Hopper sender

The entry point of the firmware is in the `main.c` file.
It includes an `app_main` function which is first function that gets called
when the device boots up.

It contains initialization code for all the sensors and the LoRa radio
as well as the main deep_sleep_task which handles putting the devices into deep sleep
after taking a sensor reading and transmitting it via LoRa radio.

The `lora.c` file contains initialization code for the LoRa radio,
while the `bme.c` file contains initialization code for the BME280 sensor.

The firmware is developed using ESP-IDF v5.1-dirty (stable) (Espressif IoT Development Framework)
which is standard for the ESP32 series of microcontrollers that we use.

The source code is well documented, so if you have a burning desire
for more information feel free to read it.

There is a `components` folder includes the driver code for LoRa device
and the BME280 device in their respective folders.

Finally the `sdkconfig` file includes all the configuration for the esp32 device.
Some of this information is included in the bootloader binary to make the microcontroller
function properly, some is used merely as configuration for interfacting with the device
via serial. 

## Overview

This is the LoRa sender device functional overview. 

It is in charge of taking a measurement
(in our example temperature, humidity and pressure, but theoretically any sensor could be added)
and transmitting it via the LoRa radio.

The device firmware is designed with power consumption in mind to
allow the device to run for years on end without needing any kind of 
human to service them.

They are designed to be deployed in remote areas
where the standard networks like the GSM are not available.
The LoRa radio facilitates incredibly long range and low power (provided the radio is
configured correctly) which makes it perfect for this kind of use case.

It (the LoRa radio) supports sending any arbitrary binary data.
We have defined a standard message format though which enforces
the node_id to be the first 4 bytes of the message
and the message_id to be the next 4 bytes. Data after that
is freely interpretable.

The interpretation format is defined on our server
and linked to the node id, so when a server
receives the lora packet it interprets the data
based on the node id and stores it for further processing
and agregation.

Message IDs are based on the number of times the device has been woken up from
deep sleep (the counter starts at 1, zero is reserved for deduplication purposes). The counter is read, used as a message id, incremented by one and then stored back.

It is stored in the permanent storage (NVS) of the esp device which preserves
the id between resets, reboots or unexpected power loses.
This mechanism makes sure that the message IDs remain unique for a specific node
and consqeuntly unique within the network in combination with node_id.

## Implementation details

To achieve such low power consumption, we have programmed the
esp32 to switch between two operating modes. Deep sleep mode and the
normal operation mode.

In deep sleep (also called hibernation mode, provided more ICs are turned off) the only IC
on the esp32 left running is the RTC timer. The LoRA radio is also put into sleep mode
as well as the temperature sensor.

The RTC timer can be configured as a wakeup source for the main CPU.
This timer needs to be configured with a time period at which it sends a wakeup signal
to the main CPU.

When the signal is received, the device is woken up and enters normal operation mode.
In normal operation mode, the LoRa radio, BME280 temperature sensor and the MCU are turned
on (put into normal operation mode) to perform the readings and transmit them and then immediately put back to sleep.

Since we configured the RTC timer wakeup source, the device will be woken up again
and the cycle continues.

## Power consumption

### SX1278 LoRa transceiver

* Sleep mode - 0.27 uA
* Receive mode - 11.5 mA
* Transmit mode (active transmission with tx power 13dBm without boost) - 29 mA

### BME280 

Note: This is an example of a measure device. This system can support any type of measure devices power would then need to be recalculated depending on devices used.

* Sleep mode - 0.1 uA
* Operating mode (1Hz; Temperature, Humidity and Pressure) - 3.6 uA

### Main board - ESP32-VROOM-32D

Note: This board is completely unnecessary it has many features that are not in use by our system but this is what we could get in such a short amount of time. In reality power consumption by the main board would me much lower.

* Sleep mode (ULP Coprocessor turned disabled) - 2.5 uA
* Sleep mode (with ULP Coprocessor enabled) - 10 uA
* Operating mode (with unnecessary features like Wi-Fi and Bluetooth radios turned off, ESP Core and some peripherals are activeA) - 30 mA (pessimistic estimation, official docs say 20mA but with peripherals off)

### Overall calculations

Consumption while sleeping: 0.27 uA + 0.1 uA + 2.5 uA = 2.87 uA
Consumption while operational (and transmitting): 29 mA + 3.6 uA + 30 mA = (approx) 59 mA

Time sleeping (per cycle): 20 seconds
Time awake (per cycle): 0.5 seconds

Power consumption per cycle (20.5 seconds): 20 * 0.00287 + 0.5 * 59 = 29.5574 mA / 20.5 seconds

Power consumption per second = 29.5574 / 20.5 = 1.4418 mAs

This equates to around 291 days.

But because power consumption is so low (practically 0) when the devices are in deep sleep, it GREATLY improves overall consumption if we wait longer in deep sleep state.

For example if we just raise the interval between measurements from 20 seconds to 1 minute (60) seconds we get:




