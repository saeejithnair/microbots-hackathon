******************************************************************************
Webpage URL: https://microbots.io/blogs/learn/codecell-basics-your-first-steps
CodeCell Basics: Your First Steps

If you’ve just got your hands on the CodeCell, you're in for a treat. This tiny module is designed to simplify your DIY projects with multiple features packed into a penny-sized board. In this guide, we’ll walk you through:

What makes a CodeCell? And how does it circuit work?
How to setup and use your CodeCell
Getting started with Examples
Explain all the available library functions and how you can use them in your project

What makes a CodeCell?
CodeCell is a compact and versatile module featuring the ESP32-C3, multiple power options, and integrated sensors, all within a tiny 1.85 cm wide form factor. These features make it a powerful tool for a wide range of applications.

In this first section, we'll start by getting familiar with the circuitry that forms the CodeCell. After that, we'll walk through the simple steps to set up your CodeCell.

ESP32C3 Module

At the heart of the CodeCell is the ESP32C3 module, a compact microcontroller known for being maker-friendly in the IoT space. It combines an Arduino-compatible architecture with built-in Wi-Fi and Bluetooth Low Energy (BLE) capabilities. This integration offers the most popular connectivity options while maintaining a small form factor.

The ESP32C3 module's PCB antenna is positioned on one side, away from other components, to minimize interference and improve signal transmission and reception. This placement helps reduce the impact of ground planes or other conductive surfaces that could degrade antenna performance. The components on the bottom side are kept within the recommended clearance for the antenna. From testing we found that the antenna's performance remains unaffected by the minimal interference from a USB-C cable, as these cables are typically shielded.

The ESP32-C3 provides plenty of memory, with 4 MB of Flash and 400 KB of SRAM, making it capable of running most typical applications. Its 32-bit RISC-V single-core processor, running at up to 160 MHz, efficiently handles various tasks. This combination of memory and processing power makes the ESP32-C3 suitable for a wide range of uses.

The ESP32C3 module also supports a USB Serial/JTAG Controller, allowing us to make the CodeCell reflashable through the USB-C port and to send serial data for communication and debugging.

Power Management

The CodeCell offers flexibility in power supply options. It can be powered through the LiPo battery connector, a USB-C cable, or both.

The LiPo battery connector makes it easier than ever to safely connect the battery without the need for soldering or risking accidental shorting it. 

The USB-C port serves dual purposes: it is used for both powering the device and/or reprogramming it. This multi-power option is enabled through the BQ24232 battery management chip, which features dynamic power-path management (DPPM) that can power the system while simultaneously and independently charging the battery. The battery charging process is managed in three phases: conditioning precharge, constant current, and constant voltage. To protect the battery the output voltage (Vo) is regulated though the BQ24232 chip. This output supports a maximum output current of 1500mA when powered by the LiPo battery and 450mA when powered via USB.


By default, the LiPo battery charge current is set to 90mA, ensuring a balanced and a safe charge rate for the optional 170mAh LiPo battery. Further more, for those who wish to adjust the charging rate, 0402 resistor R12 have to be de-soldered and replace it with a new resistor based on the formula (R = 870/Ichrg). This is only recommended for soldering pros, who aren’t afraid of wrestling with tiny 0402 components! Check the BQ24232 datasheet for more information on the battery charging.

The CodeCell library can provides visual feedback on the battery/usb power status via the onboard addressable RGB LED:

Low Battery Warning: When the battery voltage drops below 3.3V, the LED blinks red ten times and the device enters Sleep Mode. This helps conserve power until the device is reconnected to a USB charger.
Charging Process: During charging, the CodeCell suspends application processes, lights the LED blue, and waits for the battery to reach full charge. Once fully charged, the LED performs a breathing-light animation, indicating proximity distance detected by the sensors.
Battery Powered: When the USBC is disconnected and running from the battery power the CodeCell will light green again performing a breathing-light animation, indicating proximity distance detected by the sensors.
The power regulation is further supported by multiple decoupling capacitors, including up to two bulk capacitors of 100µF each, placed next to the battery-connector. These capacitors are connected to the 3.3V and the output Vo pins to ensure stable power delivery. Additionally, the board features two TVS diodes for protection; one safeguards the USB input 5V voltage (Vin), and the other protects the output voltage (Vo). These TVS diodes provide protection against electrostatic discharges (ESD), capable of safely absorbing repetitive ESD strikes above the maximum level specified in the IEC 61000-4-2 international standard without performance degradation.

The board also includes an onboard 3.3V Low Dropout (LDO) regulator, which provides a stable power supply to its low-voltage components. This tiny NCP177 LDO chip can output up to 500mA output current with a typically low dropout voltage of 200mV at 500mA.


GPIO and Power Pins

Given the compact design, the main challenge was to maximize the use of GPIO pins. We tackled this by dividing each of the three available sides of the CodeCell into different I/O sections based on their applications. We also placed power pins along the edges of the module for easy connection to various power sources, allowing you to connect other modules, sensors, and actuators to different sides.

On the bottom side, 3 out of 5 pins are used for power: a ground pin (GD), a 3.3V logic-level power pin (3V3) and a 5V input charge pin (5V0). This 5V0 pin is connected to the USB input-voltage. This means you can use it to get 5V power when the USB is connected, or you can use it as a power input for charging instead of using the USB.. The other 2 pins are the I2C SDA & SCL pins for adding external digital sensors. If your not using any external and the light/motion sensors, these I2C pins can be set up as GPIOs.

The other two sides each have a ground pin (GD) and a voltage output pin (VO). Each side also features 3 programmable GPIO pins (IO1, IO2, IO3, IO5, IO6, IO7), which can all be configured as PWM pins (ideal for directly connecting an h-bridge for actuator/motor control). IO1, IO2, and IO3 can also be used as ADC pins.

Sensing Capabilities

The CodeCell's standout features include its onboard sensors. Each unit comes equipped with a built-in light sensor, and there's also an optional motion sensor available to elevate your project's motion detection—especially useful for robotics and wearables!


VCNL4040 Light Sensor: This sensor measures both light levels and proximity up to 20 cm. It features a 16-bit high-resolution design that combines a proximity sensor, an ambient light sensor, and a high-power IRED into a compact package. By integrating photodiodes, amplifiers, and an analog-to-digital converter onto a single chip, it provides enhanced functionality. The I2C configuration is directly embedded in the CodeCell library, ensuring the sensor is automatically initialized to optimize its sensing resolution. 
Optional 9-axis BNO085 Motion Sensor: This advanced IMU sensor is a pricey upgrade, but we believe it's well worth the investment! It upgrades the CodeCell’s capabilities with an integrated 3-axis accelerometer, 3-axis gyroscope, and 3-axis magnetometer + the BNO085’s advanced sensor fusion algorithms combines the data from these sensors to accurately determine the motion data of the sensor, like: Angular Rotational Reading (Roll, Pitch, Yaw), Motion State (e.g., On Table, Stationary, Motion), Motion Activity (e.g., Driving, Walking, Running), Accelerometer Readings, Gyro Reading, Magnetometer Reading, Gravity Reading, Linear Acceleration Reading, Tap detection, Step Counter. 
Next we'll dive into how the CodeCell library simplifies both configuring these sensors and reading their data.


What about the BOOT Pin?

Some ESP32 development boards include both a RST (Reset) button and a BOOT button to manually put the device into programming mode. However, the ESP32-C3, such as the one on the CodeCell module, can automatically enter boot mode through the serial interface when using the Arduino IDE. This means the CodeCell doesn't need dedicated RST or BOOT buttons, which allowed us to make it as small as it is.

In the rare case that your CodeCell freezes or encounters an exception (causing it to continuously reset), you can manually force it into boot mode to reflash the firmware. To do this, simply connect a wire between the SCL pin and the GND pin. After that, unplug the USB or disconnect the battery, then reconnect it. This will reset the device and allow it to enter boot mode, enabling you to reprogram your CodeCell.

How to setup and use your CodeCell?
To make programming even easier, the CodeCell library provides a wide array of functions for initializing, reading, and managing sensors and power. In this section we're going to explain everything you need to know about setup up your device and it's library.

Unboxing Your CodeCell

Let’s start with what you’ll find inside the box. Depending on the options you selected during checkout, in the box you'll find:

CodeCell: The star of the show, your tiny yet mighty board featuring the ESP32-C3 module, programmable GPIO pins, and sensors.
Screws: Four M1.2 x 6mm screws to mount the CodeCell securely in your projects.
Headers: Three sets of female headers which can come unsoldered or soldered, depending on your choice.
Battery/Cable: Depending on your selection during checkout, you’ll either receive a free battery cable for connecting your own battery to the CodeCell's onboard connector or a 170mAh 20C LiPo battery with a pre-soldered cable. This optional battery measures 23 x 17.5 x 8.7 mm and weighing just 4.6 grams. Click here if you like to access the battery's datasheet.
Powering Up Your CodeCell for the First Time

Let's start by plugging in a USB-C cable! Once your CodeCell receives power it should: 

Initialization: It sets up the internal peripherals and configures the onboard sensors. Once this is being performed it outputs a Hello World message on the Serial Monitor. 
Power Check: It monitors the power status, check if the battery is connected and whether it's charging. If no battery is connected for charging, it will run a breathing light animation with the onboard RGB LED. The animation speed changes based on the proximity of the light sensor ~ Bring your hand close to it to slow down.. Move your hand away to speed it back up! The LED flashes blue or green depending on whether the board is powered by USB or battery. If the battery is charging, the LED stays static blue until fully charged.
Setting Up Your CodeCell

Next step is to connect the CodeCell to Arduino IDE and run a sketch:

USB Connection: Connect your CodeCell to your PC using a standard USB-C cable. This cable not only powers the board but also allows for reprogramming.
Install the Arduino IDE: If your new to the Arduino-World no worries, just download and install the latest free version of the Arduino IDE software from the official Arduino website. 
Add ESP32 Board Manager URL: If you have never used an ESP32, open the Arduino IDE and navigate to 'File > Preferences'. In the 'Additional Board Manager URLs' field, enter: `https://dl.espressif.com/dl/package_esp32_index.json` and then click ok. Then go to 'Tools > Board > Boards Manager'. Search for 'ESP32' and click 'Install'.
Select the Board: The next step is to select our board. Head to 'Tools > Board' and choose 'ESP32C3 Dev Module'. In a few weeks, you'll be able to search directly for 'CodeCell', but for now, selecting 'ESP32C3 Dev Module' works perfectly fine, since it's the microcontroller used onboard the CodeCell.
Select Port: Go to 'Tools > Port' and select the COM port corresponding to your CodeCell.
Other Settings:  Navigate to 'Tools > USB_CDC_On_Boot' and ensure it's enabled if you plan to use the Serial Monitor. Also, set the clock speed to 160MHz.
With your IDE all set up, we can now go ahead an install the "CodeCell" library. To do this go to 'Sketch>Include Library>Manage Libraries' - the 'Library Manager' should open up. Just type "CodeCell" and click 'Install' to download the latest version of the CodeCell.

We are continuously updating and adding new features to this library, so make sure you're using the latest version.

To quickly get familiar with this library, go to 'File > Examples > CodeCell,' where you'll find multiple examples you can use and modify for your projects. We recommend starting with the 'GettingStarted' example, which contains just a few lines of code but explains all the sensing functionalities available with CodeCell.

Once you select an example sketch, click the 'Upload' button to flash the code onto your CodeCell. After uploading, open the Serial Monitor 'Tools > Serial Monitor' to see serial data from your CodeCell.

Here are some additional CodeCell tutorials to help you get started with various applications:

Depth Gestures
Tap Detection
Proximity 
Auto Dimming
Step Counter
Angle Control
Personal Activity Guessing
Wireless Remote
Ai Prompt
Alexa Light Control
 CodeCell Library Functions

To explore the code further, let's break down all the functions and explain what each one does:

Initializing CodeCell

The first step in using the CodeCell is to initialize it. This is done using the `myCodeCell.Init()` function, which allows you to specify the sensors you want to enable.

Available Sensing Macros:

LIGHT - Enables Light Sensing.
MOTION_ACCELEROMETER - Enables Accelerometer Sensing.
MOTION_GYRO - Enables Gyroscope Sensing.
MOTION_MAGNETOMETER - Enables Magnetometer Sensing.
MOTION_LINEAR_ACC - Enables Linear Acceleration Sensing.
MOTION_GRAVITY - Enables Gravity Sensing.
MOTION_ROTATION - Enables Rotation Sensing.
MOTION_ROTATION_NO_MAG - Enables Rotation Sensing without Magnetometer.
MOTION_STEP_COUNTER - Enables Walking Step Counter.
MOTION_STATE - Enables Motion State Detection.
MOTION_TAP_DETECTOR - Enables Tap Detector.
MOTION_ACTIVITY - Enables Motion Activity Recognition.


You can combine multiple macros using the `+` operator to initialize multiple sensors at once.

Managing Power

The `myCodeCell.Run()` function is crucial for power management. This function should be called within the `loop()` function to handle battery status and ensure optimal power usage.

Function Behavior:

The function returns `true` every 100ms, which can be used for time-based operations.
It controls the onboard LED to indicate different power states:
Red Blinking (Blinking 10 times) - Battery voltage below 3.3V, entering Sleep Mode.
Green LED (Breathing Animation) - Powered by battery
Blue LED (Static) - Battery is Charging
Blue LED (Breathing Animation) - Fully charged and ready for operation.
Reading Sensor Data

After initializing the sensors, you can read their data using various functions provided by the library. Here's a quick rundown of the available functions:

Light Sensor Functions:

Light_ProximityRead() - Reads the proximity value from the light sensor.
Light_WhiteRead() - Reads the white light intensity from the light sensor.
Light_AmbientRead() - Reads the ambient light intensity from the light sensor.
Motion Sensor Functions:

Motion_AccelerometerRead(float &x, float &y, float &z) - Reads acceleration data along the x, y, and z axes.
Motion_GyroRead(float &x, float &y, float &z) - Reads rotational velocity data along the x, y, and z axes.
Motion_MagnetometerRead(float &x, float &y, float &z) - Reads magnetic field strength data along the x, y, and z axes.
Motion_GravityRead(float &x, float &y, float &z) - Reads gravity vector data along the x, y, and z axes.
Motion_LinearAccRead(float &x, float &y, float &z) - Reads linear acceleration data along the x, y, and z axes.
Motion_TapRead(uint16_t &x) - Reads the number of taps detected.
Motion_StepCounterRead(uint16_t &x) - Reads the number of steps counted.
Motion_RotationRead(float &roll, float &pitch, float &yaw) - Reads angular rotational data (roll, pitch, yaw).
Motion_RotationNoMagRead(float &roll, float &pitch, float &yaw) - Reads angular rotational data without using the magnetometer.
Motion_StateRead(uint16_t &x) - Reads the current state (e.g., On Table, Stationary, Motion).
Motion_ActivityRead(uint16_t &x) - Reads the current activity (e.g., Driving, Walking, Running).
Example Usage:



Sleep, Power-Saving, Diagnostic & LED Functions

The CodeCell includes several functions to manage sleep and power-saving modes:

WakeUpCheck() - Checks the wake-up reason of the device. If the device wakes up from a timer event, it returns `true`; otherwise, it returns `false`
Sleep(uint16_t sleep_sec) - Puts the device into deep sleep mode for a specified duration in seconds. It configures the necessary pins and sensors for low power consumption before entering sleep mode.
USBSleep(bool cable_polarity) - Manages the device's sleep mode when the battery level is low or when the device is charging via USB power. It shuts down the application and prepares the device for sleep to allow reprogramming if needed.
PrintSensors() - Prints the current readings from all initialized sensors to the serial monitor. This function is especially useful for debugging and data logging.
BatteryRead() - This function reads and returns the battery voltage when the USB-C port is disconnected.
LED_Breathing(uint32_t rgb_color_24bit) - This function is used in the Run() handler to control the onboard LED & create a breathing effect with a specific color. The `rgb_color_24bit` parameter is a 24-bit color value representing the RGB color. When using this function be careful as the 'Run' function might overwrite your LED color.
LED(uint8_t r, uint8_t g, uint8_t b) - This function sets the color of the onboard addressable LED using the RGB model where `r`, `g`, and `b` are 8-bit values representing the red, green, and blue components of the color. Each component ranges from 0 to 255. When using this function be careful as the 'Run' function might overwrite your LED color.
Congratulations!
You've now taken your first steps with CodeCell. Dive deeper into the library examples, explore sensor integrations, and start bringing your innovative projects to life with CodeCell!

******************************************************************************
Webpage URL: https://microbots.io/products/codecell?variant=49714638717261
https://github.com/microbotsio/CodeCell
Sensing
Get started with sensing light, proximity, motion & activity tracking.

CodeCell features an Arduino-compatible ESP32-C3 with both WiFi and BLE, a VCNL4040 light sensor, and an optional BNO085 9-axis motion-fusion sensor. We made these sensors beginner-friendly with a simple library and easy-to-follow examples.


Incredibly Small!
It's just 1.85 cm wide, with 8 GPIO pins, powered by USB-C or battery.

The USB-C port allows you to easily reprogram the CodeCell, read serial data, and recharge the LiPo battery. The Software Library can directly manage the power and battery, indicating the status with the onboard RGB LED.


Specifications
Electrical:

Processor: ESP32-C3 32-bit RISC-V Single-Core
Memory: 4 MB (Flash) 400 KB (SRAM)
Clock Speed: 160MHz
Average Sleep Current: 689μA
LiPo Battery Charge Current: 90mA
Maximum Output Current: 1500mA (Battery) / 450mA (USB)
Mechanical:

Dimensions: 9.4mm H x 18.5mm L x 18.5mm W (+5.2mm antenna)
Castellated Pins Pitch: 2.54mm
Screws: M1.2 (included)
Weight: 3.4 grams
3D Model

Notes:

The CodeCell is a compact 1.85 cm wide module featuring an Arduino-compatible ESP32-C3 that offers both Wi-Fi and BLE connectivity. It provides 6 programmable GPIO pins, along with 2 I2C pins that can be reconfigured as GPIOs if not needed for communication. The module also has multiple power pins for connecting additional modules, sensors, and actuators. CodeCell can be powered through a LiPo battery connector or via USB-C, which is also used for reprogramming and charging the battery. The power management system of the CodeCell is built around the BQ24232 battery management chip, which enables dynamic power-path control. This allows the battery to charge while the system continues to operate. The charging process is broken into preconditioning, constant current, and constant voltage phases. By default, the charge current is set to 90mA for the optional 170mAh LiPo battery. The RGB LED provides clear visual indications of power and charging states, with animations showing red for low battery, green for battery power, and blue when charging/powered via USB. The CodeCell includes a VCNL4040 light sensor, allowing it to measure ambient light and proximity up to 20 cm. For more advanced projects, an optional BNO085 motion sensor is available, which adds 9-axis sensing capabilities. This is an expensive sensor but combines a 3-axis accelerometer, 3-axis gyroscope, and 3-axis magnetometer, using sensor fusion algorithms to provide detailed motion data such as angular readings (roll, pitch, yaw), motion state, personal activity guessing, linear acceleration, tap detection, and even step counts. This feature makes the CodeCell especially suitable for robotics and wearable applications. To make interacting with the sensors easier, the 'CodeCell.h' library provides easy-to-use functions and multiple examples. Use the Init() function to configure the module and enable its sensing features, and the Run() function in the main loop to handle power management tasks automatically. The CodeCell will ship with a default software that initializes the light sensor, runs the power management, and displays a breathing-light animation controlled by proximity sensing. Check out this tutorial to easily get started with CodeCell.
The box will include the CodeCell, a set of four M1.2 screws and 3 set of female headers (soldering is optional). It will also include a battery-cable (1.25mm pitch) or the optional battery.
The optional 170mAh 20C LiPo battery measures 23 x 17.5 x 8.7 mm and weight 4.6 grams. It has a 1.25mm female-connector wired to it, which can be plugged directly to the onboard connector. Click here for the full battery datasheet.
Schematics for this module is available here.
This PCB is ROHS Compliant & follows IPC-A-600 II standard.
Please note that the CodeCell is intended to be used as a DIY maker kit. For commercial purposes, please get in touch.

******************************************************************************
Webpage URL: https://microbots.io/products/drivecell
Key Features

Ultra Small
1 cm x 0.8cm (0.39 x 0.31 inch)
That's as small as it can get!


Easy to Solder
Castellated pins makes it simple to solder it with wires, headers & on-top of other Circuits


Drive More!
Use the same input signal to drive multiple motors & actuators ~ Just keep it under 1.8A


Specifications
Electrical:

Input Maximum Voltage: 5V
Output Maximum Current: 1.8Amps
Mechanical:

Dimensions: 10x8mm
Thickness: 1.8mm
Weight: 0.16 grams
Screws: M1.2
Connector Pitch: 2.54mm
3D Model
Notes:

The DriveCell is a tiny h-bridge module with the DRV8837 chip. It has 2 input pins which are typically driven in opposite polarities, to control the output current direction. This allows the CoilPad/FlatFlap to be driven magnetically north or south, or a motor to be rotate clockwise or anticlockwise.
The DRV8837 chip has Overcurrent Protection, Undervoltage Lockout & Thermal Shutdown. Caution must be taken not to reverse its supply rails.
Schematics for this module is available here.
This electronics kit is ROHS Compliant & follows IPC-A-600 II standard.
The box will include the DriveCell, a set of two M1.2 screws and an individual (unsoldered) 4-pin male header.
Please note that the DriveCell is intended to be used as a DIY maker kit. For commercial purposes, please get in touch.

******************************************************************************
Webpage URL: https://microbots.io/blogs/learn/codecell-wifi-remote
CodeCell: Wifi Remote

In this guide, we will explore how to configure the CodeCell's ESP32-C3 to be used as a WiFi remote, communicating between two devices. 

The CodeCell's ESP32-C3 comes with WiFi capability, allowing it to communicate wirelessly. Using ESP-NOW, we can establish direct device-to-device communication with minimal setup. This guide will demonstrate how to pair two devices, using one as a sensor-based remote and the other to receive and act on the transmitted data.


What You'll Learn
How to set up two CodeCell devices for communication using WiFi and ESP-NOW protocol.
How to use one CodeCell to control the onboard LED of another device using proximity sensing.
How to send angular readings from one CodeCell device to another, adjusting motor speed accordingly.

Project Overview
In this example, we will pair two CodeCell devices. Device 1 will gather sensor data and send it to Device 2 via WiFi using the ESP-NOW protocol. We'll start with a simple setup where the proximity sensor on Device 1 controls the onboard LED on Device 2. In the second example, Device 1 will send angular data, and Device 2 will process the data to adjust motor speeds.

Getting the MAC Address of the Receiver
Before you can establish communication between the two CodeCell devices, you first need to get the MAC address of the receiver. This MAC address will be used in the sender's code to ensure the correct device receives the data.

Follow these steps to obtain the MAC address of the receiver:

First, upload the receiver (Device 2) code to your CodeCell. This intialize the Wifi and print its unique MAC address to the serial monitor.
After uploading the code to Device 2, open the Arduino's Serial Monitor. You should see the MAC address printed once the device is initialized.
The MAC address will be printed in the format XX:XX:XX:XX:XX:XX. Copy this address, as you'll need it for the sender's code.
Replace the placeholder MAC address in the sender's code (Device 1) with the MAC address you just obtained from Device 2. This will ensure that Device 1 sends data to the correct receiver.
Example 1: Controlling the LED Remotely with Proximity Sensing
This example demonstrates how to send proximity sensor data from Device 1 to Device 2, which will use the data to turn on or off its onboard LED.

Device 1 (Sender)

#include <esp_now.h>
#include <WiFi.h>
#include <CodeCell.h>

CodeCell myCodeCell;
uint8_t receiverMAC[] = { 0xXX, 0xXX, 0xXX, 0xXX, 0xXX, 0xXX };  // Replace with receiver's MAC address

void setup() {
  Serial.begin(115200);
  myCodeCell.Init(LIGHT);  // Initializes Light Sensing

  // Initialize WiFi in Station mode
  WiFi.mode(WIFI_STA);
  Serial.println(WiFi.macAddress());

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register peer
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, receiverMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
}

void loop() {
  if (myCodeCell.Run()) {
    uint16_t ProxRead = (myCodeCell.Light_ProximityRead()) >> 4;  // Get Proximity Value and Divide by 16
    Serial.println(ProxRead);
    esp_err_t result = esp_now_send(receiverMAC, (uint8_t *)&ProxRead, sizeof(ProxRead));

    if (result == ESP_OK) {
      Serial.println("Data sent successfully");
    } else {
      Serial.println("Sending Error");
    }
  }
}
Device 2 (Receiver)

#include <esp_now.h>
#include <WiFi.h>
#include <CodeCell.h>

CodeCell myCodeCell;

void setup() {
  Serial.begin(115200);
  myCodeCell.Init(LIGHT);  // Initializes Light Sensing

  // Initialize WiFi in Station mode
  WiFi.mode(WIFI_STA);
  Serial.println(WiFi.macAddress());

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register the receive callback
  esp_now_register_recv_cb(onDataRecv);
}

// Receive callback function
void onDataRecv(const esp_now_recv_info *recvInfo, const uint8_t *incomingData, int len) {
  uint16_t Remote_Value;
  memcpy(&Remote_Value, incomingData, sizeof(Remote_Value));
  Serial.println(Remote_Value);
  myCodeCell.LED(0, Remote_Value, 0);  // Control onboard LED brightness
}

void loop() {
  // Nothing to do here
}
Example 2: Sending Angular Data to Control Motor Speed
In this second example, we connect two motors with a two DriveCells to the receiver. Device 1 reads angular data from its motion sensors and sends it to Device 2, which adjusts the speed of two motors based on the received data. 

If you are using different devices for this example, remember to read the new MAC address of the receiver and replace the placeholder MAC address in the sender's code.

Device 1 (Sender)

#include <esp_now.h>
#include <WiFi.h>
#include <CodeCell.h>

CodeCell myCodeCell;
uint8_t receiverMAC[] = {0xXX, 0xXX, 0xXX, 0xXX, 0xXX, 0xXX};  // Replace with receiver's MAC address
int Roll_Control = 0;

float Roll = 0.0;
float Pitch = 0.0;
float Yaw = 0.0;

void setup() {
  Serial.begin(115200);
  myCodeCell.Init(MOTION_ROTATION);  // Initialize motion sensing

  // Initialize WiFi in Station mode
  WiFi.mode(WIFI_STA);
  Serial.println(WiFi.macAddress());

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register peer
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, receiverMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
}

void loop() {
  if (myCodeCell.Run()) {
    myCodeCell.Motion_RotationRead(Roll, Pitch, Yaw);

    Roll = Roll + 180;
    Roll = (Roll * 100) / 180;
    Roll = constrain(Roll, 0, 200) / 2;
    Roll_Control = (uint8_t)Roll;

    Serial.println(Roll_Control);
    esp_now_send(receiverMAC, (uint8_t *)&Roll_Control, sizeof(Roll_Control));
  }
}
Device 2 (Receiver)

#include <esp_now.h>
#include <WiFi.h>
#include <CodeCell.h>
#include <DriveCell.h>

#define IN1_pin1 2
#define IN1_pin2 3
#define IN2_pin1 5
#define IN2_pin2 6

CodeCell myCodeCell;
DriveCell Motor1(IN1_pin1, IN1_pin2);
DriveCell Motor2(IN2_pin1, IN2_pin2);

void setup() {
  Serial.begin(115200);
  myCodeCell.Init(LIGHT);  // Initialize Light Sensing
  Motor1.Init();
  Motor2.Init();

  // Initialize WiFi in Station mode
  WiFi.mode(WIFI_STA);
  Serial.println(WiFi.macAddress());

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register the receive callback
  esp_now_register_recv_cb(onDataRecv);
}

void onDataRecv(const esp_now_recv_info *recvInfo, const uint8_t *incomingData, int len) {
  int Roll_Speed = 0;
  memcpy(&Roll_Speed, incomingData, sizeof(Roll_Speed));

  if (Roll_Speed > 50) {
    Motor1.Drive(1, Roll_Speed);
    Motor2.Drive(1, Roll_Speed);
  } else {
    Roll_Speed = 100 - Roll_Speed;
    Motor1.Drive(0, Roll_Speed);
    Motor2.Drive(0, Roll_Speed);
  }
  Serial.println(Roll_Speed);
}

void loop() {
  if (myCodeCell.Run()) {}
}
Conclusion
By following these examples, you can configure two CodeCell devices to communicate over WiFi using ESP-NOW. The examples highlight how to send proximity and angular data between devices and utilize the data for real-time control of LEDs and motors.

Feel free to expand on these projects by incorporating more sensors or additional features to enhance the functionality of your remote system!

******************************************************************************

Webpage URL: https://microbots.io/blogs/learn/codecell-ai-prompt
CodeCell: Ai Prompt

In this build, we'll explore how to configure the CodeCell's ESP32-C3 to use Google's Gemini AI for Arduino prompt-based interactions. You'll learn how to send a prompt via the Serial Monitor and, in a second example, how the CodeCell can automatically trigger a joke based on proximity sensing. This project is ideal for anyone looking to add AI capabilities to their IoT projects.



What You'll Learn
How to set up the CodeCell for AI prompt-based interactions via the ESP32-C3.
How to send prompts to Gemini AI using the Arduino IDE and Serial Monitor.
How to use CodeCell’s proximity sensor to trigger a joke automatically.
About CodeCell and Google Gemini
In this example, we use Google’s Gemini model for generating content based on user input or sensor data. Through out this tutorial we will use and modify the code example made by 'techiesms' - Watch the full tutorial here.

With the ESP32-C3's WiFi capabilities, you can make HTTP requests to Google’s Gemini API, allowing real-time interaction with the AI. Whether you’re asking for text responses or generating creative outputs like jokes, this integration is straightforward to implement.

Project Overview
In the first example, you’ll send prompts directly via the Serial Monitor, and the CodeCell will send this input to Google Gemini AI for processing. The AI's response is printed back to the Serial Monitor, limited by 100 tokens. In the second example, the CodeCell’s proximity sensor will trigger a prompt to the AI, asking it to generate a joke when it detects an object. This setup can be used for fun interactive projects where the device responds to its environment using AI-based content.

How to Get the Gemini API Key
Before we integrate the Gemini AI into our ESP32-C3 setup, we first need to generate an API key and test it. Follow the steps below to create your API key and then you can also test it using a software like Postman.

Step 1: Generating the API Key for Google Gemini
Open your browser and search for Gemini API
Click on the first result that appears. This will take you to Google’s API documentation page.
Next, click on the button that says 'Get API Key in Google AI Studio', then click 'Create API Key'.
Once the API key is generated, copy it somewhere safe. You will need this for the next steps.
Step 2: Testing the API with Postman
Now that we have the API key, we can test it using the Postman application. Postman is a free tool that allows you to make HTTP requests and see the responses.

First, download and install Postman from the official website. You can find the download link here.
After installation, open Postman and create a new account if required.
Once you're logged in, click the + icon to create a new request.
Select POST as the request type, as we will be sending a prompt to the Gemini AI.
For the URL, modify this URL with your API Key, you generated in Step 1: https://generativelanguage.googleapis.com/v1beta/models/gemini-1.5-flash:generateContent?key=YOUR_API_KEY
Step 3: Configuring Headers and Body in Postman
Once you’ve entered the URL, we need to set up the request headers and body.

Go to the Headers section in Postman and add a new header.
Set the header key to Content-Type and the value to application/json.
Now, click on the Body tab and select raw and set the format to JSON.
In the body, paste the following JSON code:

{
    "contents": [
        {
            "parts": [
                {
                    "text": "Who are you?"
                }
            ]
        }
    ],
    "generationConfig": {
        "maxOutputTokens": 100
    }
}
    
In this example, we are asking the AI a simple question: "Who are you?" and setting the maximum number of tokens to 100. Tokens control the length of the response generated by the AI. If you lower the token limit (e.g., 20 tokens), the response will be shorter. You can experiment with different values for maxOutputTokens to see how it affects the response length.

Step 4: Sending the Request
Once everything is set up, click the Send button.
Postman will send the request to the Gemini AI, and after a few seconds, you should see a response.
If everything was successful, you’ll see a status of 200 OK, indicating that the request was processed without errors.
The response body will contain the answer generated by the AI. In this case, you should see something like: "I am a large language model trained by Google."
How to prompt with CodeCell?
Once you've generated and verified that the API works, you can proceed with the next step: integrating this API into your CodeCell project.

Example 1: AI Prompt via Serial Monitor
Below is the example code to get you started. In this example, the AI will respond to text prompts you send via the Serial Monitor. Remember to replace the placeholders with your WiFi credentials and Gemini API token.


#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <CodeCell.h>

CodeCell myCodeCell;

const char* ssid = "SSID"; //Enter your SSID
const char* password = "PASSWORD"; //Enter your password
const char* Gemini_Token = ""; //Enter your Gemini token
const char* Gemini_Max_Tokens = "100";
String res = "";

void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  while (!Serial);

  // Wait for WiFi connection
  WiFi.begin(ssid, password);
  Serial.print("Connecting to ");
  Serial.println(ssid);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void loop() {
  while (!Serial.available());

  while (Serial.available()) {
    char add = Serial.read();
    res += add;
    delay(1);
  }

  int len = res.length();
  res = res.substring(0, len - 1);
  res = "\"" + res + "\"";

  HTTPClient https;

  if (https.begin("https://generativelanguage.googleapis.com/v1beta/models/gemini-1.5-flash:generateContent?key=" + String(Gemini_Token))) {
    https.addHeader("Content-Type", "application/json");
    String payload = "{\"contents\": [{\"parts\":[{\"text\":" + res + "}]}],\"generationConfig\": {\"maxOutputTokens\": " + String(Gemini_Max_Tokens) + "}}";

    int httpCode = https.POST(payload);

    if (httpCode == HTTP_CODE_OK) {
      String response = https.getString();
      DynamicJsonDocument doc(1024);
      deserializeJson(doc, response);
      String answer = doc["candidates"][0]["content"]["parts"][0]["text"];
      answer.trim();

      Serial.println(answer);
    } else {
      Serial.printf("[HTTPS] POST failed, error: %s\n", https.errorToString(httpCode).c_str());
    }
    https.end();
  } else {
    Serial.printf("[HTTPS] Unable to connect\n");
  }

  res = "";
}
Example 2: Trigger AI Prompt via Proximity Sensor
This example uses the CodeCell's proximity sensor to trigger a prompt when an object is detected nearby. The AI will respond with a joke.


#include <Arduino.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <CodeCell.h>

CodeCell myCodeCell;

const char* ssid = "SSID"; //Enter your SSID
const char* password = "PASSWORD"; //Enter your password
const char* Gemini_Token = ""; //Enter your Gemini token
const char* Gemini_Max_Tokens = "100";
String res = "";

void setup() {
  Serial.begin(115200);

  myCodeCell.Init(LIGHT);  // Initializes proximity sensing

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  while (!Serial);

  WiFi.begin(ssid, password);
  Serial.print("Connecting to ");
  Serial.println(ssid);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void loop() {
  if (myCodeCell.Run()) {
    uint16_t proximity = myCodeCell.Light_ProximityRead();

    if (proximity > 100) {
      Serial.println("Here's a new joke...");
      myCodeCell.LED(0, 0xFF, 0);  // Set LED to Green when proximity is detected

      res = "\"Tell me a unique joke\"";

      HTTPClient https;

      if (https.begin("https://generativelanguage.googleapis.com/v1beta/models/gemini-1.5-flash:generateContent?key=" + String(Gemini_Token))) {
        https.addHeader("Content-Type", "application/json");
        String payload = "{\"contents\": [{\"parts\":[{\"text\":" + res + "}]}],\"generationConfig\": {\"maxOutputTokens\": " + String(Gemini_Max_Tokens) + "}}";

        int httpCode = https.POST(payload);

        if (httpCode == HTTP_CODE_OK) {
          String response = https.getString();
          DynamicJsonDocument doc(1024);
          deserializeJson(doc, response);
          String answer = doc["candidates"][0]["content"]["parts"][0]["text"];
          answer.trim();

          Serial.println(answer);
        } else {
          Serial.printf("[HTTPS] POST failed, error: %s\n", https.errorToString(httpCode).c_str());
        }
        https.end();
      } else {
        Serial.printf("[HTTPS] Unable to connect\n");
      }

      res = "";
    }
  }
}
Tips for Customization
Different Prompts: Try customizing the prompts to ask the AI different questions or give commands for creative outputs.
Experiment with Other Sensors: You can trigger different AI responses based on input from other CodeCell sensors like motion or light sensing.
Conclusion
This project showcases how to integrate AI responses into your CodeCell projects using Google’s Gemini API. By leveraging the ESP32-C3’s WiFi capabilities, you can create interactive devices that react to user input or environmental factors, making your IoT builds smarter and more engaging.

Experiment with the code and customize the prompts to suit your projects!

******************************************************************************

Webpage URL: https://microbots.io/blogs/learn/codecell-alexa-lighting

CodeCell: Alexa Lighting

In this build, we'll explore how to configure the CodeCell's onboard LED light using the Espalexa library, which allows Alexa to control devices like smart lights. We'll walk you through the process of connecting the CodeCell to your Wi-Fi, setting up the Espalexa library, and enabling voice control for the onboard LED through Alexa.



What You'll Learn
How to set up the CodeCell to connect to Wi-Fi.
How to use the Espalexa library to control the onboard LED via Alexa.
How to configure Alexa to recognize the CodeCell as a smart light.
About the Espalexa Library
The Espalexa library simplifies Alexa integration for ESP32 projects. It creates a virtual smart light, which Alexa can control via voice commands, without needing complex setup or cloud services. By using this library, your CodeCell can function as a smart device, like a light bulb, that Alexa can turn on, off, or dim.

Project Overview
In this project, the CodeCell is set up to connect to your Wi-Fi network. Once connected, Alexa can control the onboard LED light using voice commands, whether it's fully on (green) or off (no color).

Example:
Below is the example code to get you started. Update the Wi-Fi credentials with your network details, and follow the comments in the code to understand each step.


#include <Espalexa.h>
#include <WiFi.h>
#include <CodeCell.h>

CodeCell myCodeCell;

// WiFi credentials
const char* ssid = "SSID"; //Change to your SSID
const char* password = "PASSWORD"; // Change to your password 

// Alexa object
Espalexa espalexa;

// Function to handle Alexa commands
void alexaCallback(uint8_t brightness) {
  // Handle brightness (or ON/OFF) commands here
  if (brightness == 255) {
    myCodeCell.LED(0, 0xFF, 0);  // Full brightness, green light
  } else if (brightness == 0) {
    myCodeCell.LED(0, 0, 0);     // Turn off the LED
  }
}

void setup() {
  // Initialize serial for debugging
  Serial.begin(115200);
  myCodeCell.Init(LIGHT); /*Initializes Light Sensing*/

  // Connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected");

  // Add a device to Alexa
  espalexa.addDevice("MyLED", alexaCallback);

  // Start Espalexa
  espalexa.begin();
}

void loop() {  
  espalexa.loop(); // Handle Alexa requests
}
How to Add Your Device in the Alexa App
After uploading the code and connecting the CodeCell to Wi-Fi, the next step is to add the device to your Alexa app. Follow these steps to pair it with Alexa:

Open the Alexa App: On your smartphone, open the Alexa app.
Go to Devices: Tap the "Devices" tab at the bottom of the screen.
Add a New Device: Tap the "+" icon at the top-right corner, and select "Add Device."
Select Light: Since the CodeCell will appear as a smart light, choose "Light" as the device type.
Search for Devices: Alexa will now scan for new devices on your network. Wait for it to detect "MyLED" (or the name you've used in your code).
Complete Setup: Once detected, tap on your CodeCell device, and follow the prompts to complete the setup.
Test the Device: After the setup is complete, try giving a command like, "Alexa, turn on MyLED" or "Alexa, turn off my MyLED" to control the onboard LED!
With these steps, your CodeCell's onboard LED is now fully integrated into your smart home setup, and you can control it with Alexa voice commands or the Alexa app!

Tips for Customization
Experiment with Colors: Modify the LED color output in the alexaCallback() function to use different colors based on Alexa’s brightness level. You can use RGB values to create various effects.
Add more LEDs: Control RGB strip lights or NeoPixels through the CodeCell's GPIOs.
Additional Controls: Expand the project by adding multiple LED control points or combining with other CodeCell features like motion sensing or light monitoring.
Conclusion
This project demonstrates how to integrate the CodeCell with Alexa using the Espalexa library to control the onboard LED light. By following this example, you can easily build voice-activated projects with CodeCell, bringing IoT capabilities into your hands!

Get creative with the customization options and bring more of your projects to life with Alexa integration!


******************************************************************************
Webpage URL: https://microbots.io/blogs/learn/codecell-depth-gestures

CodeCell: Depth Gestures

In this build, we'll explore how to use the CodeCell's onboard proximity sensor to detect depth gestures and control two FlatFlaps, varying their angles based on the proximity values. This project demonstrates a unique way to create interactive robots, actuators, motors or light that respond to hand movements.


What You'll Learn
How to set up the CodeCell for depth gesture recognition using its proximity sensor.
How to control two FlatFlaps using proximity data from the CodeCell.
An understanding of how to integrate the tiny DriveCell, to control FlatFlaps.
About the CodeCell's Proximity Sensor
The CodeCell is equipped with a VCNL4040 proximity sensor that can measure distances up to 20 cm. By using an infrared light, the sensor detects objects within its range, measuring the reflection of emitted IR light to approximate distance. This allows you to create responsive behaviors based on how close an object is, making it ideal for interactive gestures.

Depth gestures are based on the proximity data from the CodeCell's onboard sensor. By moving your hand or other objects closer or further from the sensor, you can create dynamic inputs that drive various actions. In this project, the proximity data is used to control the angle of two FlatFlaps, which are connected to two DriveCells (H-bridge drivers). 

Project Overview
In this example, the CodeCell continuously reads proximity data and adjusts the angle of two FlatFlaps based on how close the object is. As the object moves closer or further, the angle of the FlatFlaps changes, demonstrating a simple yet effective method for depth gesture-based control.

The two FlatFlaps, are soldered to two DriveCells (H-bridge drivers), which are pin to pin compatible with the CodeCell. These components are then connected on 3D printed mount, to create a cute little Flappy-Bot! Don't forget to add a googly-eye to give it more personality!

Below is the example code to get you started. Ensure your CodeCell is properly connected via USB-C, and the FlatFlaps are connected to the two DriveCells. Follow the comments in the code to understand each step.

Example
#include <CodeCell.h>
#include <DriveCell.h>

#define IN1_pin1 2
#define IN1_pin2 3
#define IN2_pin1 5
#define IN2_pin2 6

DriveCell FlatFlap1(IN1_pin1, IN1_pin2);
DriveCell FlatFlap2(IN2_pin1, IN2_pin2);

CodeCell myCodeCell;

void setup() {
    Serial.begin(115200); // Set Serial baud rate to 115200. Ensure Tools/USB_CDC_On_Boot is enabled if using Serial.

    myCodeCell.Init(LIGHT); // Initializes Light Sensing

    FlatFlap1.Init();
    FlatFlap2.Init();

    FlatFlap1.Tone();
    FlatFlap2.Tone();
}

void loop() {
    if (myCodeCell.Run()) {
        // Runs every 100ms
        uint16_t proximity = myCodeCell.Light_ProximityRead();
        Serial.println(proximity);
        if (proximity < 100) {
            // If proximity is detected, the FlatFlaps flap
            FlatFlap1.Run(1, 100, 400);
            FlatFlap2.Run(1, 100, 400);
        } else {
            // Adjust FlatFlap angle based on proximity
            proximity = proximity - 100;
            proximity = proximity / 10;
            if (proximity > 100) {
                proximity = 100;
            }
            FlatFlap1.Drive(0, proximity);
            FlatFlap2.Drive(0, proximity);
        }
    }
}
Tips for Customization
Adjust Thresholds: Modify the proximity thresholds and scaling factors in the code to fine-tune the responsiveness of the FlatFlaps based on your preference.
Expand Functionality: Consider adding more CodeCell sensing functions, like motion tap detection or light sensing to add more functionality to your Flap-Bot!
Create Custom Designs: Use 3D printing to create a more unique robot, making it more personalized.
Conclusion
This project shows how to use the CodeCell's proximity sensor for depth gestures, driving the angles of FlatFlaps based on object distance. Experiment with the code, customize the parameters, and bring your own flappy bot to life!

******************************************************************************

Webpage URL: https://microbots.io/blogs/learn/codecell-personal-activity-guessing

CodeCell: Personal Activity Guessing

In this build, we'll explore how to configure the CodeCell's onboard motion sensor to try and guess the personal activate you're doing, and display it on an OLED screen. Its meant to track different states such as walking, running, cycling, climbing stairs and driving!



What You'll Learn
How to set up the CodeCell for personal activity sensing using its motion sensor.
How to connect an OLED display to the CodeCell.
Understanding how the CodeCell’s motion sensor categorizes different activities.
About the CodeCell's Personal Activity Sensing
The CodeCell's motion sensor is capable of categorizing various personal activities based on movement patterns. Based on these patterns the BNO085 sensor will try to guess which activity is being performed. These activities include walking, running, cycling, driving and more.

The CodeCell library makes it easy for you to directly read the activity without any complex code.

Project Overview
In this example, the CodeCell continuously monitors the BNO085's personal activity guess. The activity with the highest chance is then displayed on an OLED screen using the Adafruit SSD1306 library. This setup is ideal for creating wearable activity monitors or fitness trackers that provide real-time feedback on physical activities.

Note that some activities might take between 10-30 seconds to start getting recognized, as it will mainly depend on orientation of the CodeCell and where it is mounted. 

Example:
Below is the example code to get you started. Make sure your CodeCell is connected via USB-C and your OLED display is wired correctly to the CodeCell’s lower side, using its ground, 3V3, and I2C pins (SDA and SCL).

Follow the comments in the code to understand each step.


#include <CodeCell.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

CodeCell myCodeCell;

/* Configure the OLED Display */
#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 32  // OLED display height, in pixels

#define OLED_RESET -1        // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C  // Address of the OLED display
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

int read_timer = 0;

void setup() {
    Serial.begin(115200); // Set Serial baud rate to 115200. Ensure Tools/USB_CDC_On_Boot is enabled if using Serial.

    myCodeCell.Init(MOTION_ACTIVITY); // Initializes activity sensing.

    if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println(F("SSD1306 allocation failed"));
    }

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.display();
    delay(2000);
}

void loop() {
    if (myCodeCell.Run()) {
        if (read_timer < 10) {
            read_timer++;
        } else {
            // Update every 1 sec
            read_timer = 0;
            display.clearDisplay();
            display.setCursor(32, 16);
            display.print(F("Activity: "));
            display.setCursor(32, 24);
            switch (myCodeCell.Motion_ActivityRead()) {
                case 1:
                    display.print("Driving");
                    break;
                case 2:
                    display.print("Cycling");
                    break;
                case 3:
                case 6:
                    display.print("Walking");
                    break;
                case 4:
                    display.print("Still");
                    break;
                case 5:
                    display.print("Tilting");
                    break;
                case 7:
                    display.print("Running");
                    break;
                case 8:
                    display.print("Stairs");
                    break;
                default:
                    display.print("Reading..");
                    break;
            }
            display.display();
        }
    }
}
Tips for Customization
Combine with Other Sensors: Integrate additional sensing available inside the CodeCell, like step counting for more comprehensive fitness monitoring.
Conclusion
This project demonstrates how to use the CodeCell’s motion sensor to monitor personal activities and display the results on an OLED screen. This basic setup provides a foundation for developing more advanced activity monitoring systems.

Experiment with the code and settings to create your own personalized wearable!


******************************************************************************
Webpage URL: https://microbots.io/blogs/learn/codecell-step-counter

CodeCell: Step Counter

In this build, we'll explore how to use the CodeCell's onboard motion sensor to measure step counts and display these counts on an OLED display. This project demonstrates how to create a step counter, ideal for fitness trackers, pedometers, or any other DIY project that requires activity monitoring.


What You'll Learn
How to set up the CodeCell for step counting using its motion sensor.
How to connect the OLED display to the CodeCell
Understand how the CodeCell's motion sensor works for step counting.
About the CodeCell's Step Counting
The CodeCell is equipped with a motion sensor that can track step counts by using its onboard sensors to detect specific movement patterns. This algorithm is performed inside the BNO085 sensor, and the CodeCell library help you easily reads these step counts. 

Project Overview
In this example, the CodeCell continuously monitors for steps and updates the count. This count is then displayed on an OLED screen using the Adafruit SSD1306 library. 

Example: 
Below is the example code to get you started. Ensure your CodeCell is properly connected via USB-C and your OLED display is correctly wired to the CodeCell's lower side. There you can use its ground, 3V3 and I2C pins (SDA and SCL).

Follow the comments in the code to understand each step.


#include <CodeCell.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

CodeCell myCodeCell;

/* Configure the OLED Display */
#define SCREEN_WIDTH 64  // OLED display width, in pixels
#define SCREEN_HEIGHT 32  // OLED display height, in pixels

#define OLED_RESET -1        // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C  // Address of the OLED display
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

uint16_t step_counter = 0;

void setup() {
    Serial.begin(115200); // Set Serial baud rate to 115200. Ensure Tools/USB_CDC_On_Boot is enabled if using Serial.

    myCodeCell.Init(MOTION_STEP_COUNTER); // Initializes step counting and activity sensing.

    if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println("Display Error");
    }

    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.display();
    delay(2000);
}

void loop() {
    if (myCodeCell.Run()) {
        // Read step count from the CodeCell motion sensor.
        myCodeCell.Motion_StepCounterRead(step_counter);

        // Clear the display and show the step count.
        display.clearDisplay();
        display.setCursor(32, 16); // Start at top-left corner
        display.print(F("Steps: "));
        display.print(step_counter);
        display.display();    
    }
}
Tips for Customization
Expand Functionality: Integrate additional features like step goals, distance approximations, or activity tracking over time.
Combine with Other Sensors: Add other sensing functions like the CodeCell's Motion Personal Activity Sensing, to create a more comprehensive fitness tracker.
Conclusion
This project demonstrates how to use the CodeCell’s motion sensor to count steps and display the count on an OLED screen. Experiment with the code,to create your own wearable fitness device!


******************************************************************************

Webpage URL: https://microbots.io/blogs/learn/codecell-servo-angle-control

CodeCell: Servo Angle Control

In this build, we'll explore how to use the CodeCell's onboard 9-axis motion sensor to read roll, pitch, and yaw angles, and use these angles to control a servo motor. This project demonstrates how to create interactive motion-based controls, perfect for robotics, gimbals, or any project that requires responsive rotation control.


What You'll Learn
How to set up the CodeCell for reading motion angles (roll, pitch, and yaw).
How to write code to control a servo motor based on the pitch angle from the motion sensor.
An understanding of how the CodeCell's BNO085 motion sensor works for rotation sensing.
About the CodeCell's Rotation Sensing
The CodeCell is equipped with a BNO085 motion sensor, which provides precise motion sensing capabilities, including roll, pitch, and yaw angles. By reading these angles, you can create interactive motion controls for various applications, such as stabilizing platforms or create a response to device orientation.


How Rotation Sensing Works
The BNO085 motion sensor read the acceleromter, gyroscope and magnetometer reading and compute the rotation vectors. These vectors are sent to the CodeCell which it then transforms into angular data to obtain the roll, pitch, and yaw based on the device's orientation in space. These angles represent the device's rotation along three axes. In this example, we'll use the pitch angle to control the position of a servo motor, allowing it to respond dynamically to changes in orientation.

Project Overview
In this example, the CodeCell continuously monitors the pitch angle. The pitch value is used to set the servo motor's position, allowing it to rotate based on how you tilt the device. This basic functionality can be expanded to create more complex interactions, such as controlling multiple servos, stabilizing a platform, or adding responsive movements to a robot.

Code Example: Servo Control with Pitch Angle
Below is the example code to get you started. Make sure your CodeCell is properly connected via USB-C. Also, make sure that your servo-motor can be power via USB-C and add its angular limits to the code.

For this example you have to download the ESp32Servo libraryto control the servo-motor with your CodeCell. Follow the comments in the code to understand each step.


#include <CodeCell.h>
#include <ESP32Servo.h>

CodeCell myCodeCell;
Servo myservo;  

float Roll = 0.0;
float Pitch = 0.0;
float Yaw = 0.0;
int servo_angle = 0;

void setup() {
    Serial.begin(115200); // Set Serial baud rate to 115200. Ensure Tools/USB_CDC_On_Boot is enabled if using Serial
    myCodeCell.Init(MOTION_ROTATION); // Initializes rotation sensing
    myservo.attach(1);  // Attaches the servo on pin 1 to the servo object
}

void loop() {
    if (myCodeCell.Run()) {
        // Read rotation angles from the BNO085 sensor
        myCodeCell.Motion_RotationRead(Roll, Pitch, Yaw);
        
        // Convert the pitch angle to a servo angle
        servo_angle = abs((int)Pitch);
        servo_angle = (180 - servo_angle);
        
        // Limit the servo angle to the range 0-60 degrees
        if (servo_angle > 60) {
            servo_angle = 60;
        } else if (servo_angle < 0) {
            servo_angle = 0;
        }
        
        Serial.println(servo_angle); // Print the servo angle for debugging
        myservo.write(servo_angle);  // Set the servo position
    }
}
Tips for Customization
Adjust Range: Modify the range of the servo movement to suit your specific servo-motor by adjusting the limits set for servo_angle in the code. In this example we are using a 60deg range micro-servo. Note that some servo-motors are not mechanically linear so you might also have to compensate for its angular-mechanical error.
Combine with Other Sensors: Use additional sensors, like CodeCell's proximity and light sensing, to add more responsive actions to your project.
Use for Stabilization: Implement this setup for stabilizing platforms, like gimbals, to keep devices level during movement.
Conclusion
This project introduces the basics of using rotation sensing with CodeCell to control a servo, opening up numerous possibilities for motion-responsive projects. Experiment with the code, tweak the settings, and create your own dynamic builds!

******************************************************************************
Webpage URL: https://microbots.io/blogs/learn/codecell-tap-detection

CodeCell: Tap Detection

In this build, we'll explore how to use the CodeCell's onboard 9-axis motion sensor to detect taps. This project demonstrates how to use tap detection for interactive controls, making it perfect for creating responsive actions with a simple tap on the device.


What You'll Learn
How to set up the CodeCell for tap detection.
How to write code to detect taps and control external components.
An understanding of how the CodeCell's BNO085 motion sensor works for tap detection.
About the CodeCell's Tap Detection
The CodeCell is equipped with a BNO085 motion sensor, which offers a variety of sensing capabilities, including tap detection. This feature uses accelerometer data to detect taps, making it ideal for interactive controls like toggling lights, triggering sound effects, or other actions based on a simple tap gesture.

How Tap Detection Works
The BNO085 motion sensor detects taps by monitoring sudden accelerations along its axes. When a tap is detected, the sensor registers the event, allowing you to trigger actions like lighting up an LED or toggling other devices. This functionality is particularly useful for creating touch-based interactions without the need for mechanical buttons.

Project Overview
In this example, the CodeCell continuously monitors for taps. When a tap is detected, the onboard LED lights up in yellow for one second. You can expand on this basic functionality to create more complex interactions, such as controlling multiple LEDs, motors, or other connected devices based on tap inputs.

Example 1: Basic Tap Detection
Below is the example code to get you started. Make sure your CodeCell is properly connected via USB-C, and follow the comments in the code to understand each step.


#include <CodeCell.h>

CodeCell myCodeCell;

void setup() {
    Serial.begin(115200); // Set Serial baud rate to 115200. Ensure Tools/USB_CDC_On_Boot is enabled if using Serial
    myCodeCell.Init(MOTION_TAP_DETECTOR); // Initializes tap detection sensing
}

void loop() {
    if (myCodeCell.Run()) {
        // Runs every 100ms to check for taps
        if (myCodeCell.Motion_TapRead()) {
            // If a tap is detected, shine the LED yellow for 1 second
            myCodeCell.LED(0xA0, 0x60, 0x00); // Set LED to yellow
            delay(1000); // Keep the LED on for 1 second
        }
    }
}
Code Example 2: Tap Detection + CoilCell
In this next example, we use a CoilCell to flip its polarity, and actuate a flip-dot. This expands the interactivity by using tap detection to control external devices, creating a more dynamic response.


#include <CoilCell.h>
#include <CodeCell.h>

#define IN1_pin1 5
#define IN1_pin2 6

CoilCell myCoilCell(IN1_pin1, IN1_pin2);
CodeCell myCodeCell;

void setup() {
    Serial.begin(115200); // Set Serial baud rate to 115200. Ensure Tools/USB_CDC_On_Boot is enabled if using Serial.
    myCodeCell.Init(MOTION_TAP_DETECTOR); // Initializes tap detection sensing.
    myCoilCell.Init(); // Initializes the CoilCell.
    myCoilCell.Tone(); // Plays a tone to confirm initialization.
}

void loop() {
    if (myCodeCell.Run()) {
        // Runs every 100ms to check for taps.
        if (myCodeCell.Motion_TapRead()) {
            // If a tap is detected, shine the LED yellow and flip the CoilCell's polarity.
            myCodeCell.LED(0xA0, 0x60, 0x00); // Set LED to yellow.
            myCoilCell.Toggle(100); // Toggle the CoilCell's polarity.
            delay(1000); // Delay to keep the LED on and polarity flipped for 1 second.
        }
    }
}
Tips for Customization
Change LED Colors: Experiment with different LED colors and use the taps to increase the LED's brightness.
Expand Actions: Use tap detection to trigger other devices, like buzzers, motors, or displays, to add more layers of interactivity.
Combine with Other Sensors: Integrate tap detection with other sensors, like proximity or light, to create multi-sensor responsive projects.
Conclusion
This project introduces the basics of using tap detection with CodeCell. Experiment with the code, customize the responses, and explore the potential of tap detection in your next project!

******************************************************************************

Webpage URL: https://microbots.io/blogs/learn/codecell-proximity-sensing

CodeCell: Proximity Sensing

In this build, we'll explore how to use the CodeCell's onboard proximity sensor to detect objects. 


What You'll Learn
How to set up the CodeCell
How to write code to read proximity data and control onboard LED.
An understanding of how the CodeCell's VCNL4040 proximity sensor works.
About the CodeCell's Proximity Sensor
The CodeCell is equipped with a VCNL4040 sensor that can measure proximity up to 20 cm. This sensor uses I2C communication and is automatically initialized through the CodeCell library, allowing for seamless integration into your projects. Whether you're looking to add simple gesture depth control or detect nearby objects, the VCNL4040 makes it easy to add proximity sensing into your builds.

How Proximity Sensing Works
The VCNL4040 proximity sensor uses infrared light to detect objects within its range. It measures the reflection of emitted IR light to approximate how close an object is, allowing you to create responsive behaviors based on proximity. This feature is particularly useful for creating interactive lighting, robotics, touchless switches, or other proximity-based actions.

Project Overview
In this example, the CodeCell continuously monitors proximity data and turns on a red LED when an object is detected. You can expand on this basic functionality to create more complex interactions, such as varying the LED color or brightness based on distance, or triggering different actions based on proximity.

Code Example
Below is the example code to get you started. Ensure your CodeCell is properly connected via USB-C, and follow the comments in the code to understand each step.


#include <CodeCell.h>

CodeCell myCodeCell;

void setup() {
    Serial.begin(115200); // Set Serial baud rate to 115200. Ensure Tools/USB_CDC_On_Boot is enabled if using Serial
    myCodeCell.Init(LIGHT); // Initializes light sensing, including proximity
}

void loop() {
    if (myCodeCell.Run()) {
        // Runs every 100ms to check proximity
        uint16_t proximity = myCodeCell.Light_ProximityRead();
        
        // Check if an object is within range
        if (proximity > 100) {
            myCodeCell.LED(0xFF, 0, 0); // Set LED to Red when proximity is detected
            delay(1000); // Keep the LED on for 1 second
        } else {
            // No action if the object is out of range
        }
    }
}
Tips for Customization
Adjust Proximity Threshold: Modify the threshold value (100 in the example) to adjust the sensitivity of proximity detection based on your application.
Change LED Colors: Experiment with different LED colors using the myCodeCell.LED() function to create multi-colored responses to proximity.
Add More: Consider adding a buzzer or  motor to provide audio or motion feedback when objects are detected within range.
Use Proximity with Light Sensing: Combine proximity and light sensing to create more complex behaviors, such as adjusting lights based on both distance and ambient light levels.
Conclusion
This project introduces the basics of using proximity sensing with CodeCell, opening up a range of interactive possibilities. Experiment with the code, tweak the settings, and make it your own!

******************************************************************************
Webpage URL: https://microbots.io/blogs/learn/codecell-auto-dimming

CodeCell: Auto Dimming

In this build, we'll explore how to use the CodeCell to sense white light and automatically adjust the brightness of LEDs. This project demonstrates the CodeCell's onboard light sensor, helping you create responsive lighting effects that adapt to changing light conditions.


What You'll Learn
How to set up the CodeCell
How to write the code to read light levels and control LED brightness.
An understanding of how the CodeCell's VCNL4040 light sensor works.
About the CodeCell's Light Sensor
The CodeCell features a built-in VCNL4040 sensor that can measure both light levels and proximity up to 20 cm. This sensor uses I2C communication, and talks to the ESP32 in the CodeCell library, where the sensor is automatically initialized to optimize its sensing resolution. This makes the setup straightforward, so you can focus on building your project.

Ambient Light Sensing vs. White Light Sensing
The VCNL4040 sensor on the CodeCell is capable of both ambient light sensing and white light sensing, each serving distinct purposes:

Ambient Light Sensing: This mode measures the overall light intensity from all sources in the environment, including natural and artificial light. It's ideal for applications that require a general understanding of the light levels in a space, such as adjusting screen brightness or activating night mode in devices.
White Light Sensing: This mode specifically measures the intensity of white light, which is particularly useful when the goal is to assess light sources that resemble daylight or LED lighting. White light sensing is beneficial in scenarios where you want to distinguish between different lighting conditions, such as separating white light from other colored lights or in situations where accurate color temperature is important.
In this project, we're using the white light sensing feature to directly influence LED brightness based on the detected white light levels, creating a more targeted response compared to general ambient light sensing.

Project Overview
In this example, the CodeCell continuously measures ambient white light and adjusts the brightness of an onboard LED based on the detected light level. As the room gets darker, the LED will dim, providing a smooth transition that you can tweak and customize for your own lighting projects.

Code Example
Below is the example code to get you started. Make sure your CodeCell is connected properly, and follow the comments in the code to understand each step.


#include <CodeCell.h>

CodeCell myCodeCell;

void setup() {
    Serial.begin(115200); // Set Serial baud rate to 115200. Ensure Tools/USB_CDC_On_Boot is enabled if using Serial.
    myCodeCell.Init(LIGHT); // Initializes light sensing.
}

void loop() {
    delay(100); // Small delay - You can adjust it accordingly
    
    // Read white light from the sensor and adjust brightness for 8-bit
    uint16_t brightness = (myCodeCell.Light_WhiteRead()) >> 3; 
    Serial.println(brightness); // Print the brightness value to the serial monitor for debugging.

    // Limit the sensor values to the LED's brightness range (1 to 254)
    if (brightness == 0U) {
        brightness = 1U; // Set a minimum brightness to avoid turning off the LED completely
    } else if (brightness > 254U) {
        brightness = 254U; // Cap the brightness to the LED's maximum level
    }

    brightness = 255U - brightness; // Invert the brightness so the LED dims as it gets brighter
    myCodeCell.LED(0, 0, brightness); // Shine the onboard blue RGB LED with the new adjusted brightness
}
Tips for Customization
Adjust Brightness Sensitivity: Adjust the brightness limits based on your room's light levels.
Change LED Colors: The myCodeCell.LED() function allows you to specify RGB values. Try experimenting with different colors based on light levels.
Add More LEDs: Connect more LEDs or even NeoPixels for more lighting effects and adjust their brightness with the same technique. 
Add Proximity Control: Add the proximity sensor to add more interactive effects, like depth gestures to act like a switching.


******************************************************************************

Webpage URL: https://microbots.io/blogs/learn/coilpad-basics-your-first-steps

CoilPad Basics: Your First Steps

The CoilPad is an incredibly thin and innovative actuator that brings motion to your projects in a compact form factor. To understand how it works, let's dive into its unique design and the principles behind its operation.

In this tutorial we will explain:

What is a CoilPad and how does it work?
How to control its polarity, position, and speed
Making the CoilPad more interactive with the CodeCell sensors

What is a CoilPad?

The CoilPad is an actuator made from a flexible planar coil that adheres seamlessly to any smooth surface. By adding a magnet, it transforms into a device capable of magnetic movement, buzzing, or even heating. It’s designed to convert electrical energy into mechanical movement with ease.

How Does It Work?

The CoilPad features a flat, ultra-thin coil that interacts with external magnets. When an electric current passes through the coil, it generates a magnetic field that either attracts or repels the magnet, causing movement. By alternating the direction of the current, you can control the motion of the CoilPad. Applying a square wave signal makes the CoilPad oscillate continuously, with adjustable speed and intensity. For smooth organic motions, we will explore the DriveCell PWM library.

Installing CoilPad

The CoilPad design makes it easy to install. It comes with a peelable adhesive back, ensuring that it stays firmly attached to any smooth surface. 

Getting Your CoilPad Moving

You can start testing it, by pulling one of its pins to 5V and the other to ground, then switch them around. In one instance, the magnet will be repelled, and in the other, it will be attracted. You can connect it to your own transistors or H-bridge module to switch these pins automatically. However, to make it even easier, you can purchase our tiny DriveCell module. The DriveCell is a compact, pin-to-pin compatible H-bridge driver that simplifies the process of controlling actuators like the CoilPad. Its open-source Arduino software library makes actuator control easy, especially for beginners, by providing straightforward software functions and easy-to-follow examples.

For an in-depth guide on the DriveCell Software Library, check out this article. But here’s a quick recap of how you can use its functions to enhance the CoilPad actuation. Don’t worry, it’s quite simple! Start by downloading the "DriveCell" library from Arduino's Library Manager. Once installed, you’ll be ready to control your device. Before we start, make sure you connect the DriveCell to your microcontroller. We recommend using a CodeCell, which is pin-to-pin compatible, supports all the library functions, and can add wireless control and interactive sensing to your CoilPad.

1. Init()

First, we need a basic setup code to get you started:

#include <DriveCell.h> // This line includes the DriveCell library

DriveCell myCoilPad(IN1, IN2); // Replace IN1 and IN2 with your specific pins

void setup() {
    myCoilPad.Init(); // Initializes your DriveCell connected to a CoilPad
}
This code gives the name 'myCoilPad' to your DriveCell and tells it to start up and initialize all the necessary peripherals.

2. Pulse(bool direction, uint8_t ms_duration)

This function sends a brief burst of power to the CoilPad in a specified polarity. This quick energizing and de-energizing can cause a short, sharp movement of the CoilPad, depending on the polarity.

myCoilPad.Pulse(1, 10); // Sends a short burst for 10 milliseconds in the specified direction
3. Buzz(uint16_t us_buzz)

This function makes the CoilPad vibrate like a buzzer, which is useful for creating audible feedback. 

myCoilPad.Buzz(100); // Makes the CoilPad buzz with a 100 microsecond pulses
4. Tone()

The Tone function makes the CoilPad play a tone. This can be used for audible feedback or creative applications where sound is part of the interaction.

myCoilPad.Tone(); // Plays a tone by varying the frequency
5. Toggle(uint8_t power_percent)

This function toggles the CoilPad polarity, which can be useful for creating a rapid flapping movement or reversing direction quickly in your code.

myCoilPad.Toggle(100); // Toggles direction at 100% power
6. Run(bool smooth, uint8_t power_percent, uint16_t flip_speed_ms)

This function allows you to continuously flip the polarity of the CoilPad and control its motion speed and smoothness. If smooth is set to true, the actuation will be less sharp and smoothed, which is ideal for slower, controlled movements.

myCoilPad.Run(true, 50, 1000); // Runs the CoilPad smoothly at 50% power, flipping every 1000 milliseconds
7. Drive(bool direction, uint8_t power_percent)

This function lets you control the CoilPad polarity and its magnetic field strength by adjusting the power level.

myCoilPad.Drive(true, 75); // Moves the CoilPad forward at 75% power 
Examples:
Here's an example where we configure two CoilPads and actuate them at two different speeds:

#include <DriveCell.h>

#define IN1_pin1 2
#define IN1_pin2 3
#define IN2_pin1 5
#define IN2_pin2 6

DriveCell CoilPad1(IN1_pin1, IN1_pin2);
DriveCell CoilPad2(IN2_pin1, IN2_pin2);

uint16_t c_counter = 0;

void setup() {
  CoilPad1.Init();
  CoilPad2.Init();

  CoilPad1.Tone();
  CoilPad2.Tone();
}

void loop() {
  delay(1);
  c_counter++;
  if (c_counter < 2000U) {
    CoilPad1.Run(0, 100, 100);
    CoilPad2.Run(0, 100, 100);
  }
  else if (c_counter < 8000U) {
    CoilPad1.Run(1, 100, 1000);
    CoilPad2.Run(1, 100, 1000);
  } else {
    c_counter = 0U;
  }
}
Combining with CodeCell Sensors

To make it even more interactive you can combine the CoilPad and DriveCell with the tiny CodeCell Sensor Module. CodeCell is pin-to-pin compatible with DriveCell, supports all library functions, and can adds wireless control and interactive sensing to your project. This allows you to create more advanced, responsive elements with your CoilPad actuators. 

With this next example the CodeCell controls two CoilPad that stops flapping when proximity is detected. Their magnetic field gets adjusted dynamically based on how close your hands gets. If no hand is detected it flips the CoilPad polarity every 400 milliseconds.

#include <CodeCell.h>
#include <DriveCell.h>

#define IN1_pin1 2
#define IN1_pin2 3
#define IN2_pin1 5
#define IN2_pin2 6

DriveCell CoilPad1(IN1_pin1, IN1_pin2);
DriveCell CoilPad2(IN2_pin1, IN2_pin2);

CodeCell myCodeCell;

void setup() {
  Serial.begin(115200);

 /* Set Serial baud rate to 115200. Ensure Tools/USB_CDC_On_Boot is enabled if using Serial. */

  myCodeCell.Init(LIGHT); /*Initializes Light Sensing*/

  CoilPad1.Init();
  CoilPad2.Init();

  CoilPad1.Tone();
  CoilPad2.Tone();
}

void loop() {
  if (myCodeCell.Run()) {
    /*Runs  every 100ms*/
    uint16_t proximity = myCodeCell.Light_ProximityRead();
    Serial.println(proximity);
    if (proximity < 100) {
      CoilPad1.Run(1, 100, 400);
      CoilPad2.Run(1, 100, 400);
    } else {
      proximity = proximity - 100;
      proximity = proximity / 10;
      if (proximity > 100) {
        proximity = 100;
      }
      CoilPad1.Drive(0, (proximity));
      CoilPad2.Drive(0, (proximity));
    }
  }
}
Feel free to tweak the code with your own creative ideas, or add motion sensing for a new reaction! Get started today with our Arduino libraries! If you have any more question about the CoilPad feel free to email us and we will gladly help out!

******************************************************************************

Webpage URL: https://microbots.io/blogs/learn/flatflap-basics-your-first-steps

FlatFlap Basics: Your First Steps

The FlatFlap is an incredibly thin and innovative actuator that brings motion to your projects in a compact form factor. To understand how it works, let's dive into its unique design and the principles behind its operation.


In this tutorial we will explain:

What is a FlatFlap and how does it work?
How to control its polarity, position and speed
Make the FlatFlap more interactive with the CodeCell sensors

What is a FlatFlap?

It's Flat and its a Flap ~ the FlatFlap is an actuator made from a flexible PCB (printed circuit board) and aluminum stiffeners, folded together to create a low-force flapping motion. It’s magnetic system converts electrical energy into mechanical movement.

How Does It Work?

The FlatFlap features a thin 10mm N52 neodymium magnet on its back, which interacts with the planar copper coil embedded within the flexible PCB. When an electric current passes through the coil, it generates a small magnetic field that either attracts or repels the magnet, causing the flap to move. By alternating the direction of the current, you can control the flapping motion of the actuator. Applying a square wave signal makes the FlatFlap flap continuously, with speeds of up to 25Hz. For smooth organic motions, we will explore the DriveCell PWM  library.

Installing FlatFlap

The FlatFlap design makes it easy to install. It comes with a peelable adhesive back and optional M1.2 screws (included) for added security, ensuring that it stays firmly attached to any surface, whether smooth or textured. The adhesive is 3M467, which provides a strong bond but can be removed with tweezers if needed.


Getting Your FlatFlap Moving

If you purchased the FlatFlap as a stand-alone actuator, you can start by pulling one of its pins to 5V and the other to ground, then switch them around. In one instance, the flap will be repelled, and in the other, it will be attracted. You can connect it to your own transistors or H-bridge module to switch these pins automatically. However, to make it even easier, you can purchase the FlatFlap directly soldered to our tiny DriveCell module. The DriveCell is a compact, pin-to-pin compatible H-bridge driver that simplifies the process of controlling actuators like the FlatFlap. Its open-source Arduino software library makes actuator control easy, especially for beginners, by providing straightforward software functions and easy-to-follow examples.

For an in-depth guide on the DriveCell Software Library, check out this article. But here’s a quick recap of how you can use its functions to enhance the FlatFlap actuation. Don’t worry, it’s quite simple! Start by downloading the "DriveCell" library from Arduino's Library Manager. Once installed, you’ll be ready to control your device. Before we start, make sure you connect the DriveCell to your microcontroller. We recommend using a CodeCell, which is pin-to-pin compatible, supports all the library functions, and can add wireless control and interactive sensing to your FlatFlap.

1. Init()

First we need a basic setup code to get you started:

#include <DriveCell.h> // This line includes the DriveCell library

DriveCell myFlatFlap(IN1, IN2); // Replace IN1 and IN2 with your specific pins

void setup() {
    myFlatFlap.Init(); // Initializes your DriveCell connected to a FlatFlap
}
This code gives the name 'myFlatFlap' to your DriveCell and tells it to start up and initialize all the necessary peripherals.

 2. Pulse(bool direction, uint8_t ms_duration)

This function sends a brief burst of power to the FlatFlap in a specified polarity. This quick energizing and de-energizing can cause a short, sharp movement of the FlatFlap, depending on the polarity.

myFlatFlap.Pulse(1, 10); // Sends a short burst for 10 milliseconds in the specified direction
2. Buzz(uint16_t us_buzz)

This function makes the FlatFlap vibrate like a buzzer, which is useful for creating audible feedback. 

myFlatFlap.Buzz(100); // Makes the FlatFlap buzz with a 100 microsecond pulses
3. Tone()

The Tone function makes the FlatFlap play a tone. This can be used for audible feedback or creative applications where sound is part of the interaction.

myFlatFlap.Tone(); // Plays a tone by varying the frequency
4. Toggle(uint8_t power_percent)

This function switches the FlatFlap direction, which can be useful for creating a rapid flapping movement or reversing direction quickly in your code.

myFlatFlap.Toggle(100); // Toggles direction at 100% power
5. Run(bool smooth, uint8_t power_percent, uint16_t flip_speed_ms)

This function allows you to continuously flip the polarity of the FlatFlap and control its motion speed and smoothness. If smooth is set to true, the flapping will be less sharp and smoothed, which is ideal for slower, controlled movements.

myFlatFlap.Run(true, 50, 1000); // Runs the FlatFlap smoothly at 50% power, flipping every 1000 milliseconds
6. Drive(bool direction, uint8_t power_percent)

This function lets you control the FlatFlap polarity and angular position of the flap by adjusting the power level, basically adjusting how strong the magnetic pull or push is.

myFlatFlap.Drive(true, 75); // Moves the FlatFlap forward at 75% power 
Examples:
Here's an example where we configure two FlatFlaps and flap them at different speeds:

#include <DriveCell.h>

#define IN1_pin1 2
#define IN1_pin2 3
#define IN2_pin1 5
#define IN2_pin2 6

DriveCell FlatFlap1(IN1_pin1, IN1_pin2);
DriveCell FlatFlap2(IN2_pin1, IN2_pin2);

uint16_t flap_counter = 0;

void setup() {
  FlatFlap1.Init();
  FlatFlap2.Init();

  FlatFlap1.Tone();
  FlatFlap2.Tone();
}

void loop() {
  delay(1);
  flap_counter++;
  if (flap_counter < 2000U) {
    FlatFlap1.Run(0, 100, 100);
    FlatFlap2.Run(0, 100, 100);
  }
  else if (flap_counter < 8000U) {
    FlatFlap1.Run(1, 100, 1000);
    FlatFlap2.Run(1, 100, 1000);
  } else {
    flap_counter = 0U;
    FlatFlap1.Drive(0, 100);
    FlatFlap2.Drive(1, 100);
    delay(500);
    FlatFlap1.Drive(1, 100);
    FlatFlap2.Drive(1, 100);
    delay(500);
    FlatFlap1.Drive(1, 100);
    FlatFlap2.Drive(0, 100);
    delay(500);
    FlatFlap1.Drive(1, 100);
    FlatFlap2.Drive(1, 100);
    delay(500);
    FlatFlap1.Drive(0, 100);
    FlatFlap2.Drive(0, 100);
    delay(500);
    FlatFlap1.Drive(1, 100);
    FlatFlap2.Drive(1, 100);
    delay(500);
    FlatFlap1.Tone();
    FlatFlap2.Tone();
  }
}

Combining with CodeCell Sensors

To make it even more interactive you can combine the FlatFlap and DriveCell with the tiny CodeCell Sensor Module. CodeCell is pin-to-pin compatible with DriveCell, supports all library functions, and adds wireless control and interactive sensing to your project. This allows you to create more advanced, responsive elements with your FlatFlap actuators. 

With this next example the CodeCell controls two FlatFlap that stops flapping when proximity is detected. Their angle gets adjusted dynamically based on how close your hands gets.

#include <CodeCell.h>
#include <DriveCell.h>

#define IN1_pin1 2
#define IN1_pin2 3
#define IN2_pin1 5
#define IN2_pin2 6

DriveCell FlatFlap1(IN1_pin1, IN1_pin2);
DriveCell FlatFlap2(IN2_pin1, IN2_pin2);

CodeCell myCodeCell;

void setup() {
  Serial.begin(115200); /* Set Serial baud rate to 115200. Ensure Tools/USB_CDC_On_Boot is enabled if using Serial. */

  myCodeCell.Init(LIGHT); /*Initializes Light Sensing*/

  FlatFlap1.Init();
  FlatFlap2.Init();

  FlatFlap1.Tone();
  FlatFlap2.Tone();
}

void loop() {
  if (myCodeCell.Run()) {
    /*Runs  every 100ms*/
    uint16_t proximity = myCodeCell.Light_ProximityRead();
    Serial.println(proximity);
    if (proximity < 100) {
      FlatFlap1.Run(1, 100, 400);
      FlatFlap2.Run(1, 100, 400);
    } else {
      proximity = proximity - 100;
      proximity = proximity / 10;
      if (proximity > 100) {
        proximity = 100;
      }
      FlatFlap1.Drive(0, (proximity));
      FlatFlap2.Drive(0, (proximity));
    }
  }
}
Feel free to tweak the code with your own creative ideas, or add motion sensing for a new reaction! With FlatFlap, you can bring your creative projects to life with motion in a sleek, compact package. Whether you're adding dynamic elements to art, experimenting with robotics, or developing interactive mechanical displays, the FlatFlap provides a versatile and easy-to-use solution. Get started today with our Arduino libraries! If you have any more question about the FlatFlap feel free to email us and we will gladly help out!

******************************************************************************

Webpage URL: https://microbots.io/blogs/learn/coilcell-basics-your-first-steps

CoilCell Basics: Your First Steps

CoilCell is a compact planar coil designed for various DIY projects. Whether you're just starting or are an experienced maker, CoilCell offers easy integration to simplify your creations. In this tutorial we will explain:

What is a CoilCell and how does it work?
Getting started with its Arduino software library
Program a ball-magnet to bounce every few milliseconds
Make the CoilCell more interactive with the CodeCell sensors
What is CoilCell?
CoilCell is a thin, planar coil built on a multi-layered PCB, with an integrated driver that simplifies controlling magnetic polarity and strength. It is available in two configurations:

1W CoilCell: 70 turns, with a peak magnetic field of 2.3 mT.
2.5W CoilCell: 200 turns, with a peak magnetic field of 10 mT, which can be upgraded to 17 mT using an iron back-plate.
Magnetic Applications

N52 Magnets: Use lightweight N52 ball or disk magnets for dynamic interactions like bouncing or shaking objects by finding their resonant frequency.
FlipDot: Create interactive mechanical pixel by pivoting a magnet to flip. Build your own 3D-printed magnetic FlipDot pixel, and add more for a mini displpay. Both your ears and eyes will love it!
Iron Back-Plate: Upgrade the 2.5W CoilCell to boost its peak strength to 17 mT, turning it into a weak electromagnet suitable for attracting small metallic objects like paper clips.
Magnetic Dice: Perform fun tricks with our special dice containing a hidden magnet inside, allowing you to create automatic shaking or influencing the roll outcome with the 2.5W CoilCell.

Safety Tips
While using the 2.5W 200-Turns CoilCell, it can potentially heating up to 110°C, especially when combined with the iron back-plate. Follow these precautions:

Keep hands away from hot surfaces and turn off the coil when not in use.
Ensure that 3D-printed parts and materials can withstand high temperatures. 
Also, use eye protection when working with small magnets that may be repelled at high speeds.
How Does CoilCell Work?
CoilCell utilizes an on-board DRV8837 H-bridge chip to control current flow through the coil, allowing it to switch magnetic polarity:

North: IN1 = VCC/PWM, IN2 = GND
South: IN1 = GND, IN2 = VCC/PWM
Off: IN1 = GND, IN2 = GND
The DRV8837 chip provides overcurrent protection, undervoltage lockout, and thermal shutdown features, ensuring safe operation.



Getting Started with CoilCell
Wiring one of the input-pins to VCC will instantly turn on the CoilCell. But to make it smarter we also developed a Arduino Software Library to make it easier for you to get started.

You will need to write some basic code to tell CoilCell what to do. Don’t worry, it’s quite simple! Start by downloading the "CoilCell" library from the Arduino's Library Manager. Once this is installed, we are ready to control your device. There are multiple examples that can help you get started but next we will breakdown and understand all the functions:

Before we start make sure you connect the CoilCell to your microcontroller -- We recommend using a CodeCell which is pin to pin compatible, the same size, supports all the library functions, and can add wireless control + interactive sensing. 

1. Initialize CoilCell

#include <CoilCell.h>

CoilCell myCoilCell(IN1, IN2); // Replace IN1 and IN2 with your specific pins

void setup() {
    myCoilCell.Init(); // Initializes the CoilCell
}
This code configures the CoilCell, setting it up for magnetic control based on your selected pins and microcontroller.

2. Bounce(bool direction, uint8_t ms_duration)
The Bounce() function makes a magnet bounce up and down. The first parameter, sets the polarity of the CoilCell and the delay_ms, sets the duration for which the magnet is repelled./

myCoilCell.Bounce(true, 20); //Bounce the magnet up for 20ms
3. Buzz(uint16_t us_buzz)
Create a buzzing sound by rapidly alternating the coil’s polarity. Adjust 'us_buzz' to control the frequency of the buzz.

myCoilCell.Buzz(80); // Generates a buzzing effect at 80 microseconds intervals
4. Tone()
This function plays a default tone by making the CoilCell vibrate at different saved frequencies.

myCoilCell.Tone(); // Plays varying tones
5. Drive(bool direction, uint8_t power_percent)
By using the CodeCell or any other ESP32 microcontroller, this function lets control the coil’s magnetic polarity and strength. The magnetic strength is adjusted by the 'power_percent', which controls how far the magnet is pushed from the coil.

myCoilCell.Drive(true, 75); // Drive the coil north with 75% strength
6. Toggle(uint8_t power_percent)
By using the CodeCell or any other ESP32 microcontroller, this function toggles the coil’s polarity at a set power level, useful for simple magnetic flipping actions.

myCoilCell.Toggle(60); // Toggle polarity at 60% power
For other Arduino devices, this command makes the coilcell flip its direction at full power. 

myCoilCell.Toggle(); // Toggle polarity at 100% power
7. Vibrate(bool smooth, uint8_t power_percent, uint16_t vib_speed_ms)

This function flips the coil’s polarity at a specified speed and power. Setting 'smooth' to true creates smoother motions, ideal for slow frequencies below 2 Hz.

myCoilCell.Vibrate(true, 50, 1000); // Smooth vibration at 50% power every 1000 ms
For other Arduino devices, this command makes the coilcell flip its polarity at full power. 

myCoilCell.Vibrate(100); // Vibrate at 100% power every 100 ms
 

Here's an example where we initialize a CoilCell to make a 5mm diameter ball magnet bounce. In this example, the CoilCell is initialized with pins 5 and 6. The setup() function calls myCoilCell.Init() to configure the CoilCell. In the loop(), the Bounce() function is used to make the magnet bounce upwards for 20 milliseconds, followed by a 600 milliseconds delay that attracts the magnet back down.

#include <CoilCell.h>

#define IN1_pin1 5
#define IN1_pin2 6

CoilCell myCoilCell(IN1_pin1, IN1_pin2);

void setup() {
    myCoilCell.Init(); /*Initialize the CoilCell*/
}

void loop() {
    myCoilCell.Bounce(0, 20); /*Bounce the magnet up for 20ms*/
    delay(600); /*Attract the magnet back down for 600ms*/
}
    
In this next example we use the CodeCell's Motion Sensor to detect tapping. When a new tap is detected the CoilCell flips its magnetic polarity and sets a 1 second delay to flash the onboard LED to yellow.

#include <CodeCell.h>
#include <CoilCell.h>

#define IN1_pin1 5
#define IN1_pin2 6

CoilCell myCoilCell(IN1_pin1, IN1_pin2);
CodeCell myCodeCell;

void setup() {
  Serial.begin(115200); /* Set Serial baud rate to 115200. Ensure Tools/USB_CDC_On_Boot is enabled if using Serial. */

  myCodeCell.Init(MOTION_TAP_DETECTOR); /*Initializes Tap Detection Sensing*/
  myCoilCell.Init();
  myCoilCell.Tone();
}

void loop() {
  if (myCodeCell.Run()) {
    /*Runs  every 100ms*/
    if (myCodeCell.Motion_TapRead()) {
      /*If Tap is detected shine the LED Yellow for 1 sec and flip the CoilCell's polarity*/
      myCodeCell.LED(0XA0, 0x60, 0x00U);
      myCoilCell.Toggle(100);
      delay(1000);
    }
  }
}
With these basic functions, you can start experimenting with CoilCell in your projects. Whether you’re controlling magnets, creating interactive displays, or experimenting with magnetic forces, CoilCell provides a simple and effective solution.

If you have any more question about the CoilCell feel free to email us and we will gladly help out!

******************************************************************************

Webpage URL: https://microbots.io/blogs/learn/drivecell-library-guide

DriveCell Basics: Your First Steps

DriveCell is a tiny but powerful device that helps you easily control motors, actuators, and high-power LED lights for your DIY projects. DriveCell makes controlling these components easy, even if you're new to programming or electronics. 


In this tutorial we will explain:

What is a DriveCell and how does it work?
Getting started with its Arduino software library
Control two dc-motors and drive them at different speed
Make the DriveCell more interactive with the CodeCell sensors
What is DriveCell?
Imagine you want to make a small robot and control its motor's speed and direction. This can be complex for beginners and usually would require bulky modules. DriveCell simplifies this process because:

It’s a super small driver (just about the size of your fingertip)
Can be easily soldered with castellated pins
Works with Arduino microcontrollers and has easy to use functions that directly let you control motors, actuators, and LEDs without diving into complicated programming.
How Does DriveCell Work?
DriveCell utilizes an on-board DRV8837 H-bridge chip to control current flow through the output pins, controlled by the state of the input pins:

Forward Current: IN1 = VCC/PWM, IN2 = GND
Reverse Current: IN1 = GND, IN2 = VCC/PWM
Off: IN1 = GND, IN2 = GND
The DRV8837 chip provides overcurrent protection, undervoltage lockout, and thermal shutdown features, ensuring safe operation.


Getting Started with DriveCell
Before you can start using DriveCell, you need to set it up and connect it to your project. Here’s a simple guide to get you started:

Connecting DriveCell: 
First, solder the DriveCell output pins to your motor or actuator, and the input and power pins to your microcontroller. Think of these pins like connecting the pieces of a puzzle, allowing the DriveCell to pass current between the output pins.These output pins are controlled by the state of the input pin ~ so pulling IN1 high, will set OUT1 high, and pulling IN2 high, will set OUT2 high. IN1 and IN2 are usually set in opposite polarities to allow current to pass through. These technical details can be all handled by the DriveCell software library.

Coding DriveCell: 
Wiring one of the input-pins to VCC will instantly turn on the DriveCell. But to make it smarter we also developed a Arduino Software Library to make it easier for you to get started. You will need to write some basic code to tell DriveCell what to do. Don’t worry, it’s quite simple! Start by downloading the "DriveCell" library from the Arduino's Library Manager. Once this is installed, we are ready to control your device. There are multiple examples that can help you get started but next we will breakdown and understand all the functions.
Before we start make sure you connect the DriveCell to your microcontroller. We recommend using a CodeCell which is pin to pin compatible, supports all the library functions, and can add wireless control + interactive sensing. 
 
1. Init()

First we need a basic setup code to get you started:

#include <DriveCell.h> // This line includes the DriveCell library

DriveCell myDriveCell(IN1, IN2); // Replace IN1 and IN2 with your specific pins

void setup() {
    myDriveCell.Init(); // Initializes the DriveCell
}
This code tells the DriveCell to start up and get ready to control your motor or actuator. The Init function makes sure all the necessary peripherals are configured.

2. Pulse(bool direction, uint8_t ms_duration)

This command sends a short burst of power in a specified polarity, to quickly energize the actuator and turn it back off. 

myDriveCell.Pulse(true, 10); // Short burst for 10 milliseconds
3. Buzz(uint16_t us_buzz) 
This makes the actuator vibrate like a buzzer. It’s great for creating sounds for feedback.

myDriveCell.Buzz(100); // Creates a 100us buzzing sound
4. Tone()
This function plays a default tone by making the actuator vibrate at different saved frequencies.

myDriveCell.Tone(); // Plays varying tones
5. Toggle(uint8_t power_percent)
This function simply switches the direction at the full 100% power, which can be useful for reversing the spinning direction of a brushed motor or creating simple flapping movements.

myDriveCell.Toggle(); // Switches direction
By using the CodeCell or any other ESP32 microcontroller, you can also adjust the duty cycle 'power_percent'. For magnetic actuators the 'power_percent' controls the magnetic strength, while for brushed motors this adjusts the speed. 

6. Run(bool smooth, uint8_t power_percent, uint16_t flip_speed_ms)

By using the CodeCell or any other ESP32 microcontroller, this function lets you flip the polarity of an actuator or reverse a motor every 'flip_speed_ms' at the duty cycle 'power_percent'. Setting 'smooth' to 1, smooths out the motion, which is ideal when driving the FlatFlap or CoilPad with slow motions (less than 2Hz).

myDriveCell.Run(true, 50, 1000); // Smooth drive at 50% power every 1000 ms
For other Arduino devices, this command makes the motor/actuator flip its direction (forward and backward) at full speed. For example:

myDriveCell.Run(500); // Motor changes direction every 500 milliseconds
7. Drive(bool direction, uint8_t power_percent)
By using the CodeCell or any other ESP32 microcontroller, this function lets you control the speed and direction of your motor. You can make the motor go forward or backward and adjust how fast it goes.

myDriveCell.Drive(true, 75); // Moves forward at 75% power

Examples:

By using any of these functions in a loop, you can create the desired sequence for your motor, actuator, or high-power LEDs. Here's an example where we initialize two dc-motors and drive them at different speeds:

#include <DriveCell.h>

#define IN1_pin1 2
#define IN1_pin2 3
#define IN2_pin1 5
#define IN2_pin2 6

DriveCell Motor1(IN1_pin1, IN1_pin2);
DriveCell Motor2(IN2_pin1, IN2_pin2);

uint8_t mode = 0;
uint8_t speed_percentage = 0;

void setup() {
  Motor1.Init();
  Motor2.Init();
  speed_percentage = 80; /* Set Motor to 80% power */
}

void loop() {
  delay(3000);
  mode++;
  switch (mode) {
    case 1:
      /* Move forward */
      Motor1.Drive(1, speed_percentage);
      Motor2.Drive(1, speed_percentage);
      break;
    case 2:
      /* Move backward */
      Motor1.Drive(0, speed_percentage);
      Motor2.Drive(0, speed_percentage);
      break;
    case 3:
      /* Turn left */
      Motor1.Drive(1, speed_percentage);
      Motor2.Drive(0, speed_percentage);
      break;
    case 4:
      /* Turn right */
      Motor1.Drive(0, speed_percentage);
      Motor2.Drive(1, speed_percentage);
      break;
    case 5:
      /* Turn off both motors */
      Motor1.Drive(1, 0);
      Motor2.Drive(1, 0);
      if (speed_percentage < 95) {
        speed_percentage += 5; /* Increment speed */
      } else {
        speed_percentage = 50; /* Reset to 50% power */
      }
      mode = 0;
      break;
  }
}
In this next example we use the CodeCell's Proximity Sensor to activate the motors. This sensor will act as a gesture switch, and activate when a hand is within 5cm away. 

#include <CodeCell.h>
#include <DriveCell.h>

#define IN1_pin1 2
#define IN1_pin2 3
#define IN2_pin1 5
#define IN2_pin2 6

CodeCell myCodeCell;
DriveCell Motor1(IN1_pin1, IN1_pin2);
DriveCell Motor2(IN2_pin1, IN2_pin2);

uint8_t speed_percentage = 0;
bool on_flag = 0;

void setup() {
  Serial.begin(115200); /* Set Serial baud rate to 115200. Ensure Tools/USB_CDC_On_Boot is enabled if using Serial. */

  myCodeCell.Init(LIGHT); /*Initializes Light Sensing*/
  Motor1.Init();
  Motor2.Init();
  speed_percentage = 100;
}

void loop() {
  if (myCodeCell.Run()) {
    /*Runs  every 100ms - Put your code here*/
    if (myCodeCell.Light_ProximityRead() > 3000) {
      /*If Tap is detected shine the LED Yellow for 1 sec*/
      myCodeCell.LED(0XA0, 0x60, 0x00U);
      Motor1.Drive(0, 0);
      Motor2.Drive(0, 0);
      delay(1000);
      on_flag = !on_flag;
    }
    if (on_flag) {
      /*Move forward*/
      Motor1.Drive(1, speed_percentage);
      Motor2.Drive(1, speed_percentage);
    } else {
      Motor1.Drive(0, 0);
      Motor2.Drive(0, 0);
    }
  }
}
If you have any more question about the DriveCell feel free to email us and we will gladly help out!

******************************************************************************

Webpage URL: https://microbots.io/blogs/learn/how-does-a-flatflap-work
How does a FlatFlap work?

The FlatFlap is an incredibly thin and innovative actuator that brings motion to your projects in a compact form factor. To understand how it works, let's dive into the unique design and the principles behind its operation.

The Structure

The FlatFlap is designed from a flexible PCB (printed circuit board) and aluminum stiffeners. These components are carefully folded together to form the actuator.

The flexible PCB serves as the foundation of the actuator. Unlike rigid PCBs, the flexible version can bend and twist without breaking, which is essential for creating the flapping motion. The flexibility of the PCB allows the FlatFlap to move freely while still maintaining its structural integrity.


The Aluminum Stiffeners provide the necessary rigidity to hold the magnet that directs the flapping motion, ensuring that the movement is both precise and consistent.

The Magnetic System

The FlatFlap is powered by a clever magnetic system that converts electrical energy into mechanical motion. This system consists of a magnet on the back of the actuator and a planar copper coil embedded within the flexible PCB.

The FlatFlap features a 10mm N52 neodymium magnet attached to its back. This magnet plays a crucial role in the actuator's operation, interacting with the magnetic field generated by the copper coil. This coil is inside the flexible PCB, and is responsible for creating the magnetic field when an electric current is passed through it. Flapping motion is achieved by pulsing the current through the copper coil in different directions.



Depending on the direction of the current, this magnetic field interacts with the magnet on the back of the FlatFlap. By alternating the direction of the current, the magnetic field can either attract or repel the magnet, causing the flap to move. The voltage can also be varied via PWM, to control the distance of the coil from the magnet.

By rapidly pulsing the current in different directions, creating a square wave, the FlatFlap can produce a continuous flapping motion. The speed and frequency of this motion can be controlled by adjusting the rate at which the current is pulsed. In its optimal configuration. The FlatFlap can achieve a speed of up to 25Hz, creating a fast and responsive movement.



Easy Attachment and Secure Installation

Installing the FlatFlap is a breeze, thanks to its peelable adhesive back and optional screws. The adhesive provides a strong bond that keeps the actuator securely in place, while the screws offer an additional layer of security if needed. This dual installation method ensures flawless adhesion, whether you're attaching the FlatFlap to a smooth surface or something more textured.



Ultra-Thin and Compact Design

One of the standout features of the FlatFlap is its incredibly slim profile. With a flap that’s only 0.3mm thin and an actuator measuring just 2.6mm, this sleek design can integrate seamlessly into any flat surface. Its low profile ensures that it doesn't interfere with the aesthetics of your project, making it ideal for applications where space is at a limited.

The FlatFlap is perfect for a wide range of applications. It’s particularly well-suited for creating kinetic sculptures and experimenting with robotics. Its ability to add eye-catching motion to lightweight objects, such as thin 3D-printed plastic or paper origami, opens up a world of creative possibilities. 

******************************************************************************

Webpage URL: https://microbots.io/blogs/learn/how-does-a-pcb-motor-work

How does a PCB Motor work?

A PCB motor is an innovative solution that integrates the motor's mechanics into the electronic components, using the PCB itself as the structure of the motor.

What is a PCB Motor?

A PCB motor is a unique type of motor that uses the copper traces on a printed circuit board (PCB) to create a magnetic field that drives the motor. This concept is inspired by how radio systems use PCB traces as antennas. The same principle is applied to generate a magnetic field strong enough to turn a rotor. This type of motor is known as an axial flux brushless motor, where the PCB stator and the rotor are aligned in parallel.

The Design and Construction of a PCB Motor
The first step in creating a PCB motor is designing the PCB stator coils. In traditional motors, these coils are often made from wire wound into dense, three-dimensional shapes. In a PCB motor, the coils are instead manufactured as flat spiral traces printed onto the layers of a PCB.


One of the challenging aspects of these planar motors is fitting enough turns in the small space available to generate sufficient magnetic flux. These coils have to be connected in a star or delta configuration to create a 3-phase system. In our 6-slotted star-configured MotorCell design, we were able to stack the coils on four layers, utilizing two additional layers for the controller, to produce the required magnetic field to spin the rotor.

Over the years, we have also learned that the design of the rotor is crucial for improving the motor's efficiency. It is important to use high-quality ceramic ball bearings and to align the bearings precisely to achieve the optimal mechanical solution. This typically requires specialized tools, so we also offer the MotorCell 's rotor individually, allowing you to easily integrate it with your custom PCB.

Achieving Synchronization in the PCB Motor
One of the most critical aspects of driving a brushless motor is ensuring that the rotor stays in sync with the stator's electromagnetic field. In traditional motors with brushes, synchronization is mechanically managed by the brushes themselves. However, in a three-phase brushless motor like a PCB motor, sensory feedback is necessary to keep the motor running smoothly.

Back EMF is typically used as feedback to control the motor's speed. Back EMF is the voltage generated by the spinning motor itself, which can be measured to determine the rotor’s speed. This information is then fed into the motor’s control electronics, ensuring that the stator coils are driven in sync with the rotor's motion. For the MotorCell, all of this is handled directly by the onboard chip, simplifying the process.

******************************************************************************

```.ino
// GettingStarted.ino
#include <CodeCell.h>

CodeCell myCodeCell;

/*
 * Overview:
 * This code shows you how easy it is to set up and run the CodeCell. You just needs to specify which sensors to enable by 
 * passing the appropriate macros to the `Init()` function and use the `Run()` function to handle the power and battery management.
 *
 **** Initialization (`Init` function):
 * 
 * To initialize the CodeCell, use the `myCodeCell.Init()` function with one or more of the predefined macros. Each macro
 * corresponds to a specific sensing function. Here are the available macros:
 * 
 * - `LIGHT`                          // Enables Light Sensing
 * - `MOTION_ACCELEROMETER`           // Enables Accelerometer Sensing
 * - `MOTION_GYRO`                    // Enables Gyroscope Sensing
 * - `MOTION_MAGNETOMETER`            // Enables Magnetometer Sensing
 * - `MOTION_LINEAR_ACC`              // Enables Linear Acceleration Sensing
 * - `MOTION_GRAVITY`                 // Enables Gravity Sensing
 * - `MOTION_ROTATION`                // Enables Rotation Sensing
 * - `MOTION_ROTATION_NO_MAG`         // Enables Rotation Sensing without Magnetometer
 * - `MOTION_STEP_COUNTER`            // Enables Walking Step Counter
 * - `MOTION_STATE`                   // Enables Motion State Detection
 * - `MOTION_TAP_DETECTOR`            // Enables Tap Detector
 * - `MOTION_ACTIVITY`                // Enables Motion Activity Recognition
 * 
 * Example Usage:
 * - `myCodeCell.Init(LIGHT);`                                      // Initializes Light Sensing
 * - `myCodeCell.Init(LIGHT + MOTION_ROTATION);`                    // Initializes Light Sensing and Angular Rotation Sensing
 * - `myCodeCell.Init(LIGHT + MOTION_ROTATION + MOTION_STATE);`     // Initializes Light Sensing, Angular Rotation Sensing, and State Detection
 * 
 * Note: You can combine multiple macros using the `+` operator to initialize multiple sensors.
 * 
 **** Running the Code (`Run` function):
 * 
 * Call the `myCodeCell.Run()` function in the `loop()` to manage battery and power. This function returns true every 100ms and
 * also handles the onboard LED to indicate power status. When the battery voltage falls below 3.3V, the LED will blink red 10 times 
 * and then go into Sleep Mode until the USB cable is connected for charging. While charging, the CodeCell will shut down the application, 
 * light the LED blue, and wait until the battery is fully charged. Once fully charged, it will start a breathing-light animation with a speed
 * corresponding to the proximity distance. The LED will shine green when powered by the battery and blue when powered via USB.
 * 
 **** Reading Sensor Data:
 * 
 * After initializing the sensors, you can use the following functions to read data from them:
 * 
 * Sensor Read Functions:
 * 
 * - `Light_ProximityRead()`                                            // Reads the proximity value from the light sensor
 * - `Light_WhiteRead()`                                                // Reads the white light intensity from the light sensor
 * - `Light_AmbientRead()`                                              // Reads the ambient light intensity from the light sensor
 * - `Motion_TapRead()`                                                  // Reads the number of taps detected (tap = 1, no tap = 0)
 * - `Motion_StateRead()`                                               // Reads the current state (On Table = 1, Stationary = 2, Stable = 3, Motion = 4)
 * - `Motion_ActivityRead()`                                            // Reads the current activity (Driving = 1, Cycling = 2, Walking = 3/6, Still = 4, Tilting = 5, Running = 7, Climbing Stairs = 8)
 * - `Motion_AccelerometerRead(float &x, float &y, float &z)`           // Reads acceleration data along the x, y, and z axes
 * - `Motion_GyroRead(float &x, float &y, float &z)`                    // Reads rotational velocity data along the x, y, and z axes
 * - `Motion_MagnetometerRead(float &x, float &y, float &z)`            // Reads magnetic field strength data along the x, y, and z axes
 * - `Motion_GravityRead(float &x, float &y, float &z)`                 // Reads gravity vector data along the x, y, and z axes
 * - `Motion_LinearAccRead(float &x, float &y, float &z)`               // Reads linear acceleration data along the x, y, and z axes
 * - `Motion_StepCounterRead(uint16_t &x)`                              // Reads the number of steps counted
 * - `Motion_RotationRead(float &roll, float &pitch, float &yaw)`       // Reads angular rotational data (roll, pitch, yaw)
 * - `Motion_RotationNoMagRead(float &roll, float &pitch, float &yaw)`  // Reads angular rotational data without magnetometer
 *
 * Example Usage:
 * - `uint16_t proximity = myCodeCell.Light_ProximityRead();`
 * - `myCodeCell.Motion_AccelerometerRead(myX, myY, myZ);`
 * - `myCodeCell.Motion_RotationRead(myRoll, myPitch, myYaw);`
 * 
 * Note: You can use `myCodeCell.PrintSensors()`to prints the values of all enabled sensors on the Serial Monitor.
 * 
 */

void setup() {
  Serial.begin(115200); /* Set Serial baud rate to 115200. Ensure Tools/USB_CDC_On_Boot is enabled if using Serial. */

  myCodeCell.Init(LIGHT); /*Initializes Light Sensing*/
}

void loop() {
  if (myCodeCell.Run()) {
    /*Runs  every 100ms - Put your code here*/
    myCodeCell.PrintSensors(); /*Print Sensors data every 100ms*/
  }
}
```

******************************************************************************
```
// LightAutoDimming.ino

/*
 * Overview:
 * This code demonstrates the CodeCell's Light Sensor.
 * In this example, the CodeCell continuously measures white light and updates the brightness of the LED according to how dark it gets.
 * The LED will get dimmer as the room gets darker - Feel free to tweak the code with your own creative ideas! 
 * Learn more about this example here - https://microbots.io/blogs/learn/codecell-auto-dimming
 */

#include <CodeCell.h>

CodeCell myCodeCell;

void setup() {
    Serial.begin(115200); // Set Serial baud rate to 115200. Ensure Tools/USB_CDC_On_Boot is enabled if using Serial.
    myCodeCell.Init(LIGHT); // Initializes light sensing.
}

void loop() {
    delay(100); // Small delay - You can adjust it accordingly
    
    // Read white light from the sensor and adjust brightness for 8-bit
    uint16_t brightness = (myCodeCell.Light_WhiteRead()) >> 3; 
    Serial.println(brightness); // Print the brightness value to the serial monitor for debugging.

    // Limit the sensor values to the LED's brightness range (1 to 254)
    if (brightness == 0U) {
        brightness = 1U; // Set a minimum brightness to avoid turning off the LED completely
    } else if (brightness > 254U) {
        brightness = 254U; // Cap the brightness to the LED's maximum level
    }

    brightness = 255U - brightness; // Invert the brightness so the LED dims as it gets brighter
    myCodeCell.LED(0, 0, brightness); // Shine the onboard blue RGB LED with the new adjusted brightness
}
```
******************************************************************************

```
// LightDepthGesture.ino
/*
 * Overview:
 * In this example, we'll explore how to use the proximity sensor to read depth gestures and brings your Flappy Bot to life! 
 * With this example the CodeCell controls two FlatFlaps that stops flapping when proximity is detected.
 * Their angle gets adjusted dynamically based on how close something gets.
 * Feel free to tweak the code with your own creative ideas, or add motion sensing for a new reaction! 
 * Learn more about this example here - https://microbots.io/blogs/learn/codecell-depth-gestures
 */

#include <CodeCell.h>
#include <DriveCell.h>

#define IN1_pin1 2
#define IN1_pin2 3
#define IN2_pin1 5
#define IN2_pin2 6

DriveCell FlatFlap1(IN1_pin1, IN1_pin2);
DriveCell FlatFlap2(IN2_pin1, IN2_pin2);

CodeCell myCodeCell;

void setup() {
  Serial.begin(115200); /* Set Serial baud rate to 115200. Ensure Tools/USB_CDC_On_Boot is enabled if using Serial. */

  myCodeCell.Init(LIGHT); /*Initializes Light Sensing*/

  FlatFlap1.Init();
  FlatFlap2.Init();

  FlatFlap1.Tone();
  FlatFlap2.Tone();
}

void loop() {
  if (myCodeCell.Run()) {
    /*Runs  every 100ms*/
    uint16_t proximity = myCodeCell.Light_ProximityRead();
    Serial.println(proximity);
    if (proximity < 100) {
      FlatFlap1.Run(1, 100, 400);
      FlatFlap2.Run(1, 100, 400);
    } else {
      proximity = proximity - 100;
      proximity = proximity / 10;
      if (proximity > 100) {
        proximity = 100;
      }
      FlatFlap1.Drive(0, (proximity));
      FlatFlap2.Drive(0, (proximity));
    }
  }
}
```
******************************************************************************

```
// Light_ProximityDetector.ino
/*
 * Overview:
 * This code demonstrates the CodeCell's prximity sensing.
 * In this example, the CodeCell continuously measures proximity and set the onboard LED to Red when proximity is detected
 * Feel free to tweak the code with your own creative ideas! 
 * Learn more about this example here - https://microbots.io/blogs/learn/codecell-proximity-sensing
 */

#include <CodeCell.h>
CodeCell myCodeCell;

void setup() {
    Serial.begin(115200); // Set Serial baud rate to 115200. Ensure Tools/USB_CDC_On_Boot is enabled if using Serial
    myCodeCell.Init(LIGHT); // Initializes light sensing
}

void loop() {
    if (myCodeCell.Run()) {
        // Runs every 100ms to check proximity
        uint16_t proximity = myCodeCell.Light_ProximityRead();
        
        // Check if an object is within range
        if (proximity > 100) {
            myCodeCell.LED(0xFF, 0, 0); // Set LED to Red when proximity is detected
            delay(1000); // Keep the LED on for 1 second
        } else {
            // No action if the object is out of range
        }
    }
}
```
******************************************************************************
```
// Light_ProxRemote_Receiver.ino
/*
 * Overview:
 * This code demonstrates how to configure the CodeCell's ESP32-C3 to be used as a WiFi remote.
 * In this example, the CodeCell (Device 2) will receive proximity data from Device 1 and adjusts the LED based on the received data. 
 * Feel free to tweak the code with your own creative ideas!
 * Learn more about this example here - https://microbots.io/blogs/learn/codecell-wifi-remote
 */

#include <esp_now.h>
#include <WiFi.h>
#include <CodeCell.h>

CodeCell myCodeCell;

void setup() {
  Serial.begin(115200);
  myCodeCell.Init(LIGHT);  // Initializes Light Sensing

  // Initialize WiFi in Station mode
  WiFi.mode(WIFI_STA);
  Serial.println(WiFi.macAddress());

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register the receive callback
  esp_now_register_recv_cb(onDataRecv);
}

// Receive callback function
void onDataRecv(const esp_now_recv_info *recvInfo, const uint8_t *incomingData, int len) {
  uint16_t Remote_Value;
  memcpy(&Remote_Value, incomingData, sizeof(Remote_Value));

  Serial.println(Remote_Value);
  myCodeCell.LED(0, Remote_Value, 0);
}

void loop() {
  // Nothing to do here
}
```
******************************************************************************
```
// Light_ProxRemote_Sender.ino
/*
 * Overview:
 * This code demonstrates how to configure the CodeCell's ESP32-C3 to be used as a WiFi remote
 * In this example, the CodeCell (Device 1) reads the proximity sensor and sends it to Device 2
 * Feel free to tweak the code with your own creative ideas!
 * Learn more about this example here - https://microbots.io/blogs/learn/codecell-wifi-remote
 */

#include <esp_now.h>
#include <WiFi.h>
#include <CodeCell.h>

CodeCell myCodeCell;
uint8_t receiverMAC[] = { 0x18, 0x8B, 0x0E, 0x06, 0xB5, 0xCC };  // Replace with your receiver's MAC address

void setup() {
  Serial.begin(115200);

  myCodeCell.Init(LIGHT);  // Initializes Light Sensing

  // Initialize WiFi in Station mode
  WiFi.mode(WIFI_STA);
  Serial.println(WiFi.macAddress());

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register peer
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, receiverMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
}

void loop() {
  if (myCodeCell.Run()) {
    uint16_t ProxRead = (myCodeCell.Light_ProximityRead()) >> 4;  //Get Proximity Value and Divide by 16
    Serial.println(ProxRead);
    esp_err_t result = esp_now_send(receiverMAC, (uint8_t *)&ProxRead, sizeof(ProxRead));

    if (result == ESP_OK) {
      Serial.println("Data sent successfully");
    } else {
      Serial.println("Sending Error");
    }
  }
}
```

******************************************************************************
```
// Light_SleepExample.ino
/*
 * Overview:
 * This code illustrates a CodeCell that wakes-up upon detecting proximity and enters a low-power sleep mode when no proximity is present.
 * During sleep, the CodeCell consumes approximately 750uA and briefly wakes up every second for 100ms to check proximity, consuming around 15mA during this short time.
 * You can modify the code for your specific application needs, or configure it to wake up using a different sensor.
 */

#include <CodeCell.h>

CodeCell myCodeCell;

void setup() {
  Serial.begin(115200); /*If not enabled, set Serial baud rate to 115200*/

  delay(60);
  if (myCodeCell.WakeUpCheck()) {
    /*Waking up from Sleep - Initialize sensor*/
    while (myCodeCell.Light_Init() == 1) {
      delay(1);
      myCodeCell.LightReset(); /*If sensor not set up, reset it*/
    }
    delay(40);
    if (myCodeCell.Light_ProximityRead() < 10) {
      myCodeCell.Sleep(1); /*If Proxity still not detected go to sleep & check again after 1 sec*/
    }
  }

  myCodeCell.Init(LIGHT); /*Initializes CodeCell*/
}

void loop() {
  if (myCodeCell.Run()) {
    /*Runs  every 100ms*/
    if (myCodeCell.Light_ProximityRead() < 10) {
      myCodeCell.Sleep(1); /*If Proxity not detected go to sleep & check again after 1 sec*/
    }
  }
}
```
******************************************************************************

```
// Motion_AngularRemote_Receiver.ino
/*
 * Overview:
 * This code demonstrates how to configure the CodeCell's ESP32-C3 to be used as a WiFi remote.
 * In this example, we connect two motors with a two DriveCells to this receiver.
 * The CodeCell (Device 2) will receive angular data from Device 1 and adjusts the speed of two motors based on the received data. 
 * Feel free to tweak the code with your own creative ideas!
 * Learn more about this example here - https://microbots.io/blogs/learn/codecell-wifi-remote
 */
 
#include <esp_now.h>
#include <WiFi.h>
#include <CodeCell.h>
#include <DriveCell.h>

#define IN1_pin1 2
#define IN1_pin2 3
#define IN2_pin1 5
#define IN2_pin2 6

CodeCell myCodeCell;
DriveCell Motor1(IN1_pin1, IN1_pin2);
DriveCell Motor2(IN2_pin1, IN2_pin2);

void setup() {
  Serial.begin(115200);
  myCodeCell.Init(LIGHT);  // Initializes Light Sensing
  Motor1.Init();
  Motor2.Init();

  // Initialize WiFi in Station mode
  WiFi.mode(WIFI_STA);
  Serial.println(WiFi.macAddress());

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register the receive callback
  esp_now_register_recv_cb(onDataRecv);
}

// Receive callback function
void onDataRecv(const esp_now_recv_info *recvInfo, const uint8_t *incomingData, int len) {
  int Roll_Speed = 0;
  memcpy(&Roll_Speed, incomingData, sizeof(Roll_Speed));

  if (Roll_Speed > 50) {
    Motor1.Drive(1, Roll_Speed);
    Motor2.Drive(1, Roll_Speed);
  } else {
    Roll_Speed = 100 - Roll_Speed;  //Add Motor Dead Zone
    Motor1.Drive(0, Roll_Speed);
    Motor2.Drive(0, Roll_Speed);
  }
  Serial.println(Roll_Speed);
}

void loop() {
  if (myCodeCell.Run()) {}
}
```
******************************************************************************

```
// Motion_Angular_RemoteSender.ino
/*
 * Overview:
 * This code demonstrates how to configure the CodeCell's ESP32-C3 to be used as a WiFi remote
 * In this example, the CodeCell (Device 1) reads angular data from its motion sensors and sends it to Device 2
 * Feel free to tweak the code with your own creative ideas!
 * Learn more about this example here - https://microbots.io/blogs/learn/codecell-wifi-remote
 */

#include <esp_now.h>
#include <WiFi.h>
#include <CodeCell.h>

CodeCell myCodeCell;
uint8_t receiverMAC[] = {0x84, 0xFC, 0xE6, 0xFC, 0xEC, 0xA0};  // Replace with your receiver's MAC address
int Roll_Control = 0;

float Roll = 0.0;
float Pitch = 0.0;
float Yaw = 0.0;

void setup() {
  Serial.begin(115200);

  myCodeCell.Init(MOTION_ROTATION);  // Initializes Light Sensing

  // Initialize WiFi in Station mode
  WiFi.mode(WIFI_STA);
  Serial.println(WiFi.macAddress());

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register peer
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, receiverMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
}

void loop() {
  if (myCodeCell.Run()) {
    myCodeCell.Motion_RotationRead(Roll, Pitch, Yaw);

    Roll = Roll + 180;
    Roll = (Roll * 100)/180;
    if (Roll > 200) {
      Roll = 200;
    } else if (Roll < 0) {
      Roll = 0;
    } else {
      //Skip
    }
    Roll = Roll / 2;
    Roll_Control = (uint8_t)Roll;
    Serial.println(Roll_Control);

    esp_err_t result = esp_now_send(receiverMAC, (uint8_t *)&Roll_Control, sizeof(Roll_Control));

    if (result == ESP_OK) {
      Serial.println("Data sent successfully");
    } else {
      Serial.println("Sending Error");
    }
  }
}
```
******************************************************************************

```
// Motion_PersonalActivity.ino
/*
 * Overview:
 * In this example we'll explore how to configure the CodeCell's onboard motion sensor to try and guess the personal activate you're doing. a
 * This activity guess will be read every 1000ms and displayed on an OLED display.
 * Feel free to tweak the code with your own creative ideas! 
 * Learn more about this example here - https://microbots.io/blogs/learn/codecell-personal-activity-guessing
 */

#include <CodeCell.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <CodeCell.h>

CodeCell myCodeCell;

/*Configure up the OLED Displaay*/
#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 32  // OLED display height, in pixels

#define OLED_RESET -1        // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C  ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

int read_timer = 0;

void setup() {
  Serial.begin(115200); /* Set Serial baud rate to 115200. Ensure Tools/USB_CDC_On_Boot is enabled if using Serial. */

  myCodeCell.Init(MOTION_ACTIVITY); /*Initializes Step Counter & Activity Sensing*/

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.display();
  delay(2000);
}

void loop() {
  if (myCodeCell.Run()) {
    /*Runs  every 100ms*/
    if (read_timer < 10) {
      read_timer++;
    } else {
      /*Update every 1 sec*/
      read_timer = 0;
      display.clearDisplay();
      display.setCursor(32, 16);
      display.print(F("Activity: "));
      display.setCursor(32, 24);
      switch (myCodeCell.Motion_ActivityRead()) {
        case 1:
          display.print("Driving");
          break;
        case 2:
          display.print("Cycling");
          break;
        case 3:
        case 6:
          display.print("Walking");
          break;
        case 4:
          display.print("Still");
          break;
        case 5:
          display.print("Tilting");
          break;
        case 7:
          display.print("Running");
          break;
        case 8:
          display.print("Stairs");
          break;
        default:
          display.print("Reading..");
          break;
      }
      display.display();
    }
  }
}
```
******************************************************************************
```
// Motion_RollPitchYaw.ino
/*
 * Overview:
 * This code demonstrates the CodeCell's Rotation Sensing features.
 * In this example, the CodeCell continuously reads the angular rotations 
 * Every 100ms it prints the Roll, Pitch, and Yaw angles on the Serial Monitor
 * Feel free to tweak the code with your own creative ideas!
 */

#include <CodeCell.h>

CodeCell myCodeCell;

float Roll = 0.0;
float Pitch = 0.0;
float Yaw = 0.0;

void setup() {
  Serial.begin(115200); /* Set Serial baud rate to 115200. Ensure Tools/USB_CDC_On_Boot is enabled if using Serial. */

  myCodeCell.Init(MOTION_ROTATION); /*Initializes Rotation Sensing*/
}

void loop() {
  if (myCodeCell.Run()) {
    /*Runs  every 100ms*/
    myCodeCell.Motion_RotationRead(Roll, Pitch, Yaw);
    Serial.printf("Roll: %.2f°, Pitch: %.2f°, Yaw: %.2f°\n", Roll, Pitch, Yaw);
  }
}
```
******************************************************************************

```
// Motion_ServoAngle.ino
/*
 * Overview:
 * This code demonstrates the CodeCell's Rotation Sensing features.
 * In this example, the CodeCell continuously reads the angular rotations and use these angles to control a servo motor
 * Feel free to tweak the code with your own creative ideas!
 * Learn more about this example here - https://microbots.io/blogs/learn/codecell-servo-angle-control
 */

#include <CodeCell.h>
#include <ESP32Servo.h>

CodeCell myCodeCell;
Servo myservo;  

float Roll = 0.0;
float Pitch = 0.0;
float Yaw = 0.0;
int servo_angle = 0;

void setup() {
    Serial.begin(115200); /*Set Serial baud rate to 115200. Ensure Tools/USB_CDC_On_Boot is enabled if using Serial*/
    
    myCodeCell.Init(MOTION_ROTATION); /*Initializes rotation sensing */
    myservo.attach(1);  /*Attaches the servo on pin 1 to the servo object*/
}

void loop() {
    if (myCodeCell.Run()) { /*Runs  every 100ms*/
        
        myCodeCell.Motion_RotationRead(Roll, Pitch, Yaw); /* Read rotation angles from the BNO085 sensor*/
        
        
        servo_angle = abs((int)Pitch);/*Convert the pitch angle to a servo angle*/
        servo_angle = (180 - servo_angle);
        
        /*Limit the servo angle to the range 0-60 degrees*/
        if (servo_angle > 60) {
            servo_angle = 60;
        } else if (servo_angle < 0) {
            servo_angle = 0;
        }
        
        Serial.println(servo_angle); /* Print the servo angle for debugging */
        myservo.write(servo_angle);  /* Set the servo position */
    }
}
```
******************************************************************************

```
// Motion_StateRead.ino
/*
 * Overview:
 * This code demonstrates the CodeCell's Motion State Sensing feature.
 * In this example, the CodeCell continuously monitors its motion status.
 * On the Serial Monitor, it outputs whether it's On-Table, In Motion, Stabilizing, or Stationary.
 * Feel free to tweak the code with your own creative ideas! 
 */

#include <CodeCell.h>

CodeCell myCodeCell;

void setup() {
  Serial.begin(115200); /* Set Serial baud rate to 115200. Ensure Tools/USB_CDC_On_Boot is enabled if using Serial. */

  myCodeCell.Init(MOTION_STATE); /*Initializes Motion State Sensing*/
}

void loop() {
  if (myCodeCell.Run()) {
    /*Runs  every 100ms*/
    Serial.print("State: ");
    switch (myCodeCell.Motion_StateRead()) {
      case MOTION_STATE_STABLE:
        Serial.println("Motion Stopped - Stabilizing");
        break;
      case MOTION_STATE_ONTABLE:
        Serial.println("On Table");
        break;
      case MOTION_STATE_STATIONARY:
        Serial.println("Stationary");
        break;
      case MOTION_STATE_MOTION:
        Serial.println("In Motion");
        break;
      default:
        Serial.println("Unkown");
        break;
    }
  }
}
```
******************************************************************************
```
// Motion_StepCounter.ino
/*
 * Overview:
 * In this example we'll explore how to use the CodeCell's onboard motion sensor to measure step counts.
 * These counts are read every 100ms and displayed on an OLED display.
 * Feel free to tweak the code with your own creative ideas! 
 * Learn more about this example here - https://microbots.io/blogs/learn/codecell-step-counter
 */
#include <CodeCell.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

CodeCell myCodeCell;

/*Configure up the OLED Displaay*/
#define SCREEN_WIDTH 128  // OLED display width, in pixels
#define SCREEN_HEIGHT 32  // OLED display height, in pixels

#define OLED_RESET -1        // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C  ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

uint16_t step_counter = 0;

void setup() {
  Serial.begin(115200); /* Set Serial baud rate to 115200. Ensure Tools/USB_CDC_On_Boot is enabled if using Serial. */

  myCodeCell.Init(MOTION_STEP_COUNTER); /*Initializes Step Counter Sensing*/

  /*Set up OLED display*/
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("Display Error");
  }

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.display();
  delay(2000);
}

void loop() {
  if (myCodeCell.Run()) {
    /*Runs  every 100ms*/    
    myCodeCell.Motion_StepCounterRead(step_counter); /*Read if a new Step was made*/

    /*Update Display*/
    display.clearDisplay();
    display.setCursor(32, 16);  // Start at top-left corner
    display.print(F("Steps: "));
    display.print(step_counter);
    display.display();    
  }
}
```
******************************************************************************

```
// Motion_TapDetection.ino
/*
 * Overview:
 * This code demonstrates the CodeCell's Tap Detection feature.
 * In this example, the CodeCell continuously searches for a tap. When one is detected, it shines the LED yellow for 1 second.
 * Feel free to tweak the code with your own creative ideas! 
 * Learn more about this example here - https://microbots.io/blogs/learn/codecell-tap-detection
 */

#include <CodeCell.h>

CodeCell myCodeCell;

void setup() {
    Serial.begin(115200); // Set Serial baud rate to 115200. Ensure Tools/USB_CDC_On_Boot is enabled if using Serial
    myCodeCell.Init(MOTION_TAP_DETECTOR); // Initializes tap detection sensing
}

void loop() {
    if (myCodeCell.Run()) {
        // Runs every 100ms to check for taps
        if (myCodeCell.Motion_TapRead()) {
            // If a tap is detected, shine the LED yellow for 1 second
            myCodeCell.LED(0xA0, 0x60, 0x00); // Set LED to yellow
            delay(1000); // Keep the LED on for 1 second
        }
    }
}
```

File: src/BNO085.cpp

#include "BNO085.h"

static TwoWire *_i2cPort = NULL;		//The generic connection to user's chosen I2C hardware
static uint8_t _address = 0x4A; //Keeps track of I2C address. setI2CAddress changes this.

static sh2_SensorValue_t *_sensor_value = NULL;
static bool _reset_occurred = false;

static int i2chal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len);
static int i2chal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us);
static void i2chal_close(sh2_Hal_t *self);
static int i2chal_open(sh2_Hal_t *self);

static uint32_t hal_getTimeUs(sh2_Hal_t *self);
static void hal_callback(void *cookie, sh2_AsyncEvent_t *pEvent);
static void sensorHandler(void *cookie, sh2_SensorEvent_t *pEvent);

size_t _maxBufferSize = 32;
size_t maxBufferSize();		


bool BNO085::begin(TwoWire &wirePort)
{
  	_address = 0x4A;
  	_i2cPort = &wirePort;

  	if (isConnected() == false) {
    	return (false); /*no response*/
    }

    _HAL.open = i2chal_open;
    _HAL.close = i2chal_close;
    _HAL.read = i2chal_read;
    _HAL.write = i2chal_write;
    _HAL.getTimeUs = hal_getTimeUs;

    return _init();
}

bool BNO085::_init(int32_t sensor_id) {
  int status;

  // Open SH2 interface (also registers non-sensor event handler.)
  status = sh2_open(&_HAL, hal_callback, NULL);
  if (status != SH2_OK) {
    return false;
  }

  // Check connection partially by getting the product id's
  memset(&prodIds, 0, sizeof(prodIds));
  status = sh2_getProdIds(&prodIds);
  if (status != SH2_OK) {
    return false;
  }

  // Register sensor listener
  sh2_setSensorCallback(sensorHandler, NULL);

  return true;
}

bool BNO085::getSensorEvent() {
  _sensor_value = &sensorValue;

  _sensor_value->timestamp = 0;

  sh2_service();

  if (_sensor_value->timestamp == 0 && _sensor_value->sensorId != SH2_GYRO_INTEGRATED_RV) {
    // no new events
    return false;
  }

  return true;
}

bool BNO085::enableReport(sh2_SensorId_t sensorId, uint32_t interval_us,
							   	   uint32_t sensorSpecific) {
  static sh2_SensorConfig_t config;

  // These sensor options are disabLED or not used in most cases
  config.changeSensitivityEnabled = false;
  config.wakeupEnabled = false;
  config.changeSensitivityRelative = false;
  config.alwaysOnEnabled = false;
  config.changeSensitivity = 0;
  config.batchInterval_us = 0;
  config.sensorSpecific = sensorSpecific;

  config.reportInterval_us = interval_us;
  
  int status = sh2_setSensorConfig(sensorId, &config);

  if (status != SH2_OK) {
    return false;
  }

  return true;
}

static int i2chal_open(sh2_Hal_t *self) {
  
  uint8_t softreset_pkt[] = {5, 0, 1, 0, 1};
  bool success = false;
  for (uint8_t attempts = 0; attempts < 5; attempts++) {
    if (I2CWrite(_address,softreset_pkt, 5)) {
      success = true;
      break;
    }
    delay(30);
  }
  if (!success)
    return -1;
  delay(300);
  return 0;
}

static void i2chal_close(sh2_Hal_t *self) {
}

static int i2chal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us) {
  uint8_t header[4];
  if (!I2CRead(_address,header, 4)) {
    return 0;
  }

  uint16_t packet_size = (uint16_t)header[0] | (uint16_t)header[1] << 8;
  // Unset the "continue" bit
  packet_size &= ~0x8000;


  size_t i2c_buffer_max = maxBufferSize();

  if (packet_size > len) {
    return 0;
  }
  uint16_t cargo_remaining = packet_size;
  uint8_t i2c_buffer[i2c_buffer_max];
  uint16_t read_size;
  uint16_t cargo_read_amount = 0;
  bool first_read = true;

  while (cargo_remaining > 0) {
    if (first_read) {
      read_size = min(i2c_buffer_max, (size_t)cargo_remaining);
    } else {
      read_size = min(i2c_buffer_max, (size_t)cargo_remaining + 4);
    }

    if (!I2CRead(_address,i2c_buffer, read_size)) {
      return 0;
    }

    if (first_read) {
      // The first time we're saving the "original" header, so include it in the
      // cargo count
      cargo_read_amount = read_size;
      memcpy(pBuffer, i2c_buffer, cargo_read_amount);
      first_read = false;
    } else {
      // this is not the first read, so copy from 4 bytes after the beginning of
      // the i2c buffer to skip the header included with every new i2c read and
      // don't include the header in the amount of cargo read
      cargo_read_amount = read_size - 4;
      memcpy(pBuffer, i2c_buffer + 4, cargo_read_amount);
    }
    // advance our pointer by the amount of cargo read
    pBuffer += cargo_read_amount;
    // mark the cargo as received
    cargo_remaining -= cargo_read_amount;
  }
  return packet_size;
}

static int i2chal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len) {
  size_t i2c_buffer_max = maxBufferSize();

  uint16_t write_size = min(i2c_buffer_max, len);

  if (!I2CWrite(_address,pBuffer, write_size)) {
    return 0;
  }

  return write_size;
}


static uint32_t hal_getTimeUs(sh2_Hal_t *self) {
  uint32_t t = millis() * 1000;
  // Serial.print("I2C HAL get time: %d\n", t);
  return t;
}

static void hal_callback(void *cookie, sh2_AsyncEvent_t *pEvent) {
  // If we see a reset, set a flag so that sensors will be reconfigured.
  if (pEvent->eventId == SH2_RESET) {
    // Serial.println("Reset!");
    _reset_occurred = true;
  }
}

// Handle sensor events.
static void sensorHandler(void *cookie, sh2_SensorEvent_t *event) {
  int rc;

  // Serial.println("Got an event!");

  rc = sh2_decodeSensorEvent(_sensor_value, event);
  if (rc != SH2_OK) {
    //Serial.println("BNO085 - Error decoding sensor event");
    _sensor_value->timestamp = 0;
    return;
  }
}

//Return the sensorID
uint8_t BNO085::getSensorEventID()
{
	return _sensor_value->sensorId;
}

//Returns true if I2C device ack's
bool BNO085::isConnected()
{
  	_i2cPort->beginTransmission((uint8_t)_address);
  	if (_i2cPort->endTransmission() != 0)
    	return (false); //Sensor did not ACK
  	return (true);
}

bool I2CWrite(uint8_t add, uint8_t *buffer, size_t size) {
  Wire.beginTransmission(add);
  // Write the data buffer
  if (Wire.write(buffer, size) != size) {
    // If the number of bytes written is not equal to the length, return false
    Wire.endTransmission();  // Ensure to end transmission even if writing fails
    return false;
  }

  if (Wire.endTransmission() == 0) {
    return true;
  } else {
    return false;
  }
}


bool I2CRead(uint8_t add, uint8_t *buffer, size_t size) {
  size_t pos = 0;
  while (pos < size) {
    size_t read_size;
    if ((size - pos) > maxBufferSize()) {
      read_size = maxBufferSize();
    } else {
      read_size = size - pos;
    }
    
    size_t recv =  Wire.requestFrom(add, read_size);

    if (recv != size) {
      return false;
    }

    for (uint16_t i = 0; i < size; i++) {
      buffer[i] = Wire.read();
    }
    pos += read_size;
  }
  return true;
}

size_t maxBufferSize() { return _maxBufferSize; }


float BNO085::getRot_I()
{
	return _sensor_value->un.rotationVector.i;
}

float BNO085::getRot_J()
{
	return _sensor_value->un.rotationVector.j;
}

float BNO085::getRot_K()
{
	return _sensor_value->un.rotationVector.k;
}

float BNO085::getRot_R()
{
	return _sensor_value->un.rotationVector.real;
}

//Return the rotation vector radian accuracy
float BNO085::getRadianAccuracy()
{
	return _sensor_value->un.rotationVector.accuracy;
}

//Return the rotation vector sensor event report status accuracy
uint8_t BNO085::getRot_Accuracy()
{
	return _sensor_value->status;
}

//Return the game rotation vector quaternion I
float BNO085::getGameI()
{
	return _sensor_value->un.gameRotationVector.i;
}

//Return the game rotation vector quaternion J
float BNO085::getGameJ()
{
	return _sensor_value->un.gameRotationVector.j;
}

//Return the game rotation vector quaternion K
float BNO085::getGameK()
{
	return _sensor_value->un.gameRotationVector.k;
}

//Return the game rotation vector quaternion Real
float BNO085::getGameReal()
{
	return _sensor_value->un.gameRotationVector.real;
}

//Return the acceleration component
float BNO085::getAccelX()
{
	return _sensor_value->un.accelerometer.x;
}

//Return the acceleration component
float BNO085::getAccelY()
{
	return _sensor_value->un.accelerometer.y;
}

//Return the acceleration component
float BNO085::getAccelZ()
{
	return _sensor_value->un.accelerometer.z;
}

//Return the acceleration component
uint8_t BNO085::getAccelAccuracy()
{
	return _sensor_value->status;
}

float BNO085::getLinAccelX()
{
	return _sensor_value->un.linearAcceleration.x;
}

//Return the acceleration component
float BNO085::getLinAccelY()
{
	return _sensor_value->un.linearAcceleration.y;
}

//Return the acceleration component
float BNO085::getLinAccelZ()
{
	return _sensor_value->un.linearAcceleration.z;
}

//Return the acceleration component
uint8_t BNO085::getLinAccelAccuracy()
{
	return _sensor_value->status;
}

//Return the gyro component
float BNO085::getGyroX()
{
	return _sensor_value->un.gyroscope.x;
}

//Return the gyro component
float BNO085::getGyroY()
{
	return _sensor_value->un.gyroscope.y;
}

//Return the gyro component
float BNO085::getGyroZ()
{
	return _sensor_value->un.gyroscope.z;
}

//Return the gyro component
uint8_t BNO085::getGyroAccuracy()
{
	return (gyroAccuracy);
}

//Return the gyro component
float BNO085::getUncalibratedGyroX()
{
	return _sensor_value->un.gyroscopeUncal.x;
}
//Return the gyro component
float BNO085::getUncalibratedGyroY()
{
	return _sensor_value->un.gyroscopeUncal.y;
}
//Return the gyro component
float BNO085::getUncalibratedGyroZ()
{
	return _sensor_value->un.gyroscopeUncal.z;
}
//Return the gyro component
float BNO085::getUncalibratedGyroBiasX()
{
	return _sensor_value->un.gyroscopeUncal.biasX;
}
//Return the gyro component
float BNO085::getUncalibratedGyroBiasY()
{
	return _sensor_value->un.gyroscopeUncal.biasY;
}
//Return the gyro component
float BNO085::getUncalibratedGyroBiasZ()
{
	return _sensor_value->un.gyroscopeUncal.biasZ;
}

//Return the gyro component
uint8_t BNO085::getUncalibratedGyroAccuracy()
{
	return (UncalibGyroAccuracy);
}

float BNO085::getGravityX()
{
	return _sensor_value->un.gravity.x;
}

//Return the gravity component
float BNO085::getGravityY()
{
	return _sensor_value->un.gravity.y;
}

//Return the gravity component
float BNO085::getGravityZ()
{
	return _sensor_value->un.gravity.z;
}

uint8_t BNO085::getGravityAccuracy()
{
	return _sensor_value->status;
}

//Return the magnetometer component
float BNO085::getMagX()
{
	return _sensor_value->un.magneticField.x;
}

//Return the magnetometer component
float BNO085::getMagY()
{
	return _sensor_value->un.magneticField.y;
}

//Return the magnetometer component
float BNO085::getMagZ()
{
	return _sensor_value->un.magneticField.z;
}

//Return the mag component
uint8_t BNO085::getMagAccuracy()
{
	return _sensor_value->status;
}

//Return the tap detector
uint8_t BNO085::getTapDetector()
{
	uint8_t previousTapDetector = tapDetector;
	tapDetector = 0; //Reset so user code sees exactly one tap
	return (previousTapDetector);
}

//Return the step count
uint16_t BNO085::getStepCount()
{
	return _sensor_value->un.stepCounter.steps;
}

//Return the stability classifier
uint8_t BNO085::getStabilityClassifier()
{
	return _sensor_value->un.stabilityClassifier.classification;
}

//Return the activity classifier
uint8_t BNO085::getActivityClassifier()
{
	return _sensor_value->un.personalActivityClassifier.mostLikelyState;
}

//Return the activity confindence
uint8_t BNO085::getActivityConfidence(uint8_t activity)
{
	return _sensor_value->un.personalActivityClassifier.confidence[activity];
}

//Return the time stamp
uint64_t BNO085::getTimeStamp()
{
	return _sensor_value->timestamp;
}

//Return raw mems value for the accel
int16_t BNO085::getRawAccelX()
{
	return _sensor_value->un.rawAccelerometer.x;
}
//Return raw mems value for the accel
int16_t BNO085::getRawAccelY()
{
	return _sensor_value->un.rawAccelerometer.y;
}
//Return raw mems value for the accel
int16_t BNO085::getRawAccelZ()
{
	return _sensor_value->un.rawAccelerometer.z;
}

//Return raw mems value for the gyro
int16_t BNO085::getRawGyroX()
{
	return _sensor_value->un.rawGyroscope.x;
}
int16_t BNO085::getRawGyroY()
{
	return _sensor_value->un.rawGyroscope.y;
}
int16_t BNO085::getRawGyroZ()
{
	return _sensor_value->un.rawGyroscope.z;
}

//Return raw mems value for the mag
int16_t BNO085::getRawMagX()
{
	return _sensor_value->un.rawMagnetometer.x;
}
int16_t BNO085::getRawMagY()
{
	return _sensor_value->un.rawMagnetometer.y;
}
int16_t BNO085::getRawMagZ()
{
	return _sensor_value->un.rawMagnetometer.z;
}

bool BNO085::serviceBus(void)
{
  sh2_service();
  return true;
}

//Send command to reset IC
bool BNO085::softReset(void)
{
  int status = sh2_devReset();

  if (status != SH2_OK) {
    return false;
  }

  return true;	
}

//Set the operating mode to "On"
//(This one is for @jerabaul29)
bool BNO085::modeOn(void)
{
  int status = sh2_devOn();

  if (status != SH2_OK) {
    return false;
  }

  return true;	
}

//Set the operating mode to "Sleep"
//(This one is for @jerabaul29)
bool BNO085::modeSleep(void)
{
  int status = sh2_devSleep();

  if (status != SH2_OK) {
    return false;
  }

  return true;	
}

//Get the reason for the last reset
//1 = POR, 2 = Internal reset, 3 = Watchdog, 4 = External reset, 5 = Other
uint8_t BNO085::getResetReason()
{
	return prodIds.entry[0].resetCause;
}

//Sends the packet to enable the rotation vector
bool BNO085::enableRotationVector(uint16_t timeBetweenReports)
{
	timeBetweenReports  = timeBetweenReports * 1000; // ms to us
	return enableReport(SH2_ROTATION_VECTOR, timeBetweenReports);
}

//Sends the packet to enable the geomagnetic rotation vector
bool BNO085::enableGeomagneticRotationVector(uint16_t timeBetweenReports)
{
	timeBetweenReports  = timeBetweenReports * 1000; // ms to us
	return enableReport(SH2_GEOMAGNETIC_ROTATION_VECTOR, timeBetweenReports);
}

//Sends the packet to enable the ar/vr stabilized rotation vector
bool BNO085::enableARVRStabilizedRotationVector(uint16_t timeBetweenReports)
{
	timeBetweenReports  = timeBetweenReports * 1000; // ms to us
	return enableReport(SENSOR_REPORTID_AR_VR_STABILIZED_ROTATION_VECTOR, timeBetweenReports);
}

//Sends the packet to enable the rotation vector
bool BNO085::enableGameRotationVector(uint16_t timeBetweenReports)
{
	timeBetweenReports  = timeBetweenReports * 1000; // ms to us
	return enableReport(SH2_GAME_ROTATION_VECTOR, timeBetweenReports);
}

//Sends the packet to enable the ar/vr stabilized rotation vector
bool BNO085::enableARVRStabilizedGameRotationVector(uint16_t timeBetweenReports)
{
	timeBetweenReports  = timeBetweenReports * 1000; // ms to us
	return enableReport(SENSOR_REPORTID_AR_VR_STABILIZED_GAME_ROTATION_VECTOR, timeBetweenReports);
}

//Sends the packet to enable the accelerometer
bool BNO085::enableAccelerometer(uint16_t timeBetweenReports)
{
	timeBetweenReports  = timeBetweenReports * 1000; // ms to us
	return enableReport(SH2_ACCELEROMETER, timeBetweenReports);
}

//Sends the packet to enable the accelerometer
bool BNO085::enableLinearAccelerometer(uint16_t timeBetweenReports)
{
	timeBetweenReports  = timeBetweenReports * 1000; // ms to us
	return enableReport(SENSOR_REPORTID_LINEAR_ACCELERATION, timeBetweenReports);	
}

//Sends the packet to enable the gravity vector
bool BNO085::enableGravity(uint16_t timeBetweenReports)
{
	timeBetweenReports  = timeBetweenReports * 1000; // ms to us
	return enableReport(SENSOR_REPORTID_GRAVITY, timeBetweenReports);	
}

//Sends the packet to enable the gyro
bool BNO085::enableGyro(uint16_t timeBetweenReports)
{
	timeBetweenReports  = timeBetweenReports * 1000; // ms to us
	return enableReport(SENSOR_REPORTID_GYROSCOPE_CALIBRATED, timeBetweenReports);		
}

//Sends the packet to enable the uncalibrated gyro
bool BNO085::enableUncalibratedGyro(uint16_t timeBetweenReports)
{
	timeBetweenReports  = timeBetweenReports * 1000; // ms to us
	return enableReport(SENSOR_REPORTID_UNCALIBRATED_GYRO, timeBetweenReports);		
}

//Sends the packet to enable the magnetometer
bool BNO085::enableMagnetometer(uint16_t timeBetweenReports)
{
	timeBetweenReports  = timeBetweenReports * 1000; // ms to us
	return enableReport(SENSOR_REPORTID_MAGNETIC_FIELD, timeBetweenReports);		
}

//Sends the packet to enable the tap detector
bool BNO085::enableTapDetector(uint16_t timeBetweenReports)
{
	timeBetweenReports  = timeBetweenReports * 1000; // ms to us
	return enableReport(SENSOR_REPORTID_TAP_DETECTOR, timeBetweenReports);		
}

//Sends the packet to enable the step counter
bool BNO085::enableStepCounter(uint16_t timeBetweenReports)
{
	timeBetweenReports  = timeBetweenReports * 1000; // ms to us
	return enableReport(SENSOR_REPORTID_STEP_COUNTER, timeBetweenReports);		
}

//Sends the packet to enable the Stability Classifier
bool BNO085::enableStabilityClassifier(uint16_t timeBetweenReports)
{
	timeBetweenReports  = timeBetweenReports * 1000; // ms to us
	return enableReport(SENSOR_REPORTID_STABILITY_CLASSIFIER, timeBetweenReports);		
}

//Sends the packet to enable the raw accel readings
//Note you must enable basic reporting on the sensor as well
bool BNO085::enableRawAccelerometer(uint16_t timeBetweenReports)
{
	timeBetweenReports  = timeBetweenReports * 1000; // ms to us
	return enableReport(SENSOR_REPORTID_RAW_ACCELEROMETER, timeBetweenReports);		
}

//Sends the packet to enable the raw accel readings
//Note you must enable basic reporting on the sensor as well
bool BNO085::enableRawGyro(uint16_t timeBetweenReports)
{
	timeBetweenReports  = timeBetweenReports * 1000; // ms to us
	return enableReport(SENSOR_REPORTID_RAW_GYROSCOPE, timeBetweenReports);		
}

//Sends the packet to enable the raw accel readings
//Note you must enable basic reporting on the sensor as well
bool BNO085::enableRawMagnetometer(uint16_t timeBetweenReports)
{
	timeBetweenReports  = timeBetweenReports * 1000; // ms to us
	return enableReport(SENSOR_REPORTID_RAW_MAGNETOMETER, timeBetweenReports);		
}

//Sends the packet to enable the various activity classifiers
bool BNO085::enableActivityClassifier(uint16_t timeBetweenReports, uint32_t activitiesToEnable)
{
	timeBetweenReports  = timeBetweenReports * 1000; // ms to us
	return enableReport(SENSOR_REPORTID_PERSONAL_ACTIVITY_CLASSIFIER, timeBetweenReports, activitiesToEnable);
}

// See 2.2 of the Calibration Procedure document 1000-4044
// Set the desired sensors to have active dynamic calibration
bool BNO085::setCalibrationConfig(uint8_t sensors)
{
  int status = sh2_setCalConfig(sensors);

  if (status != SH2_OK) {
    return false;
  }

  return true;	
}

bool BNO085::tareNow(bool zAxis, sh2_TareBasis_t basis)
{
  int status = sh2_setTareNow(zAxis ? TARE_AXIS_Z : TARE_AXIS_ALL, basis);

  if (status != SH2_OK) {
    return false;
  }

  return true;	
}

bool BNO085::saveTare()
{
  int status = sh2_persistTare();

  if (status != SH2_OK) {
    return false;
  }

  return true;
}

bool BNO085::clearTare()
{
  int status = sh2_clearTare();

  if (status != SH2_OK) {
    return false;
  }

  return true;
}


bool BNO085::saveCalibration()
{
  int status = sh2_saveDcdNow();
  if (status != SH2_OK) {
    return false;
  }
  return true;	
}





File: src/BNO085.h

#include "sh2.h"
#include "sh2_SensorValue.h"
#include "sh2_err.h"

#pragma once

#include "Arduino.h"
#include <Wire.h>

//All the ways we can configure or talk to the BNO085, figure 34, page 36 reference manual
//These are used for low level communication with the sensor, on channel 2
#define SHTP_REPORT_COMMAND_RESPONSE 0xF1
#define SHTP_REPORT_COMMAND_REQUEST 0xF2
#define SHTP_REPORT_FRS_READ_RESPONSE 0xF3
#define SHTP_REPORT_FRS_READ_REQUEST 0xF4
#define SHTP_REPORT_PRODUCT_ID_RESPONSE 0xF8
#define SHTP_REPORT_PRODUCT_ID_REQUEST 0xF9
#define SHTP_REPORT_BASE_TIMESTAMP 0xFB
#define SHTP_REPORT_SET_FEATURE_COMMAND 0xFD

//All the different sensors and features we can get reports from
//These are used when enabling a given sensor
#define SENSOR_REPORTID_ACCELEROMETER SH2_ACCELEROMETER
#define SENSOR_REPORTID_GYROSCOPE_CALIBRATED SH2_GYROSCOPE_CALIBRATED
#define SENSOR_REPORTID_MAGNETIC_FIELD SH2_MAGNETIC_FIELD_CALIBRATED
#define SENSOR_REPORTID_LINEAR_ACCELERATION SH2_LINEAR_ACCELERATION
#define SENSOR_REPORTID_ROTATION_VECTOR SH2_ROTATION_VECTOR
#define SENSOR_REPORTID_GRAVITY SH2_GRAVITY
#define SENSOR_REPORTID_UNCALIBRATED_GYRO SH2_GYROSCOPE_UNCALIBRATED
#define SENSOR_REPORTID_GAME_ROTATION_VECTOR 0x08
#define SENSOR_REPORTID_GEOMAGNETIC_ROTATION_VECTOR 0x09
#define SENSOR_REPORTID_GYRO_INTEGRATED_ROTATION_VECTOR SH2_GYRO_INTEGRATED_RV
#define SENSOR_REPORTID_TAP_DETECTOR 0x10
#define SENSOR_REPORTID_STEP_COUNTER SH2_STEP_COUNTER
#define SENSOR_REPORTID_STABILITY_CLASSIFIER SH2_STABILITY_CLASSIFIER
#define SENSOR_REPORTID_RAW_ACCELEROMETER 0x14
#define SENSOR_REPORTID_RAW_GYROSCOPE 0x15
#define SENSOR_REPORTID_RAW_MAGNETOMETER 0x16
#define SENSOR_REPORTID_PERSONAL_ACTIVITY_CLASSIFIER SH2_PERSONAL_ACTIVITY_CLASSIFIER
#define SENSOR_REPORTID_AR_VR_STABILIZED_ROTATION_VECTOR 0x28
#define SENSOR_REPORTID_AR_VR_STABILIZED_GAME_ROTATION_VECTOR 0x29

// Reset complete packet (BNO085 Datasheet p.24 Figure 1-27)
#define EXECUTABLE_RESET_COMPLETE 0x1

//Command IDs from section 6.4, page 42
//These are used to calibrate, initialize, set orientation, tare etc the sensor
#define COMMAND_ERRORS 1
#define COMMAND_COUNTER 2
#define COMMAND_TARE 3
#define COMMAND_INITIALIZE 4
#define COMMAND_DCD 6
#define COMMAND_ME_CALIBRATE 7
#define COMMAND_DCD_PERIOD_SAVE 9
#define COMMAND_OSCILLATOR 10
#define COMMAND_CLEAR_DCD 11

#define CALIBRATE_ACCEL 0
#define CALIBRATE_GYRO 1
#define CALIBRATE_MAG 2
#define CALIBRATE_PLANAR_ACCEL 3
#define CALIBRATE_ACCEL_GYRO_MAG 4
#define CALIBRATE_STOP 5

#define TARE_AXIS_ALL 0x07
#define TARE_AXIS_Z   0x04

#define TARE_ROTATION_VECTOR 0
#define TARE_GAME_ROTATION_VECTOR 1
#define TARE_GEOMAGNETIC_ROTATION_VECTOR 2
#define TARE_GYRO_INTEGRATED_ROTATION_VECTOR 3
#define TARE_AR_VR_STABILIZED_ROTATION_VECTOR 4
#define TARE_AR_VR_STABILIZED_GAME_ROTATION_VECTOR 5

bool I2CWrite(uint8_t add, uint8_t *buffer, size_t size);
bool I2CRead(uint8_t add, uint8_t *buffer, size_t size);

class BNO085
{
public:
	bool begin(TwoWire &wirePort = Wire); 
	bool isConnected();

    sh2_ProductIds_t prodIds; ///< The product IDs returned by the sensor
	sh2_SensorValue_t sensorValue;


	uint8_t getResetReason(); // returns prodIds->resetCause

    bool enableReport(sh2_SensorId_t sensor, uint32_t interval_us = 10000, uint32_t sensorSpecific = 0);
    bool getSensorEvent();
	uint8_t getSensorEventID();

	bool softReset();	  //Try to reset the IMU via software
	bool serviceBus(void);	
	uint8_t resetReason(); //Query the IMU for the reason it last reset
	bool modeOn();	  //Use the executable channel to turn the BNO on
	bool modeSleep();	  //Use the executable channel to put the BNO to sleep

	bool enableRotationVector(uint16_t timeBetweenReports = 10);
	bool enableGeomagneticRotationVector(uint16_t timeBetweenReports = 10);
	bool enableGameRotationVector(uint16_t timeBetweenReports = 10);
	bool enableARVRStabilizedRotationVector(uint16_t timeBetweenReports);
	bool enableARVRStabilizedGameRotationVector(uint16_t timeBetweenReports);
	bool enableAccelerometer(uint16_t timeBetweenReports = 10);
	bool enableLinearAccelerometer(uint16_t timeBetweenReports = 10);
	bool enableGravity(uint16_t timeBetweenReports = 10);
	bool enableGyro(uint16_t timeBetweenReports = 10);
	bool enableUncalibratedGyro(uint16_t timeBetweenReports = 10);
	bool enableMagnetometer(uint16_t timeBetweenReports = 10);
	bool enableTapDetector(uint16_t timeBetweenReports = 10);
	bool enableStepCounter(uint16_t timeBetweenReports = 10);
	bool enableStabilityClassifier(uint16_t timeBetweenReports = 10);
	bool enableActivityClassifier(uint16_t timeBetweenReports, uint32_t activitiesToEnable);
	bool enableRawAccelerometer(uint16_t timeBetweenReports = 10);
	bool enableRawGyro(uint16_t timeBetweenReports = 10);
	bool enableRawMagnetometer(uint16_t timeBetweenReports = 10);

	float getRot_I();
	float getRot_J();
	float getRot_K();
	float getRot_R();
	float getRadianAccuracy();
	uint8_t getRot_Accuracy();

	float getGameI();
	float getGameJ();
	float getGameK();
	float getGameReal();	

	float getAccelX();
	float getAccelY();
	float getAccelZ();
	uint8_t getAccelAccuracy();

	float getLinAccelX();
	float getLinAccelY();
	float getLinAccelZ();
	uint8_t getLinAccelAccuracy();

	float getGyroX();
	float getGyroY();
	float getGyroZ();
	uint8_t getGyroAccuracy();

	float getUncalibratedGyroX();
	float getUncalibratedGyroY();
	float getUncalibratedGyroZ();
	float getUncalibratedGyroBiasX();
	float getUncalibratedGyroBiasY();
	float getUncalibratedGyroBiasZ();
	uint8_t getUncalibratedGyroAccuracy();

	float getMagX();
	float getMagY();
	float getMagZ();
	uint8_t getMagAccuracy();

	float getGravityX();
	float getGravityY();
	float getGravityZ();
	uint8_t getGravityAccuracy();

	bool setCalibrationConfig(uint8_t sensors);
	bool saveCalibration();

	bool tareNow(bool zAxis=false, sh2_TareBasis_t basis=SH2_TARE_BASIS_ROTATION_VECTOR);
	bool saveTare();
	bool clearTare();
	
	uint8_t getTapDetector();
	uint64_t getTimeStamp();
	uint16_t getStepCount();
	uint8_t getStabilityClassifier();
	uint8_t getActivityClassifier();
	uint8_t getActivityConfidence(uint8_t activity);

	int16_t getRawAccelX();
	int16_t getRawAccelY();
	int16_t getRawAccelZ();

	int16_t getRawGyroX();
	int16_t getRawGyroY();
	int16_t getRawGyroZ();

	int16_t getRawMagX();
	int16_t getRawMagY();
	int16_t getRawMagZ();

	float getRoll();
	float getPitch();
	float getYaw();

//	void sendCommand(uint8_t command);
//	void sendCalibrateCommand(uint8_t thingToCalibrate);

	//Metadata functions
	// int16_t getQ1(uint16_t recordID);
	// int16_t getQ2(uint16_t recordID);
	// int16_t getQ3(uint16_t recordID);
	// float getResolution(uint16_t recordID);
	// float getRange(uint16_t recordID);
	// uint32_t readFRSword(uint16_t recordID, uint8_t wordNumber);
	// void frsReadRequest(uint16_t recordID, uint16_t readOffset, uint16_t blockSize);
	// bool readFRSdata(uint16_t recordID, uint8_t startLocation, uint8_t wordsToRead);

	//Global Variables
	// uint8_t shtpHeader[4]; //Each packet has a header of 4 bytes
	// uint8_t shtpData[MAX_PACKET_SIZE];
	// uint8_t sequenceNumber[6] = {0, 0, 0, 0, 0, 0}; //There are 6 com channels. Each channel has its own seqnum
	// uint8_t commandSequenceNumber = 0;				//Commands have a seqNum as well. These are inside command packet, the header uses its own seqNum per channel
	// uint32_t metaData[MAX_METADATA_SIZE];			//There is more than 10 words in a metadata record but we'll stop at Q point 3

//	unsigned long _spiPortSpeed; //Optional user defined port speed
//	uint8_t _cs;				 //Pins needed for SPI

private:

	Stream *_debugPort;			 //The stream to send debug messages to if enabLED. Usually Serial.
	bool _printDebug = false; //Flag to print debugging variables

	//These are the raw sensor values (without Q applied) pulLED from the user requested Input Report
	uint16_t rawAccelX, rawAccelY, rawAccelZ, accelAccuracy;
	uint16_t rawLinAccelX, rawLinAccelY, rawLinAccelZ, accelLinAccuracy;
	uint16_t rawGyroX, rawGyroY, rawGyroZ, gyroAccuracy;
	uint16_t rawUncalibGyroX, rawUncalibGyroY, rawUncalibGyroZ, rawBiasX, rawBiasY, rawBiasZ, UncalibGyroAccuracy;
	uint16_t rawMagX, rawMagY, rawMagZ, magAccuracy;
	uint16_t rawQuatI, rawQuatJ, rawQuatK, rawQuatReal, rawQuatRadianAccuracy, quatAccuracy;
	uint16_t rawFastGyroX, rawFastGyroY, rawFastGyroZ;
	uint16_t gravityX, gravityY, gravityZ, gravityAccuracy;
	uint8_t tapDetector;
	uint16_t stepCount;
	uint32_t timeStamp;
	uint8_t stabilityClassifier;
	uint8_t activityClassifier;
	uint8_t calibrationStatus;							  //Byte R0 of ME Calibration Response
	uint16_t memsRawAccelX, memsRawAccelY, memsRawAccelZ; //Raw readings from MEMS sensor
	uint16_t memsRawGyroX, memsRawGyroY, memsRawGyroZ;	//Raw readings from MEMS sensor
	uint16_t memsRawMagX, memsRawMagY, memsRawMagZ;		  //Raw readings from MEMS sensor

	//These Q values are defined in the datasheet but can also be obtained by querying the meta data records
	//See the read metadata example for more info
	int16_t rotationVector_Q1 = 14;
	int16_t rotationVectorAccuracy_Q1 = 12; //Heading accuracy estimate in radians. The Q point is 12.
	int16_t accelerometer_Q1 = 8;
	int16_t linear_accelerometer_Q1 = 8;
	int16_t gyro_Q1 = 9;
	int16_t magnetometer_Q1 = 4;
	int16_t angular_velocity_Q1 = 10;
	int16_t gravity_Q1 = 8;

protected:
	virtual bool _init(int32_t sensor_id = 0);
	sh2_Hal_t _HAL; ///< The struct representing the SH2 Hardware Abstraction Layer
};


File: src/CodeCell.cpp

#include "CodeCell.h"
#include "BNO085.h"

bool serial_flag = 0;
bool cc_timeflag = 1;
esp_sleep_wakeup_cause_t cc_wakeup_reason;
hw_timer_t *cctimer = NULL;

BNO085 Motion;


void IRAM_ATTR cc_Timer() {
  cc_timeflag = 1;
}

CodeCell::CodeCell() {
}

void CodeCell::Init(uint16_t sense_motion) {
  uint8_t light_timer = 0;

  delay(1300); /*Uart Setup delay*/

  if (Serial) {
    serial_flag = 1;
  }

  if (WakeUpCheck()) {
    Serial.println("Waking up..");
    _charge_state = POWER_INIT;
    _chrg_counter = 0;
  } else if (cc_wakeup_reason == ESP_SLEEP_WAKEUP_GPIO) {
    Serial.println("Waking up..");
    _charge_state = POWER_INIT;
    _chrg_counter = 0;
  } else {
    Serial.println(" ");
    Serial.println("Hello World! I am your CodeCell");
  }

  _msense = sense_motion;

  Serial.print("Please wait while I setup my sensors.. ");

  while (Light_Init() == 1) {
    delay(10);
    LightReset();
    if (light_timer < 50U) {
      light_timer++;
    } else {
      Serial.println(" CodeCell Error - Light Sensor not found");
      esp_restart();
    }
  }

  if (_msense != MOTION_DISABLE) {
    Serial.print("Light Sensing Enabled + ");
    Motion_Init();
  } else {
    Serial.println("Light Sensing Enabled..");
  }
  Serial.print("Printing Sensors Readings.. ");
  
  pinMode(LED_PIN, OUTPUT);   /*Set LED pin as output*/
  digitalWrite(LED_PIN, LOW); /*Init Set up to output low*/
  delay(1);
  rmtInit(LED_PIN, RMT_TX_MODE, RMT_MEM_NUM_BLOCKS_1, 10000000); /*Configure RMT to run the onboard addressable LED*/

  LED(0, 0, 0);
  delay(1);
  LED(0, 0, LED_SLEEP_BRIGHTNESS);
  delay(80);
  LED(LED_SLEEP_BRIGHTNESS, 0, 0);
  delay(80);
  LED(0, LED_SLEEP_BRIGHTNESS, 0);
  delay(80);
  LED(0, 0, 0);


  pinMode(0, INPUT); /*Set up Cable indication pin*/
  pinMode(4, INPUT); /*Set up Battery Voltage Read pin*/
  analogSetAttenuation(ADC_11db);

  pinMode(1, OUTPUT);
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  digitalWrite(1, LOW); /*Init Set up to output low*/
  digitalWrite(2, LOW); /*Init Set up to output low*/
  digitalWrite(3, LOW); /*Init Set up to output low*/
  digitalWrite(5, LOW); /*Init Set up to output low*/
  digitalWrite(6, LOW); /*Init Set up to output low*/
  digitalWrite(7, LOW); /*Init Set up to output low*/

  PrintSensors();

  cctimer = timerBegin(1000000);            /*Set timer frequency to 1Mhz*/
  timerAttachInterrupt(cctimer, &cc_Timer); /*Attach interrupt*/
  timerAlarm(cctimer, 100000, true, 0);     /*Set alarm to trigger every 100ms with auto-reload*/

  Serial.println("Now running my default code v" CODECELL_SW_VERSION);
}

void CodeCell::Test() {
  digitalWrite(1, HIGH); /*Init Set up to output HIGH*/
  digitalWrite(2, HIGH); /*Init Set up to output HIGH*/
  digitalWrite(3, HIGH); /*Init Set up to output HIGH*/
  digitalWrite(5, HIGH); /*Init Set up to output HIGH*/
  digitalWrite(6, HIGH); /*Init Set up to output HIGH*/
  digitalWrite(7, HIGH); /*Init Set up to output HIGH*/
  delay(500);
  digitalWrite(1, LOW); /*Init Set up to output low*/
  digitalWrite(2, LOW); /*Init Set up to output low*/
  digitalWrite(3, LOW); /*Init Set up to output low*/
  digitalWrite(5, LOW); /*Init Set up to output low*/
  digitalWrite(6, LOW); /*Init Set up to output low*/
  digitalWrite(7, LOW); /*Init Set up to output low*/
}


bool CodeCell::WakeUpCheck() {

  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();
  cc_wakeup_reason = wakeup_reason;

  if (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER) {
    return true;
  } else {
    return false;
  }
}

void CodeCell::Sleep(uint16_t sleep_sec) {
  pinMode(1, INPUT);
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(5, INPUT);
  pinMode(6, INPUT);
  pinMode(7, INPUT);

  _i2c_write_array[_i2c_write_size++] = VCNL4040_PS_CONF3_MS_REG; /*Address*/
  _i2c_write_array[_i2c_write_size++] = (0x800 & 0xFF);           /*LSB*/
  _i2c_write_array[_i2c_write_size++] = ((0x800 >> 8) & 0xFF);    /*MSB*/
  if (!I2CWrite(VCNL4040_ADDRESS, _i2c_write_array, _i2c_write_size)) {
    Serial.println("CodeCell Error - Light Sensor not found");
  }
  _i2c_write_size = 0;
  _i2c_write_array[_i2c_write_size++] = VCNL4040_ALS_CONF_REG; /*Address*/
  _i2c_write_array[_i2c_write_size++] = (0x01 & 0xFF);         /*LSB*/
  _i2c_write_array[_i2c_write_size++] = ((0x01 >> 8) & 0xFF);  /*MSB*/
  if (!I2CWrite(VCNL4040_ADDRESS, _i2c_write_array, _i2c_write_size)) {
    Serial.println("CodeCell Error - Light Sensor not found");
  }
  _i2c_write_size = 0;
  _i2c_write_array[_i2c_write_size++] = VCNL4040_PS_CONF1_2_REG; /*Address*/
  _i2c_write_array[_i2c_write_size++] = (0x01 & 0xFF);           /*LSB*/
  _i2c_write_array[_i2c_write_size++] = ((0x01 >> 8) & 0xFF);    /*MSB*/
  if (!I2CWrite(VCNL4040_ADDRESS, _i2c_write_array, _i2c_write_size)) {
    Serial.println("CodeCell Error - Light Sensor not found");
  }
  _i2c_write_size = 0;

  if (_msense != MOTION_DISABLE) {
    Motion.modeSleep();
  }

  digitalWrite(10, LOW);
  pinMode(10, INPUT);
  Wire.end();  // Stop I2C

  pinMode(8, INPUT);
  pinMode(9, INPUT);
  LED(0, 0, 0);

  esp_sleep_enable_timer_wakeup(sleep_sec * 1000000ULL);
  esp_deep_sleep_start();
}

void CodeCell::USBSleep(bool cable_polarity) {
  pinMode(1, INPUT);
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(5, INPUT);
  pinMode(6, INPUT);
  pinMode(7, INPUT);

  if (cable_polarity) {
    Serial.println("Shutting down application.."); /*Shut down but do not go to sleep to allow reprogramming*/
    delay(100);
    while (digitalRead(0) == 0) {
      delay(1);
    }
    esp_restart();
  } else {
    for (int er = 0; er < 10; er++) {
      LED(255U, 0, 0); /*Set LED to the minimum brightness Red*/
      delay(1000);
      LED(0, 0, 0); /*Set LED to the minimum brightness Red*/
      delay(1000);
      if (digitalRead(0) == 0) {
        /*USB Connected stop blink & Reset*/
        esp_restart();
      }
    }

    _i2c_write_array[_i2c_write_size++] = VCNL4040_PS_CONF3_MS_REG; /*Address*/
    _i2c_write_array[_i2c_write_size++] = (0x800 & 0xFF);           /*LSB*/
    _i2c_write_array[_i2c_write_size++] = ((0x800 >> 8) & 0xFF);    /*MSB*/
    if (!I2CWrite(VCNL4040_ADDRESS, _i2c_write_array, _i2c_write_size)) {
      Serial.println("CodeCell Error - Light Sensor not found");
    }
    _i2c_write_size = 0;
    _i2c_write_array[_i2c_write_size++] = VCNL4040_ALS_CONF_REG; /*Address*/
    _i2c_write_array[_i2c_write_size++] = (0x01 & 0xFF);         /*LSB*/
    _i2c_write_array[_i2c_write_size++] = ((0x01 >> 8) & 0xFF);  /*MSB*/
    if (!I2CWrite(VCNL4040_ADDRESS, _i2c_write_array, _i2c_write_size)) {
      Serial.println("CodeCell Error - Light Sensor not found");
    }
    _i2c_write_size = 0;
    _i2c_write_array[_i2c_write_size++] = VCNL4040_PS_CONF1_2_REG; /*Address*/
    _i2c_write_array[_i2c_write_size++] = (0x01 & 0xFF);           /*LSB*/
    _i2c_write_array[_i2c_write_size++] = ((0x01 >> 8) & 0xFF);    /*MSB*/
    if (!I2CWrite(VCNL4040_ADDRESS, _i2c_write_array, _i2c_write_size)) {
      Serial.println("CodeCell Error - Light Sensor not found");
    }
    _i2c_write_size = 0;

    if (_msense != MOTION_DISABLE) {
      Motion.modeSleep();
    }
    delay(1000);

    digitalWrite(10, LOW);
    pinMode(10, INPUT);
    Wire.end();  // Stop I2C

    pinMode(8, INPUT);
    pinMode(9, INPUT);

    esp_deep_sleep_enable_gpio_wakeup(1 << 0, ESP_GPIO_WAKEUP_GPIO_LOW); /*Set Gpio0 as wakeup pin*/
    esp_deep_sleep_start();
    /*Waiting to wake up*/
  }
}

void CodeCell::PrintSensors() {
  Serial.print("Proximity: ");
  Serial.print(Light_ProximityRead());
  Serial.print(" cts, White: ");
  Serial.print(Light_WhiteRead());
  Serial.print(" lx, Ambient: ");
  Serial.print(Light_AmbientRead());
  Serial.print(" lx");

  if (_msense != MOTION_DISABLE) {
    if ((_msense & MOTION_ACCELEROMETER) == MOTION_ACCELEROMETER) {
      Motion_AccelerometerRead(xx, yy, zz);
      Serial.printf(", Acc XYZ: %.2f, %.2f, %.2f m/s^2, ", xx, yy, zz);
    }
    if ((_msense & MOTION_GYRO) == MOTION_GYRO) {
      Motion_GyroRead(xx, yy, zz);
      Serial.printf(", Gyro XYZ: %.2f, %.2f, %.2f rad/sec, ", xx, yy, zz);
    }
    if ((_msense & MOTION_MAGNETOMETER) == MOTION_MAGNETOMETER) {
      Motion_MagnetometerRead(xx, yy, zz);
      Serial.printf(", Magnetometer XYZ: %.2f, %.2f, %.2f uT, ", xx, yy, zz);
    }
    if ((_msense & MOTION_LINEAR_ACC) == MOTION_LINEAR_ACC) {
      Motion_LinearAccRead(xx, yy, zz);
      Serial.printf(", Linear Acc XYZ: %.2f, %.2f, %.2f m/s^2, ", xx, yy, zz);
    }
    if ((_msense & MOTION_GRAVITY) == MOTION_GRAVITY) {
      Motion_GravityRead(xx, yy, zz);
      Serial.printf(", Gravity XYZ: %.2f, %.2f, %.2f m/s^2, ", xx, yy, zz);
    }
    if ((_msense & MOTION_ROTATION) == MOTION_ROTATION) {
      Motion_RotationRead(xx, yy, zz);
      Serial.printf(", Roll: %.2f°, Pitch: %.2f°, Yaw: %.2f°, ", xx, yy, zz);
    }
    if ((_msense & MOTION_ROTATION_NO_MAG) == MOTION_ROTATION_NO_MAG) {
      Motion_RotationNoMagRead(xx, yy, zz);
      Serial.printf(", Roll: %.2f°, Pitch: %.2f°, Yaw: %.2f°, ", xx, yy, zz);
    }
    if ((_msense & MOTION_STEP_COUNTER) == MOTION_STEP_COUNTER) {
      Motion_StepCounterRead(xxtemp);
      Serial.print(", Steps: ");
      Serial.print(xxtemp);
    }
    if ((_msense & MOTION_TAP_DETECTOR) == MOTION_TAP_DETECTOR) {
      Serial.print(", Tap: ");
      Serial.print(Motion_TapRead());
    }
    if ((_msense & MOTION_STATE) == MOTION_STATE) {
      Serial.print(", Current State: ");
      switch (Motion_StateRead()) {
        case MOTION_STATE_STABLE:
          Serial.print("Stable");
          break;
        case MOTION_STATE_ONTABLE:
          Serial.print("On Table");
          break;
        case MOTION_STATE_STATIONARY:
          Serial.print("Stationary");
          break;
        case MOTION_STATE_MOTION:
          Serial.print("Motion");
          break;
        default:
          Serial.print("Unkown");
          break;
      }
    }
    if ((_msense & MOTION_ACTIVITY) == MOTION_ACTIVITY) {
      Serial.print(", Activity: ");
      switch (Motion_ActivityRead()) {
        case 1:
          Serial.print("Driving");
          break;
        case 2:
          Serial.print("Cycling");
          break;
        case 3:
        case 6:
          Serial.print("Walking");
          break;
        case 4:
          Serial.print("Still");
          break;
        case 5:
          Serial.print("Tilting");
          break;
        case 7:
          Serial.print("Running");
          break;
        case 8:
          Serial.print("Stairs");
          break;
        default:
          Serial.print("Reading..");
          break;
      }
    }
  }
  Serial.println(" ");
}

uint16_t CodeCell::BatteryRead() {
  double voltage = (double)analogRead(4) * 1.448;  //* 5930 / 4095
  uint16_t uvoltage = (uint16_t)voltage;
  return uvoltage;
}

bool CodeCell::Run() {
  bool tt_100ms = 0;
  if (cc_timeflag) {
    tt_100ms = 1;
    cc_timeflag = 0;
    uint16_t battery_voltage = BatteryRead();

    if ((battery_voltage > USB_VOLTAGE) || (digitalRead(0) == 0)) {
      _lowvoltage_counter = 0;
      _chrg_counter = 0;
      if (_charge_state == POWER_BAT_CHRG) {
        esp_restart(); /*Restart the CodeCell*/
      }
      if (digitalRead(0) == 0) {
        Serial.print("Battery is now charging.. ");
        LED(0, 0, LED_SLEEP_BRIGHTNESS); /*Set LED to the minimum Blue*/
        USBSleep(1);
        battery_voltage = BatteryRead();
        if (battery_voltage > USB_VOLTAGE) {
          _charge_state = POWER_INIT;
        } else {
          _charge_state = POWER_BAT_CHRG_FULL;
        }
      } else if (_charge_state == POWER_INIT) {
        /*Battery First Time Check*/
        Serial.println("Battery not detected.. Running from USB Power..");
        _charge_state = POWER_USB;
      } else {
        /*USB Cabel connected without battery - Run application*/
        LED_Breathing(LED_COLOR_BLUE);
        if ((!Serial) && (serial_flag == 1)) {
          Serial.end();         /*Close the current Serial connection*/
          delay(500);           /*Short delay before reinitializing*/
          Serial.begin(115200); /*Reinitialize the Serial connection*/
          Serial.println("Serial connection re-established!");
          delay(100);    /*Short delay before reinitializing*/
          esp_restart(); /*Restart the CodeCell*/
        }
      }
    } else {
      if ((_charge_state == POWER_BAT_CHRG_FULL) || (_charge_state == POWER_INIT)) {
        /*Battery has been charged or USB is disconnected*/
        Serial.println("Running from Battery Power.. ");
        _charge_state = POWER_BAT_CHRG; /*Battery is at an operating voltage level*/
        _chrg_counter = 0;
        _lowvoltage_counter = 0;
      } else {
        if (battery_voltage < MIN_BATTERY_VOLTAGE) {
          /*Voltage is less than 3.3V or higher than 4.3V*/
          if (_lowvoltage_counter < 10) {
            _lowvoltage_counter++;
          } else {
            Serial.println("CodeCell Battery Volotage Error..");
            _chrg_counter = 0;
            USBSleep(0);
          }
        } else {
          _lowvoltage_counter = 0;
          if (_chrg_counter < 10) {
            _chrg_counter++;
          } else {
            _charge_state = POWER_BAT_CHRG; /*Battery is at an operating voltage level*/
            LED_Breathing(LED_COLOR_GREEN);
          }
        }
      }
    }
  } else {
    /*wait*/
  }
  return tt_100ms;
}

void CodeCell::LED(uint8_t r, uint8_t g, uint8_t b) {
  neopixelWrite(LED_PIN, r, g, b); /*RMT ESP32 function for addressable LEDs*/
}

void CodeCell::LED_Breathing(uint32_t rgb_color_24bit) {

  uint32_t r_counter = (rgb_color_24bit >> 16) & 0xFFU;
  uint32_t g_counter = (rgb_color_24bit >> 8) & 0xFFU;
  uint32_t b_counter = rgb_color_24bit & 0xFFU;
  uint16_t getProx = Light_ProximityRead();

  if (getProx > 2000U) {
    getProx = 2000U;
  }

  getProx = (getProx / 100) + 2;
  if (getProx > 10U) {
    getProx = 10U;
  }
  getProx = getProx << 1U;

  if (_LED_Breathing_flag == 1) {
    if (_LED_Breathing_counter <= getProx) {
      _LED_Breathing_counter++;
    } else {
      _LED_Breathing_flag = 0;
    }
  } else {
    if (_LED_Breathing_counter >= 1) {
      _LED_Breathing_counter--;
    } else {
      _LED_Breathing_flag = 1;
    }
  }

  r_counter = (r_counter * _LED_Breathing_counter) / 22U;
  g_counter = (g_counter * _LED_Breathing_counter) / 22U;
  b_counter = (b_counter * _LED_Breathing_counter) / 22U;

  LED(r_counter, g_counter, b_counter); /*Set LED to the default Blue*/
}

uint16_t CodeCell::Light_AmbientRead() {
  uint16_t value = 0;
  Wire.beginTransmission(VCNL4040_ADDRESS);
  Wire.write(VCNL4040_AMBIENT_REG);
  Wire.endTransmission(false); /*Don't release the bus*/
  I2CRead(VCNL4040_ADDRESS, _i2c_read_array, 2);
  Wire.endTransmission();
  value = _i2c_read_array[0];         /*LSB*/
  value |= (_i2c_read_array[1] << 8); /*MSB*/
  return value;
}

uint16_t CodeCell::Light_WhiteRead() {
  uint16_t value = 0;
  Wire.beginTransmission(VCNL4040_ADDRESS);
  Wire.write(VCNL4040_WHITE_REG);
  Wire.endTransmission(false); /*Don't release the bus*/
  I2CRead(VCNL4040_ADDRESS, _i2c_read_array, 2);
  Wire.endTransmission();
  value = _i2c_read_array[0];         /*LSB*/
  value |= (_i2c_read_array[1] << 8); /*MSB*/
  return value;
}

uint16_t CodeCell::Light_ProximityRead() {
  uint16_t value = 0;
  Wire.beginTransmission(VCNL4040_ADDRESS);
  Wire.write(VCNL4040_PROX_REG);
  Wire.endTransmission(false); /*Don't release the bus*/
  I2CRead(VCNL4040_ADDRESS, _i2c_read_array, 2);
  Wire.endTransmission();
  value = _i2c_read_array[0];         /*LSB*/
  value |= (_i2c_read_array[1] << 8); /*MSB*/
  return value;
}

void CodeCell::Motion_AccelerometerRead(float &x, float &y, float &z) {
  Motion_Read();
  if (Motion.getSensorEventID() == SENSOR_REPORTID_ACCELEROMETER) {
    x = Motion.getAccelX();
    y = Motion.getAccelY();
    z = Motion.getAccelZ();
  }
  Wire.endTransmission();
}

void CodeCell::Motion_GyroRead(float &x, float &y, float &z) {
  Motion_Read();
  if (Motion.getSensorEventID() == SENSOR_REPORTID_GYROSCOPE_CALIBRATED) {
    x = Motion.getGyroX();
    y = Motion.getGyroY();
    z = Motion.getGyroZ();
  }
  Wire.endTransmission();
}

void CodeCell::Motion_MagnetometerRead(float &x, float &y, float &z) {
  Motion_Read();
  if (Motion.getSensorEventID() == SENSOR_REPORTID_MAGNETIC_FIELD) {
    x = Motion.getMagX();
    y = Motion.getMagY();
    z = Motion.getMagZ();
  }
  Wire.endTransmission();
}

void CodeCell::Motion_GravityRead(float &x, float &y, float &z) {
  Motion_Read();
  if (Motion.getSensorEventID() == SENSOR_REPORTID_GRAVITY) {
    x = Motion.getGravityX();
    y = Motion.getGravityY();
    z = Motion.getGravityZ();
  }
  Wire.endTransmission();
}

void CodeCell::Motion_LinearAccRead(float &x, float &y, float &z) {
  Motion_Read();
  if (Motion.getSensorEventID() == SENSOR_REPORTID_LINEAR_ACCELERATION) {
    x = Motion.getLinAccelX();
    y = Motion.getLinAccelY();
    z = Motion.getLinAccelZ();
  }
  Wire.endTransmission();
}

bool CodeCell::Motion_TapRead() {
  bool x = false;
  if (Motion.getSensorEvent() == true) {
    if (Motion.getSensorEventID() == SENSOR_REPORTID_TAP_DETECTOR) {
      x = true;
    }
  }
  Wire.endTransmission();
  return x;
}

void CodeCell::Motion_StepCounterRead(uint16_t &x) {
  Motion_Read();
  if (Motion.getSensorEventID() == SENSOR_REPORTID_STEP_COUNTER) {
    x = (uint16_t)Motion.getStepCount();
  }
  Wire.endTransmission();
}

uint16_t CodeCell::Motion_StateRead() {
  uint16_t x = 0;
  Motion_Read();
  if (Motion.getSensorEventID() == SENSOR_REPORTID_STABILITY_CLASSIFIER) {
    x = Motion.getStabilityClassifier();
  }
  Wire.endTransmission();
  return x;
}

uint16_t CodeCell::Motion_ActivityRead() {
  uint16_t x = 0;
  //Motion_Read();
  Wire.beginTransmission(BNO085_ADDRESS);
  if (Motion.getSensorEvent() == true) {
    if (Motion.getSensorEventID() == SENSOR_REPORTID_PERSONAL_ACTIVITY_CLASSIFIER) {
      x = Motion.getActivityClassifier();
    }
  }
  Wire.endTransmission();
  return x;
}


void CodeCell::Motion_RotationRead(float &roll, float &pitch, float &yaw) {
  float Rr = 0;
  float Ri = 0;
  float Rj = 0;
  float Rk = 0;

  Motion_Read();
  if (Motion.getSensorEventID() == SENSOR_REPORTID_ROTATION_VECTOR) {
    Rr = Motion.getRot_R();
    Ri = Motion.getRot_I();
    Rj = Motion.getRot_J();
    Rk = Motion.getRot_K();

    roll = atan2(2.0 * (Rr * Ri + Rj * Rk), 1.0 - 2.0 * (Ri * Ri + Rj * Rj)) * RAD_TO_DEG;  /* ROLL */
    pitch = atan2(2.0 * (Rr * Rj - Rk * Ri), 1.0 - 2.0 * (Rj * Rj + Ri * Ri)) * RAD_TO_DEG; /* PITCH */
    yaw = atan2(2.0 * (Rr * Rk + Ri * Rj), 1.0 - 2.0 * (Rj * Rj + Rk * Rk)) * RAD_TO_DEG;   /* YAW */
  }
  Wire.endTransmission();
}

void CodeCell::Motion_RotationNoMagRead(float &roll, float &pitch, float &yaw) {
  float Rr = 0;
  float Ri = 0;
  float Rj = 0;
  float Rk = 0;

  Motion_Read();
  if (Motion.getSensorEventID() == SENSOR_REPORTID_GAME_ROTATION_VECTOR) {
    Rr = Motion.getGameReal();
    Ri = Motion.getGameI();
    Rj = Motion.getGameJ();
    Rk = Motion.getGameK();

    roll = atan2(2.0 * (Rr * Ri + Rj * Rk), 1.0 - 2.0 * (Ri * Ri + Rj * Rj)) * RAD_TO_DEG;  /* ROLL */
    pitch = atan2(2.0 * (Rr * Rj - Rk * Ri), 1.0 - 2.0 * (Rj * Rj + Ri * Ri)) * RAD_TO_DEG; /* PITCH */
    yaw = atan2(2.0 * (Rr * Rk + Ri * Rj), 1.0 - 2.0 * (Rj * Rj + Rk * Rk)) * RAD_TO_DEG;   /* YAW */
  }
  Wire.endTransmission();
}

void CodeCell::Motion_Read() {
  uint8_t imu_read_timer = 0U;
  Wire.beginTransmission(BNO085_ADDRESS);
  while (Motion.getSensorEvent() == false) {
    if (imu_read_timer > 300U) {
      Serial.println(" CodeCell Error - Motion Sensor not found");
      Serial.println("Reseting");
      delay(100);
      esp_restart();
    } else {
      imu_read_timer++;
    }
    delay(1);
  }
}

bool CodeCell::Light_Init() {
  bool light_error = 0;

  Wire.begin();
  Wire.beginTransmission(VCNL4040_ADDRESS);
  /*Configure - Continuous conversion mode, high dynamic range, integration time of 80 ms*/
  _i2c_write_array[_i2c_write_size++] = VCNL4040_ALS_CONF_REG; /*Address*/
  _i2c_write_array[_i2c_write_size++] = (0x000 & 0xFF);        /*LSB*/
  _i2c_write_array[_i2c_write_size++] = ((0x000 >> 8) & 0xFF); /*MSB*/
  if (!I2CWrite(VCNL4040_ADDRESS, _i2c_write_array, _i2c_write_size)) {
    light_error = 1;
  }
  _i2c_write_size = 0;
  /*Configure - duty cycle to 1/40, 16-bit resolution*/
  _i2c_write_array[_i2c_write_size++] = VCNL4040_PS_CONF1_2_REG; /*Address*/
  _i2c_write_array[_i2c_write_size++] = (0x80E & 0xFF);          /*LSB*/
  _i2c_write_array[_i2c_write_size++] = ((0x80E >> 8) & 0xFF);   /*MSB*/
  if (!I2CWrite(VCNL4040_ADDRESS, _i2c_write_array, _i2c_write_size)) {
    light_error = 1;
  }
  _i2c_write_size = 0;
  /*Configure - Set LED current to 200 mA, No interrupt settings*/
  _i2c_write_array[_i2c_write_size++] = VCNL4040_PS_CONF3_MS_REG; /*Address*/
  _i2c_write_array[_i2c_write_size++] = (0x4710 & 0xFF);          /*LSB*/
  _i2c_write_array[_i2c_write_size++] = ((0x4710 >> 8) & 0xFF);   /*MSB*/
  if (!I2CWrite(VCNL4040_ADDRESS, _i2c_write_array, _i2c_write_size)) {
    light_error = 1;
  }
  _i2c_write_size = 0;
  Wire.endTransmission();

  return light_error;
}

void CodeCell::LightReset() {
  _i2c_write_array[_i2c_write_size++] = VCNL4040_ALS_CONF_REG; /*Address*/
  _i2c_write_array[_i2c_write_size++] = (0x01 & 0xFF);         /*LSB*/
  if (!I2CWrite(VCNL4040_ADDRESS, _i2c_write_array, _i2c_write_size)) {
    Serial.println("CodeCell Error - Light Sensor not found");
  }
  _i2c_write_size = 0;
  _i2c_write_array[_i2c_write_size++] = VCNL4040_PS_CONF1_2_REG; /*Address*/
  _i2c_write_array[_i2c_write_size++] = (0x01 & 0xFF);           /*LSB*/
  if (!I2CWrite(VCNL4040_ADDRESS, _i2c_write_array, _i2c_write_size)) {
    Serial.println("CodeCell Error - Light Sensor not found");
  }
  _i2c_write_size = 0;
}

void CodeCell::Motion_Init() {
  uint8_t imu_timer = 0U;
  Wire.begin();
  Wire.beginTransmission(BNO085_ADDRESS);
  while (Motion.begin() == false) {
    if (imu_timer > 100U) {
      Serial.println(" CodeCell Error - Motion Sensor not found");
      Serial.println("Reseting");
      delay(100);
      esp_restart();
    } else {
      imu_timer++;
    }
    delay(10);
    Motion.softReset();
  }
  if ((_msense & MOTION_ACCELEROMETER) == MOTION_ACCELEROMETER) {
    if (Motion.enableAccelerometer() == true) {
      Serial.println("Accelerometer Motion Sensing Enabled.. ");
    } else {
      Serial.println("Accelerometer Motion Sensing Failed.. ");
    }
  }
  if ((_msense & MOTION_GYRO) == MOTION_GYRO) {
    if (Motion.enableGyro() == true) {
      Serial.println("Gyro Motion Sensing Enabled.. ");
    } else {
      Serial.println("Gyro Motion Sensing Failed.. ");
    }
  }
  if ((_msense & MOTION_MAGNETOMETER) == MOTION_MAGNETOMETER) {
    if (Motion.enableMagnetometer() == true) {
      Serial.println("Magnetometer Motion Sensing Enabled.. ");
    } else {
      Serial.println("Magnetometer Motion Sensing Failed.. ");
    }
  }
  if ((_msense & MOTION_LINEAR_ACC) == MOTION_LINEAR_ACC) {
    if (Motion.enableLinearAccelerometer() == true) {
      Serial.println("Linear Accelerometer Motion Sensing Enabled.. ");
    } else {
      Serial.println("Linear Accelerometer Motion Sensing Failed.. ");
    }
  }
  if ((_msense & MOTION_GRAVITY) == MOTION_GRAVITY) {
    if (Motion.enableGravity() == true) {
      Serial.println("Gravity Motion Sensing Enabled.. ");
    } else {
      Serial.println("Gravity Motion Sensing Failed.. ");
    }
  }
  if ((_msense & MOTION_ROTATION) == MOTION_ROTATION) {
    if (Motion.enableRotationVector() == true) {
      Serial.println("Rotation Motion Sensing Enabled.. ");
    } else {
      Serial.println("Rotation Motion Sensing Failed.. ");
    }
  }
  if ((_msense & MOTION_ROTATION_NO_MAG) == MOTION_ROTATION_NO_MAG) {
    if (Motion.enableGameRotationVector() == true) {
      Serial.println("Rotation Motion (Magnetometer DisabLED) Sensing Enabled.. ");
    } else {
      Serial.println("Rotation Motion (Magnetometer DisabLED) Sensing Failed.. ");
    }
  }
  if ((_msense & MOTION_STEP_COUNTER) == MOTION_STEP_COUNTER) {
    if (Motion.enableStepCounter() == true) {
      Serial.println("Step Counter Motion Sensing Enabled.. ");
    } else {
      Serial.println("Step Counter Rotation Motion Sensing Failed.. ");
    }
  }
  if ((_msense & MOTION_TAP_DETECTOR) == MOTION_TAP_DETECTOR) {
    if (Motion.enableTapDetector() == true) {
      Serial.println("Tap Detector Motion Sensing Enabled.. ");
    } else {
      Serial.println("Tap Detector Motion Sensing Failed.. ");
    }
  }
  if ((_msense & MOTION_STATE) == MOTION_STATE) {
    if (Motion.enableStabilityClassifier() == true) {
      Serial.println("Motion State Sensing Enabled.. ");
    } else {
      Serial.println("Motion State Sensing Failed.. ");
    }
  }
  if ((_msense & MOTION_ACTIVITY) == MOTION_ACTIVITY) {
    if (Motion.enableActivityClassifier(1000, 0x1F) == true) {
      Serial.println("Motion Activity Sensing Enabled.. ");
    } else {
      Serial.println("Motion Activity Sensing Failed.. ");
    }
  }
  _i2c_write_size = 0;
  Wire.endTransmission();
}



File: src/CodeCell.h

#ifndef CODECELL_H
#define CODECELL_H

#include <Arduino.h>
#include <Wire.h>

#define CODECELL_SW_VERSION "1.2.3"

#define POWER_BAT_CHRG 0U
#define POWER_USB 1U
#define POWER_INIT 2U
#define POWER_BAT_CHRG_FULL 4U
#define USB_VOLTAGE 4100
#define MIN_BATTERY_VOLTAGE 3350

#define LED_PIN 10U
#define LED_DEFAULT_BRIGHTNESS 10U
#define LED_SLEEP_BRIGHTNESS 5U

#define LED_COLOR_RED 0XFF0000U
#define LED_COLOR_ORANGE 0XC04000U
#define LED_COLOR_YELLOW 0XA06000U
#define LED_COLOR_GREEN 0X00FF00U
#define LED_COLOR_AQUA 0X00A030U
#define LED_COLOR_PINK 0XC00020U
#define LED_COLOR_BLUE 0X0000FFU
#define LED_COLOR_WHITE 0XFFFFFFU

#define LIGHT 0
#define VCNL4040_ADDRESS 0x60
#define VCNL4040_ALS_CONF_REG 0x00
#define VCNL4040_PS_CONF1_2_REG 0x03
#define VCNL4040_PS_CONF3_MS_REG 0x04
#define VCNL4040_PROX_REG 0x08
#define VCNL4040_AMBIENT_REG 0x09
#define VCNL4040_WHITE_REG 0x0A

#define BNO085_ADDRESS 0x4A
#define MOTION_DISABLE 0
#define MOTION_ACCELEROMETER 0b1
#define MOTION_GYRO 0b10
#define MOTION_MAGNETOMETER 0b100
#define MOTION_LINEAR_ACC 0b1000
#define MOTION_GRAVITY 0b10000
#define MOTION_ROTATION 0b100000
#define MOTION_ROTATION_NO_MAG 0b10000000
#define MOTION_STEP_COUNTER 0b100000000
#define MOTION_STATE 0b1000000000
#define MOTION_TAP_DETECTOR 0b10000000000
#define MOTION_ACTIVITY 0b100000000000

#define MOTION_STATE_UNKNOWN 0
#define MOTION_STATE_ONTABLE 1
#define MOTION_STATE_STATIONARY 2
#define MOTION_STATE_STABLE 3
#define MOTION_STATE_MOTION 4
#define MOTION_STATE_STABLE 3

class CodeCell {
private:
  bool _LED_Breathing_flag = 0;
  uint16_t _msense = 0U;
  uint16_t _LED_Breathing_counter = 0U;
  uint8_t _chrg_counter = 0U;
  uint8_t 
          _lowvoltage_counter = 0;
  uint8_t _charge_state = POWER_INIT;
  uint8_t _i2c_write_array[10] = { 0 };
  uint8_t _i2c_read_array[10] = { 0 };
  uint8_t _i2c_write_size = 0;
  uint16_t xxtemp = 0;
  float xx = 0;
  float yy = 0;
  float zz = 0;

public:
  CodeCell();
  void Init(uint16_t sense_motion);
  void PrintSensors();
  void Test();
  void USBSleep(bool cable_polarity);
  void Sleep(uint16_t sleep_sec);
  bool WakeUpCheck();
  bool Run();
  uint16_t BatteryRead();

  void LED_Breathing(uint32_t rgb_color_24bit);
  void LED(uint8_t r, uint8_t g, uint8_t b);

  bool Light_Init();
  void LightReset();
  uint16_t Light_ProximityRead();
  uint16_t Light_WhiteRead();
  uint16_t Light_AmbientRead();

  void Motion_Init();
  void Motion_Read();
  void Motion_AccelerometerRead(float &x, float &y, float &z);
  void Motion_GyroRead(float &x, float &y, float &z);
  void Motion_MagnetometerRead(float &x, float &y, float &z);
  void Motion_GravityRead(float &x, float &y, float &z);
  void Motion_LinearAccRead(float &x, float &y, float &z);
  void Motion_StepCounterRead(uint16_t &x);
  void Motion_RotationRead(float &roll, float &pitch, float &yaw);
  void Motion_RotationNoMagRead(float &roll, float &pitch, float &yaw);
  bool Motion_TapRead();
  uint16_t Motion_StateRead();
  uint16_t Motion_ActivityRead();
};

#endif



File: src/NOTICE.txt

This software is licensed from CEVA, Inc.
Copyright (c) CEVA, Inc. and its licensors. 
All rights reserved.  

CEVA and the CEVA logo are trademarks of CEVA, Inc.

You can contact CEVA, Sensor Fusion Business Unit on the web at 
https://www.ceva-dsp.com/app/motion-sensing/ or at
15245 Shady Grove Road, Suite 400, Rockville, MD 20850 USA




File: src/sh2.c

/*
 * Copyright 2015-2018 Hillcrest Laboratories, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License and 
 * any applicable agreements you may have with Hillcrest Laboratories, Inc.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file sh2.c
 * @author David Wheeler
 * @date 22 Sept 2015
 * @brief API Definition for Hillcrest SH-2 Sensor Hub.
 *
 * The sh2 API provides functions for opening a session with
 * the sensor hub and performing all supported operations with it.
 * This includes enabling sensors and reading events as well as
 * other housekeeping functions.
 *
 */

#include "sh2.h"
#include "sh2_err.h"
#include "shtp.h"
#include "sh2_util.h"

#include <string.h>

// ------------------------------------------------------------------------
// Private type definitions

#define GUID_EXECUTABLE (1)
#define GUID_SENSORHUB (2)

// executable/device channel responses
#define EXECUTABLE_DEVICE_CMD_RESET (1)
#define EXECUTABLE_DEVICE_CMD_ON    (2)
#define EXECUTABLE_DEVICE_CMD_SLEEP (3)

// executable/device channel responses
#define EXECUTABLE_DEVICE_RESP_RESET_COMPLETE (1)

// Tags for sensorhub app advertisements.
#define TAG_SH2_VERSION (0x80)
#define TAG_SH2_REPORT_LENGTHS (0x81)

// Max length of sensorhub version string.
#define MAX_VER_LEN (16)

// Max number of report ids supported
#define SH2_MAX_REPORT_IDS (64)

#if defined(_MSC_VER)
#define PACKED_STRUCT struct
#pragma pack(push, 1)
#elif defined(__GNUC__)
#define PACKED_STRUCT struct __attribute__((packed))
#else 
#define PACKED_STRUCT __packed struct
#endif

#define ADVERT_TIMEOUT_US (200000)

// Command and Subcommand values
#define SH2_CMD_ERRORS                 1
#define SH2_CMD_COUNTS                 2
#define     SH2_COUNTS_GET_COUNTS          0
#define     SH2_COUNTS_CLEAR_COUNTS        1
#define SH2_CMD_TARE                   3
#define     SH2_TARE_TARE_NOW              0
#define     SH2_TARE_PERSIST_TARE          1
#define     SH2_TARE_SET_REORIENTATION     2
#define SH2_CMD_INITIALIZE             4
#define     SH2_INIT_SYSTEM                1
#define     SH2_INIT_UNSOLICITED           0x80
// #define SH2_CMD_FRS                    5 /* Depreciated */
#define SH2_CMD_DCD                    6
#define SH2_CMD_ME_CAL                 7
#define SH2_CMD_DCD_SAVE               9
#define SH2_CMD_GET_OSC_TYPE           0x0A
#define SH2_CMD_CLEAR_DCD_AND_RESET    0x0B
#define SH2_CMD_CAL                    0x0C
#define     SH2_CAL_START                   0
#define     SH2_CAL_FINISH                  1
#define SH2_CMD_BOOTLOADER             0x0D     /* SH-2 Reference Manual 6.4.12 */
#define     SH2_BL_MODE_REQ                 0
#define     SH2_BL_STATUS_REQ               1
#define SH2_CMD_INTERACTIVE_ZRO        0x0E     /* SH-2 Reference Manual 6.4.13 */

// SENSORHUB_COMMAND_REQ
#define SENSORHUB_COMMAND_REQ        (0xF2)
#define COMMAND_PARAMS (9)
typedef PACKED_STRUCT {
    uint8_t reportId;
    uint8_t seq;
    uint8_t command;
    uint8_t p[COMMAND_PARAMS];
} CommandReq_t;

// SENSORHUB_COMMAND_RESP
#define SENSORHUB_COMMAND_RESP       (0xF1)
#define RESPONSE_VALUES (11)
typedef PACKED_STRUCT {
    uint8_t reportId;
    uint8_t seq;
    uint8_t command;
    uint8_t commandSeq;
    uint8_t respSeq;
    uint8_t r[RESPONSE_VALUES];
} CommandResp_t;

// SENSORHUB_PROD_ID_REQ
#define SENSORHUB_PROD_ID_REQ        (0xF9)
typedef PACKED_STRUCT {
    uint8_t reportId;  
    uint8_t reserved;
} ProdIdReq_t;

// SENSORHUB_PROD_ID_RESP
#define SENSORHUB_PROD_ID_RESP       (0xF8)
typedef PACKED_STRUCT {
    uint8_t reportId;
    uint8_t resetCause;
    uint8_t swVerMajor;
    uint8_t swVerMinor;
    uint32_t swPartNumber;
    uint32_t swBuildNumber;
    uint16_t swVerPatch;
    uint8_t reserved0;
    uint8_t reserved1;
} ProdIdResp_t;

// Report definitions
// Bit fields for Feature Report flags
#define FEAT_CHANGE_SENSITIVITY_RELATIVE (1)
#define FEAT_CHANGE_SENSITIVITY_ABSOLUTE (0)
#define FEAT_CHANGE_SENSITIVITY_ENABLED  (2)
#define FEAT_CHANGE_SENSITIVITY_DISABLED (0)
#define FEAT_WAKE_ENABLED                (4)
#define FEAT_WAKE_DISABLED               (0)
#define FEAT_ALWAYS_ON_ENABLED           (8)
#define FEAT_ALWAYS_ON_DISABLED          (0)

// GET_FEATURE_REQ
#define SENSORHUB_GET_FEATURE_REQ    (0xFE)
typedef PACKED_STRUCT{
    uint8_t reportId;
    uint8_t featureReportId;
} GetFeatureReq_t;

// SENSORHUB_GET_FEATURE_RESP
#define SENSORHUB_GET_FEATURE_RESP   (0xFC)
typedef PACKED_STRUCT{
    uint8_t reportId;
    uint8_t featureReportId;      // sensor id
    uint8_t flags;                // FEAT_... values
    uint16_t changeSensitivity;
    uint32_t reportInterval_uS;
    uint32_t batchInterval_uS;
    uint32_t sensorSpecific;
} GetFeatureResp_t;


typedef struct sh2_s sh2_t;

typedef int (sh2_OpStart_t)(sh2_t *pSh2);
typedef void (sh2_OpRx_t)(sh2_t *pSh2, const uint8_t *payload, uint16_t len);

typedef struct sh2_Op_s {
    uint32_t timeout_us;
    sh2_OpStart_t *start;
    sh2_OpRx_t *rx;
} sh2_Op_t;

// Parameters and state information for the operation in progress
typedef union {
    struct {
        CommandReq_t req;
    } sendCmd;
    struct {
        sh2_ProductIds_t *pProdIds;
        uint8_t nextEntry;
        uint8_t expectedEntries;
    } getProdIds;
    struct {
        sh2_SensorConfig_t *pConfig;
        sh2_SensorId_t sensorId;
    } getSensorConfig;
    struct {
        const sh2_SensorConfig_t *pConfig;
        sh2_SensorId_t sensorId;
    } setSensorConfig;
    struct {
        uint16_t frsType;
        uint32_t *pData;
        uint16_t *pWords;
        uint16_t nextOffset;
    } getFrs;
    struct {
        uint16_t frsType;
        uint32_t *pData;
        uint16_t words;
        uint16_t offset;
    } setFrs;
    struct {
        uint8_t severity;
        sh2_ErrorRecord_t *pErrors;
        uint16_t *pNumErrors;
        uint16_t errsRead;
    } getErrors;
    struct {
        sh2_SensorId_t sensorId;
        sh2_Counts_t *pCounts;
    } getCounts;
    struct {
        uint8_t sensors;
    } calConfig;
    struct {
        uint8_t *pSensors;
    } getCalConfig;
    struct {
        sh2_SensorId_t sensorId;
    } forceFlush;
    struct {
        sh2_OscType_t *pOscType;
    } getOscType;
    struct {
        uint32_t interval_us;
    } startCal;
    struct {
        sh2_CalStatus_t status;
    } finishCal;
} sh2_OpData_t;

// Max length of an FRS record, words.
#define MAX_FRS_WORDS (72)

struct sh2_s {
    // Pointer to the SHTP HAL
    sh2_Hal_t *pHal;

    // associated SHTP instance
    void *pShtp;
    
    volatile bool resetComplete;
    bool advertDone;
    uint8_t executableChan;
    uint8_t controlChan;
    char version[MAX_VER_LEN+1];

    // Report lengths
    struct {
        uint8_t id;
        uint8_t len;
    } report[SH2_MAX_REPORT_IDS];

    // Multi-step operation support
    const sh2_Op_t *pOp;
    int opStatus;
    sh2_OpData_t opData;
    uint8_t lastCmdId;
    uint8_t cmdSeq;
    uint8_t nextCmdSeq;
    
    // Event callback and it's cookie
    sh2_EventCallback_t *eventCallback;
    void * eventCookie;

    // Sensor callback and it's cookie
    sh2_SensorCallback_t *sensorCallback;
    void * sensorCookie;

    // Storage space for reading sensor metadata
    uint32_t frsData[MAX_FRS_WORDS];
    uint16_t frsDataLen;

    // Stats
    uint32_t execBadPayload;
    uint32_t emptyPayloads;
    uint32_t unknownReportIds;

};

#define SENSORHUB_BASE_TIMESTAMP_REF (0xFB)
typedef PACKED_STRUCT {
    uint8_t reportId;
    uint32_t timebase;
} BaseTimestampRef_t;

#define SENSORHUB_TIMESTAMP_REBASE   (0xFA)
typedef PACKED_STRUCT {
    uint8_t reportId;
    int32_t timebase;
} TimestampRebase_t;

// SENSORHUB_FORCE_SENSOR_FLUSH
#define SENSORHUB_FORCE_SENSOR_FLUSH (0xF0)
typedef PACKED_STRUCT {
    uint8_t reportId;
    uint8_t sensorId;
} ForceFlushReq_t;

// SENSORHUB_FLUSH_COMPLETED    
#define SENSORHUB_FLUSH_COMPLETED    (0xEF)
typedef PACKED_STRUCT {
    uint8_t reportId;
    uint8_t sensorId;
} ForceFlushResp_t;

// ------------------------------------------------------------------------
// Private data

// SH2 state
sh2_t _sh2;

// SH2 Async Event Message
static sh2_AsyncEvent_t sh2AsyncEvent;

// ------------------------------------------------------------------------
// Private functions

// SH-2 transaction phases
static int opStart(sh2_t *pSh2, const sh2_Op_t *pOp)
{
    // return error if another operation already in progress
    if (pSh2->pOp) return SH2_ERR_OP_IN_PROGRESS;

    // Establish this operation as the new operation in progress
    pSh2->pOp = pOp;
    pSh2->opStatus = SH2_OK;
    int rc = pOp->start(pSh2);  // Call start method
    if (rc != SH2_OK) {
        // Unregister this operation
        pSh2->opStatus = rc;
        pSh2->pOp = 0;
    }

    return rc;
}

static void opRx(sh2_t *pSh2, const uint8_t *payload, uint16_t len)
{ 
    if ((pSh2->pOp != 0) &&                      // An operation is in progress
        (pSh2->pOp->rx != 0)) {                  // and it has an rx method
        pSh2->pOp->rx(pSh2, payload, len);  // Call receive method
    }
}

static void sensorhubAdvertHdlr(void *cookie, uint8_t tag, uint8_t len, uint8_t *value)
{
    sh2_t *pSh2 = (sh2_t *)cookie;
    
    switch (tag) {
        case TAG_SH2_VERSION:
            strcpy(pSh2->version, (const char *)value);
            break;

        case TAG_SH2_REPORT_LENGTHS:
        {
            uint8_t reports = len/2;
            if (reports > SH2_MAX_REPORT_IDS) {
                // Hub gave us more report lengths than we can store!
                reports = SH2_MAX_REPORT_IDS;
            }
        
            for (int n = 0; n < reports; n++) {
                pSh2->report[n].id = value[n*2];
                pSh2->report[n].len = value[n*2 + 1];
            }
            break;
        }
    
        case 0:
        {
            // 0 tag indicates end of advertisements for this app
            // At this time, the SHTP layer can give us our channel numbers
            pSh2->executableChan = shtp_chanNo(pSh2->pShtp, "executable", "device");
            pSh2->controlChan = shtp_chanNo(pSh2->pShtp, "sensorhub", "control");

            pSh2->advertDone = true;
            break;
        }
        
        default:
            break;
    }
}

static void sensorhubControlHdlr(void *cookie, uint8_t *payload, uint16_t len, uint32_t timestamp)
{
    sh2_t *pSh2 = (sh2_t *)cookie;

    uint16_t cursor = 0;
    uint32_t count = 0;
    CommandResp_t * pResp = 0;
    
    if (len == 0) {
        pSh2->emptyPayloads++;
        return;
    }

    while (cursor < len) {
        // Get next report id
        count++;
        uint8_t reportId = payload[cursor];

        // Determine report length
        uint8_t reportLen = 0;
        for (int n = 0; n < SH2_MAX_REPORT_IDS; n++) {
            if (pSh2->report[n].id == reportId) {
                reportLen = pSh2->report[n].len;
                break;
            }
        }
        if (reportLen == 0) {
            // An unrecognized report id
            pSh2->unknownReportIds++;
            return;
        }
        else {
            // Check for unsolicited initialize response
            if (reportId == SENSORHUB_COMMAND_RESP) {
                pResp = (CommandResp_t *)(payload+cursor);
                if ((pResp->command == (SH2_CMD_INITIALIZE | SH2_INIT_UNSOLICITED)) &&
                    (pResp->r[1] == SH2_INIT_SYSTEM)) {
                    // This is an unsolicited INIT message.
                    // Is it time to call reset callback?
                }

            } // Check for Get Feature Response
            else if (reportId == SENSORHUB_GET_FEATURE_RESP) {
                if (pSh2->eventCallback) {
                    GetFeatureResp_t * pGetFeatureResp;
                    pGetFeatureResp = (GetFeatureResp_t *)(payload + cursor);

                    sh2AsyncEvent.eventId = SH2_GET_FEATURE_RESP;
                    sh2AsyncEvent.sh2SensorConfigResp.sensorId = pGetFeatureResp->featureReportId;
                    sh2AsyncEvent.sh2SensorConfigResp.sensorConfig.changeSensitivityEnabled = ((pGetFeatureResp->flags & FEAT_CHANGE_SENSITIVITY_ENABLED) != 0);
                    sh2AsyncEvent.sh2SensorConfigResp.sensorConfig.changeSensitivityRelative = ((pGetFeatureResp->flags & FEAT_CHANGE_SENSITIVITY_RELATIVE) != 0);
                    sh2AsyncEvent.sh2SensorConfigResp.sensorConfig.wakeupEnabled = ((pGetFeatureResp->flags & FEAT_WAKE_ENABLED) != 0);
                    sh2AsyncEvent.sh2SensorConfigResp.sensorConfig.alwaysOnEnabled = ((pGetFeatureResp->flags & FEAT_ALWAYS_ON_ENABLED) != 0);
                    sh2AsyncEvent.sh2SensorConfigResp.sensorConfig.changeSensitivity = pGetFeatureResp->changeSensitivity;
                    sh2AsyncEvent.sh2SensorConfigResp.sensorConfig.reportInterval_us = pGetFeatureResp->reportInterval_uS;
                    sh2AsyncEvent.sh2SensorConfigResp.sensorConfig.batchInterval_us = pGetFeatureResp->batchInterval_uS;
                    sh2AsyncEvent.sh2SensorConfigResp.sensorConfig.sensorSpecific = pGetFeatureResp->sensorSpecific;

                    pSh2->eventCallback(pSh2->eventCookie, &sh2AsyncEvent);
                }
            }

            // Hand off to operation in progress, if any
            opRx(pSh2, payload+cursor, reportLen);
            cursor += reportLen;
        }
    }
}

static int opCompleted(sh2_t *pSh2, int status)
{
    // Record status
    pSh2->opStatus = status;

    // Signal that op is done.
    pSh2->pOp = 0;

    return SH2_OK;
}

static int opProcess(sh2_t *pSh2, const sh2_Op_t *pOp)
{
    int status = SH2_OK;
    uint32_t start_us = 0;

    start_us = pSh2->pHal->getTimeUs(pSh2->pHal);
    
    status = opStart(&_sh2, pOp);
    if (status != SH2_OK) {
        return status;
    }

    uint32_t now_us = start_us;
    
    // While op not complete and not timed out.
    while ((pSh2->pOp != 0) &&
           ((pOp->timeout_us == 0) ||
            ((now_us-start_us) < pOp->timeout_us))) {
        // Service SHTP to poll the device.
        shtp_service(pSh2->pShtp);

        // Update the time
        now_us = pSh2->pHal->getTimeUs(pSh2->pHal);
    }

    if (pSh2->pOp != 0) {
        // Operation has timed out.  Clean up.
        pSh2->pOp = 0;
        pSh2->opStatus = SH2_ERR_TIMEOUT;
    }

    return pSh2->opStatus;
}

static uint8_t getReportLen(sh2_t *pSh2, uint8_t reportId)
{
    for (int n = 0; n < SH2_MAX_REPORT_IDS; n++) {
        if (pSh2->report[n].id == reportId) {
            return pSh2->report[n].len;
        }
    }

    return 0;
}

// Produce 64-bit microsecond timestamp for a sensor event
static uint64_t touSTimestamp(uint32_t hostInt, int32_t referenceDelta, uint16_t delay)
{
    static uint32_t lastHostInt = 0;
    static uint32_t rollovers = 0;
    uint64_t timestamp;

    // Count times hostInt timestamps rolLED over to produce upper bits
    if (hostInt < lastHostInt) {
        rollovers++;
    }
    lastHostInt = hostInt;
    
    timestamp = ((uint64_t)rollovers << 32);
    timestamp += hostInt + (referenceDelta + delay) * 100;

    return timestamp;
}

static void sensorhubInputHdlr(sh2_t *pSh2, uint8_t *payload, uint16_t len, uint32_t timestamp)
{
    sh2_SensorEvent_t event;
    uint16_t cursor = 0;

    uint32_t referenceDelta;

    referenceDelta = 0;

    while (cursor < len) {
        // Get next report id
        uint8_t reportId = payload[cursor];

        // Determine report length
        uint8_t reportLen = getReportLen(pSh2, reportId);
        if (reportLen == 0) {
            // An unrecognized report id
            pSh2->unknownReportIds++;
            return;
        }
        else {
            if (reportId == SENSORHUB_BASE_TIMESTAMP_REF) {
                const BaseTimestampRef_t *rpt = (const BaseTimestampRef_t *)(payload+cursor);
                
                // store base timestamp reference
                referenceDelta = -rpt->timebase;
            }
            else if (reportId == SENSORHUB_TIMESTAMP_REBASE) {
                const TimestampRebase_t *rpt = (const TimestampRebase_t *)(payload+cursor);

                referenceDelta += rpt->timebase;
            }
            else if (reportId == SENSORHUB_FLUSH_COMPLETED) {
                // Route this as if it arrived on command channel.
                opRx(pSh2, payload+cursor, reportLen);
            }
            else {
                uint8_t *pReport = payload+cursor;
                uint16_t delay = ((pReport[2] & 0xFC) << 6) + pReport[3];
                event.timestamp_uS = touSTimestamp(timestamp, referenceDelta, delay);
                event.reportId = reportId;
                memcpy(event.report, pReport, reportLen);
                event.len = reportLen;
                if (pSh2->sensorCallback != 0) {
                    pSh2->sensorCallback(pSh2->sensorCookie, &event);
                }
            }
            cursor += reportLen;
        }
    }
}

static void sensorhubInputNormalHdlr(void *cookie, uint8_t *payload, uint16_t len, uint32_t timestamp)
{
    sh2_t *pSh2 = (sh2_t *)cookie;

    sensorhubInputHdlr(pSh2, payload, len, timestamp);
}

static void sensorhubInputWakeHdlr(void *cookie, uint8_t *payload, uint16_t len, uint32_t timestamp)
{
    sh2_t *pSh2 = (sh2_t *)cookie;
    
    sensorhubInputHdlr(pSh2, payload, len, timestamp);
}

static void sensorhubInputGyroRvHdlr(void *cookie, uint8_t *payload, uint16_t len, uint32_t timestamp)
{
    sh2_t *pSh2 = (sh2_t *)cookie;
    sh2_SensorEvent_t event;
    uint16_t cursor = 0;

    uint8_t reportId = SH2_GYRO_INTEGRATED_RV;
    uint8_t reportLen = getReportLen(pSh2, reportId);

    while (cursor < len) {
        event.timestamp_uS = timestamp;
        event.reportId = reportId;
        memcpy(event.report, payload+cursor, reportLen);
        event.len = reportLen;

        if (pSh2->sensorCallback != 0) {
            pSh2->sensorCallback(pSh2->sensorCookie, &event);
        }

        cursor += reportLen;
    }
}

static void executableAdvertHdlr(void *cookie, uint8_t tag, uint8_t len, uint8_t *value)
{
    // Ignore.  No known TLV tags for this app.
}

static void executabLEDeviceHdlr(void *cookie, uint8_t *payload, uint16_t len, uint32_t timestamp)
{
    sh2_t *pSh2 = (sh2_t *)cookie;

    // Discard if length is bad
    if (len != 1) {
        pSh2->execBadPayload++;
        return;
    }
    
    switch (payload[0]) {
        case EXECUTABLE_DEVICE_RESP_RESET_COMPLETE:
            // reset process is now done.
            pSh2->resetComplete = true;
            
            // Notify client that reset is complete.
            sh2AsyncEvent.eventId = SH2_RESET;
            if (pSh2->eventCallback) {
                pSh2->eventCallback(pSh2->eventCookie, &sh2AsyncEvent);
            }
            break;
        default:
            pSh2->execBadPayload++;
            break;
    }
}

static int sendExecutable(sh2_t *pSh2, uint8_t cmd)
{
    return shtp_send(pSh2->pShtp, pSh2->executableChan, &cmd, 1);
}

static int sendCtrl(sh2_t *pSh2, const uint8_t *data, uint16_t len)
{
    return shtp_send(pSh2->pShtp, pSh2->controlChan, data, len);
}

static int16_t toQ14(double x)
{
    int16_t retval = (int16_t)(x * (1<<14));
    
    return retval;
}

// ------------------------------------------------------------------------
// Get Product ID support

// Get Product ID Op handler
static int getProdIdStart(sh2_t *pSh2)
{
    int rc = SH2_OK;
    ProdIdReq_t req;
    
    pSh2->opData.getProdIds.nextEntry = 0;
    pSh2->opData.getProdIds.expectedEntries = 4;  // Most products supply 4 product ids.
                                                // When the first arrives, we'll know if
                                                // we need to adjust this.
    
    // Set up request to issue
    memset(&req, 0, sizeof(req));
    req.reportId = SENSORHUB_PROD_ID_REQ;
    rc = sendCtrl(pSh2, (uint8_t *)&req, sizeof(req));

    return rc;
}

static void getProdIdRx(sh2_t *pSh2, const uint8_t *payload, uint16_t len)
{
    ProdIdResp_t *resp = (ProdIdResp_t *)payload;
    
    // skip this if it isn't the product id response.
    if (resp->reportId != SENSORHUB_PROD_ID_RESP) return;

    // Store this product id, if we can
    sh2_ProductIds_t *pProdIds = pSh2->opData.getProdIds.pProdIds;
    
    if (pProdIds) {
        // Store the product id response
        if (pSh2->opData.getProdIds.nextEntry < pSh2->opData.getProdIds.expectedEntries) {
            sh2_ProductId_t *pProdId = &pProdIds->entry[pSh2->opData.getProdIds.nextEntry];
            
            pProdId->resetCause = resp->resetCause;
            pProdId->swVersionMajor = resp->swVerMajor;
            pProdId->swVersionMinor = resp->swVerMinor;
            pProdId->swPartNumber = resp->swPartNumber;
            pProdId->swBuildNumber = resp->swBuildNumber;
            pProdId->swVersionPatch = resp->swVerPatch;
            pProdId->reserved0 = resp->reserved0;
            pProdId->reserved1 = resp->reserved1;

            if (pProdId->swPartNumber == 10004095) {
                // FSP200 has 5 product id entries
                pSh2->opData.getProdIds.expectedEntries = 5;
            }


            pSh2->opData.getProdIds.nextEntry++;
        }
    }

    // Complete this operation if there is no storage for more product ids
    if ((pSh2->opData.getProdIds.pProdIds == 0) ||
        (pSh2->opData.getProdIds.nextEntry >= pSh2->opData.getProdIds.expectedEntries)) {
        
        pSh2->opData.getProdIds.pProdIds->numEntries = pSh2->opData.getProdIds.nextEntry;
        opCompleted(pSh2, SH2_OK);
    }

    return;
}

const sh2_Op_t getProdIdOp = {
    .start = getProdIdStart,
    .rx = getProdIdRx,
};

// ------------------------------------------------------------------------
// Set Sensor Config

static int getSensorConfigStart(sh2_t *pSh2)
{
    int rc = SH2_OK;
    GetFeatureReq_t req;
    
    // set up request to issue
    memset(&req, 0, sizeof(req));
    req.reportId = SENSORHUB_GET_FEATURE_REQ;
    req.featureReportId = pSh2->opData.getSensorConfig.sensorId;
    rc = sendCtrl(pSh2, (uint8_t *)&req, sizeof(req));

    return rc;
}

static void getSensorConfigRx(sh2_t *pSh2, const uint8_t *payload, uint16_t len)
{
    GetFeatureResp_t *resp = (GetFeatureResp_t *)payload;
    sh2_SensorConfig_t *pConfig;
    
    // skip this if it isn't the response we're waiting for.
    if (resp->reportId != SENSORHUB_GET_FEATURE_RESP) return;
    if (resp->featureReportId != pSh2->opData.getSensorConfig.sensorId) return;

    // Copy out data
    pConfig = pSh2->opData.getSensorConfig.pConfig;
    
    pConfig->changeSensitivityEnabled = ((resp->flags & FEAT_CHANGE_SENSITIVITY_ENABLED) != 0);
    pConfig->changeSensitivityRelative = ((resp->flags & FEAT_CHANGE_SENSITIVITY_RELATIVE) != 0);
    pConfig->wakeupEnabled = ((resp->flags & FEAT_WAKE_ENABLED) != 0);
    pConfig->alwaysOnEnabled = ((resp->flags & FEAT_ALWAYS_ON_ENABLED) != 0);
    pConfig->changeSensitivity = resp->changeSensitivity;
    pConfig->reportInterval_us = resp->reportInterval_uS;
    pConfig->batchInterval_us = resp->batchInterval_uS;
    pConfig->sensorSpecific = resp->sensorSpecific;

    // Complete this operation
    opCompleted(pSh2, SH2_OK);

    return;
}

const sh2_Op_t getSensorConfigOp = {
    .start = getSensorConfigStart,
    .rx = getSensorConfigRx,
};

// ------------------------------------------------------------------------
// Set Sensor Config

// SENSORHUB_SET_FEATURE_CMD
#define SENSORHUB_SET_FEATURE_CMD    (0xFD)
typedef PACKED_STRUCT {
    uint8_t reportId;             // 0xFD
    uint8_t featureReportId;      // sensor id
    uint8_t flags;                // FEAT_... values
    uint16_t changeSensitivity;
    uint32_t reportInterval_uS;
    uint32_t batchInterval_uS;
    uint32_t sensorSpecific;
} SetFeatureReport_t;

static int setSensorConfigStart(sh2_t *pSh2)
{
    SetFeatureReport_t req;
    uint8_t flags = 0;
    int rc;
    sh2_SensorConfig_t *pConfig = pSh2->opData.getSensorConfig.pConfig;
    
    if (pConfig->changeSensitivityEnabled)  flags |= FEAT_CHANGE_SENSITIVITY_ENABLED;
    if (pConfig->changeSensitivityRelative) flags |= FEAT_CHANGE_SENSITIVITY_RELATIVE;
    if (pConfig->wakeupEnabled)             flags |= FEAT_WAKE_ENABLED;
    if (pConfig->alwaysOnEnabled)           flags |= FEAT_ALWAYS_ON_ENABLED;

    memset(&req, 0, sizeof(req));
    req.reportId = SENSORHUB_SET_FEATURE_CMD;
    req.featureReportId = pSh2->opData.setSensorConfig.sensorId;
    req.flags = flags;
    req.changeSensitivity = pConfig->changeSensitivity;
    req.reportInterval_uS = pConfig->reportInterval_us;
    req.batchInterval_uS = pConfig->batchInterval_us;
    req.sensorSpecific = pConfig->sensorSpecific;

    rc = sendCtrl(pSh2, (uint8_t *)&req, sizeof(req));
    opCompleted(pSh2, rc);

    return rc;
}

const sh2_Op_t setSensorConfigOp = {
    .start = setSensorConfigStart,
};

// ------------------------------------------------------------------------
// Get FRS.

// SENSORHUB_FRS_WRITE_REQ
#define SENSORHUB_FRS_WRITE_REQ      (0xF7)
typedef PACKED_STRUCT {
    uint8_t reportId;
    uint8_t reserved;
    uint16_t length;
    uint16_t frsType;
} FrsWriteReq_t;

// SENSORHUB_FRS_WRITE_DATA_REQ
#define SENSORHUB_FRS_WRITE_DATA_REQ (0xF6)
typedef PACKED_STRUCT {
    uint8_t reportId;
    uint8_t reserved;
    uint16_t offset;
    uint32_t data0;
    uint32_t data1;
} FrsWriteDataReq_t;

// FRS write status values
#define FRS_WRITE_STATUS_RECEIVED (0)
#define FRS_WRITE_STATUS_UNRECOGNIZED_FRS_TYPE (1)
#define FRS_WRITE_STATUS_BUSY (2)
#define FRS_WRITE_STATUS_WRITE_COMPLETED (3)
#define FRS_WRITE_STATUS_READY (4)
#define FRS_WRITE_STATUS_FAILED (5)
#define FRS_WRITE_STATUS_NOT_READY (6) // data received when not in write mode
#define FRS_WRITE_STATUS_INVALID_LENGTH (7)
#define FRS_WRITE_STATUS_RECORD_VALID (8)
#define FRS_WRITE_STATUS_INVALID_RECORD (9)
#define FRS_WRITE_STATUS_DEVICE_ERROR (10)
#define FRS_WRITE_STATUS_READ_ONLY (11)

// SENSORHUB_FRS_WRITE_RESP
#define SENSORHUB_FRS_WRITE_RESP     (0xF5)
typedef PACKED_STRUCT {
    uint8_t reportId;
    uint8_t status;
    uint16_t wordOffset;
} FrsWriteResp_t;

// RESP_FRS_READ_REQ
#define SENSORHUB_FRS_READ_REQ       (0xF4)
typedef PACKED_STRUCT {
    uint8_t reportId;
    uint8_t reserved;
    uint16_t readOffset;
    uint16_t frsType;
    uint16_t blockSize;
} FrsReadReq_t;

// Get Datalen portion of len_status field
#define FRS_READ_DATALEN(x) ((x >> 4) & 0x0F)

// Get status portion of len_status field
#define FRS_READ_STATUS(x) ((x) & 0x0F)

// Status values
#define FRS_READ_STATUS_NO_ERROR                        0
#define FRS_READ_STATUS_UNRECOGNIZED_FRS_TYPE           1
#define FRS_READ_STATUS_BUSY                            2
#define FRS_READ_STATUS_READ_RECORD_COMPLETED           3
#define FRS_READ_STATUS_OFFSET_OUT_OF_RANGE             4
#define FRS_READ_STATUS_RECORD_EMPTY                    5
#define FRS_READ_STATUS_READ_BLOCK_COMPLETED            6
#define FRS_READ_STATUS_READ_BLOCK_AND_RECORD_COMPLETED 7
#define FRS_READ_STATUS_DEVICE_ERROR                    8

// SENSORHUB_FRS_READ_RESP
#define SENSORHUB_FRS_READ_RESP      (0xF3)
typedef PACKED_STRUCT {
    uint8_t reportId;
    uint8_t len_status;  // See FRS_READ... macros above
    uint16_t wordOffset;
    uint32_t data0;
    uint32_t data1;
    uint16_t frsType;
    uint8_t reserved0;
    uint8_t reserved1;
} FrsReadResp_t;

static int getFrsStart(sh2_t *pSh2)
{
    int rc = SH2_OK;
    FrsReadReq_t req;

    pSh2->opData.getFrs.nextOffset = 0;
    
    // set up request to issue
    memset(&req, 0, sizeof(req));
    req.reportId = SENSORHUB_FRS_READ_REQ;
    req.reserved = 0;
    req.readOffset = 0; // read from start
    req.frsType = pSh2->opData.getFrs.frsType;
    req.blockSize = 0;  // read all avail data

    rc = sendCtrl(pSh2, (uint8_t *)&req, sizeof(req));

    return rc;
}

static void getFrsRx(sh2_t *pSh2, const uint8_t *payload, uint16_t len)
{
    FrsReadResp_t *resp = (FrsReadResp_t *)payload;
    uint8_t status;

    // skip this if it isn't the response we're looking for
    if (resp->reportId != SENSORHUB_FRS_READ_RESP) return;

    // Check for errors: Unrecognized FRS type, Busy, Out of range, Device error
    status = FRS_READ_STATUS(resp->len_status);
    if ((status == FRS_READ_STATUS_UNRECOGNIZED_FRS_TYPE) ||
        (status == FRS_READ_STATUS_BUSY) ||
        (status == FRS_READ_STATUS_OFFSET_OUT_OF_RANGE) ||
        (status == FRS_READ_STATUS_DEVICE_ERROR)
        ) {
        // Operation faiLED
        opCompleted(pSh2, SH2_ERR_HUB);
        return;
    }

    if (status == FRS_READ_STATUS_RECORD_EMPTY) {
        // Empty record, return zero length.
        *(pSh2->opData.getFrs.pWords) = 0;
        opCompleted(pSh2, SH2_OK);
    }

    // Store the contents from this response
    uint16_t offset = resp->wordOffset;

    // check for missed offsets, resulting in error.
    if (offset != pSh2->opData.getFrs.nextOffset) {
        // Some data was dropped.
        *(pSh2->opData.getFrs.pWords) = 0;
        opCompleted(pSh2, SH2_ERR_IO);
    }
    
    // store first word, if we have room
    if ((*(pSh2->opData.getFrs.pWords) == 0) ||
        (offset <= *(pSh2->opData.getFrs.pWords))) {
        pSh2->opData.getFrs.pData[offset] = resp->data0;
        pSh2->opData.getFrs.nextOffset = offset+1;
    }

    // store second word if there is one and we have room
    if ((FRS_READ_DATALEN(resp->len_status) == 2)  &&
        ((*(pSh2->opData.getFrs.pWords) == 0) ||
         (offset <= *(pSh2->opData.getFrs.pWords)))) {
        pSh2->opData.getFrs.pData[offset+1] = resp->data1;
        pSh2->opData.getFrs.nextOffset = offset+2;
    }

    // If read is done, complete the operation
    if ((status == FRS_READ_STATUS_READ_RECORD_COMPLETED) ||
        (status == FRS_READ_STATUS_READ_BLOCK_COMPLETED) ||
        (status == FRS_READ_STATUS_READ_BLOCK_AND_RECORD_COMPLETED)) {
        *(pSh2->opData.getFrs.pWords) = pSh2->opData.getFrs.nextOffset;

        opCompleted(pSh2, SH2_OK);
    }

    return;
}

const sh2_Op_t getFrsOp = {
    .start = getFrsStart,
    .rx = getFrsRx,
};

// ------------------------------------------------------------------------
// Support for sh2_getMetadata

const static struct {
    sh2_SensorId_t sensorId;
    uint16_t recordId;
} sensorToRecordMap[] = {
    { SH2_RAW_ACCELEROMETER,            FRS_ID_META_RAW_ACCELEROMETER },
    { SH2_ACCELEROMETER,                FRS_ID_META_ACCELEROMETER },
    { SH2_LINEAR_ACCELERATION,          FRS_ID_META_LINEAR_ACCELERATION },
    { SH2_GRAVITY,                      FRS_ID_META_GRAVITY },
    { SH2_RAW_GYROSCOPE,                FRS_ID_META_RAW_GYROSCOPE },
    { SH2_GYROSCOPE_CALIBRATED,         FRS_ID_META_GYROSCOPE_CALIBRATED },
    { SH2_GYROSCOPE_UNCALIBRATED,       FRS_ID_META_GYROSCOPE_UNCALIBRATED },
    { SH2_RAW_MAGNETOMETER,             FRS_ID_META_RAW_MAGNETOMETER },
    { SH2_MAGNETIC_FIELD_CALIBRATED,    FRS_ID_META_MAGNETIC_FIELD_CALIBRATED },
    { SH2_MAGNETIC_FIELD_UNCALIBRATED,  FRS_ID_META_MAGNETIC_FIELD_UNCALIBRATED },
    { SH2_ROTATION_VECTOR,              FRS_ID_META_ROTATION_VECTOR },
    { SH2_GAME_ROTATION_VECTOR,         FRS_ID_META_GAME_ROTATION_VECTOR },
    { SH2_GEOMAGNETIC_ROTATION_VECTOR,  FRS_ID_META_GEOMAGNETIC_ROTATION_VECTOR },
    { SH2_PRESSURE,                     FRS_ID_META_PRESSURE },
    { SH2_AMBIENT_LIGHT,                FRS_ID_META_AMBIENT_LIGHT },
    { SH2_HUMIDITY,                     FRS_ID_META_HUMIDITY },
    { SH2_PROXIMITY,                    FRS_ID_META_PROXIMITY },
    { SH2_TEMPERATURE,                  FRS_ID_META_TEMPERATURE },
    { SH2_TAP_DETECTOR,                 FRS_ID_META_TAP_DETECTOR },
    { SH2_STEP_DETECTOR,                FRS_ID_META_STEP_DETECTOR },
    { SH2_STEP_COUNTER,                 FRS_ID_META_STEP_COUNTER },
    { SH2_SIGNIFICANT_MOTION,           FRS_ID_META_SIGNIFICANT_MOTION },
    { SH2_STABILITY_CLASSIFIER,         FRS_ID_META_STABILITY_CLASSIFIER },
    { SH2_SHAKE_DETECTOR,               FRS_ID_META_SHAKE_DETECTOR },
    { SH2_FLIP_DETECTOR,                FRS_ID_META_FLIP_DETECTOR },
    { SH2_PICKUP_DETECTOR,              FRS_ID_META_PICKUP_DETECTOR },
    { SH2_STABILITY_DETECTOR,           FRS_ID_META_STABILITY_DETECTOR },
    { SH2_PERSONAL_ACTIVITY_CLASSIFIER, FRS_ID_META_PERSONAL_ACTIVITY_CLASSIFIER },
    { SH2_SLEEP_DETECTOR,               FRS_ID_META_SLEEP_DETECTOR },
    { SH2_TILT_DETECTOR,                FRS_ID_META_TILT_DETECTOR },
    { SH2_POCKET_DETECTOR,              FRS_ID_META_POCKET_DETECTOR },
    { SH2_CIRCLE_DETECTOR,              FRS_ID_META_CIRCLE_DETECTOR },
};

static void stuffMetadata(sh2_SensorMetadata_t *pData, uint32_t *frsData)
{
    // Populate the sensorMetadata structure with results
    pData->meVersion        = (frsData[0] >> 0) & 0xFF;
    pData->mhVersion        = (frsData[0] >> 8) & 0xFF;
    pData->shVersion        = (frsData[0] >> 16) & 0xFF;
    pData->range            = frsData[1];
    pData->resolution       = frsData[2];
    pData->power_mA         = (frsData[3] >> 0) & 0xFFFF;    // 16.10 format
    pData->revision         = (frsData[3] >> 16) & 0xFFFF;
    pData->minPeriod_uS     = frsData[4];
    pData->maxPeriod_uS     = 0;  // ...unless reading format 4 metadata below
    pData->fifoMax          = (frsData[5] >> 0) & 0xFFFF;
    pData->fifoReserved     = (frsData[5] >> 16) & 0xFFFF;
    pData->batchBufferBytes = (frsData[6] >> 0) & 0xFFFF;;
    pData->vendorIdLen      = (frsData[6] >> 16) & 0xFFFF;

    // Init fields that may not be present, depending on metadata revision
    pData->qPoint1           = 0;
    pData->qPoint2           = 0;
    pData->qPoint3           = 0;
    pData->sensorSpecificLen = 0;
    strcpy(pData->vendorId, ""); // init with empty string in case vendorIdLen == 0

    int vendorIdOffset = 8;
    // Get revision-specific fields
    if (pData->revision == 0) {
        // No fixed fields, vendor id copied after if-else block
    }
    else if (pData->revision == 1) {
        pData->qPoint1        = (frsData[7] >> 0) & 0xFFFF;
        pData->qPoint2        = (frsData[7] >> 16) & 0xFFFF;
    }
    else if (pData->revision == 2) {
        pData->qPoint1        = (frsData[7] >> 0) & 0xFFFF;
        pData->qPoint2        = (frsData[7] >> 16) & 0xFFFF;
        pData->sensorSpecificLen = (frsData[8] >> 0) & 0xFFFF;
        memcpy(pData->sensorSpecific, (uint8_t *)&frsData[9], pData->sensorSpecificLen);
        vendorIdOffset = 9 + ((pData->sensorSpecificLen+3)/4); // 9 + one word for every 4 bytes of SS data
    }
    else if (pData->revision == 3) {
        pData->qPoint1        = (frsData[7] >> 0) & 0xFFFF;
        pData->qPoint2        = (frsData[7] >> 16) & 0xFFFF;
        pData->sensorSpecificLen = (frsData[8] >> 0) & 0xFFFF;
        pData->qPoint3        = (frsData[8] >> 16) & 0xFFFF;
        memcpy(pData->sensorSpecific, (uint8_t *)&frsData[9], pData->sensorSpecificLen);
        vendorIdOffset = 9 + ((pData->sensorSpecificLen+3)/4); // 9 + one word for every 4 bytes of SS data
    }
    else if (pData->revision == 4) {
        pData->qPoint1        = (frsData[7] >> 0) & 0xFFFF;
        pData->qPoint2        = (frsData[7] >> 16) & 0xFFFF;
        pData->sensorSpecificLen = (frsData[8] >> 0) & 0xFFFF;
        pData->qPoint3        = (frsData[8] >> 16) & 0xFFFF;
        pData->maxPeriod_uS   = frsData[9];
        memcpy(pData->sensorSpecific, (uint8_t *)&frsData[10], pData->sensorSpecificLen);
        vendorIdOffset = 10 + ((pData->sensorSpecificLen+3)/4); // 9 + one word for every 4 bytes of SS data
    }
    else {
        // Unrecognized revision!
    }

    // Copy vendor id
    memcpy(pData->vendorId, (uint8_t *)&frsData[vendorIdOffset],
           pData->vendorIdLen);
}

// ------------------------------------------------------------------------
// Set FRS.

static int setFrsStart(sh2_t *pSh2)
{
    int rc = SH2_OK;
    FrsWriteReq_t req;

    pSh2->opData.setFrs.offset = 0;
    
    // set up request to issue
    memset(&req, 0, sizeof(req));
    req.reportId = SENSORHUB_FRS_WRITE_REQ;
    req.reserved = 0;
    req.length = pSh2->opData.setFrs.words;
    req.frsType = pSh2->opData.getFrs.frsType;

    rc = sendCtrl(pSh2, (uint8_t *)&req, sizeof(req));

    return rc;
}

static void setFrsRx(sh2_t *pSh2, const uint8_t *payload, uint16_t len)
{
    FrsWriteResp_t *resp = (FrsWriteResp_t *)payload;
    FrsWriteDataReq_t req;
    uint8_t status;
    bool sendMoreData = false;
    bool completed = false;
    int rc = SH2_OK;

    // skip this if it isn't the response we're looking for.
    if (resp->reportId != SENSORHUB_FRS_WRITE_RESP) return;

    // Check for errors: Unrecognized FRS type, Busy, Out of range, Device error
    status = resp->status;
    switch(status) {
        case FRS_WRITE_STATUS_RECEIVED:
        case FRS_WRITE_STATUS_READY:
            sendMoreData = true;
            break;
        case FRS_WRITE_STATUS_UNRECOGNIZED_FRS_TYPE:
        case FRS_WRITE_STATUS_BUSY:
        case FRS_WRITE_STATUS_FAILED:
        case FRS_WRITE_STATUS_NOT_READY:
        case FRS_WRITE_STATUS_INVALID_LENGTH:
        case FRS_WRITE_STATUS_INVALID_RECORD:
        case FRS_WRITE_STATUS_DEVICE_ERROR:
        case FRS_WRITE_STATUS_READ_ONLY:
            rc = SH2_ERR_HUB;
            completed = true;
            break;
        case FRS_WRITE_STATUS_WRITE_COMPLETED:
            // Successful completion
            rc = SH2_OK;
            completed = true;
            break;
        case FRS_WRITE_STATUS_RECORD_VALID:
            // That's nice, keep waiting
            break;
    }

    // if we should send more data, do it.
    if (sendMoreData &&
        (pSh2->opData.setFrs.offset < pSh2->opData.setFrs.words)) {
        uint16_t offset = pSh2->opData.setFrs.offset;
        
        memset(&req, 0, sizeof(req));
        req.reportId = SENSORHUB_FRS_WRITE_DATA_REQ;
        req.reserved = 0;
        req.offset = offset;
        req.data0 = pSh2->opData.setFrs.pData[offset++];
        if (offset < pSh2->opData.setFrs.words) {
            req.data1 = pSh2->opData.setFrs.pData[offset++];
        } else {
            req.data1 = 0;
        }
        pSh2->opData.setFrs.offset = offset;
        
        rc = sendCtrl(pSh2, (uint8_t *)&req, sizeof(req));
    }

    // if the operation is done or has to be aborted, complete it
    if (completed) {
        opCompleted(pSh2, rc);
    }

    return;
}

const sh2_Op_t setFrsOp = {
    .start = setFrsStart,
    .rx = setFrsRx,
};

// ------------------------------------------------------------------------
// Support for sending commands

static int sendCmd(sh2_t *pSh2, uint8_t cmd, uint8_t p[COMMAND_PARAMS])
{
    int rc = SH2_OK;
    CommandReq_t req;

    // Clear request structure
    memset(&req, 0, sizeof(req));
    
    // Create a command sequence number for this command
    pSh2->lastCmdId = cmd;
    pSh2->cmdSeq = pSh2->nextCmdSeq++;
    
    // set up request to issue
    req.reportId = SENSORHUB_COMMAND_REQ;
    req.seq = pSh2->cmdSeq;
    req.command = cmd;
    for (int n = 0; n < COMMAND_PARAMS; n++) {
        req.p[n] = p[n];
    }
    
    rc = sendCtrl(pSh2, (uint8_t *)&req, sizeof(req));
    
    return rc;
}

// Send a command with 0 parameters
static int sendCmd0(sh2_t *pSh2, uint8_t cmd)
{
    uint8_t p[COMMAND_PARAMS];

    memset(p, 0, COMMAND_PARAMS);

    return sendCmd(pSh2, cmd, p);
}

// Send a command with 1 parameter
static int sendCmd1(sh2_t *pSh2, uint8_t cmd, uint8_t p0)
{
    uint8_t p[COMMAND_PARAMS];

    memset(p, 0, COMMAND_PARAMS);

    p[0] = p0;
    return sendCmd(pSh2, cmd, p);
}

// Send a command with 2 parameters
static int sendCmd2(sh2_t *pSh2, uint8_t cmd, uint8_t p0, uint8_t p1)
{
    uint8_t p[COMMAND_PARAMS];

    memset(p, 0, COMMAND_PARAMS);
    
    p[0] = p0;
    p[1] = p1;
    return sendCmd(pSh2, cmd, p);
}

static bool wrongResponse(sh2_t *pSh2, CommandResp_t *resp)
{
    if (resp->reportId != SENSORHUB_COMMAND_RESP) return true;
    if (resp->command != pSh2->lastCmdId) return true;
    if (resp->commandSeq != pSh2->cmdSeq) return true;

    return false;
}

// ------------------------------------------------------------------------
// Get Errors

static int getErrorsStart(sh2_t *pSh2)
{
    // Initialize state
    pSh2->opData.getErrors.errsRead = 0;

    return sendCmd1(pSh2, SH2_CMD_ERRORS, pSh2->opData.getErrors.severity);
}

static void getErrorsRx(sh2_t *pSh2, const uint8_t *payload, uint16_t len)
{
    CommandResp_t *resp = (CommandResp_t *)payload;
    
    // skip this if it isn't the right response
    if (wrongResponse(pSh2, resp)) return;

    if (resp->r[2] == 255) {
        // No error to report, operation is complete
        *(pSh2->opData.getErrors.pNumErrors) = pSh2->opData.getErrors.errsRead;
        opCompleted(pSh2, SH2_OK);
    } else {
        // Copy data for invoker.
        unsigned int index = pSh2->opData.getErrors.errsRead;
        if (index < *(pSh2->opData.getErrors.pNumErrors)) {
            // We have room for this one.
            pSh2->opData.getErrors.pErrors[index].severity = resp->r[0];
            pSh2->opData.getErrors.pErrors[index].sequence = resp->r[1];
            pSh2->opData.getErrors.pErrors[index].source = resp->r[2];
            pSh2->opData.getErrors.pErrors[index].error = resp->r[3];
            pSh2->opData.getErrors.pErrors[index].module = resp->r[4];
            pSh2->opData.getErrors.pErrors[index].code = resp->r[5];

            pSh2->opData.getErrors.errsRead++;
        }
    }

    return;
}

const sh2_Op_t getErrorsOp = {
    .start = getErrorsStart,
    .rx = getErrorsRx,
};

// ------------------------------------------------------------------------
// Get Counts

static int getCountsStart(sh2_t *pSh2)
{
    return sendCmd2(pSh2, SH2_CMD_COUNTS, SH2_COUNTS_GET_COUNTS, pSh2->opData.getCounts.sensorId);
}

static void getCountsRx(sh2_t *pSh2, const uint8_t *payload, uint16_t len)
{
    CommandResp_t *resp = (CommandResp_t *)payload;

    if (wrongResponse(pSh2, resp)) return;
    
    // Store results
    if (resp->respSeq == 0) {
        pSh2->opData.getCounts.pCounts->offered = readu32(&resp->r[3]);
        pSh2->opData.getCounts.pCounts->accepted = readu32(&resp->r[7]);
    }
    else {
        pSh2->opData.getCounts.pCounts->on = readu32(&resp->r[3]);
        pSh2->opData.getCounts.pCounts->attempted = readu32(&resp->r[7]);
    }
    
    // Complete this operation if we've received last response
    if (resp->respSeq == 1) {
        opCompleted(pSh2, SH2_OK);
    }

    return;
}

const sh2_Op_t getCountsOp = {
    .start = getCountsStart,
    .rx = getCountsRx,
};

// ------------------------------------------------------------------------
// Generic Send Command

static int sendCmdStart(sh2_t *pSh2)
{
    int status = sendCmd(pSh2, pSh2->opData.sendCmd.req.command,
                     pSh2->opData.sendCmd.req.p);

    opCompleted(pSh2, status);

    return status;
}

const sh2_Op_t sendCmdOp = {
    .start = sendCmdStart,
};

// ------------------------------------------------------------------------
// Reinit

static int reinitStart(sh2_t *pSh2)
{
    int status = sendCmd1(pSh2, SH2_CMD_INITIALIZE, SH2_INIT_SYSTEM);

    return status;
}

static void reinitRx(sh2_t *pSh2, const uint8_t *payload, uint16_t len)
{
    CommandResp_t *resp = (CommandResp_t *)payload;

    // Ignore message if it doesn't pertain to this operation
    if (wrongResponse(pSh2, resp)) return;

    // Get return status
    int status = SH2_OK;
    if (resp->r[0] != 0) {
        status = SH2_ERR_HUB;
    }

    opCompleted(pSh2, status);
}

const sh2_Op_t reinitOp = {
    .start = reinitStart,
    .rx = reinitRx,
};

// ------------------------------------------------------------------------
// Save DCD Now

static int saveDcdNowStart(sh2_t *pSh2)
{
    int status = sendCmd0(pSh2, SH2_CMD_DCD);

    return status;
}

static void saveDcdNowRx(sh2_t *pSh2, const uint8_t *payload, uint16_t len)
{
    CommandResp_t *resp = (CommandResp_t *)payload;

    // Ignore message if it doesn't pertain to this operation
    if (wrongResponse(pSh2, resp)) return;

    // Get return status
    int status = SH2_OK;
    if (resp->r[0] != 0) {
        status = SH2_ERR_HUB;
    }

    opCompleted(pSh2, status);
}

const sh2_Op_t saveDcdNowOp = {
    .start = saveDcdNowStart,
    .rx = saveDcdNowRx,
};

// ------------------------------------------------------------------------
// Get Osc Type

static int getOscTypeStart(sh2_t *pSh2)
{
    return sendCmd0(pSh2, SH2_CMD_GET_OSC_TYPE);
}

static void getOscTypeRx(sh2_t *pSh2, const uint8_t *payload, uint16_t len)
{
    CommandResp_t *resp = (CommandResp_t *)payload;
    sh2_OscType_t *pOscType;
    
    // Ignore message if it doesn't pertain to this operation
    if (wrongResponse(pSh2, resp)) return;

    // Read out data
    pOscType = pSh2->opData.getOscType.pOscType;
    *pOscType = (sh2_OscType_t)resp->r[0];

    // Complete this operation
    opCompleted(pSh2, SH2_OK);
}

const sh2_Op_t getOscTypeOp = {
    .start = getOscTypeStart,
    .rx = getOscTypeRx,
};

// ------------------------------------------------------------------------
// Set Cal Config

static int setCalConfigStart(sh2_t *pSh2)
{
    uint8_t p[COMMAND_PARAMS];

    // Clear p.  (Importantly, set subcommand in p[3] to 0, CONFIGURE)
    memset(p, 0, COMMAND_PARAMS);
    
    // Which cal processes to enable/disable
    p[0] = (pSh2->opData.calConfig.sensors & SH2_CAL_ACCEL) ? 1 : 0; // accel cal
    p[1] = (pSh2->opData.calConfig.sensors & SH2_CAL_GYRO)  ? 1 : 0; // gyro cal
    p[2] = (pSh2->opData.calConfig.sensors & SH2_CAL_MAG)   ? 1 : 0; // mag cal
    p[4] = (pSh2->opData.calConfig.sensors & SH2_CAL_PLANAR) ? 1 : 0; // planar cal
    
    return sendCmd(pSh2, SH2_CMD_ME_CAL, p);
}

static void setCalConfigRx(sh2_t *pSh2, const uint8_t *payload, uint16_t len)
{
    CommandResp_t *resp = (CommandResp_t *)payload;
    
    // Ignore message if it doesn't pertain to this operation
    if (wrongResponse(pSh2, resp)) return;

    // Read out data
    int status = SH2_OK;
    if (resp->r[0] != 0) {
        status = SH2_ERR_HUB;
    }

    // Complete this operation
    opCompleted(pSh2, status);
}

const sh2_Op_t setCalConfigOp = {
    .start = setCalConfigStart,
    .rx = setCalConfigRx,
};

// ------------------------------------------------------------------------
// Get Cal Config

static int getCalConfigStart(sh2_t *pSh2)
{
    uint8_t p[COMMAND_PARAMS];

    // Clear p.  (Importantly, set subcommand in p[3] to 0, CONFIGURE)
    memset(p, 0, COMMAND_PARAMS);
    
    // Subcommand: Get ME Calibration
    p[3] = 0x01;
    
    return sendCmd(pSh2, SH2_CMD_ME_CAL, p);
}

static void getCalConfigRx(sh2_t *pSh2, const uint8_t *payload, uint16_t len)
{
    CommandResp_t *resp = (CommandResp_t *)payload;
    
    // Ignore message if it doesn't pertain to this operation
    if (wrongResponse(pSh2, resp)) return;

    // Read out data
    int status = SH2_OK;
    if (resp->r[0] != 0) {
        status = SH2_ERR_HUB;
    }
    else {
        // Unload results into pSensors
        uint8_t sensors = 0;
        if (resp->r[1]) sensors |= SH2_CAL_ACCEL;
        if (resp->r[2]) sensors |= SH2_CAL_GYRO;
        if (resp->r[3]) sensors |= SH2_CAL_MAG;
        if (resp->r[4]) sensors |= SH2_CAL_PLANAR;
        *(pSh2->opData.getCalConfig.pSensors) = sensors;
    }
    
    // Complete this operation
    opCompleted(pSh2, status);
}


const sh2_Op_t getCalConfigOp = {
    .start = getCalConfigStart,
    .rx = getCalConfigRx,
};

// ------------------------------------------------------------------------
// Force Flush

static int forceFlushStart(sh2_t *pSh2)
{
    ForceFlushReq_t req;

    memset(&req, 0, sizeof(req));
    req.reportId = SENSORHUB_FORCE_SENSOR_FLUSH;
    req.sensorId = pSh2->opData.forceFlush.sensorId;
    
    return sendCtrl(pSh2, (uint8_t *)&req, sizeof(req));
}

static void forceFlushRx(sh2_t *pSh2, const uint8_t *payload, uint16_t len)
{
    ForceFlushResp_t *resp = (ForceFlushResp_t *)payload;
    
    // Ignore message if it doesn't pertain to this operation
    if (resp->reportId != SENSORHUB_FLUSH_COMPLETED) return;
    if (resp->sensorId != pSh2->opData.forceFlush.sensorId) return;

    // Complete this operation
    opCompleted(pSh2, SH2_OK);
}

const sh2_Op_t forceFlushOp = {
    .start = forceFlushStart,
    .rx = forceFlushRx,
};

// ------------------------------------------------------------------------
// Start Cal

static int startCalStart(sh2_t *pSh2)
{
    uint8_t p[COMMAND_PARAMS];

    // Clear p.  (Importantly, set subcommand in p[3] to 0, CONFIGURE)
    memset(p, 0, COMMAND_PARAMS);
    
    // Subcommand: Get ME Calibration
    p[0] = SH2_CAL_START;
    p[1] = pSh2->opData.startCal.interval_us & 0xFF;          // LSB
    p[2] = (pSh2->opData.startCal.interval_us >> 8) & 0xFF;
    p[3] = (pSh2->opData.startCal.interval_us >> 16) & 0xFF;
    p[4] = (pSh2->opData.startCal.interval_us >> 24) & 0xFF;  // MSB
    
    return sendCmd(pSh2, SH2_CMD_CAL, p);
}

static void startCalRx(sh2_t *pSh2, const uint8_t *payload, uint16_t len)
{
    CommandResp_t *resp = (CommandResp_t *)payload;
    
    // Ignore message if it doesn't pertain to this operation
    if (wrongResponse(pSh2, resp)) return;

    // Complete this operation
    opCompleted(pSh2, SH2_OK);
}

const sh2_Op_t startCalOp = {
    .start = startCalStart,
    .rx = startCalRx,
};

// ------------------------------------------------------------------------
// Start Cal

static int finishCalStart(sh2_t *pSh2)
{
    return sendCmd1(pSh2, SH2_CMD_CAL, SH2_CAL_FINISH);
}

static void finishCalRx(sh2_t *pSh2, const uint8_t *payload, uint16_t len)
{
    CommandResp_t *resp = (CommandResp_t *)payload;
    
    // Ignore message if it doesn't pertain to this operation
    if (wrongResponse(pSh2, resp)) return;

    pSh2->opData.finishCal.status = (sh2_CalStatus_t)resp->r[1];

    // Complete this operation
    if (pSh2->opData.finishCal.status == SH2_CAL_SUCCESS) {
        opCompleted(pSh2, SH2_OK);
    }
    else {
        opCompleted(pSh2, SH2_ERR_HUB);
    }
}

const sh2_Op_t finishCalOp = {
    .start = finishCalStart,
    .rx = finishCalRx,
};


// ------------------------------------------------------------------------
// SHTP Event Callback

static void shtpEventCallback(void *cookie, shtp_Event_t shtpEvent) {
    sh2_t *pSh2 = &_sh2;

    sh2AsyncEvent.eventId = SH2_SHTP_EVENT;
    sh2AsyncEvent.shtpEvent = shtpEvent;
    if (pSh2->eventCallback) {
        pSh2->eventCallback(pSh2->eventCookie, &sh2AsyncEvent);
    }
}

// ------------------------------------------------------------------------
// Public functions

/**
 * @brief Open a session with a sensor hub.
 *
 * This function should be calLED before others in this API.
 * An instance of an SH2 HAL should be passed in.
 * This call will result in the open() function of the HAL being calLED.
 *
 * As part of the initialization process, a callback function is registered that will
 * be invoked when the device generates certain events.  (See sh2_AsyncEventId)
 *
 * @param pHal Pointer to an SH2 HAL instance, provided by the target system.
 * @param  eventCallback Will be calLED when events, such as reset complete, occur.
 * @param  eventCookie Will be passed to eventCallback.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_open(sh2_Hal_t *pHal,
             sh2_EventCallback_t *eventCallback, void *eventCookie)
{
    sh2_t *pSh2 = &_sh2;
    
    // Validate parameters
    if (pHal == 0) return SH2_ERR_BAD_PARAM;

    // Clear everything in sh2 structure.
    memset(&_sh2, 0, sizeof(_sh2));
        
    pSh2->resetComplete = false;  // will go true after reset response from SH.
    pSh2->controlChan = 0xFF;  // An invalid value since we don't know yet.
    
    // Store reference to HAL for future use.
    pSh2->pHal = pHal;
    pSh2->eventCallback = eventCallback;
    pSh2->eventCookie = eventCookie;
    pSh2->sensorCallback = 0;
    pSh2->sensorCookie = 0;

    // Open SHTP layer
    pSh2->pShtp = shtp_open(pSh2->pHal);
    if (pSh2->pShtp == 0) {
        // Error opening SHTP
        return SH2_ERR;
    }

    // Register SHTP event callback
    shtp_setEventCallback(pSh2->pShtp, shtpEventCallback, &_sh2);

    // Register with SHTP
    // Register SH2 handlers
    shtp_listenAdvert(pSh2->pShtp, GUID_SENSORHUB, sensorhubAdvertHdlr, &_sh2);
    shtp_listenChan(pSh2->pShtp, GUID_SENSORHUB, "control", sensorhubControlHdlr, &_sh2);
    shtp_listenChan(pSh2->pShtp, GUID_SENSORHUB, "inputNormal", sensorhubInputNormalHdlr, &_sh2);
    shtp_listenChan(pSh2->pShtp, GUID_SENSORHUB, "inputWake", sensorhubInputWakeHdlr, &_sh2);
    shtp_listenChan(pSh2->pShtp, GUID_SENSORHUB, "inputGyroRv", sensorhubInputGyroRvHdlr, &_sh2);

    // Register EXECUTABLE handlers
    shtp_listenAdvert(pSh2->pShtp, GUID_EXECUTABLE, executableAdvertHdlr, &_sh2);
    shtp_listenChan(pSh2->pShtp, GUID_EXECUTABLE, "device", executabLEDeviceHdlr, &_sh2);

    // Wait for reset notifications to arrive.
    // The client can't talk to the sensor hub until that happens.
    uint32_t start_us = pSh2->pHal->getTimeUs(pSh2->pHal);
    uint32_t now_us = start_us;
    while (((now_us - start_us) < ADVERT_TIMEOUT_US) &&
           (!pSh2->resetComplete))
    {
        shtp_service(pSh2->pShtp);
        now_us = pSh2->pHal->getTimeUs(pSh2->pHal);
    }
    
    // No errors.
    return SH2_OK;
}

/**
 * @brief Close a session with a sensor hub.
 *
 * This should be calLED at the end of a sensor hub session.  
 * The underlying SHTP and HAL instances will be closed.
 */
void sh2_close(void)
{
    sh2_t *pSh2 = &_sh2;
    
    shtp_close(pSh2->pShtp);

    // Clear everything in sh2 structure.
    memset(pSh2, 0, sizeof(sh2_t));
}

/**
 * @brief Service the SH2 device, reading any data that is available and dispatching callbacks.
 *
 * This function should be calLED periodically by the host system to service an open sensor hub.
 */
void sh2_service(void)
{
    sh2_t *pSh2 = &_sh2;
    
    shtp_service(pSh2->pShtp);
}

/**
 * @brief Register a function to receive sensor events.
 *
 * @param  callback A function that will be calLED each time a sensor event is received.
 * @param  cookie  A value that will be passed to the sensor callback function.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_setSensorCallback(sh2_SensorCallback_t *callback, void *cookie)
{
    sh2_t *pSh2 = &_sh2;
    
    pSh2->sensorCallback = callback;
    pSh2->sensorCookie = cookie;

    return SH2_OK;
}

/**
 * @brief Reset the sensor hub device by sending RESET (1) command on "device" channel.
 *
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_devReset(void)
{
    sh2_t *pSh2 = &_sh2;

    return sendExecutable(pSh2, EXECUTABLE_DEVICE_CMD_RESET);
}

/**
 * @brief Turn sensor hub on by sending RESET (1) command on "device" channel.
 *
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_devOn(void)
{
    sh2_t *pSh2 = &_sh2;

    return sendExecutable(pSh2, EXECUTABLE_DEVICE_CMD_ON);
}

/**
 * @brief Put sensor hub in sleep state by sending SLEEP (2) command on "device" channel.
 *
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_devSleep(void)
{
    sh2_t *pSh2 = &_sh2;

    return sendExecutable(pSh2, EXECUTABLE_DEVICE_CMD_SLEEP);
}

/**
 * @brief Get Product ID information from Sensorhub.
 *
 * @param  prodIds Pointer to structure that will receive results.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_getProdIds(sh2_ProductIds_t *prodIds)
{
    sh2_t *pSh2 = &_sh2;
    
    // clear opData
    memset(&pSh2->opData, 0, sizeof(sh2_OpData_t));
    
    pSh2->opData.getProdIds.pProdIds = prodIds;

    return opProcess(pSh2, &getProdIdOp);
}

/**
 * @brief Get sensor configuration.
 *
 * @param  sensorId Which sensor to query.
 * @param  config SensorConfig structure to store results.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_getSensorConfig(sh2_SensorId_t sensorId, sh2_SensorConfig_t *pConfig)
{
    sh2_t *pSh2 = &_sh2;
    
    // clear opData
    memset(&pSh2->opData, 0, sizeof(sh2_OpData_t));
    
    // Set up operation
    pSh2->opData.getSensorConfig.sensorId = sensorId;
    pSh2->opData.getSensorConfig.pConfig = pConfig;

    return opProcess(pSh2, &getSensorConfigOp);
}

/**
 * @brief Set sensor configuration. (e.g enable a sensor at a particular rate.)
 *
 * @param  sensorId Which sensor to configure.
 * @param  pConfig Pointer to structure holding sensor configuration.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_setSensorConfig(sh2_SensorId_t sensorId, const sh2_SensorConfig_t *pConfig)
{
    sh2_t *pSh2 = &_sh2;
    
    // clear opData
    memset(&pSh2->opData, 0, sizeof(sh2_OpData_t));
    
    // Set up operation
    pSh2->opData.setSensorConfig.sensorId = sensorId;
    pSh2->opData.setSensorConfig.pConfig = pConfig;

    return opProcess(pSh2, &setSensorConfigOp);
}

/**
 * @brief Get metadata related to a sensor.
 *
 * @param  sensorId Which sensor to query.
 * @param  pData Pointer to structure to receive the results.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_getMetadata(sh2_SensorId_t sensorId, sh2_SensorMetadata_t *pData)
{
    sh2_t *pSh2 = &_sh2;
    
    // pData must be non-null
    if (pData == 0) return SH2_ERR_BAD_PARAM;
  
    // Convert sensorId to metadata recordId
    int i;
    for (i = 0; i < ARRAY_LEN(sensorToRecordMap); i++) {
        if (sensorToRecordMap[i].sensorId == sensorId) {
            break;
        }
    }
    if (i >= ARRAY_LEN(sensorToRecordMap)) {
        // no match was found
        return SH2_ERR_BAD_PARAM;
    }
    uint16_t recordId = sensorToRecordMap[i].recordId;
    
    // clear opData
    memset(&pSh2->opData, 0, sizeof(sh2_OpData_t));
    
    // Set up an FRS read operation
    pSh2->opData.getFrs.frsType = recordId;
    pSh2->opData.getFrs.pData = pSh2->frsData;
    pSh2->frsDataLen = ARRAY_LEN(pSh2->frsData);
    pSh2->opData.getFrs.pWords = &pSh2->frsDataLen;

    // Read an FRS record
    int status = opProcess(pSh2, &getFrsOp);
    
    // Copy the results into pData
    if (status == SH2_OK) {
        stuffMetadata(pData, pSh2->frsData);
    }

    return status;
}

/**
 * @brief Get an FRS record.
 *
 * @param  recordId Which FRS Record to retrieve.
 * @param  pData pointer to buffer to receive the results
 * @param[in] words Size of pData buffer, in 32-bit words.
 * @param[out] words Number of 32-bit words retrieved.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_getFrs(uint16_t recordId, uint32_t *pData, uint16_t *words)
{
    sh2_t *pSh2 = &_sh2;
    
    if ((pData == 0) || (words == 0)) {
        return SH2_ERR_BAD_PARAM;
    }
    
    // clear opData
    memset(&pSh2->opData, 0, sizeof(sh2_OpData_t));
    
    // Store params for this op
    pSh2->opData.getFrs.frsType = recordId;
    pSh2->opData.getFrs.pData = pData;
    pSh2->opData.getFrs.pWords = words;

    return opProcess(pSh2, &getFrsOp);
}

/**
 * @brief Set an FRS record
 *
 * @param  recordId Which FRS Record to set.
 * @param  pData pointer to buffer containing the new data.
 * @param  words number of 32-bit words to write.  (0 to delete record.)
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_setFrs(uint16_t recordId, uint32_t *pData, uint16_t words)
{
    sh2_t *pSh2 = &_sh2;
    
    if ((pData == 0) && (words != 0)) {
        return SH2_ERR_BAD_PARAM;
    }
    
    // clear opData
    memset(&pSh2->opData, 0, sizeof(sh2_OpData_t));
    
    pSh2->opData.setFrs.frsType = recordId;
    pSh2->opData.setFrs.pData = pData;
    pSh2->opData.setFrs.words = words;

    return opProcess(pSh2, &setFrsOp);
}

/**
 * @brief Get error counts.
 *
 * @param  severity Only errors of this severity or greater are returned.
 * @param  pErrors Buffer to receive error codes.
 * @param  numErrors size of pErrors array
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_getErrors(uint8_t severity, sh2_ErrorRecord_t *pErrors, uint16_t *numErrors)
{
    sh2_t *pSh2 = &_sh2;
    
    // clear opData
    memset(&pSh2->opData, 0, sizeof(sh2_OpData_t));
    
    pSh2->opData.getErrors.severity = severity;
    pSh2->opData.getErrors.pErrors = pErrors;
    pSh2->opData.getErrors.pNumErrors = numErrors;
    
    return opProcess(pSh2, &getErrorsOp);
}

/**
 * @brief Read counters related to a sensor.
 *
 * @param  sensorId Which sensor to operate on.
 * @param  pCounts Pointer to Counts structure that will receive data.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_getCounts(sh2_SensorId_t sensorId, sh2_Counts_t *pCounts)
{
    sh2_t *pSh2 = &_sh2;
    
    // clear opData
    memset(&pSh2->opData, 0, sizeof(sh2_OpData_t));
    
    pSh2->opData.getCounts.sensorId = sensorId;
    pSh2->opData.getCounts.pCounts = pCounts;
    
    return opProcess(pSh2, &getCountsOp);
}

/**
 * @brief Clear counters related to a sensor.
 *
 * @param  sensorId which sensor to operate on.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_clearCounts(sh2_SensorId_t sensorId)
{
    sh2_t *pSh2 = &_sh2;

    // clear opData
    memset(&pSh2->opData, 0, sizeof(sh2_OpData_t));
    
    
    pSh2->opData.sendCmd.req.command = SH2_CMD_COUNTS;
    pSh2->opData.sendCmd.req.p[0] = SH2_COUNTS_CLEAR_COUNTS;
    pSh2->opData.sendCmd.req.p[1] = sensorId;

    return opProcess(pSh2, &sendCmdOp);
}

/**
 * @brief Perform a tare operation on one or more axes.
 *
 * @param  axes Bit mask specifying which axes should be tared.
 * @param  basis Which rotation vector to use as the basis for Tare adjustment.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_setTareNow(uint8_t axes,    // SH2_TARE_X | SH2_TARE_Y | SH2_TARE_Z
                   sh2_TareBasis_t basis)
{
    sh2_t *pSh2 = &_sh2;

    // clear opData
    memset(&pSh2->opData, 0, sizeof(sh2_OpData_t));
    
    
    pSh2->opData.sendCmd.req.command = SH2_CMD_TARE;
    pSh2->opData.sendCmd.req.p[0] = SH2_TARE_TARE_NOW;
    pSh2->opData.sendCmd.req.p[1] = axes;
    pSh2->opData.sendCmd.req.p[2] = basis;

    return opProcess(pSh2, &sendCmdOp);
}

/**
 * @brief Clears the previously applied tare operation.
 *
 * @return SH2_OK \n");
 */
int sh2_clearTare(void)
{
    sh2_t *pSh2 = &_sh2;

    // clear opData
    memset(&pSh2->opData, 0, sizeof(sh2_OpData_t));
    
    
    pSh2->opData.sendCmd.req.command = SH2_CMD_TARE;
    pSh2->opData.sendCmd.req.p[0] = SH2_TARE_SET_REORIENTATION;

    return opProcess(pSh2, &sendCmdOp);
}

/**
 * @brief Persist the results of last tare operation to flash.
 *
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_persistTare(void)
{
    sh2_t *pSh2 = &_sh2;

    // clear opData
    memset(&pSh2->opData, 0, sizeof(sh2_OpData_t));
    
    
    pSh2->opData.sendCmd.req.command = SH2_CMD_TARE;
    pSh2->opData.sendCmd.req.p[0] = SH2_TARE_PERSIST_TARE;

    return opProcess(pSh2, &sendCmdOp);
}

/**
 * @brief Set the current run-time sensor reorientation. (Set to zero to clear tare.)
 *
 * @param  orientation Quaternion rotation vector to apply as new tare.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_setReorientation(sh2_Quaternion_t *orientation)
{
    sh2_t *pSh2 = &_sh2;

    // clear opData
    memset(&pSh2->opData, 0, sizeof(sh2_OpData_t));
    
    
    pSh2->opData.sendCmd.req.command = SH2_CMD_TARE;
    pSh2->opData.sendCmd.req.p[0] = SH2_TARE_SET_REORIENTATION;

    // save me a lot of typing and you a lot of reading
    uint8_t *p = pSh2->opData.sendCmd.req.p;

    // Put new orientation in command parameters
    writeu16(&p[1], toQ14(orientation->x));
    writeu16(&p[3], toQ14(orientation->y));
    writeu16(&p[5], toQ14(orientation->z));
    writeu16(&p[7], toQ14(orientation->w));

    return opProcess(pSh2, &sendCmdOp);
}

/**
 * @brief Command the sensorhub to reset.
 *
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_reinitialize(void)
{
    sh2_t *pSh2 = &_sh2;

    return opProcess(pSh2, &reinitOp);
}

/**
 * @brief Save Dynamic Calibration Data to flash.
 *
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_saveDcdNow(void)
{
    sh2_t *pSh2 = &_sh2;

    return opProcess(pSh2, &saveDcdNowOp);
}

/**
 * @brief Get Oscillator type.
 *
 * @param  pOscType pointer to data structure to receive results.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_getOscType(sh2_OscType_t *pOscType)
{
    sh2_t *pSh2 = &_sh2;

    pSh2->opData.getOscType.pOscType = pOscType;

    return opProcess(pSh2, &getOscTypeOp);
}

/**
 * @brief Enable/Disable dynamic calibration for certain sensors
 *
 * @param  sensors Bit mask to configure which sensors are affected.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_setCalConfig(uint8_t sensors)
{
    sh2_t *pSh2 = &_sh2;

    pSh2->opData.calConfig.sensors = sensors;

    return opProcess(pSh2, &setCalConfigOp);
}

/**
 * @brief Get dynamic calibration configuration settings.
 *
 * @param  pSensors pointer to Bit mask, set on return.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_getCalConfig(uint8_t *pSensors)
{
    sh2_t *pSh2 = &_sh2;

    pSh2->opData.getCalConfig.pSensors = pSensors;

    return opProcess(pSh2, &getCalConfigOp);
}

/**
 * @brief Configure automatic saving of dynamic calibration data.
 *
 * @param  enabLED Enable or Disable DCD auto-save.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_setDcdAutoSave(bool enabLED)
{
    sh2_t *pSh2 = &_sh2;

    // clear opData
    memset(&pSh2->opData, 0, sizeof(sh2_OpData_t));
    
    pSh2->opData.sendCmd.req.command = SH2_CMD_DCD_SAVE;
    pSh2->opData.sendCmd.req.p[0] = enabLED ? 0 : 1;

    return opProcess(pSh2, &sendCmdOp);
}

/**
 * @brief Immediately issue all buffered sensor reports from a given sensor.
 *
 * @param  sensorId Which sensor reports to flush.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_flush(sh2_SensorId_t sensorId)
{
    sh2_t *pSh2 = &_sh2;

    // clear opData
    memset(&pSh2->opData, 0, sizeof(sh2_OpData_t));
    
    pSh2->opData.forceFlush.sensorId = sensorId;

    return opProcess(pSh2, &forceFlushOp);
}

/**
 * @brief Command clear DCD in RAM, then reset sensor hub.
 *
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_clearDcdAndReset(void)
{
    sh2_t *pSh2 = &_sh2;

    // clear opData
    memset(&pSh2->opData, 0, sizeof(sh2_OpData_t));
    
    pSh2->opData.sendCmd.req.command = SH2_CMD_CLEAR_DCD_AND_RESET;

    return opProcess(pSh2, &sendCmdOp);
}

/**
 * @brief Start simple self-calibration procedure.
 *
 * @parameter interval_us sensor report interval, uS.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_startCal(uint32_t interval_us)
{
    sh2_t *pSh2 = &_sh2;

    // clear opData
    memset(&pSh2->opData, 0, sizeof(sh2_OpData_t));
    
    pSh2->opData.startCal.interval_us = interval_us;

    return opProcess(pSh2, &startCalOp);
}

/**
 * @brief Finish simple self-calibration procedure.
 *
 * @parameter status contains calibration status code on return.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_finishCal(sh2_CalStatus_t *status)
{
    sh2_t *pSh2 = &_sh2;

    // clear opData
    memset(&pSh2->opData, 0, sizeof(sh2_OpData_t));
    
    return opProcess(pSh2, &finishCalOp);
}

/**
 * @brief send Interactive ZRO Request.
 *
 * @parameter intent Inform the sensor hub what sort of motion should be in progress.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_setIZro(sh2_IZroMotionIntent_t intent)
{
    sh2_t *pSh2 = &_sh2;

    // clear opData
    memset(&pSh2->opData, 0, sizeof(sh2_OpData_t));

    // set up opData for iZRO request
    pSh2->opData.sendCmd.req.command = SH2_CMD_INTERACTIVE_ZRO;
    pSh2->opData.sendCmd.req.p[0] = intent;

    // Send command
    return opProcess(pSh2, &sendCmdOp);
}



File: src/sh2.h

/*
 * Copyright 2015-2018 Hillcrest Laboratories, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License and 
 * any applicable agreements you may have with Hillcrest Laboratories, Inc.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file sh2.h
 * @author David Wheeler
 * @date 22 Sept 2015
 * @brief API Definition for Hillcrest SH-2 Sensor Hub.
 *
 * The sh2 API provides an interface to the Hillcrest Labs sensor hub devices.
 */

#ifndef SH2_H
#define SH2_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#include "sh2_hal.h"

/***************************************************************************************
 * Public type definitions
 ***************************************************************************************/

/**
 * @brief Sensor Event
 *
 * See the SH-2 Reference Manual for more detail.
 */
#define SH2_MAX_SENSOR_EVENT_LEN (16)
typedef struct sh2_SensorEvent {
    uint64_t timestamp_uS;
    uint8_t len;
    uint8_t reportId;
    uint8_t report[SH2_MAX_SENSOR_EVENT_LEN];
} sh2_SensorEvent_t;

typedef void (sh2_SensorCallback_t)(void * cookie, sh2_SensorEvent_t *pEvent);

/**
 * @brief Product Id value
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef struct sh2_ProductId_s {
    uint8_t resetCause;
    uint8_t swVersionMajor;
    uint8_t swVersionMinor;
    uint32_t swPartNumber;
    uint32_t swBuildNumber;
    uint16_t swVersionPatch;
    uint8_t reserved0;
    uint8_t reserved1;
} sh2_ProductId_t;

#define SH2_MAX_PROD_ID_ENTRIES (5)
typedef struct sh2_ProductIds_s {
    sh2_ProductId_t entry[SH2_MAX_PROD_ID_ENTRIES];
    uint8_t numEntries;
} sh2_ProductIds_t;

/**
 * @brief List of sensor types supported by the hub
 *
 * See the SH-2 Reference Manual for more information on each type.
 */
enum sh2_SensorId_e {
    SH2_RAW_ACCELEROMETER = 0x14,
    SH2_ACCELEROMETER = 0x01,
    SH2_LINEAR_ACCELERATION = 0x04,
    SH2_GRAVITY = 0x06,
    SH2_RAW_GYROSCOPE = 0x15,
    SH2_GYROSCOPE_CALIBRATED = 0x02,
    SH2_GYROSCOPE_UNCALIBRATED = 0x07,
    SH2_RAW_MAGNETOMETER = 0x16,
    SH2_MAGNETIC_FIELD_CALIBRATED = 0x03,
    SH2_MAGNETIC_FIELD_UNCALIBRATED = 0x0f,
    SH2_ROTATION_VECTOR = 0x05,
    SH2_GAME_ROTATION_VECTOR = 0x08,
    SH2_GEOMAGNETIC_ROTATION_VECTOR = 0x09,
    SH2_PRESSURE = 0x0a,
    SH2_AMBIENT_LIGHT = 0x0b,
    SH2_HUMIDITY = 0x0c,
    SH2_PROXIMITY = 0x0d,
    SH2_TEMPERATURE = 0x0e,
    SH2_RESERVED = 0x17,
    SH2_TAP_DETECTOR = 0x10,
    SH2_STEP_DETECTOR = 0x18,
    SH2_STEP_COUNTER = 0x11,
    SH2_SIGNIFICANT_MOTION = 0x12,
    SH2_STABILITY_CLASSIFIER = 0x13,
    SH2_SHAKE_DETECTOR = 0x19,
    SH2_FLIP_DETECTOR = 0x1a,
    SH2_PICKUP_DETECTOR = 0x1b,
    SH2_STABILITY_DETECTOR = 0x1c,
    SH2_PERSONAL_ACTIVITY_CLASSIFIER = 0x1e,
    SH2_SLEEP_DETECTOR = 0x1f,
    SH2_TILT_DETECTOR = 0x20,
    SH2_POCKET_DETECTOR = 0x21,
    SH2_CIRCLE_DETECTOR = 0x22,
    SH2_HEART_RATE_MONITOR = 0x23,
    SH2_ARVR_STABILIZED_RV = 0x28,
    SH2_ARVR_STABILIZED_GRV = 0x29,
    SH2_GYRO_INTEGRATED_RV = 0x2A,
    SH2_IZRO_MOTION_REQUEST = 0x2B,

    // UPDATE to reflect greatest sensor id
    SH2_MAX_SENSOR_ID = 0x2B,
};
typedef uint8_t sh2_SensorId_t;

/**
 * @brief Sensor Configuration settings
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef struct sh2_SensorConfig {
    /* Change sensitivity enabLED */
    bool changeSensitivityEnabled;  /**< @brief Enable reports on change */

    /* Change sensitivity - true if relative; false if absolute */
    bool changeSensitivityRelative;  /**< @brief Change reports relative (vs absolute) */

    /* Wake-up enabLED */
    bool wakeupEnabled;  /**< @brief Wake host on event */

    /* Always on enabLED */
    bool alwaysOnEnabled;  /**< @brief Sensor remains on in sleep state */
    /* 16-bit signed fixed point integer representing the value a
     * sensor output must exceed in order to trigger another input
     * report. A setting of 0 causes all reports to be sent.
     */
    uint16_t changeSensitivity;  /**< @brief Report-on-change threshold */

    /* Interval in microseconds between asynchronous input reports. */
    uint32_t reportInterval_us;  /**< @brief [uS] Report interval */

    /* Reserved field, not used. */
    uint32_t batchInterval_us;  /**< @brief [uS] Batch interval */

    /* Meaning is sensor specific */
    uint32_t sensorSpecific;  /**< @brief See SH-2 Reference Manual for details. */
} sh2_SensorConfig_t;

/**
 * @brief Sensor Metadata Record
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef struct sh2_SensorMetadata {
    uint8_t meVersion;   /**< @brief Motion Engine Version */
    uint8_t mhVersion;  /**< @brief Motion Hub Version */
    uint8_t shVersion;  /**< @brief SensorHub Version */
    uint32_t range;  /**< @brief Same units as sensor reports */
    uint32_t resolution;  /**< @brief Same units as sensor reports */
    uint16_t revision;  /**< @brief Metadata record format revision */
    uint16_t power_mA;    /**< @brief [mA] Fixed point 16Q10 format */
    uint32_t minPeriod_uS;  /**< @brief [uS] */
    uint32_t maxPeriod_uS;  /**< @brief [uS] */
    uint32_t fifoReserved;  /**< @brief (Unused) */
    uint32_t fifoMax;  /**< @brief (Unused) */
    uint32_t batchBufferBytes;  /**< @brief (Unused) */
    uint16_t qPoint1;     /**< @brief q point for sensor values */
    uint16_t qPoint2;     /**< @brief q point for accuracy or bias fields */
    uint16_t qPoint3;     /**< @brief q point for sensor data change sensitivity */
    uint32_t vendorIdLen; /**< @brief [bytes] */
    char vendorId[48];  /**< @brief Vendor name and part number */
    uint32_t sensorSpecificLen;  /**< @brief [bytes] */
    uint8_t sensorSpecific[48];  /**< @brief See SH-2 Reference Manual */
} sh2_SensorMetadata_t;

/**
 * @brief SensorHub Error Record
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef struct sh2_ErrorRecord {
    uint8_t severity;   /**< @brief Error severity, 0: most severe. */
    uint8_t sequence;   /**< @brief Sequence number (by severity) */
    uint8_t source;     /**< @brief 1-MotionEngine, 2-MotionHub, 3-SensorHub, 4-Chip  */
    uint8_t error;      /**< @brief See SH-2 Reference Manual */
    uint8_t module;     /**< @brief See SH-2 Reference Manual */
    uint8_t code;       /**< @brief See SH-2 Reference Manual */
} sh2_ErrorRecord_t;

/**
 * @brief SensorHub Counter Record
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef struct sh2_Counts {
    uint32_t offered;   /**< @brief [events] */
    uint32_t accepted;  /**< @brief [events] */
    uint32_t on;        /**< @brief [events] */
    uint32_t attempted; /**< @brief [events] */
} sh2_Counts_t;

/**
 * @brief Values for specifying tare basis
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef enum sh2_TareBasis {
    SH2_TARE_BASIS_ROTATION_VECTOR = 0,             /**< @brief Use Rotation Vector */
    SH2_TARE_BASIS_GAMING_ROTATION_VECTOR = 1,      /**< @brief Use Game Rotation Vector */
    SH2_TARE_BASIS_GEOMAGNETIC_ROTATION_VECTOR = 2, /**< @brief Use Geomagnetic R.V. */
} sh2_TareBasis_t;

/**
 * @brief Bit Fields for specifying tare axes.
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef enum sh2_TareAxis {
    SH2_TARE_X = 1,  /**< @brief sh2_tareNow() axes bit field */
    SH2_TARE_Y = 2,  /**< @brief sh2_tareNow() axes bit field */
    SH2_TARE_Z = 4,  /**< @brief sh2_tareNow() axes bit field */
} sh2_TareAxis_t;

/**
 * @brief Quaternion (double precision floating point representation.)
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef struct sh2_Quaternion {
    double x;
    double y;
    double z;
    double w;
} sh2_Quaternion_t;

/**
 * @brief Oscillator type: Internal or External
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef enum {
    SH2_OSC_INTERNAL    = 0,
    SH2_OSC_EXT_CRYSTAL = 1,
    SH2_OSC_EXT_CLOCK   = 2,
} sh2_OscType_t;

/**
 * @brief Calibration result
 *
 * See the SH-2 Reference Manual, Finish Calibration Response.
 */
typedef enum {
    SH2_CAL_SUCCESS = 0,
    SH2_CAL_NO_ZRO,
    SH2_CAL_NO_STATIONARY_DETECTION,
    SH2_CAL_ROTATION_OUTSIDE_SPEC,
    SH2_CAL_ZRO_OUTSIDE_SPEC,
    SH2_CAL_ZGO_OUTSIDE_SPEC,
    SH2_CAL_GYRO_GAIN_OUTSIDE_SPEC,
    SH2_CAL_GYRO_PERIOD_OUTSIDE_SPEC,
    SH2_CAL_GYRO_DROPS_OUTSIDE_SPEC,
} sh2_CalStatus_t;

// FRS Record Ids
#define STATIC_CALIBRATION_AGM                   (0x7979)
#define NOMINAL_CALIBRATION                      (0x4D4D)
#define STATIC_CALIBRATION_SRA                   (0x8A8A)
#define NOMINAL_CALIBRATION_SRA                  (0x4E4E)
#define DYNAMIC_CALIBRATION                      (0x1F1F)
#define ME_POWER_MGMT                            (0xD3E2)
#define SYSTEM_ORIENTATION                       (0x2D3E)
#define ACCEL_ORIENTATION                        (0x2D41)
#define SCREEN_ACCEL_ORIENTATION                 (0x2D43)
#define GYROSCOPE_ORIENTATION                    (0x2D46)
#define MAGNETOMETER_ORIENTATION                 (0x2D4C)
#define ARVR_STABILIZATION_RV                    (0x3E2D)
#define ARVR_STABILIZATION_GRV                   (0x3E2E)
#define TAP_DETECT_CONFIG                        (0xC269)
#define SIG_MOTION_DETECT_CONFIG                 (0xC274)
#define SHAKE_DETECT_CONFIG                      (0x7D7D)
#define MAX_FUSION_PERIOD                        (0xD7D7)
#define SERIAL_NUMBER                            (0x4B4B)
#define ES_PRESSURE_CAL                          (0x39AF)
#define ES_TEMPERATURE_CAL                       (0x4D20)
#define ES_HUMIDITY_CAL                          (0x1AC9)
#define ES_AMBIENT_LIGHT_CAL                     (0x39B1)
#define ES_PROXIMITY_CAL                         (0x4DA2)
#define ALS_CAL                                  (0xD401)
#define PROXIMITY_SENSOR_CAL                     (0xD402)
#define PICKUP_DETECTOR_CONFIG                   (0x1B2A)
#define FLIP_DETECTOR_CONFIG                     (0xFC94)
#define STABILITY_DETECTOR_CONFIG                (0xED85)
#define ACTIVITY_TRACKER_CONFIG                  (0xED88)
#define SLEEP_DETECTOR_CONFIG                    (0xED87)
#define TILT_DETECTOR_CONFIG                     (0xED89)
#define POCKET_DETECTOR_CONFIG                   (0xEF27)
#define CIRCLE_DETECTOR_CONFIG                   (0xEE51)
#define USER_RECORD                              (0x74B4)
#define ME_TIME_SOURCE_SELECT                    (0xD403)
#define UART_FORMAT                              (0xA1A1)
#define GYRO_INTEGRATED_RV_CONFIG                (0xA1A2)
#define FRS_ID_META_RAW_ACCELEROMETER            (0xE301)
#define FRS_ID_META_ACCELEROMETER                (0xE302)
#define FRS_ID_META_LINEAR_ACCELERATION          (0xE303)
#define FRS_ID_META_GRAVITY                      (0xE304)
#define FRS_ID_META_RAW_GYROSCOPE                (0xE305)
#define FRS_ID_META_GYROSCOPE_CALIBRATED         (0xE306)
#define FRS_ID_META_GYROSCOPE_UNCALIBRATED       (0xE307)
#define FRS_ID_META_RAW_MAGNETOMETER             (0xE308)
#define FRS_ID_META_MAGNETIC_FIELD_CALIBRATED    (0xE309)
#define FRS_ID_META_MAGNETIC_FIELD_UNCALIBRATED  (0xE30A)
#define FRS_ID_META_ROTATION_VECTOR              (0xE30B)
#define FRS_ID_META_GAME_ROTATION_VECTOR         (0xE30C)
#define FRS_ID_META_GEOMAGNETIC_ROTATION_VECTOR  (0xE30D)
#define FRS_ID_META_PRESSURE                     (0xE30E)
#define FRS_ID_META_AMBIENT_LIGHT                (0xE30F)
#define FRS_ID_META_HUMIDITY                     (0xE310)
#define FRS_ID_META_PROXIMITY                    (0xE311)
#define FRS_ID_META_TEMPERATURE                  (0xE312)
#define FRS_ID_META_TAP_DETECTOR                 (0xE313)
#define FRS_ID_META_STEP_DETECTOR                (0xE314)
#define FRS_ID_META_STEP_COUNTER                 (0xE315)
#define FRS_ID_META_SIGNIFICANT_MOTION           (0xE316)
#define FRS_ID_META_STABILITY_CLASSIFIER         (0xE317)
#define FRS_ID_META_SHAKE_DETECTOR               (0xE318)
#define FRS_ID_META_FLIP_DETECTOR                (0xE319)
#define FRS_ID_META_PICKUP_DETECTOR              (0xE31A)
#define FRS_ID_META_STABILITY_DETECTOR           (0xE31B)
#define FRS_ID_META_PERSONAL_ACTIVITY_CLASSIFIER (0xE31C)
#define FRS_ID_META_SLEEP_DETECTOR               (0xE31D)
#define FRS_ID_META_TILT_DETECTOR                (0xE31E)
#define FRS_ID_META_POCKET_DETECTOR              (0xE31F)
#define FRS_ID_META_CIRCLE_DETECTOR              (0xE320)
#define FRS_ID_META_HEART_RATE_MONITOR           (0xE321)
#define FRS_ID_META_ARVR_STABILIZED_RV           (0xE322)
#define FRS_ID_META_ARVR_STABILIZED_GRV          (0xE323)
#define FRS_ID_META_GYRO_INTEGRATED_RV           (0xE324)

/**
 * @brief Interactive ZRO Motion Intent
 *
 * See the SH-2 Reference Manual, 6.4.13
 */
typedef enum {
    SH2_IZRO_MI_UNKNOWN = 0,
    SH2_IZRO_MI_STATIONARY_NO_VIBRATION,
    SH2_IZRO_MI_STATIONARY_WITH_VIBRATION,
    SH2_IZRO_MI_IN_MOTION,
} sh2_IZroMotionIntent_t;

/**
 * @brief Interactive ZRO Motion Intent
 *
 * See the SH-2 Reference Manual, 6.4.13
 */
typedef enum {
    SH2_IZRO_MR_NO_REQUEST = 0,
    SH2_IZRO_MR_STAY_STATIONARY,
    SH2_IZRO_MR_STATIONARY_NON_URGENT,
    SH2_IZRO_MR_STATIONARY_URGENT,
} sh2_IZroMotionRequest_t;


/**
* @brief Asynchronous Event
*
* Represents reset events and other non-sensor events received from SH-2 sensor hub.
*/

enum sh2_AsyncEventId_e {
    SH2_RESET,
    SH2_SHTP_EVENT,
    SH2_GET_FEATURE_RESP,
};
typedef enum sh2_AsyncEventId_e sh2_AsyncEventId_t;

enum sh2_ShtpEvent_e {
    SH2_SHTP_TX_DISCARD = 0,
    SH2_SHTP_SHORT_FRAGMENT = 1,
    SH2_SHTP_TOO_LARGE_PAYLOADS = 2,
    SH2_SHTP_BAD_RX_CHAN = 3,
    SH2_SHTP_BAD_TX_CHAN = 4,
};
typedef uint8_t sh2_ShtpEvent_t;

typedef struct sh2_SensorConfigResp_e {
    sh2_SensorId_t sensorId;
    sh2_SensorConfig_t sensorConfig;
} sh2_SensorConfigResp_t;

typedef struct sh2_AsyncEvent {
    uint32_t eventId;
    union {
        sh2_ShtpEvent_t shtpEvent;
        sh2_SensorConfigResp_t sh2SensorConfigResp;
    };
} sh2_AsyncEvent_t;

typedef void (sh2_EventCallback_t)(void * cookie, sh2_AsyncEvent_t *pEvent);


/***************************************************************************************
 * Public API
 **************************************************************************************/

/**
 * @brief Open a session with a sensor hub.
 *
 * This function should be calLED before others in this API.
 * An instance of an SH2 HAL should be passed in.
 * This call will result in the open() function of the HAL being calLED.
 *
 * As part of the initialization process, a callback function is registered that will
 * be invoked when the device generates certain events.  (See sh2_AsyncEventId)
 *
 * @param pHal Pointer to an SH2 HAL instance, provided by the target system.
 * @param  eventCallback Will be calLED when events, such as reset complete, occur.
 * @param  eventCookie Will be passed to eventCallback.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_open(sh2_Hal_t *pHal,
             sh2_EventCallback_t *eventCallback, void *eventCookie);

/**
 * @brief Close a session with a sensor hub.
 *
 * This should be calLED at the end of a sensor hub session.  
 * The underlying SHTP and HAL instances will be closed.
 *
 */
void sh2_close(void);

/**
 * @brief Service the SH2 device, reading any data that is available and dispatching callbacks.
 *
 * This function should be calLED periodically by the host system to service an open sensor hub.
 *
 */
void sh2_service(void);

/**
 * @brief Register a function to receive sensor events.
 *
 * @param  callback A function that will be calLED each time a sensor event is received.
 * @param  cookie  A value that will be passed to the sensor callback function.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_setSensorCallback(sh2_SensorCallback_t *callback, void *cookie);

/**
 * @brief Reset the sensor hub device by sending RESET (1) command on "device" channel.
 *
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_devReset(void);

/**
 * @brief Turn sensor hub on by sending ON (2) command on "device" channel.
 *
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_devOn(void);

/**
 * @brief Put sensor hub in sleep state by sending SLEEP (3) command on "device" channel.
 *
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_devSleep(void);

/**
 * @brief Get Product ID information from Sensorhub.
 *
 * @param  prodIds Pointer to structure that will receive results.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_getProdIds(sh2_ProductIds_t *prodIds);

/**
 * @brief Get sensor configuration.
 *
 * @param  sensorId Which sensor to query.
 * @param  config SensorConfig structure to store results.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_getSensorConfig(sh2_SensorId_t sensorId, sh2_SensorConfig_t *config);

/**
 * @brief Set sensor configuration. (e.g enable a sensor at a particular rate.)
 *
 * @param  sensorId Which sensor to configure.
 * @param  pConfig Pointer to structure holding sensor configuration.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_setSensorConfig(sh2_SensorId_t sensorId, const sh2_SensorConfig_t *pConfig);

/**
 * @brief Get metadata related to a sensor.
 *
 * @param  sensorId Which sensor to query.
 * @param  pData Pointer to structure to receive the results.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_getMetadata(sh2_SensorId_t sensorId, sh2_SensorMetadata_t *pData);

/**
 * @brief Get an FRS record.
 *
 * @param  recordId Which FRS Record to retrieve.
 * @param  pData pointer to buffer to receive the results
 * @param[in] words Size of pData buffer, in 32-bit words.
 * @param[out] words Number of 32-bit words retrieved.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_getFrs(uint16_t recordId, uint32_t *pData, uint16_t *words);

/**
 * @brief Set an FRS record
 *
 * @param  recordId Which FRS Record to set.
 * @param  pData pointer to buffer containing the new data.
 * @param  words number of 32-bit words to write.  (0 to delete record.)
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_setFrs(uint16_t recordId, uint32_t *pData, uint16_t words);

/**
 * @brief Get error counts.
 *
 * @param  severity Only errors of this severity or greater are returned.
 * @param  pErrors Buffer to receive error codes.
 * @param  numErrors size of pErrors array
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_getErrors(uint8_t severity, sh2_ErrorRecord_t *pErrors, uint16_t *numErrors);

/**
 * @brief Read counters related to a sensor.
 *
 * @param  sensorId Which sensor to operate on.
 * @param  pCounts Pointer to Counts structure that will receive data.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_getCounts(sh2_SensorId_t sensorId, sh2_Counts_t *pCounts);

/**
 * @brief Clear counters related to a sensor.
 *
 * @param  sensorId which sensor to operate on.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_clearCounts(sh2_SensorId_t sensorId);

/**
 * @brief Perform a tare operation on one or more axes.
 *
 * @param  axes Bit mask specifying which axes should be tared.
 * @param  basis Which rotation vector to use as the basis for Tare adjustment.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_setTareNow(uint8_t axes,    // SH2_TARE_X | SH2_TARE_Y | SH2_TARE_Z
                   sh2_TareBasis_t basis);

/**
 * @brief Clears the previously applied tare operation.
 *
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_clearTare(void);

/**
 * @brief Persist the results of last tare operation to flash.
 *
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_persistTare(void);

/**
 * @brief Set the current run-time sensor reorientation. (Set to zero to clear tare.)
 *
 * @param  orientation Quaternion rotation vector to apply as new tare.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_setReorientation(sh2_Quaternion_t *orientation);

/**
 * @brief Command the sensorhub to reset.
 *
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_reinitialize(void);

/**
 * @brief Save Dynamic Calibration Data to flash.
 *
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_saveDcdNow(void);

/**
 * @brief Get Oscillator type.
 *
 * @param  pOscType pointer to data structure to receive results.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_getOscType(sh2_OscType_t *pOscType);

// Flags for sensors field of sh_calConfig
#define SH2_CAL_ACCEL (0x01)
#define SH2_CAL_GYRO  (0x02)
#define SH2_CAL_MAG   (0x04)
#define SH2_CAL_PLANAR (0x08)

/**
 * @brief Enable/Disable dynamic calibration for certain sensors
 *
 * @param  sensors Bit mask to configure which sensors are affected.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_setCalConfig(uint8_t sensors);

/**
 * @brief Get dynamic calibration configuration settings.
 *
 * @param  pSensors pointer to Bit mask, set on return.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_getCalConfig(uint8_t *pSensors);

/**
 * @brief Configure automatic saving of dynamic calibration data.
 *
 * @param  enabLED Enable or Disable DCD auto-save.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_setDcdAutoSave(bool enabLED);

/**
 * @brief Immediately issue all buffered sensor reports from a given sensor.
 *
 * @param  sensorId Which sensor reports to flush.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_flush(sh2_SensorId_t sensorId);

/**
 * @brief Command clear DCD in RAM, then reset sensor hub.
 *
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_clearDcdAndReset(void);

/**
 * @brief Start simple self-calibration procedure.
 *
 * @parameter interval_us sensor report interval, uS.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_startCal(uint32_t interval_us);

/**
 * @brief Finish simple self-calibration procedure.
 *
 * @parameter status contains calibration status code on return.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_finishCal(sh2_CalStatus_t *status);

/**
 * @brief send Interactive ZRO Request.
 *
 * @parameter intent Inform the sensor hub what sort of motion should be in progress.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_setIZro(sh2_IZroMotionIntent_t intent);

#ifdef __cplusplus
} // extern "C"
#endif

#endif



File: src/sh2_SensorValue.c

/*
 * Copyright 2015-16 Hillcrest Laboratories, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License and 
 * any applicable agreements you may have with Hillcrest Laboratories, Inc.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * BNO08x Sensor Event decoding
 */

#include "sh2_SensorValue.h"
#include "sh2_err.h"
#include "sh2_util.h"

#define SCALE_Q(n) (1.0f / (1 << n))

const float scaleRadToDeg = 180.0 / 3.14159265358;

// ------------------------------------------------------------------------
// Forward declarations

static int decodeRawAccelerometer(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event);
static int decodeAccelerometer(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event);
static int decodeLinearAcceleration(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event);
static int decodeGravity(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event);
static int decodeRawGyroscope(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event);
static int decodeGyroscopeCalibrated(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event);
static int decodeGyroscopeUncal(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event);
static int decodeRawMagnetometer(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event);
static int decodeMagneticFieldCalibrated(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event);
static int decodeMagneticFieldUncal(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event);
static int decodeRotationVector(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event);
static int decodeGameRotationVector(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event);
static int decodeGeomagneticRotationVector(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event);
static int decodePressure(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event);
static int decodeAmbientLight(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event);
static int decodeHumidity(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event);
static int decodeProximity(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event);
static int decodeTemperature(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event);
static int decodeReserved(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event);
static int decodeTapDetector(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event);
static int decodeStepDetector(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event);
static int decodeStepCounter(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event);
static int decodeSignificantMotion(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event);
static int decodeStabilityClassifier(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event);
static int decodeShakeDetector(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event);
static int decodeFlipDetector(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event);
static int decodePickupDetector(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event);
static int decodeStabilityDetector(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event);
static int decodePersonalActivityClassifier(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event);
static int decodeSleepDetector(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event);
static int decodeTiltDetector(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event);
static int decodePocketDetector(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event);
static int decodeCircLEDetector(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event);
static int decodeHeartRateMonitor(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event);
static int decodeArvrStabilizedRV(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event);
static int decodeArvrStabilizedGRV(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event);
static int decodeGyroIntegratedRV(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event);
static int decodeIZroRequest(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event);

// ------------------------------------------------------------------------
// Public API

int sh2_decodeSensorEvent(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event)
{
    // Fill out fields of *value based on *event, converting data from message representation
    // to natural representation.

    int rc = SH2_OK;

    value->sensorId = event->reportId;
    value->timestamp = event->timestamp_uS;

    if (value->sensorId != SH2_GYRO_INTEGRATED_RV) {
        value->sequence = event->report[1];
        value->status = event->report[2] & 0x03;
    }
    else {
        value->sequence = 0;
        value->status = 0;
    }

    // extract delay field (100uS units)
    
    
    switch (value->sensorId) {
        case SH2_RAW_ACCELEROMETER:
            rc = decodeRawAccelerometer(value, event);
            break;
        case SH2_ACCELEROMETER:
            rc = decodeAccelerometer(value, event);
            break;
        case SH2_LINEAR_ACCELERATION:
            rc = decodeLinearAcceleration(value, event);
            break;
        case SH2_GRAVITY:
            rc = decodeGravity(value, event);
            break;
        case SH2_RAW_GYROSCOPE:
            rc = decodeRawGyroscope(value, event);
            break;
        case SH2_GYROSCOPE_CALIBRATED:
            rc = decodeGyroscopeCalibrated(value, event);
            break;
        case SH2_GYROSCOPE_UNCALIBRATED:
            rc = decodeGyroscopeUncal(value, event);
            break;
        case SH2_RAW_MAGNETOMETER:
            rc = decodeRawMagnetometer(value, event);
            break;
        case SH2_MAGNETIC_FIELD_CALIBRATED:
            rc = decodeMagneticFieldCalibrated(value, event);
            break;
        case SH2_MAGNETIC_FIELD_UNCALIBRATED:
            rc = decodeMagneticFieldUncal(value, event);
            break;
        case SH2_ROTATION_VECTOR:
            rc = decodeRotationVector(value, event);
            break;
        case SH2_GAME_ROTATION_VECTOR:
            rc = decodeGameRotationVector(value, event);
            break;
        case SH2_GEOMAGNETIC_ROTATION_VECTOR:
            rc = decodeGeomagneticRotationVector(value, event);
            break;
        case SH2_PRESSURE:
            rc = decodePressure(value, event);
            break;
        case SH2_AMBIENT_LIGHT:
            rc = decodeAmbientLight(value, event);
            break;
        case SH2_HUMIDITY:
            rc = decodeHumidity(value, event);
            break;
        case SH2_PROXIMITY:
            rc = decodeProximity(value, event);
            break;
        case SH2_TEMPERATURE:
            rc = decodeTemperature(value, event);
            break;
        case SH2_RESERVED:
            rc = decodeReserved(value, event);
            break;
        case SH2_TAP_DETECTOR:
            rc = decodeTapDetector(value, event);
            break;
        case SH2_STEP_DETECTOR:
            rc = decodeStepDetector(value, event);
            break;
        case SH2_STEP_COUNTER:
            rc = decodeStepCounter(value, event);
            break;
        case SH2_SIGNIFICANT_MOTION:
            rc = decodeSignificantMotion(value, event);
            break;
        case SH2_STABILITY_CLASSIFIER:
            rc = decodeStabilityClassifier(value, event);
            break;
        case SH2_SHAKE_DETECTOR:
            rc = decodeShakeDetector(value, event);
            break;
        case SH2_FLIP_DETECTOR:
            rc = decodeFlipDetector(value, event);
            break;
        case SH2_PICKUP_DETECTOR:
            rc = decodePickupDetector(value, event);
            break;
        case SH2_STABILITY_DETECTOR:
            rc = decodeStabilityDetector(value, event);
            break;
        case SH2_PERSONAL_ACTIVITY_CLASSIFIER:
            rc = decodePersonalActivityClassifier(value, event);
            break;
        case SH2_SLEEP_DETECTOR:
            rc = decodeSleepDetector(value, event);
            break;
        case SH2_TILT_DETECTOR:
            rc = decodeTiltDetector(value, event);
            break;
        case SH2_POCKET_DETECTOR:
            rc = decodePocketDetector(value, event);
            break;
        case SH2_CIRCLE_DETECTOR:
            rc = decodeCircLEDetector(value, event);
            break;
        case SH2_HEART_RATE_MONITOR:
            rc = decodeHeartRateMonitor(value, event);
            break;
        case SH2_ARVR_STABILIZED_RV:
            rc = decodeArvrStabilizedRV(value, event);
            break;
        case SH2_ARVR_STABILIZED_GRV:
            rc = decodeArvrStabilizedGRV(value, event);
            break;
        case SH2_GYRO_INTEGRATED_RV:
            rc = decodeGyroIntegratedRV(value, event);
            break;
        case SH2_IZRO_MOTION_REQUEST:
            rc = decodeIZroRequest(value, event);
            break;
        default:
            // Unknown report id
            rc = SH2_ERR;
            break;
    }

    return rc;
}

// ------------------------------------------------------------------------
// Private utility functions

static int decodeRawAccelerometer(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event)
{
    value->un.rawAccelerometer.x = read16(&event->report[4]);
    value->un.rawAccelerometer.y = read16(&event->report[6]);
    value->un.rawAccelerometer.z = read16(&event->report[8]);
    value->un.rawAccelerometer.timestamp = read32(&event->report[12]);

    return SH2_OK;
}

static int decodeAccelerometer(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event)
{
    value->un.accelerometer.x = read16(&event->report[4]) * SCALE_Q(8);
    value->un.accelerometer.y = read16(&event->report[6]) * SCALE_Q(8);
    value->un.accelerometer.z = read16(&event->report[8]) * SCALE_Q(8);

    return SH2_OK;
}

static int decodeLinearAcceleration(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event)
{
    value->un.linearAcceleration.x = read16(&event->report[4]) * SCALE_Q(8);
    value->un.linearAcceleration.y = read16(&event->report[6]) * SCALE_Q(8);
    value->un.linearAcceleration.z = read16(&event->report[8]) * SCALE_Q(8);

    return SH2_OK;
}

static int decodeGravity(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event)
{
    value->un.gravity.x = read16(&event->report[4]) * SCALE_Q(8);
    value->un.gravity.y = read16(&event->report[6]) * SCALE_Q(8);
    value->un.gravity.z = read16(&event->report[8]) * SCALE_Q(8);

    return SH2_OK;
}

static int decodeRawGyroscope(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event)
{
    value->un.rawGyroscope.x = read16(&event->report[4]);
    value->un.rawGyroscope.y = read16(&event->report[6]);
    value->un.rawGyroscope.z = read16(&event->report[8]);
    value->un.rawGyroscope.temperature = read16(&event->report[10]);
    value->un.rawGyroscope.timestamp = read32(&event->report[12]);

    return SH2_OK;
}

static int decodeGyroscopeCalibrated(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event)
{
    value->un.gyroscope.x = read16(&event->report[4]) * SCALE_Q(9);
    value->un.gyroscope.y = read16(&event->report[6]) * SCALE_Q(9);
    value->un.gyroscope.z = read16(&event->report[8]) * SCALE_Q(9);

    return SH2_OK;
}

static int decodeGyroscopeUncal(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event)
{
    value->un.gyroscopeUncal.x = read16(&event->report[4]) * SCALE_Q(9);
    value->un.gyroscopeUncal.y = read16(&event->report[6]) * SCALE_Q(9);
    value->un.gyroscopeUncal.z = read16(&event->report[8]) * SCALE_Q(9);

    value->un.gyroscopeUncal.biasX = read16(&event->report[10]) * SCALE_Q(9);
    value->un.gyroscopeUncal.biasY = read16(&event->report[12]) * SCALE_Q(9);
    value->un.gyroscopeUncal.biasZ = read16(&event->report[14]) * SCALE_Q(9);

    return SH2_OK;
}

static int decodeRawMagnetometer(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event)
{
    value->un.rawMagnetometer.x = read16(&event->report[4]);
    value->un.rawMagnetometer.y = read16(&event->report[6]);
    value->un.rawMagnetometer.z = read16(&event->report[8]);
    value->un.rawMagnetometer.timestamp = read32(&event->report[12]);

    return SH2_OK;
}

static int decodeMagneticFieldCalibrated(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event)
{
    value->un.magneticField.x = read16(&event->report[4]) * SCALE_Q(4);
    value->un.magneticField.y = read16(&event->report[6]) * SCALE_Q(4);
    value->un.magneticField.z = read16(&event->report[8]) * SCALE_Q(4);

    return SH2_OK;
}

static int decodeMagneticFieldUncal(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event)
{
    value->un.magneticFieldUncal.x = read16(&event->report[4]) * SCALE_Q(4);
    value->un.magneticFieldUncal.y = read16(&event->report[6]) * SCALE_Q(4);
    value->un.magneticFieldUncal.z = read16(&event->report[8]) * SCALE_Q(4);

    value->un.magneticFieldUncal.biasX = read16(&event->report[10]) * SCALE_Q(4);
    value->un.magneticFieldUncal.biasY = read16(&event->report[12]) * SCALE_Q(4);
    value->un.magneticFieldUncal.biasZ = read16(&event->report[14]) * SCALE_Q(4);

    return SH2_OK;
}

static int decodeRotationVector(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event)
{
    value->un.rotationVector.i = read16(&event->report[4]) * SCALE_Q(14);
    value->un.rotationVector.j = read16(&event->report[6]) * SCALE_Q(14);
    value->un.rotationVector.k = read16(&event->report[8]) * SCALE_Q(14);
    value->un.rotationVector.real = read16(&event->report[10]) * SCALE_Q(14);
    value->un.rotationVector.accuracy = read16(&event->report[12]) * SCALE_Q(12);

    return SH2_OK;
}

static int decodeGameRotationVector(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event)
{
    value->un.gameRotationVector.i = read16(&event->report[4]) * SCALE_Q(14);
    value->un.gameRotationVector.j = read16(&event->report[6]) * SCALE_Q(14);
    value->un.gameRotationVector.k = read16(&event->report[8]) * SCALE_Q(14);
    value->un.gameRotationVector.real = read16(&event->report[10]) * SCALE_Q(14);

    return SH2_OK;
}

static int decodeGeomagneticRotationVector(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event)
{
    value->un.geoMagRotationVector.i = read16(&event->report[4]) * SCALE_Q(14);
    value->un.geoMagRotationVector.j = read16(&event->report[6]) * SCALE_Q(14);
    value->un.geoMagRotationVector.k = read16(&event->report[8]) * SCALE_Q(14);
    value->un.geoMagRotationVector.real = read16(&event->report[10]) * SCALE_Q(14);
    value->un.geoMagRotationVector.accuracy = read16(&event->report[12]) * SCALE_Q(12);

    return SH2_OK;
}

static int decodePressure(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event)
{
    value->un.pressure.value = read32(&event->report[4]) * SCALE_Q(20);

    return SH2_OK;
}

static int decodeAmbientLight(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event)
{
    value->un.ambientLight.value = read32(&event->report[4]) * SCALE_Q(8);

    return SH2_OK;
}

static int decodeHumidity(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event)
{
    value->un.humidity.value = read16(&event->report[4]) * SCALE_Q(8);

    return SH2_OK;
}

static int decodeProximity(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event)
{
    value->un.proximity.value = read16(&event->report[4]) * SCALE_Q(4);

    return SH2_OK;
}

static int decodeTemperature(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event)
{
    value->un.temperature.value = read16(&event->report[4]) * SCALE_Q(7);

    return SH2_OK;
}

static int decodeReserved(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event)
{
    value->un.reserved.tbd = read16(&event->report[4]) * SCALE_Q(7);

    return SH2_OK;
}

static int decodeTapDetector(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event)
{
    value->un.tapDetector.flags = event->report[4];

    return SH2_OK;
}

static int decodeStepDetector(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event)
{
    value->un.stepDetector.latency = readu32(&event->report[4]);

    return SH2_OK;
}

static int decodeStepCounter(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event)
{
    value->un.stepCounter.latency = readu32(&event->report[4]);
    value->un.stepCounter.steps = readu32(&event->report[8]);

    return SH2_OK;
}

static int decodeSignificantMotion(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event)
{
    value->un.sigMotion.motion = readu16(&event->report[4]);

    return SH2_OK;
}

static int decodeStabilityClassifier(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event)
{
    value->un.stabilityClassifier.classification = event->report[4];

    return SH2_OK;
}

static int decodeShakeDetector(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event)
{
    value->un.shakeDetector.shake = readu16(&event->report[4]);

    return SH2_OK;
}

static int decodeFlipDetector(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event)
{
    value->un.flipDetector.flip = readu16(&event->report[4]);

    return SH2_OK;
}

static int decodePickupDetector(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event)
{
    value->un.pickupDetector.pickup = readu16(&event->report[4]);

    return SH2_OK;
}

static int decodeStabilityDetector(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event)
{
    value->un.stabilityDetector.stability = readu16(&event->report[4]);

    return SH2_OK;
}

static int decodePersonalActivityClassifier(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event)
{
    value->un.personalActivityClassifier.page = event->report[4] & 0x7F;
    value->un.personalActivityClassifier.lastPage = ((event->report[4] & 0x80) != 0);
    value->un.personalActivityClassifier.mostLikelyState = event->report[5];
    for (int n = 0; n < 10; n++) {
        value->un.personalActivityClassifier.confidence[n] = event->report[6+n];
    }
    
    return SH2_OK;
}

static int decodeSleepDetector(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event)
{
    value->un.sleepDetector.sleepState = event->report[4];

    return SH2_OK;
}

static int decodeTiltDetector(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event)
{
    value->un.tiltDetector.tilt = readu16(&event->report[4]);

    return SH2_OK;
}

static int decodePocketDetector(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event)
{
    value->un.pocketDetector.pocket = readu16(&event->report[4]);

    return SH2_OK;
}

static int decodeCircLEDetector(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event)
{
    value->un.circLEDetector.circle = readu16(&event->report[4]);

    return SH2_OK;
}

static int decodeHeartRateMonitor(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event)
{
    value->un.heartRateMonitor.heartRate = readu16(&event->report[4]);

    return SH2_OK;
}

static int decodeArvrStabilizedRV(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event)
{
    value->un.arvrStabilizedRV.i = read16(&event->report[4]) * SCALE_Q(14);
    value->un.arvrStabilizedRV.j = read16(&event->report[6]) * SCALE_Q(14);
    value->un.arvrStabilizedRV.k = read16(&event->report[8]) * SCALE_Q(14);
    value->un.arvrStabilizedRV.real = read16(&event->report[10]) * SCALE_Q(14);
    value->un.arvrStabilizedRV.accuracy = read16(&event->report[12]) * SCALE_Q(12);

    return SH2_OK;
}

static int decodeArvrStabilizedGRV(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event)
{
    value->un.arvrStabilizedGRV.i = read16(&event->report[4]) * SCALE_Q(14);
    value->un.arvrStabilizedGRV.j = read16(&event->report[6]) * SCALE_Q(14);
    value->un.arvrStabilizedGRV.k = read16(&event->report[8]) * SCALE_Q(14);
    value->un.arvrStabilizedGRV.real = read16(&event->report[10]) * SCALE_Q(14);

    return SH2_OK;
}

static int decodeGyroIntegratedRV(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event)
{
    value->un.gyroIntegratedRV.i = read16(&event->report[0]) * SCALE_Q(14);
    value->un.gyroIntegratedRV.j = read16(&event->report[2]) * SCALE_Q(14);
    value->un.gyroIntegratedRV.k = read16(&event->report[4]) * SCALE_Q(14);
    value->un.gyroIntegratedRV.real = read16(&event->report[6]) * SCALE_Q(14);
    value->un.gyroIntegratedRV.angVelX = read16(&event->report[8]) * SCALE_Q(10);
    value->un.gyroIntegratedRV.angVelY = read16(&event->report[10]) * SCALE_Q(10);
    value->un.gyroIntegratedRV.angVelZ = read16(&event->report[12]) * SCALE_Q(10);

    return SH2_OK;
}

static int decodeIZroRequest(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event)
{
    value->un.izroRequest.intent = (sh2_IZroMotionIntent_t)event->report[4];
    value->un.izroRequest.request = (sh2_IZroMotionRequest_t)event->report[5];

    return SH2_OK;
}



File: src/sh2_SensorValue.h

/*
 * Copyright 2015-16 Hillcrest Laboratories, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License and 
 * any applicable agreements you may have with Hillcrest Laboratories, Inc.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/** 
 * @file sh2_SensorValue.h 
 * @author David Wheeler
 * @date 10 Nov 2015
 * @brief Support for converting sensor events (messages) into natural data structures.
 *
 */

#ifndef SH2_SENSORVALUE_H
#define SH2_SENSORVALUE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

#include "sh2.h"

/* Note on quaternion naming conventions:
 * Quaternions are values with four real components that are usually
 * interpreted as coefficients in the complex quantity, Q.
 *
 * As in, Q = W + Xi + Yj + Zk
 *
 * Where i, j and k represent the three imaginary dimensions.
 *
 * So W represents the Real components and X, Y and Z the Imaginary ones.
 *
 * In the Hillcrest datasheets and in this code, however, the four components
 * are named real, i, j and k, to make it explicit which is which.  If you 
 * need to translate these names into the "wxyz" or "xyzw" convention, then, the
 * appropriate mapping is this:
 *     w = real
 *     x = i
 *     y = j
 *     z = k
 */
	
/**
 * @brief Raw Accelerometer
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef struct sh2_RawAccelerometer {
    /* Units are ADC counts */
    int16_t x;  /**< @brief [ADC counts] */
    int16_t y;  /**< @brief [ADC counts] */
    int16_t z;  /**< @brief [ADC counts] */

    /* Microseconds */
    uint32_t timestamp;  /**< @brief [uS] */
} sh2_RawAccelerometer_t;

/**
 * @brief Accelerometer
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef struct sh2_Accelerometer {
    float x;
    float y;
    float z;
} sh2_Accelerometer_t;

/**
 * @brief Raw gyroscope
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef struct sh2_RawGyroscope {
    /* Units are ADC counts */
    int16_t x;  /**< @brief [ADC Counts] */
    int16_t y;  /**< @brief [ADC Counts] */
    int16_t z;  /**< @brief [ADC Counts] */
    int16_t temperature;  /**< @brief [ADC Counts] */

    /* Microseconds */
    uint32_t timestamp;  /**< @brief [uS] */
} sh2_RawGyroscope_t;

/**
 * @brief Gyroscope
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef struct sh2_Gyroscope {
    /* Units are rad/s */
    float x;
    float y;
    float z;
} sh2_Gyroscope_t;

/**
 * @brief Uncalibrated gyroscope
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef struct sh2_GyroscopeUncalibrated {
    /* Units are rad/s */
    float x;  /**< @brief [rad/s] */
    float y;  /**< @brief [rad/s] */
    float z;  /**< @brief [rad/s] */
    float biasX;  /**< @brief [rad/s] */
    float biasY;  /**< @brief [rad/s] */
    float biasZ;  /**< @brief [rad/s] */
} sh2_GyroscopeUncalibrated_t;

/**
 * @brief Raw Magnetometer
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef struct sh2_RawMagnetometer {
    /* Units are ADC counts */
    int16_t x;  /**< @brief [ADC Counts] */
    int16_t y;  /**< @brief [ADC Counts] */
    int16_t z;  /**< @brief [ADC Counts] */

    /* Microseconds */
    uint32_t timestamp;  /**< @brief [uS] */
} sh2_RawMagnetometer_t;

/**
 * @brief Magnetic field
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef struct sh2_MagneticField {
    /* Units are uTesla */
    float x;  /**< @brief [uTesla] */
    float y;  /**< @brief [uTesla] */
    float z;  /**< @brief [uTesla] */
} sh2_MagneticField_t;

/**
 * @brief Uncalibrated magnetic field
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef struct sh2_MagneticFieldUncalibrated {
    /* Units are uTesla */
    float x;  /**< @brief [uTesla] */
    float y;  /**< @brief [uTesla] */
    float z;  /**< @brief [uTesla] */
    float biasX;  /**< @brief [uTesla] */
    float biasY;  /**< @brief [uTesla] */
    float biasZ;  /**< @brief [uTesla] */
} sh2_MagneticFieldUncalibrated_t;

/**
 * @brief Rotation Vector with Accuracy
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef struct sh2_RotationVectorWAcc {
    float i;  /**< @brief Quaternion component i */
    float j;  /**< @brief Quaternion component j */
    float k;  /**< @brief Quaternion component k */
    float real;  /**< @brief Quaternion component, real */
    float accuracy;  /**< @brief Accuracy estimate [radians] */
} sh2_RotationVectorWAcc_t;

/**
 * @brief Rotation Vector
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef struct sh2_RotationVector {
    float i;  /**< @brief Quaternion component i */
    float j;  /**< @brief Quaternion component j */
    float k;  /**< @brief Quaternion component k */
    float real;  /**< @brief Quaternion component real */
} sh2_RotationVector_t;

/**
 * @brief Atmospheric Pressure
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef struct sh2_Pressure {
    float value;  /**< @brief Atmospheric Pressure.  [hectopascals] */
} sh2_Pressure_t;

/**
 * @brief Ambient Light
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef struct sh2_AmbientLight {
    float value;  /**< @brief Ambient Light.  [lux] */
} sh2_AmbientLight_t;

/**
 * @brief Humidity
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef struct sh2_Humidity {
    float value;  /**< @brief Relative Humidity.  [percent] */
} sh2_Humidity_t;

/**
 * @brief Proximity
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef struct sh2_Proximity {
    float value;  /**< @brief Proximity.  [cm] */
} sh2_Proximity_t;

/**
 * @brief Temperature
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef struct sh2_Temperature {
    float value;  /**< @brief Temperature.  [C] */
} sh2_Temperature_t;

/**
 * @brief Reserved
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef struct sh2_Reserved {
    float tbd;  /**< @brief Reserved */
} sh2_Reserved_t;

/**
 * @brief TapDetector
 *
 * See the SH-2 Reference Manual for more detail.
 */
#define TAPDET_X      (1)
#define TAPDET_X_POS  (2)
#define TAPDET_Y      (4)
#define TAPDET_Y_POS  (8)
#define TAPDET_Z      (16)
#define TAPDET_Z_POS  (32)
#define TAPDET_DOUBLE (64)
typedef struct sh2_TapDetector {
    uint8_t flags;  /**< @brief TapDetector.  */
} sh2_TapDetector_t;

/**
 * @brief StepDetector
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef struct sh2_StepDetector {
    uint32_t latency;  /**< @brief Step detect latency [uS].  */
} sh2_StepDetector_t;

/**
 * @brief StepCounter
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef struct sh2_StepCounter {
    uint32_t latency;  /**< @brief Step counter latency [uS].  */
    uint16_t steps;    /**< @brief Steps counted. */
} sh2_StepCounter_t;

/**
 * @brief SigMotion
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef struct sh2_SigMotion {
    uint16_t motion;
} sh2_SigMotion_t;

/**
 * @brief StabilityClassifier
 *
 * See the SH-2 Reference Manual for more detail.
 */
#define STABILITY_CLASSIFIER_UNKNOWN (0)
#define STABILITY_CLASSIFIER_ON_TABLE (1)
#define STABILITY_CLASSIFIER_STATIONARY (2)
#define STABILITY_CLASSIFIER_STABLE (3)
#define STABILITY_CLASSIFIER_MOTION (4)
#define STABILITY_CLASSIFIER_RESERVED (5)
typedef struct sh2_StabilityClassifier {
    uint8_t classification;
} sh2_StabilityClassifier_t;

/**
 * @brief ShakeDetector
 *
 * See the SH-2 Reference Manual for more detail.
 */
#define SHAKE_X (1)
#define SHAKE_Y (2)
#define SHAKE_Z (4)
typedef struct sh2_ShakeDetector {
    uint16_t shake;
} sh2_ShakeDetector_t;

/**
 * @brief flipDetector
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef struct sh2_FlipDetector {
    uint16_t flip;
} sh2_FlipDetector_t;

/**
 * @brief pickupDetector
 *
 * See the SH-2 Reference Manual for more detail.
 */
#define PICKUP_LEVEL_TO_NOT_LEVEL (1)
#define PICKUP_STOP_WITHIN_REGION (2)
typedef struct sh2_PickupDetector {
    uint16_t pickup;   /**< flag field with bits defined above. */
} sh2_PickupDetector_t;

/**
 * @brief stabilityDetector
 *
 * See the SH-2 Reference Manual for more detail.
 */
#define STABILITY_ENTERED (1)
#define STABILITY_EXITED  (2)
typedef struct sh2_StabilityDetector {
    uint16_t stability;  /**< flag field with bits defined above. */
} sh2_StabilityDetector_t;

/**
 * @brief Personal Activity Classifier
 *
 * See the SH-2 Reference Manual for more detail.
 */
#define PAC_UNKNOWN (0)
#define PAC_IN_VEHICLE (1)
#define PAC_ON_BICYCLE (2)
#define PAC_ON_FOOT (3)
#define PAC_STILL (4)
#define PAC_TILTING (5)
#define PAC_WALKING (6)
#define PAC_RUNNING (7)
typedef struct sh2_PersonalActivityClassifier {
    uint8_t page;
    bool lastPage;
    uint8_t mostLikelyState;
    uint8_t confidence[10];
} sh2_PersonalActivityClassifier_t;

/**
 * @brief sleepDetector
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef struct sh2_SleepDetector {
    uint8_t sleepState;
} sh2_SleepDetector_t;

/**
 * @brief tiltDetector
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef struct sh2_TiltDetector {
    uint16_t tilt;
} sh2_TiltDetector_t;

/**
 * @brief pocketDetector
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef struct sh2_PocketDetector {
    uint16_t pocket;
} sh2_PocketDetector_t;

/**
 * @brief circLEDetector
 *
 * See the SH-2 Reference Manual for more detail.
 */
typedef struct sh2_CircLEDetector {
    uint16_t circle;
} sh2_CircLEDetector_t;

/**
 * @brief heartRateMonitor
 *
 * See SH-2 Reference Manual for details.
 */
typedef struct sh2_HeartRateMonitor {
    uint16_t heartRate; /**< heart rate in beats per minute. */
} sh2_HeartRateMonitor_t;

/**
 * @brief Gyro Integrated Rotation Vector
 *
 * See SH-2 Reference Manual for details.
 */
typedef struct sh2_GyroIntegratedRV {
    float i;        /**< @brief Quaternion component i */
    float j;        /**< @brief Quaternion component j */
    float k;        /**< @brief Quaternion component k */
    float real;     /**< @brief Quaternion component real */
    float angVelX;  /**< @brief Angular velocity about x [rad/s] */
    float angVelY;  /**< @brief Angular velocity about y [rad/s] */
    float angVelZ;  /**< @brief Angular velocity about z [rad/s] */
} sh2_GyroIntegratedRV_t;

typedef struct sh2_IZroRequest {
    sh2_IZroMotionIntent_t intent;
    sh2_IZroMotionRequest_t request;
} sh2_IZroRequest_t;

typedef struct sh2_SensorValue {
    
    /** Which sensor produced this event. */
    uint8_t sensorId;

    /** @brief 8-bit unsigned integer used to track reports.
     *
     * The sequence number increments once for each report sent.  Gaps
     * in the sequence numbers indicate missing or dropped reports.
     */
    uint8_t sequence;

    /* Status of a sensor
     *   0 - Unreliable
     *   1 - Accuracy low
     *   2 - Accuracy medium
     *   3 - Accuracy high
     */
    uint8_t status; /**< @brief bits 7-5: reserved, 4-2: exponent delay, 1-0: Accuracy */

    uint64_t timestamp;  /**< [uS] */

    uint32_t delay; /**< @brief [uS] value is delay * 2^exponent (see status) */

    /** @brief Sensor Data
     *
     * Use the structure based on the value of the sensor
     * field.
     */
    union {
        sh2_RawAccelerometer_t rawAccelerometer;
        sh2_Accelerometer_t accelerometer; 
        sh2_Accelerometer_t linearAcceleration; 
        sh2_Accelerometer_t gravity; 
        sh2_RawGyroscope_t rawGyroscope; 
        sh2_Gyroscope_t gyroscope; 
        sh2_GyroscopeUncalibrated_t gyroscopeUncal; 
        sh2_RawMagnetometer_t rawMagnetometer; 
        sh2_MagneticField_t magneticField; 
        sh2_MagneticFieldUncalibrated_t magneticFieldUncal; 
        sh2_RotationVectorWAcc_t rotationVector; 
        sh2_RotationVector_t gameRotationVector; 
        sh2_RotationVectorWAcc_t geoMagRotationVector;
        sh2_Pressure_t pressure;
        sh2_AmbientLight_t ambientLight;
        sh2_Humidity_t humidity;
        sh2_Proximity_t proximity;
        sh2_Temperature_t temperature;
        sh2_Reserved_t reserved;
        sh2_TapDetector_t tapDetector;
        sh2_StepDetector_t stepDetector;
        sh2_StepCounter_t stepCounter;
        sh2_SigMotion_t sigMotion;
        sh2_StabilityClassifier_t stabilityClassifier;
        sh2_ShakeDetector_t shakeDetector;
        sh2_FlipDetector_t flipDetector;
        sh2_PickupDetector_t pickupDetector;
        sh2_StabilityDetector_t stabilityDetector;
        sh2_PersonalActivityClassifier_t personalActivityClassifier;
        sh2_SleepDetector_t sleepDetector;
        sh2_TiltDetector_t tiltDetector;
        sh2_PocketDetector_t pocketDetector;
        sh2_CircLEDetector_t circLEDetector;
        sh2_HeartRateMonitor_t heartRateMonitor;
        sh2_RotationVectorWAcc_t arvrStabilizedRV;
        sh2_RotationVector_t arvrStabilizedGRV;
        sh2_GyroIntegratedRV_t gyroIntegratedRV;
        sh2_IZroRequest_t izroRequest;
    } un;
} sh2_SensorValue_t;

int sh2_decodeSensorEvent(sh2_SensorValue_t *value, const sh2_SensorEvent_t *event);


#ifdef __cplusplus
} // extern "C"
#endif

#endif



File: src/sh2_err.h

/*
 * Copyright 2015-16 Hillcrest Laboratories, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License and 
 * any applicable agreements you may have with Hillcrest Laboratories, Inc.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/** 
 * @file sh2_err.h
 * @author David Wheeler
 * @date 22 May 2015
 * @brief Type definitions for Hillcrest SH-2 API.
 *
 * Struct and type definitions supporting the Hillcrest SH-2 SensorHub API.
 * 
 */


#ifndef SH2_ERR_H
#define SH2_ERR_H


#define SH2_OK                 (0)  /**< Success */
#define SH2_ERR                (-1) /**< General Error */
#define SH2_ERR_BAD_PARAM      (-2) /**< Bad parameter to an API call */
#define SH2_ERR_OP_IN_PROGRESS (-3) /**< Operation in progress */
#define SH2_ERR_IO             (-4) /**< Error communicating with hub */
#define SH2_ERR_HUB            (-5) /**< Error reported by hub */
#define SH2_ERR_TIMEOUT        (-6) /**< Operation timed out */


#endif



File: src/sh2_hal.h

/*
 * Copyright 2018 Hillcrest Laboratories, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License and 
 * any applicable agreements you may have with Hillcrest Laboratories, Inc.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * SH2 HAL Interface for Non-RTOS Applications.
 */

// Include guard
#ifndef SH2_HAL_H
#define SH2_HAL_H

#include <stdint.h>

// SH2 Implementations generally have a max out transfer len of 256
#define SH2_HAL_MAX_TRANSFER_OUT (256)
#define SH2_HAL_MAX_PAYLOAD_OUT  (256)

// Although some implementations adversize a max in transfer of 32K,
// in practice, the largest transfer performed is the advertisements
// which is 272 bytes at time of writing.
#define SH2_HAL_MAX_TRANSFER_IN  (384)
#define SH2_HAL_MAX_PAYLOAD_IN   (384)

// This needs to be a power of 2, greater than max of the above.
#define SH2_HAL_DMA_SIZE (512)

typedef struct sh2_Hal_s sh2_Hal_t;

// The SH2 interface uses these functions to access the underlying
// communications device, so the system integrator will need to
// implement these.  At system intialization time, an sh2_Hal_t
// structure should be initialized with pointers to all the hardware
// access layer functions.  A pointer to this structure must then be
// passed to sh2_open() to initialize the SH2 interface.
//
// If the DFU (download firmware update) capability is needed, the
// example DFU code also uses this interface but each function has
// somewhat different requirements.  So a separate instance of an
// sh2_Hal_t structure, pointing to different functions, is
// recommended for supporting DFU.

struct sh2_Hal_s {
    // This function initializes communications with the device.  It
    // can initialize any GPIO pins and peripheral devices used to
    // interface with the sensor hub.
    // It should also perform a reset cycle on the sensor hub to
    // ensure communications start from a known state.
    int (*open)(sh2_Hal_t *self);

    // This function completes communications with the sensor hub.
    // It should put the device in reset then de-initialize any
    // peripherals or hardware resources that were used.
    void (*close)(sh2_Hal_t *self);

    // This function supports reading data from the sensor hub.
    // It will be calLED frequently to sevice the device.
    //
    // If the HAL has received a full SHTP transfer, this function
    // should load the data into pBuffer, set the timestamp to the
    // time the interrupt was detected and return the non-zero length
    // of data in this transfer.
    //
    // If the HAL has not recevied a full SHTP transfer, this function
    // should return 0.
    //
    // Because this function is calLED regularly, it can be used to
    // perform other housekeeping operations.  (In the case of UART
    // interfacing, bytes transmitted are staggered in time and this
    // function can be used to keep the transmission flowing.)
    int (*read)(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t_us);

    // This function supports writing data to the sensor hub.
    // It is calLED each time the application has a block of data to
    // transfer to the device.
    //
    // If the device isn't ready to receive data this function can
    // return 0 without performing the transmit function.
    //
    // If the transmission can be started, this function needs to
    // copy the data from pBuffer and return the number of bytes
    // accepted.  It need not block.  The actual transmission of
    // the data can continue after this function returns.
    int (*write)(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len);

    // This function should return a 32-bit value representing a
    // microsecond counter.  The count may roll over after 2^32
    // microseconds.  
    uint32_t (*getTimeUs)(sh2_Hal_t *self);
};

// End of include guard
#endif



File: src/sh2_util.c

/*
 * Copyright 2015-16 Hillcrest Laboratories, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License and 
 * any applicable agreements you may have with Hillcrest Laboratories, Inc.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Simple Utility functions common to several SH2 files.
 */

#include "sh2_util.h"

uint8_t readu8(const uint8_t *p)
{
    uint8_t retval = p[0];
    return retval;
}

void writeu8(uint8_t * p, uint8_t value)
{
    *p = (uint8_t)(value & 0xFF);
}

uint16_t readu16(const uint8_t *p)
{
    uint16_t retval = p[0] | (p[1] << 8);
    return retval;
}

void writeu16(uint8_t * p, uint16_t value)
{
    *p++ = (uint8_t)(value & 0xFF);
    value >>= 8;
    *p = (uint8_t)(value & 0xFF);
}

uint32_t readu32(const uint8_t *p)
{
    uint32_t retval = p[0] | (p[1] << 8) | (p[2] << 16) | (p[3] << 24);
    return retval;
}

void writeu32(uint8_t * p, uint32_t value)
{
    *p++ = (uint8_t)(value & 0xFF);
    value >>= 8;
    *p++ = (uint8_t)(value & 0xFF);
    value >>= 8;
    *p++ = (uint8_t)(value & 0xFF);
    value >>= 8;
    *p = (uint8_t)(value & 0xFF);
}

int8_t read8(const uint8_t *p)
{
    int8_t retval = p[0];
    return retval;
}

void write8(uint8_t * p, int8_t value)
{
    *p = (uint8_t)(value & 0xFF);
}

int16_t read16(const uint8_t *p)
{
    int16_t retval = p[0] | (p[1] << 8);
    return retval;
}

void write16(uint8_t * p, int16_t value)
{
    *p++ = (uint8_t)(value & 0xFF);
    value >>= 8;
    *p = (uint8_t)(value & 0xFF);
}

int32_t read32(const uint8_t *p)
{
    int32_t retval = p[0] | (p[1] << 8) | (p[2] << 16) | (p[3] << 24);
    return retval;
}

void write32(uint8_t * p, int32_t value)
{
    *p++ = (uint8_t)(value & 0xFF);
    value >>= 8;
    *p++ = (uint8_t)(value & 0xFF);
    value >>= 8;
    *p++ = (uint8_t)(value & 0xFF);
    value >>= 8;
    *p = (uint8_t)(value & 0xFF);
}



File: src/sh2_util.h

/*
 * Copyright 2015-16 Hillcrest Laboratories, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License and 
 * any applicable agreements you may have with Hillcrest Laboratories, Inc.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Simple Utility functions common to several SH2 files.
 */

#ifndef SH2_UTIL_H
#define SH2_UTIL_H

#include <stdint.h>

#ifndef ARRAY_LEN
#define ARRAY_LEN(a) (sizeof(a)/sizeof(a[0]))
#endif

uint8_t  readu8(const uint8_t * buffer);
void writeu8(uint8_t * buffer, uint8_t value);
uint16_t readu16(const uint8_t * buffer);
void writeu16(uint8_t * buffer, uint16_t value);
uint32_t readu32(const uint8_t * buffer);
void writeu32(uint8_t * buffer, uint32_t value);

int8_t read8(const uint8_t * buffer);
void write8(uint8_t * buffer, int8_t value);
int16_t read16(const uint8_t * buffer);
void write16(uint8_t * buffer, int16_t value);
int32_t read32(const uint8_t * buffer);
void write32(uint8_t * buffer, int32_t value);

#endif



File: src/shtp.c

/*
 * Copyright 2015-18 Hillcrest Laboratories, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License and 
 * any applicable agreements you may have with Hillcrest Laboratories, Inc.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Hillcrest Sensor Hub Transport Protocol (SHTP) API
 */

#include "shtp.h"
#include "sh2_err.h"
#include "sh2_util.h"

#include <string.h>

// ------------------------------------------------------------------------
// Private types

#define SH2_MAX_APPS (5)
#define SHTP_APP_NAME_LEN (32)
#define SH2_MAX_CHANS (8)
#define SHTP_CHAN_NAME_LEN (32)

// Defined Globally Unique Identifiers
#define GUID_SHTP (0)

// Command Channel commands and parameters
#define SHTP_CHAN_COMMAND (0)
#define CMD_ADVERTISE (0)
#define CMD_ADVERTISE_SHTP (0)
#define CMD_ADVERTISE_ALL (1)
#define RESP_ADVERTISE (0)

#define SHTP_HDR_LEN (4)

#define TAG_SHTP_VERSION 0x80

typedef struct shtp_App_s {
    uint32_t guid;
    char appName[SHTP_APP_NAME_LEN];
} shtp_App_t;

typedef struct shtp_AppListener_s {
    uint16_t guid;
    shtp_AdvertCallback_t *callback;
    void *cookie;
} shtp_AppListener_t;

typedef struct shtp_ChanListener_s {
    uint16_t guid;
    char chanName[SHTP_CHAN_NAME_LEN];
    shtp_Callback_t *callback;
    void *cookie;
} shtp_ChanListener_t;

typedef struct shtp_Channel_s {
    uint8_t nextOutSeq;
    uint8_t nextInSeq;
    uint32_t guid;  // app id
    char chanName[SHTP_CHAN_NAME_LEN];
    bool wake;
    shtp_Callback_t *callback;
    void *cookie;
} shtp_Channel_t;

typedef enum {
    ADVERT_NEEDED,
    ADVERT_REQUESTED,
    ADVERT_IDLE,
} advert_phase_t;

// Per-instance data for SHTP
typedef struct shtp_s {
    // Associated SHTP HAL
    // If 0, this indicates the SHTP instance is available for new opens
    sh2_Hal_t *pHal;

    // Asynchronous Event callback and it's cookie
    shtp_EventCallback_t *eventCallback;
    void * eventCookie;

    // Data from adverts
    char shtpVersion[8];
    uint16_t outMaxPayload;
    uint16_t outMaxTransfer;

    // Transmit support
    uint8_t outTransfer[SH2_HAL_MAX_TRANSFER_OUT];

    // Receive support
    uint16_t inMaxTransfer;
    uint16_t inRemaining;
    uint8_t  inChan;
    uint8_t  inPayload[SH2_HAL_MAX_PAYLOAD_IN];
    uint16_t inCursor;
    uint32_t inTimestamp;
    uint8_t inTransfer[SH2_HAL_MAX_TRANSFER_IN];
    
    // What stage of advertisement processing are we in.
    advert_phase_t advertPhase;
    
    // Applications
    shtp_App_t app[SH2_MAX_APPS];
    uint8_t    nextApp;
    
    // Advert registrations
    uint8_t nextAppListener;
    shtp_AppListener_t appListener[SH2_MAX_APPS];

    // SHTP Channels
    shtp_Channel_t      chan[SH2_MAX_CHANS];

    // Channel listeners
    shtp_ChanListener_t chanListener[SH2_MAX_CHANS];
    uint8_t             nextChanListener;
    
    // Stats
    uint32_t txDiscards;
    uint32_t shortFragments;
    uint32_t tooLargePayloads;
    uint32_t badRxChan;
    uint32_t badTxChan;

} shtp_t;


// ------------------------------------------------------------------------
// Private data

// Advertisement request
static const uint8_t advertise[] = {
    CMD_ADVERTISE,
    CMD_ADVERTISE_ALL
};

#define MAX_INSTANCES (1)
static shtp_t instances[MAX_INSTANCES];

static bool shtp_initialized = false;

// ------------------------------------------------------------------------
// Private functions

static void shtp_init(void)
{
    // clear instance memory.
    // In particular, this clears the pHal pointers which are used
    // to determine if an instance is open and in-use.
    memset(instances, 0, sizeof(instances));

    // Set the initialized flag so this doesn't happen again.
    shtp_initialized = true;
}

static shtp_t *getInstance(void)
{
    for (int n = 0; n < MAX_INSTANCES; n++) {
        if (instances[n].pHal == 0) {
            // This instance is free
            return &instances[n];
        }
    }

    // Can't give an instance, none are free
    return 0;
}

// Register a listener for an app (advertisement listener)
static void addAdvertListener(shtp_t *pShtp, uint16_t guid,
                              shtp_AdvertCallback_t *callback, void * cookie)
{
    shtp_AppListener_t *pAppListener = 0;

    // Bail out if no space for more apps
    if (pShtp->nextAppListener >= SH2_MAX_APPS) return;

    // Register this app
    pAppListener = &pShtp->appListener[pShtp->nextAppListener];
    pShtp->nextAppListener++;
    pAppListener->guid = guid;
    pAppListener->callback = callback;
    pAppListener->cookie = cookie;
}

// Try to match registered listeners with their channels.
// This is performed every time the underlying Channel, App, Listener data structures are updated.
// As a result, channel number to callback association is fast when receiving packets
static void updateCallbacks(shtp_t *pShtp)
{
    // Figure out which callback is associated with each channel.
    //   Channel -> (GUID, Chan name).
    //   GUID -> App name.
    //   (App name, Chan name) -> Callback

    uint32_t guid;
    const char * chanName = 0;
    
    for (int chanNo = 0; chanNo < SH2_MAX_CHANS; chanNo++) {
        // Reset callback for this channel until we find the right one.
        pShtp->chan[chanNo].callback = 0;
            
        if (pShtp->chan[chanNo].guid == 0xFFFFFFFF) {
            // This channel entry not used.
            continue;
        }

        // Get GUID and Channel Name for this channel
        guid = pShtp->chan[chanNo].guid;
        chanName = pShtp->chan[chanNo].chanName;

        // Look for a listener registered with this guid, channel name
        for (int listenerNo = 0; listenerNo < SH2_MAX_CHANS; listenerNo++)
        {
            if ((pShtp->chanListener[listenerNo].callback != 0) &&
                (pShtp->chanListener[listenerNo].guid == guid) &&
                (strcmp(chanName, pShtp->chanListener[listenerNo].chanName) == 0))
            {
            
                // This listener is the one for this channel
                pShtp->chan[chanNo].callback = pShtp->chanListener[listenerNo].callback;
                pShtp->chan[chanNo].cookie = pShtp->chanListener[listenerNo].cookie;
                break;
            }
        }
    }
}

// Register a new channel listener
static int addChanListener(shtp_t *pShtp,
                           uint16_t guid, const char * chanName,
                           shtp_Callback_t *callback, void *cookie)
{
    shtp_ChanListener_t *pListener = 0;

    // Bail out if there are too many listeners registered
    if (pShtp->nextChanListener >= SH2_MAX_CHANS) return SH2_ERR;

    // Register channel listener
    pListener = &pShtp->chanListener[pShtp->nextChanListener];
    pShtp->nextChanListener++;
    pListener->guid = guid;
    strcpy(pListener->chanName, chanName);
    pListener->callback = callback;
    pListener->cookie = cookie;

    // re-evaluate channel callbacks
    updateCallbacks(pShtp);

    return SH2_OK;
}

static inline uint16_t min_u16(uint16_t a, uint16_t b)
{
    if (a < b) {
        return a;
    }
    else {
        return b;
    }
}

// Send a cargo as a sequence of transports
static int txProcess(shtp_t *pShtp, uint8_t chan, const uint8_t* pData, uint32_t len)
{
    int status = SH2_OK;
    
    bool continuation = false;
    uint16_t cursor = 0;
    uint16_t remaining;
    uint16_t transferLen;  // length of transfer, minus the header
    uint16_t lenField;

    cursor = 0;
    remaining = len;
    while (remaining > 0) {
        // How much data (not header) can we send in next transfer
        transferLen = min_u16(remaining, pShtp->outMaxTransfer-SHTP_HDR_LEN);
        
        // Length field will be transferLen + SHTP_HDR_LEN
        lenField = transferLen + SHTP_HDR_LEN;

        // Put the header in the out buffer
        pShtp->outTransfer[0] = lenField & 0xFF;
        pShtp->outTransfer[1] = (lenField >> 8) & 0xFF;
        if (continuation) {
            pShtp->outTransfer[1] |= 0x80;
        }
        pShtp->outTransfer[2] = chan;
        pShtp->outTransfer[3] = pShtp->chan[chan].nextOutSeq++;

        // Stage one tranfer in the out buffer
        memcpy(pShtp->outTransfer+SHTP_HDR_LEN, pData+cursor, transferLen);
        remaining -= transferLen;
        cursor += transferLen;

        // Transmit (try repeatedly while HAL write returns 0)
        status = pShtp->pHal->write(pShtp->pHal, pShtp->outTransfer, lenField);
        while (status == 0)
        {
            shtp_service(pShtp);
            status = pShtp->pHal->write(pShtp->pHal, pShtp->outTransfer, lenField);
        }
        
        if (status < 0)
        {
            // Error, throw away this cargo
            pShtp->txDiscards++;
            return status;
        }

        // For the rest of this transmission, packets are continuations.
        continuation = true;
    }

    return SH2_OK;
}

// Callback for SHTP app-specific advertisement tags
static void shtpAdvertHdlr(void *cookie, uint8_t tag, uint8_t len, uint8_t *val)
{
    shtp_t *pShtp = (shtp_t *)cookie;

    switch (tag) {
        case TAG_SHTP_VERSION:
            if (strlen((const char *)val) < sizeof(pShtp->shtpVersion)) {
                strcpy(pShtp->shtpVersion, (const char *)val);
            }
            break;
        default:
            break;
    }
}

// Add one to the set of known Apps
static void addApp(shtp_t *pShtp, uint32_t guid)
{
    shtp_App_t *pApp = 0;

    // Bail out if this GUID is already registered
    for (int n = 0; n < pShtp->nextApp; n++) {
        if (pShtp->app[n].guid == guid) return;
    }

    // Bail out if no space for more apps
    if (pShtp->nextApp >= SH2_MAX_APPS) return;

    // Register this app
    pApp = &pShtp->app[pShtp->nextApp];
    pShtp->nextApp++;
    pApp->guid = guid;
    strcpy(pApp->appName, "");

    // Re-evaluate channel callbacks
    updateCallbacks(pShtp);
}

static void setAppName(shtp_t *pShtp, uint32_t guid, const char * appName)
{
    shtp_App_t *pApp = 0;
    
    // Find the app entry with this GUID
    for (unsigned n = 0; n < pShtp->nextApp; n++) {
        if (pShtp->app[n].guid == guid) {
            pApp = &pShtp->app[n];
            strcpy(pApp->appName, appName);
            return;
        }
    }
}

// Add one to the set of known channels
static void addChannel(shtp_t *pShtp, uint8_t chanNo, uint32_t guid, const char * chanName, bool wake)
{
    if (chanNo >= SH2_MAX_CHANS) return;

    shtp_Channel_t * pChan = &pShtp->chan[chanNo];

    // Store channel definition
    pChan->guid = guid;
    strcpy(pChan->chanName, chanName);
    pChan->wake = wake;

    // Init channel-associated data
    pChan->nextOutSeq = 0;
    pChan->nextInSeq = 0;
    pChan->callback = 0;
    pChan->cookie = 0;

    // Re-evaluate channel callbacks
    updateCallbacks(pShtp);
}

static void callAdvertHandler(shtp_t *pShtp, uint32_t guid,
                              uint8_t tag, uint8_t len, uint8_t *val)
{
    // Find listener for this app
    for (int n = 0; n < SH2_MAX_APPS; n++)
    {
        if (pShtp->appListener[n].guid == guid) {
            // Found matching App entry
            if (pShtp->appListener[n].callback != 0) {
                pShtp->appListener[n].callback(pShtp->appListener[n].cookie, tag, len, val);
                return;
            }
        }
    }
}

static void processAdvertisement(shtp_t *pShtp, uint8_t *payload, uint16_t payloadLen)
{
    uint16_t x;
    uint8_t tag;
    uint8_t len;
    uint8_t *val;
    uint16_t cursor = 1;
    uint32_t guid = 0;
    char appName[SHTP_APP_NAME_LEN];
    char chanName[SHTP_CHAN_NAME_LEN];
    uint8_t chanNo = 0;
    bool wake = false;

    strcpy(appName, "");
    strcpy(chanName, "");

    pShtp->advertPhase = ADVERT_IDLE;
        
    while (cursor < payloadLen) {
        tag = payload[cursor++];
        len = payload[cursor++];
        val = payload+cursor;
        cursor += len;

        // Process tag
        switch (tag) {
            case TAG_NULL:
                // Reserved value, not a valid tag.
                break;
            case TAG_GUID:
                // A new GUID is being established so terminate advertisement process with earlier app, if any.
                callAdvertHandler(pShtp, guid, TAG_NULL, 0, 0);
                
                guid = readu32(val);
                addApp(pShtp, guid);
            
                strcpy(appName, "");
                strcpy(chanName, "");
                break;
            case TAG_MAX_CARGO_PLUS_HEADER_WRITE:
                x = readu16(val) - SHTP_HDR_LEN;
            
                if (x < SH2_HAL_MAX_PAYLOAD_OUT) {
                    pShtp->outMaxPayload = x;
                }
                break;
            case TAG_MAX_CARGO_PLUS_HEADER_READ:
                x = readu16(val) - SHTP_HDR_LEN;
                // No need to store this!
                break;
            case TAG_MAX_TRANSFER_WRITE:
                x = readu16(val) - SHTP_HDR_LEN;
                if (x < SH2_HAL_MAX_TRANSFER_OUT) {
                    pShtp->outMaxTransfer = x;
                } else {
                    pShtp->outMaxTransfer = SH2_HAL_MAX_TRANSFER_OUT;
                }
                break;
            case TAG_MAX_TRANSFER_READ:
                x = readu16(val) - SHTP_HDR_LEN;
                if (x < SH2_HAL_MAX_TRANSFER_IN) {
                    pShtp->inMaxTransfer = x;
                }
                break;
            case TAG_NORMAL_CHANNEL:
                chanNo = readu8(val);
                wake = false;
                break;
            case TAG_WAKE_CHANNEL:
                chanNo = readu8(val);
                wake = true;
                break;
            case TAG_APP_NAME:
                strcpy(appName, (const char *)val);
                setAppName(pShtp, guid, appName);
            
                break;
            case TAG_CHANNEL_NAME:
                strcpy(chanName, (const char *)val);
                addChannel(pShtp, chanNo, guid, (const char *)val, wake);

                // Store channel metadata
                if (chanNo < SH2_MAX_CHANS) {
                    pShtp->chan[chanNo].guid = guid;
                    strcpy(pShtp->chan[chanNo].chanName, chanName);
                    pShtp->chan[chanNo].wake = wake;
                }
                break;
            case TAG_ADV_COUNT:
                // Not yet supported.
                break;
            default:
                // Nothing special needs to be done with this tag.
                break;
        }
        
        // Deliver a TLV entry to the app's handler
        callAdvertHandler(pShtp, guid, tag, len, val);
    }

    // terminate advertisement process with last app
    callAdvertHandler(pShtp, guid, TAG_NULL, 0, 0);
}

// Callback for SHTP command channel
static void shtpCmdListener(void *cookie, uint8_t *payload, uint16_t len, uint32_t timestamp)
{
    shtp_t *pShtp = (shtp_t *)cookie;
    
    if ((payload == 0) || (len == 0)) return;
    
    uint8_t response = payload[0];

    switch (response) {
        case RESP_ADVERTISE:
            processAdvertisement(pShtp, payload, len);
            break;
        default:
            // unknown response
            break;
    }
}

static void rxAssemble(shtp_t *pShtp, uint8_t *in, uint16_t len, uint32_t t_us)
{
    uint16_t payloadLen;
    bool continuation;
    uint8_t chan = 0;
    uint8_t seq = 0;

    // discard invalid short fragments
    if (len < SHTP_HDR_LEN) {
        pShtp->shortFragments++;
        return;
    }
    
    // Interpret header fields
    payloadLen = (in[0] + (in[1] << 8)) & (~0x8000);
    continuation = ((in[1] & 0x80) != 0);
    chan = in[2];
    seq = in[3];
    
    if (payloadLen < SHTP_HDR_LEN) {
      pShtp->shortFragments++;

      if (pShtp->eventCallback) {
          pShtp->eventCallback(pShtp->eventCookie, SHTP_SHORT_FRAGMENT);
      }
      return;
    }
        
    if ((chan >= SH2_MAX_CHANS) ||
        (chan >= pShtp->nextChanListener)) {
        // Invalid channel id.
        pShtp->badRxChan++;

        if (pShtp->eventCallback) {
            pShtp->eventCallback(pShtp->eventCookie, SHTP_BAD_RX_CHAN);
        }
        return;
    }

    // Discard earlier assembly in progress if the received data doesn't match it.
    if (pShtp->inRemaining) {
        // Check this against previously received data.
        if (!continuation ||
            (chan != pShtp->inChan) ||
            (seq != pShtp->chan[chan].nextInSeq)) {
            // This fragment doesn't fit with previous one, discard earlier data
            pShtp->inRemaining = 0;
        }
    }

    if (pShtp->inRemaining == 0) {
        if (payloadLen > sizeof(pShtp->inPayload)) {
            // Error: This payload won't fit! Discard it.
            pShtp->tooLargePayloads++;
            
            if (pShtp->eventCallback) {
                pShtp->eventCallback(pShtp->eventCookie, SHTP_TOO_LARGE_PAYLOADS);
            }
            return;
        }

        // This represents a new payload

        // Store timestamp
        pShtp->inTimestamp = t_us;

        // Start a new assembly.
        pShtp->inCursor = 0;
        pShtp->inChan = chan;
    }

    // Append the new fragment to the payload under construction.
    if (len > payloadLen) {
        // Only use the valid portion of the transfer
        len = payloadLen;
    }
    memcpy(pShtp->inPayload + pShtp->inCursor, in+SHTP_HDR_LEN, len-SHTP_HDR_LEN);
    pShtp->inCursor += len-SHTP_HDR_LEN;
    pShtp->inRemaining = payloadLen - len;

    // If whole payload received, deliver it to channel listener.
    if (pShtp->inRemaining == 0) {

        // Call callback if there is one.
        if (pShtp->chan[chan].callback != 0) {
            pShtp->chan[chan].callback(pShtp->chan[chan].cookie,
                                       pShtp->inPayload, pShtp->inCursor,
                                       pShtp->inTimestamp);
        }
    }

    // Remember next sequence number we expect for this channel.
    pShtp->chan[chan].nextInSeq = seq + 1;
}

// ------------------------------------------------------------------------
// Public functions

// Takes HAL pointer, returns shtp ID for use in future calls.
// HAL will be opened by this call.
void *shtp_open(sh2_Hal_t *pHal)
{
    if (!shtp_initialized) {
        // Perform one-time module initialization
        shtp_init();
    }
    
    // Validate params
    if (pHal == 0) {
        // Error
        return 0;
    }

    // Find an available instance for this open
    shtp_t *pShtp = getInstance();
    if (pShtp == 0) {
        // No instances available, return error
        return 0;
    }

    // Clear the SHTP instance as a shortcut to initializing all fields
    memset(pShtp, 0, sizeof(shtp_t));
    
    // Store reference to the HAL
    pShtp->pHal = pHal;

    // Clear the asynchronous event callback point
    pShtp->eventCallback = 0;
    pShtp->eventCookie = 0;

    // Initialize state vars (be prepared for adverts)
    pShtp->outMaxPayload = SH2_HAL_MAX_PAYLOAD_OUT;
    pShtp->outMaxTransfer = SH2_HAL_MAX_TRANSFER_OUT;

    // Establish SHTP App and command channel a priori
    addApp(pShtp, GUID_SHTP);
    addChannel(pShtp, 0, GUID_SHTP, "command", false);
    
    // Register SHTP advert listener and command channel listener
    shtp_listenAdvert(pShtp, GUID_SHTP, shtpAdvertHdlr, pShtp);
    shtp_listenChan(pShtp, GUID_SHTP, "command", shtpCmdListener, pShtp);

    // When we open the HAL, it resets the device and adverts are sent automatically.
    // So we go to ADVERT_REQUESTED state.  They are on the way.
    pShtp->advertPhase = ADVERT_REQUESTED;

    // Open HAL
    pHal->open(pHal);

    return pShtp;
}

// Releases resources associated with this SHTP instance.
// HAL will not be closed.
void shtp_close(void *pInstance)
{
    shtp_t *pShtp = (shtp_t *)pInstance;

    pShtp->pHal->close(pShtp->pHal);
    
    // Clear pShtp
    // (Resetting pShtp->pHal to 0, returns this instance to the free pool)
    memset(pShtp, 0, sizeof(shtp_t));
}

// Register the pointer of the callback function for reporting asynchronous events
void shtp_setEventCallback(void *pInstance, 
                           shtp_EventCallback_t * eventCallback, 
                           void *eventCookie) {
    shtp_t *pShtp = (shtp_t *)pInstance;

    pShtp->eventCallback = eventCallback;
    pShtp->eventCookie = eventCookie;
}

// Register a listener for an SHTP channel
int shtp_listenChan(void *pInstance,
                    uint16_t guid, const char * chan,
                    shtp_Callback_t *callback, void * cookie)
{
    shtp_t *pShtp = (shtp_t *)pInstance;
    
    // Balk if channel name isn't valid
    if ((chan == 0) || (strlen(chan) == 0)) return SH2_ERR_BAD_PARAM;

    return addChanListener(pShtp, guid, chan, callback, cookie);
}

// Register a listener for SHTP advertisements 
int shtp_listenAdvert(void *pInstance,
                      uint16_t guid,
                      shtp_AdvertCallback_t *advertCallback, void * cookie)
{
    shtp_t *pShtp = (shtp_t *)pInstance;
    
    // Register the advert listener
    addAdvertListener(pShtp, guid, advertCallback, cookie);

    // Arrange for a new set of advertisements, for this listener
    if (pShtp->advertPhase == ADVERT_IDLE) {
        pShtp->advertPhase = ADVERT_NEEDED;
    }

    return SH2_OK;
}

// Look up the channel number for a particular app, channel.
uint8_t shtp_chanNo(void *pInstance,
                    const char * appName, const char * chanName)
{
    shtp_t *pShtp = (shtp_t *)pInstance;
    
    int chan = 0;
    uint32_t guid = 0xFFFFFFFF;

    // Determine GUID for this appname
    for (int n = 0; n < SH2_MAX_APPS; n++) {
        if (strcmp(pShtp->app[n].appName, appName) == 0) {
            guid = pShtp->app[n].guid;
            break;
        }
    }
    if (guid == 0xFFFFFFFF) return -1;

    for (chan = 0; chan < SH2_MAX_CHANS; chan++) {
        if ((strcmp(pShtp->chan[chan].chanName, chanName) == 0) &&
            pShtp->chan[chan].guid == guid) {
            // Found match
            return chan;
        }
    }

    // Not found
    return 0xFF;
}

// Send an SHTP payload on a particular channel
int shtp_send(void *pInstance,
              uint8_t channel, const uint8_t *payload, uint16_t len)
{
    shtp_t *pShtp = (shtp_t *)pInstance;
    int ret = SH2_OK;
    
    if (len > pShtp->outMaxPayload) {
        return SH2_ERR_BAD_PARAM;
    }
    if (channel >= SH2_MAX_CHANS) {
        pShtp->badTxChan++;
        return SH2_ERR_BAD_PARAM;
    }
    
    ret = txProcess(pShtp, channel, payload, len);

    return ret;
}

// Check for received data and process it.
void shtp_service(void *pInstance)
{
    shtp_t *pShtp = (shtp_t *)pInstance;
    uint32_t t_us = 0;

    if (pShtp->advertPhase == ADVERT_NEEDED) {
        pShtp->advertPhase = ADVERT_REQUESTED;  // do this before send, to avoid recursion.
        int status = shtp_send(pShtp, SHTP_CHAN_COMMAND, advertise, sizeof(advertise));
        if (status != SH2_OK) {
            // Oops, advert request faiLED.  Go back to needing one.
            pShtp->advertPhase = ADVERT_NEEDED;
        }
    }

    int len = pShtp->pHal->read(pShtp->pHal, pShtp->inTransfer, sizeof(pShtp->inTransfer), &t_us);
    if (len) {
        rxAssemble(pShtp, pShtp->inTransfer, len, t_us);
    }
}



File: src/shtp.h

/*
 * Copyright 2015-18 Hillcrest Laboratories, Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License and 
 * any applicable agreements you may have with Hillcrest Laboratories, Inc.
 * You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Hillcrest Sensor Hub Transport Protocol (SHTP) API
 */

#ifndef SHTP_H
#define SHTP_H

#include <stdint.h>
#include <stdbool.h>

#include "sh2_hal.h"

// Advertisement TLV tags
#define TAG_NULL 0
#define TAG_GUID 1
#define TAG_MAX_CARGO_PLUS_HEADER_WRITE 2
#define TAG_MAX_CARGO_PLUS_HEADER_READ 3
#define TAG_MAX_TRANSFER_WRITE 4
#define TAG_MAX_TRANSFER_READ 5
#define TAG_NORMAL_CHANNEL 6
#define TAG_WAKE_CHANNEL 7
#define TAG_APP_NAME 8
#define TAG_CHANNEL_NAME 9
#define TAG_ADV_COUNT 10
#define TAG_APP_SPECIFIC 0x80

typedef enum shtp_Event_e {
    SHTP_TX_DISCARD = 0,
    SHTP_SHORT_FRAGMENT = 1,
    SHTP_TOO_LARGE_PAYLOADS = 2,
    SHTP_BAD_RX_CHAN = 3,
    SHTP_BAD_TX_CHAN = 4,
} shtp_Event_t;

typedef void shtp_Callback_t(void * cookie, uint8_t *payload, uint16_t len, uint32_t timestamp);
typedef void shtp_AdvertCallback_t(void * cookie, uint8_t tag, uint8_t len, uint8_t *value);
typedef void shtp_SendCallback_t(void *cookie);
typedef void shtp_EventCallback_t(void *cookie, shtp_Event_t shtpEvent);

// Takes HAL pointer, returns shtp ID for use in future calls.
// HAL will be opened by this call.
void * shtp_open(sh2_Hal_t *pHal);

// Releases resources associated with this SHTP instance.
// HAL will not be closed.
void shtp_close(void *pShtp);

// Provide the point of the callback function for reporting SHTP asynchronous events
void shtp_setEventCallback(void *pInstance,
                           shtp_EventCallback_t * eventCallback, 
                           void *eventCookie);

// Register a listener for an SHTP channel
int shtp_listenChan(void *pShtp,
                    uint16_t guid, const char * chan,
                    shtp_Callback_t *callback, void * cookie);

// Register a listener for SHTP advertisements 
int shtp_listenAdvert(void *pShtp,
                      uint16_t guid,
                      shtp_AdvertCallback_t *advertCallback, void * cookie);

// Look up the channel number for a particular app, channel.
uint8_t shtp_chanNo(void *pShtp,
                    const char * appName, const char * chanName);

// Send an SHTP payload on a particular channel
int shtp_send(void *pShtp,
              uint8_t channel, const uint8_t *payload, uint16_t len);

// Check for received data and process it.
void shtp_service(void *pShtp);

// #ifdef SHTP_H
#endif
