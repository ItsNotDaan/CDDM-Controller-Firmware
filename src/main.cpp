/*
  Name: CDDM-Controller-Firmware
  Date: 13-11-2024
  Author: Daan Heetkamp

  Description:
  This firmware is designed for the ESP32C3 Wroom microcontroller.
  It utilizes a wake/deepsleep switch, OLED Screen (0.91 OLED) and an external gyroscope (MPU6050).
  The communication between the devices is facilitated by ESP-NOW EASY.
  Using a rotary encoder, the user can control the data on the OLED.

  Functionality:
  1. The controller, using its gyroscope, will control the robot's movements. Tilting the controller will result in corresponding movements of the robot.
  2. The controller will display the robot's status on its OLED screen.


  NOTE: Due to the mquality of the PCB that has been milled the Serial Monitor is not available. Using the OLED screen is the only way to debug the program.

  Revision:
  0.1 - Initial draft
  0.2 - Added the OLED screen and the gyroscope and the rotary encoder
  0.3 - Added the ESP-NOW EASY library with the deep sleep functionality
  0.4 - Cleaning up code and adding comments. TO-DO: Make the device OLED more user-friendly.
*/

// include libraries
#include <Arduino.h>
#include <ESPNOW-EASY.h>
#include "driver/rtc_io.h"
#include <esp_sleep.h>

// include the MPU6050 library
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// include the OLED library
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// ------- Declare Constants -------
// Debug Settings (true or false)
#define DEBUG_GYRO true
#define DEBUG_ROTARY false
#define DEBUG_ESPNOW false
#define DEBUG_SYSTEM true

// GPIO Pins
#define MCU_SDA GPIO_NUM_7
#define MCU_SCL GPIO_NUM_8

#define MCU_WAKE_DEEPSLEEP GPIO_NUM_3

#define MCU_ROTARY_ENCODING_A1 GPIO_NUM_21
#define MCU_ROTARY_ENCODING_B1 GPIO_NUM_1
#define MCU_ROTARY_BUTTON GPIO_NUM_10

// Constants for OLED display dimensions.
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32

// ESP-NOW settings
#define DEVICE_TYPE MASTER     // Device type (MASTER or SLAVE)
#define DEBUG_SETTING DEBUG_ON // Debug setting (DEBUG_ON or DEBUG_OFF)

// ----- Declare Objects -----
// Create an object of the MPU6050 Objects
Adafruit_MPU6050 mpu;
sensors_event_t a, g, temp;

// Create an object of the OLED screen
Adafruit_SSD1306 display = Adafruit_SSD1306(128, 32, &Wire);

// ----- Declare subroutines and/or functions -----
void readGyroscope();
void checkRotaryEncoder();
void checkOnOffSwitch();
void displayMenu();

// ----- Declare Global Variables -----
// MPU6050 variables
float accY, accX;

// Rotary Encoder button and turn variables
int rotaryButtonCount = 0;
bool rotaryButtonState = LOW;

int rotaryTurnCount = 0;
int CLK_state;
int prev_CLK_state;

// Setup
void setup()
{
  Serial.begin(115200); // Start the serial monitor at 115200 baud

  // Wire
  Wire.begin(MCU_SDA, MCU_SCL, 1000000); // Start the I2C communication

  // ESPNOW
  if (initESPNOW(DEVICE_TYPE, DEBUG_SETTING) == false)
  {
    Serial.println("ESP-NOW initialization failed");
    ESP.restart();
  }

  startPairingProcess();
  setReceivedMessageOnMonitor(DEBUG_ESPNOW);

  // Initialize the MPU6050 and set offset values
  if (!mpu.begin())
  {
    Serial.println("Failed to find MPU6050 chip");
    while (1)
    {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  // Initialize the MPU6050 and set offset values
  mpu.begin();

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Initialize the OLED screen
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // Address 0x3C for 128x32
  display.clearDisplay();

  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setRotation(0);

  // Init the rotary encoder pins
  pinMode(MCU_ROTARY_ENCODING_A1, INPUT_PULLUP);
  pinMode(MCU_ROTARY_ENCODING_B1, INPUT_PULLUP);
  pinMode(MCU_ROTARY_BUTTON, INPUT_PULLUP);

  // Set the wake/deepsleep switch
  pinMode(MCU_WAKE_DEEPSLEEP, INPUT);

  esp_deep_sleep_enable_gpio_wakeup(1 << MCU_WAKE_DEEPSLEEP, ESP_GPIO_WAKEUP_GPIO_HIGH);
  gpio_pulldown_en(MCU_WAKE_DEEPSLEEP);
  gpio_pullup_dis(MCU_WAKE_DEEPSLEEP);

  // Display the welcome message
  display.setCursor(0, 0);
  display.println("Hello, World!");
  display.display();

  Serial.println("Setup complete");
}

// Main loop
void loop()
{
  // Check the rotary encoder for a turn or press.
  checkRotaryEncoder();

  // Read the gyroscope values
  readGyroscope();

  // Check if the wake/deepsleep switch is pressed
  checkOnOffSwitch();

  // Check the ESP-NOW pairing mode status
  checkPairingModeStatus(5000); // Check the pairing mode status every 5 seconds if pairing mode is active.

  // Write to ESP-NOW
  sendMpuData(0, 0, 0, accX, accY, 0);
  // sendData(DATA, "Hello", 1);
}
//******************************************************************************************************//

/***********************************Check On/Off Switch******************************************/
/// @brief This function will check the on/off switch and put the ESP32C3 in deep sleep mode.
void checkOnOffSwitch()
{
  // Check if the wake/deepsleep switch is pressed
  if (digitalRead(MCU_WAKE_DEEPSLEEP) == LOW)
  {
    // Serial.println("Going to sleep");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Going to sleep!");
    display.display();

    Serial.println("Going to sleep");
    delay(1000);

    display.clearDisplay();
    display.display();

    esp_deep_sleep_start();
  }
}

/***********************************Check Rotary Encoder******************************************/
/// @brief This function will check the rotary encoder for a turn or press.
void checkRotaryEncoder()
{
  // Check the rotary encoder button for a press.
  if ((digitalRead(MCU_ROTARY_BUTTON) == LOW) && (rotaryButtonState == HIGH))
  {
    rotaryButtonCount++;
  }
  rotaryButtonState = digitalRead(MCU_ROTARY_BUTTON);

  // read the current state of the rotary encoder's CLK pin
  CLK_state = digitalRead(MCU_ROTARY_ENCODING_A1);

  // If the state of CLK is changed, then pulse occurred
  // React to only the rising edge (from LOW to HIGH) to avoid double count
  if (CLK_state != prev_CLK_state && CLK_state == HIGH)
  {
    // if the DT state is HIGH
    // the encoder is rotating in counter-clockwise direction => decrease the counter
    if (digitalRead(MCU_ROTARY_ENCODING_B1) == HIGH)
    {
      rotaryTurnCount--;
    }
    else
    {
      // the encoder is rotating in clockwise direction => increase the counter
      rotaryTurnCount++;
    }
  }

  // save last CLK state
  prev_CLK_state = CLK_state;

  // Display the rotary encoder values on the OLED screen
  if (DEBUG_ROTARY)
  {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("BTN Count: ");
    display.println(rotaryButtonCount);
    display.print("Rotary Count: ");
    display.println(rotaryTurnCount);
    display.display();
  }
}

/***********************************Menu******************************************/
/// @brief This function will display a menu on the OLED screen and navigate through it using the rotary encoder.
void displayMenu()
{
  int menuIndex = 0;
  int lastMenuIndex = -1;

  while (true)
  {
    // Read the rotary encoder
    int newMenuIndex = menuIndex; // Replace this with actual rotary encoder reading logic

    if (newMenuIndex != lastMenuIndex)
    {
      lastMenuIndex = newMenuIndex;

      display.clearDisplay();
      display.setCursor(0, 0);

      switch (newMenuIndex)
      {
      case 0:
        display.println("Menu Item 1");
        break;
      case 1:
        display.println("Menu Item 2");
        break;
      case 2:
        display.println("Menu Item 3");
        break;
      default:
        display.println("Invalid Menu Item");
        break;
      }

      display.display();
    }

    delay(100); // Adjust delay as needed
  }
}

/***********************************Read Gyroscope******************************************/
/// @brief This function will read the gyroscope values and store them in the global variables.
void readGyroscope()
{
  // Read the accelerometer
  mpu.getEvent(&a, &g, &temp);

  // Store the gyroscope values in the global variables
  accX = a.acceleration.x;
  accY = a.acceleration.y;

  if (DEBUG_GYRO)
  {
    display.clearDisplay();
    display.setCursor(0, 0);

    display.println("Accelerometer - m/s^2");
    display.print(a.acceleration.x, 1);
    display.print(", ");
    display.print(a.acceleration.y, 1);
    display.print(", ");
    display.print(a.acceleration.z, 1);
    display.println("");

    display.println("Gyroscope - rps");
    display.print(g.gyro.x, 1);
    display.print(", ");
    display.print(g.gyro.y, 1);
    display.print(", ");
    display.print(g.gyro.z, 1);
    display.println("");

    display.display();
    delay(100);
  }
}