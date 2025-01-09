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
  0.5 - Added the beginning of the menu system.
  0.6 - Added the menu system and the sub-menu system. Values on the robot can be changed using the controller.
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
#define DEBUG_GYRO false
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
int checkRotaryEncoder();
void checkOnOffSwitch();
void displayMenu(int rotaryResult);
void displaySubMenu(int rotaryResult);

// ----- Declare Global Variables -----
// MPU6050 variables
float accY, accX;

// Rotary Encoder button and turn variables
int rotaryButtonCount = 0;
bool rotaryButtonState = LOW;

int rotaryTurnCount = 0;
int CLK_state;
int prev_CLK_state;

// PID variables
float KP = 0.00;
float KI = 0.00;
float KD = 0.00;

// OLED screen variables
bool inSubMenuPress = false;
bool wasInSubMenu = false;
int lastSubMenuValue = 0;

int menuIndex = -1;
int lastMenuIndex = -1;

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
  int funcValue = checkRotaryEncoder();

  // Display the menu on the OLED screen
  displayMenu(funcValue);

  // Read the gyroscope values
  readGyroscope();

  // Check if the wake/deepsleep switch is pressed
  checkOnOffSwitch();

  // Check the ESP-NOW pairing mode status
  checkPairingModeStatus(5000); // Check the pairing mode status every 5 seconds if pairing mode is active.

  // Write to ESP-NOW
  // sendMpuData(0, 0, 0, accX, accY, 0);
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
/// @return 1 for clockwise turn, -1 for counter-clockwise turn, 2 for button press, 0 for no action.
int checkRotaryEncoder()
{
  int returnResult = 0;

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
      returnResult = 1;
    }
    else
    {
      // the encoder is rotating in clockwise direction => increase the counter
      returnResult = -1;
    }
  }

  // save last CLK state
  prev_CLK_state = CLK_state;

  // Check the rotary encoder button for a press.
  if ((digitalRead(MCU_ROTARY_BUTTON) == LOW) && (rotaryButtonState == HIGH))
  {
    returnResult = 2;
  }
  rotaryButtonState = digitalRead(MCU_ROTARY_BUTTON);

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

  return returnResult;
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

/***********************************Menu******************************************/
/// @brief This function will display a menu on the OLED screen and navigate through it using the rotary encoder.
/// @param rotaryResult The result of the rotary encoder check.
void displayMenu(int rotaryResult)
{

  if (rotaryResult == 2)
  {
    inSubMenuPress = !inSubMenuPress;
  }

  if (inSubMenuPress) // Check if the user is in the sub menu.
  {
    // function to display the sub menu
    displaySubMenu(rotaryResult);

    wasInSubMenu = true;
    return;
  }
  else // If the user is not in the sub menu display the main menu.
  {
    // Check if the rotary encoder has been turned.
    if (rotaryResult == 1)
    {
      menuIndex++;
      if (menuIndex > 9)
      {
        menuIndex = 0;
      }
    }
    else if (rotaryResult == -1)
    {
      menuIndex--;
      if (menuIndex < 0)
      {
        menuIndex = 9;
      }
    }

    // Display the menu on the OLED screen
    if ((menuIndex != lastMenuIndex) || (wasInSubMenu == true)) //
    {
      wasInSubMenu = false; // Check to see if the user was in the sub menu. This should reset the screen to the main menu.

      display.clearDisplay();
      display.setCursor(0, 0); // 22 characters per line.

      switch (menuIndex)
      {
      case 0: // Menu 1: Connect to robot
        display.println("Menu 1: Connect");
        display.println("(Re)Connect to robot.");
        display.println();
        display.println("Press rotary encoder.");
        break;
      case 1: // Menu 2: Set Controller Zero
        display.println("Menu 2: Zero Position");
        display.println("Set controller zero.");
        display.println();
        display.println("Press rotary encoder.");
        break;
      case 2: // Menu 3: Set KP value
        display.println("Menu 3: Set KP");
        display.println("Set Robot KP value.");
        display.println();
        display.println("Press rotary encoder.");
        break;
      case 3: // Menu 4: Set KI value
        display.println("Menu 4: Set KI");
        display.println("Set Robot KI value.");
        display.println();
        display.println("Press rotary encoder.");
        break;
      case 4: // Menu 5: Set KD value
        display.println("Menu 5: Set KD");
        display.println("Set Robot KD value.");
        display.println();
        display.println("Press rotary encoder.");
        break;
      case 5: // Menu 6: Set Robot to Zero
        display.println("Menu 6: Set Zero");
        display.println("Set Robot to zero.");
        display.println();
        display.println("Press rotary encoder.");
        break;
      case 6:
        display.println("Menu 7");
        break;
      case 7:
        display.println("Menu 8");
        break;
      case 8:
        display.println("Menu 9");
        break;
      case 9:
        display.println("Menu 10");
        break;
      default:
        display.println("INVALID MENU INDEX");
        display.println();
        display.println("Please rotate the");
        display.println("rotary encoder.");
        break;
      }

      display.display();
      lastMenuIndex = menuIndex;
    }
  }
}

/***********************************Sub Menu******************************************/
/// @brief This function will handle the sub menu of the OLED screen depending on the main menu selection.
void displaySubMenu(int rotaryResult)
{
  display.clearDisplay();
  display.setCursor(0, 0);

  // Check the rotary encoder for a turn, if no change of value is detecteted lastSubMenuValue.
  if (rotaryResult != lastSubMenuValue)
  {
    lastSubMenuValue = rotaryResult;

    switch (menuIndex)
    {
    case 0: // Sub Menu 1: This submenu should perform the ESPNOW reconnection.
      display.println("Sub Menu 1: Connect");
      display.println("Reconnecting...");
      display.display();

      // Reconnect to the robot
      startPairingProcess();

      delay(1000);
      // checkPairingModeStatus(5000); // Check the pairing mode status every 5 seconds if pairing mode is active.

      display.clearDisplay();
      display.setCursor(0, 0);
      display.println("Sub Menu 1: Connect");
      display.println("Connected!");
      display.display();
      delay(2000);

      // Break out of the sub menu
      inSubMenuPress = false;
      break;
    case 1: // Sub Menu 2: Set Controller Zero
      display.println("Sub Menu 2: Zero");
      display.println("Set controller zero.");

      sendData(DATA, "Hello", 1);
      // Variables still need to be created
      display.println("WORK IN PROGRESS");
      break;
    case 2: // Sub Menu 3: Set KP value
      display.println("Sub Menu 3: Set KP");
      display.print("KP: ");
      display.println(KP);
      display.println();
      display.println("Rotate the encoder");

      // The logic to change the KP value
      if (rotaryResult == 1)
      {
        KP++;
      }
      else if (rotaryResult == -1)
      {
        KP--;
      }

      sendData(DATA, "KP", KP);
      break;
    case 3: // Sub Menu 4: Set KI value
      display.println("Sub Menu 4: Set KI");
      display.print("KI: ");
      display.println(KI);
      display.println();
      display.println("Rotate the encoder");

      // The logic to change the KI value
      if (rotaryResult == 1)
      {
        KI++;
      }
      else if (rotaryResult == -1)
      {
        KI--;
      }

      sendData(DATA, "KI", KI);

      break;
    case 4: // Sub Menu 5: Set KD value
      display.println("Sub Menu 5: Set KD");
      display.print("KD: ");
      display.println(KD);
      display.println();
      display.println("Rotate the encoder");

      // The logic to change the KD value
      if (rotaryResult == 1)
      {
        KD = KD + 0.01;
      }
      else if (rotaryResult == -1)
      {
        KD = KD - 0.01;
      }

      sendData(DATA, "KD", KD);
      break;
    case 5: // Sub Menu 6: Set Robot to Zero

      sendData(DATA, "ZERO", 0);

      display.clearDisplay();
      display.setCursor(0, 0);
      display.println("Sub Menu 6: Set Zero");
      display.println("Robot set to zero.");
      display.display();
      delay(2000);

      // Break out of the sub menu
      inSubMenuPress = false;
      break;
    default:
      display.println("INVALID MENU INDEX");
      display.println();
      display.println("Create the sub");
      display.println("menu Daan.");
      break;
    }

    display.display();
  }
}
