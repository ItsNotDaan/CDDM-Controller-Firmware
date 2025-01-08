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
*/

// include libraries
#include <Arduino.h>
#include <ESPNOW-EASY.h>
#include "driver/rtc_io.h"
#include <esp_sleep.h>

// // include the MPU6050 library
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

// // include the OLED library
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

Adafruit_SSD1306 display = Adafruit_SSD1306(128, 32, &Wire);

// ----- Declare Constants -----

#define DEBUG_GYRO true
#define DEBUG_MOTOR true
#define DEBUG_ESPNOW true
#define DEBUG_SYSTEM true

/*************For the ESP32C3 Wroom SOM******************/
#define MCU_SDA GPIO_NUM_7
#define MCU_SCL GPIO_NUM_8

#define MCU_WAKE_DEEPSLEEP GPIO_NUM_3

// Constants for OLED display dimensions.
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32

// Constants for animation delay and text scrolling speed.
#define ANIMATION_DELAY 100
#define SCROLL_SPEED 2

// MCU_ROTARY_ENCODING_B1 moved from IO6 to IO2
#define MCU_ROTARY_ENCODING_A1 GPIO_NUM_21
#define MCU_ROTARY_ENCODING_B1 GPIO_NUM_1
#define MCU_ROTARY_BUTTON GPIO_NUM_10

/*************For ESP-NOW******************/
// Device type (MASTER or SLAVE)
#define DEVICE_TYPE MASTER
// Debug setting (DEBUG_ON or DEBUG_OFF)
#define DEBUG_SETTING DEBUG_ON

// ----- Declare Objects -----

// ----- Declare subroutines and/or functions -----
void readGyroscope();
void printGyroData();
void scrollText(const String &text);
void scanI2CBus();

// ----- Declare Global Variables -----
int GyroX, GyroY, GyroZ;
int AccX, AccY, AccZ;

// Rotary Encoder
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
  setReceivedMessageOnMonitor(true);

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

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Initialize the OLED screen
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // Address 0x3C for 128x32
  display.clearDisplay();

  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setRotation(0);

  // Init the rotary encoder
  pinMode(MCU_ROTARY_ENCODING_A1, INPUT_PULLUP);
  pinMode(MCU_ROTARY_ENCODING_B1, INPUT_PULLUP);
  pinMode(MCU_ROTARY_BUTTON, INPUT_PULLUP);

  // Set the wake/deepsleep switch
  //  Configure the wake-up pin as input
  pinMode(MCU_WAKE_DEEPSLEEP, INPUT);

  esp_deep_sleep_enable_gpio_wakeup(1 << MCU_WAKE_DEEPSLEEP, ESP_GPIO_WAKEUP_GPIO_HIGH);
  gpio_pulldown_en(MCU_WAKE_DEEPSLEEP);
  gpio_pullup_dis(MCU_WAKE_DEEPSLEEP);

  display.setCursor(0, 0);
  display.println("Hello, World!");
  display.display();

  Serial.println("Setup complete");
}

// Main loop
void loop()
{
  //ESPNOW
  checkPairingModeStatus(5000); // Check the pairing mode status every 5 seconds if pairing mode is active.


  // Check the rotary encoder button for a press.
  if ((digitalRead(MCU_ROTARY_BUTTON) == LOW) && (rotaryButtonState == HIGH))
  {
    rotaryButtonCount++;
  }
  rotaryButtonState = digitalRead(MCU_ROTARY_BUTTON);

  printGyroData();

  // Check the rotary encoder for a turn.

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

  display.clearDisplay();
  display.setCursor(0, 0);
  display.print("BTN Count: ");
  display.println(rotaryButtonCount);
  display.print("Rotary Count: ");
  display.println(rotaryTurnCount);
  display.display();

  // //Check if the wake/deepsleep switch is pressed
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

/***********************************Read Gyroscope******************************************/
/// @brief This function will read the gyroscope values and store them in the global variables.
void readGyroscope()
{
  // Read the accelerometer
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

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

/***********************************Print Gyroscope Data******************************************/
/// @brief This function will print the gyroscope values to the OLED screen.
void printGyroData()
{
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* Print out the values */
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");

  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" degC");

  Serial.println("");
  delay(500);
}

/***********************************PID******************************************/
/// @brief This function will do the PID calculations.
void PID_Calc() {  

  //Calculate the angle of inclination using the accelerometer and gyroscope
  //accY needs to be reversed to create the correct front.
  accAngle = atan2(-accY, accZ) * RAD_TO_DEG;
  gyroRate = map(gyroX, -32768, 32767, -250, 250);
  gyroAngle = (float)gyroRate * sampleTime;  
  currentAngle = 0.9934 * (prevAngle + gyroAngle) + 0.0066 * accAngle;
  
  targetAngle = wishedAngle + zeroAngle;

  //Calculate the error, error sum and motor power
  error = currentAngle - targetAngle;
  errorSum += error;   //This += is the same as errorSum = errorSum + error;
  errorSum = constrain(errorSum, -300, 300);
  
  //Calculate the output from P, I and D values
  motorPower = Kp * error + Ki * errorSum * sampleTime - Kd * (currentAngle - prevAngle) / sampleTime;
  prevAngle = currentAngle;

  //DEBUG information
  if(DEBUG_GYRO) {
    Serial.print("Current Angle: ");
    Serial.println(currentAngle);
    Serial.print("Target Angle: ");
    Serial.println(targetAngle);
    Serial.print("Motor Power: ");
    Serial.println(motorPower);
    Serial.println("");
  }
}

/***********************************Scan I2C Bus******************************************/
/// @brief This function will test scan the I2C bus and print the results to the OLED screen.
void scanI2CBus()
{
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Scanning I2C bus...");
  display.display();

  delay(1000);
  display.clearDisplay();
  display.setCursor(0, 0);

  byte error;
  byte address[] = {0, 0, 0, 0};
  int nDevices;

  nDevices = 0;

  // Scan the I2C bus and add the addresses to the address array
  for (byte i = 1; i < 127; i++)
  {
    Wire.beginTransmission(i);
    error = Wire.endTransmission();
    if (error == 0)
    {
      address[nDevices] = i;
      nDevices++;
    }
  }

  // Print the addresses to the OLED screen
  display.setCursor(0, 0);
  display.println("I2C devices found:");
  for (int i = 0; i < nDevices; i++)
  {
    display.print("Device ");
    display.print(i);
    display.print(":0x");
    if (address[i] < 16)
    {
      display.print("0");
    }
    display.println(address[i], HEX);
  }

  // Display the data to the OLED.
  display.display();

  delay(5000);
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
