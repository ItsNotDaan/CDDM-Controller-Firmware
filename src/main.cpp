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

// #define DEBUG 1
#define DEBUG GPIO_NUM_1

/*************For the ESP32C3 Wroom SOM******************/
//SDA moved from IO8 to IO4
//SCL moved from IO7 to IO5
// #define MCU_SDA 7
// #define MCU_SCL 8

#define MCU_SDA GPIO_NUM_7
#define MCU_SCL GPIO_NUM_8


#define MCU_WAKE_DEEPSLEEP GPIO_NUM_3

// Constants for OLED display dimensions.
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32

// Constants for animation delay and text scrolling speed.
#define ANIMATION_DELAY 100
#define SCROLL_SPEED 2

//MCU_ROTARY_ENCODING_B1 moved from IO6 to IO2
#define MCU_ROTARY_ENCODING_A1 7
#define MCU_ROTARY_ENCODING_B1 2
#define MCU_ROTARY_BUTTON 10

/*************For ESP-NOW******************/
// Device type (MASTER or SLAVE)
#define DEVICE_TYPE MASTER
// Debug setting (DEBUG_ON or DEBUG_OFF)
#define DEBUG_SETTING DEBUG_ON


// ----- Declare Objects -----

// ----- Declare subroutines and/or functions -----
void readGyroscope();
void printGyroData();
void scrollText(const String& text);
void scanI2CBus();

// ----- Declare Global Variables -----
int GyroX, GyroY, GyroZ;
int AccX, AccY, AccZ;

//Rotary Encoder
int rotaryCount = 0; 
bool rotaryButtonState = false;


// Setup
void setup()
{
  // Serial.begin(9600); // Start the serial monitor at 115200 baud

  //Wire
  Wire.begin(MCU_SDA, MCU_SCL, 1000000); // Start the I2C communication

  //ESPNOW
  // initESPNOW(DEVICE_TYPE, DEBUG_SETTING);
  // // if (initESPNOW(DEVICE_TYPE, DEBUG_SETTING) == false)
  // // {
  // //   Serial.println("ESP-NOW initialization failed");
  // //   ESP.restart();
  // // }

  // startPairingProcess();
  // setReceivedMessageOnMonitor(true);
  
  //Initialize the MPU6050 and set offset values
  mpu.begin();

  //Initialize the OLED screen
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // Address 0x3C for 128x32
  display.clearDisplay();

  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setRotation(0);

  //Init the rotary encoder
  pinMode(MCU_ROTARY_ENCODING_A1, INPUT);
  pinMode(MCU_ROTARY_ENCODING_B1, INPUT);
  pinMode(MCU_ROTARY_BUTTON, INPUT_PULLUP);

  //Set the wake/deepsleep switch
  // Configure the wake-up pin as input
  pinMode(MCU_WAKE_DEEPSLEEP, INPUT);

  // pinMode(MCU_SDA, OUTPUT);
  // pinMode(MCU_SCL, OUTPUT);
  pinMode(DEBUG, OUTPUT);

  esp_deep_sleep_enable_gpio_wakeup(1 << MCU_WAKE_DEEPSLEEP, ESP_GPIO_WAKEUP_GPIO_HIGH);
  gpio_pulldown_en(MCU_WAKE_DEEPSLEEP);  
  gpio_pullup_dis(MCU_WAKE_DEEPSLEEP);

  display.setCursor(0, 0);
  display.println("Hello, World!");
  display.display();
}

// Main loop
void loop()
{
  if ((digitalRead(MCU_ROTARY_BUTTON) == LOW) && (rotaryButtonState == false)){

    rotaryButtonState = true;
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Button pressed!");
    display.print("Rotary Count: ");
    display.println(rotaryCount);
    display.display();
    
    rotaryCount++;
  }
  else if (digitalRead(MCU_ROTARY_BUTTON) == HIGH){
    rotaryButtonState = false;
  }

  // readGyroscope();
  // printGyroData();
  // testI2CPins();

  // scanI2CBus();

  // //Check if the wake/deepsleep switch is pressed
  if (digitalRead(MCU_WAKE_DEEPSLEEP) == LOW){
    // Serial.println("Going to sleep");
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("Going to sleep!");
    display.display();
    delay(1000);

    display.clearDisplay(); 
    display.display();

    esp_deep_sleep_start();
  }

  
}



/***********************************Read Gyroscope******************************************/
/// @brief This function will read the gyroscope values and store them in the global variables.
void readGyroscope() {
  //Read the accelerometer
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
void printGyroData() {
  display.clearDisplay();
  display.setCursor(0, 0);

  display.println("Accelerometer - m/s^2");
  display.print(AccX, 1);
  display.print(", ");
  display.print(AccY, 1);
  display.print(", ");
  display.print(AccZ, 1);
  display.println("");

  display.println("Gyroscope - rps");
  display.print(GyroX, 1);
  display.print(", ");
  display.print(GyroY, 1);
  display.print(", ");
  display.print(GyroZ, 1);
  display.println("");

  display.display();
  delay(100);
}


/***********************************Scroll Text******************************************/
/// @brief This function will print the gyroscope values to the OLED screen.
// / @param text The text to be scrolled.
void scrollText(const String& text) {
  int textWidth = text.length() * 6; // Each character is 6 pixels wide
  int xPos = SCREEN_WIDTH;

  while (xPos > -textWidth) {
    display.clearDisplay();
    display.setCursor(xPos, (SCREEN_HEIGHT - 8) / 2);
    display.println(text);
    display.display();
    delay(ANIMATION_DELAY);
    xPos -= SCROLL_SPEED;
  }
}


/***********************************Scan I2C Bus******************************************/
/// @brief This function will test scan the I2C bus and print the results to the OLED screen.
void scanI2CBus() {
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Scanning I2C bus...");
  display.display();

  delay(1000);
  display.clearDisplay();
  display.setCursor(0,0); 

  byte error;
  byte address[] = {0, 0, 0, 0};
  int nDevices;

  nDevices = 0;

  //Scan the I2C bus and add the addresses to the address array
  for (byte i = 1; i < 127; i++) {
    Wire.beginTransmission(i);
    error = Wire.endTransmission();
    if (error == 0) {
      address[nDevices] = i;
      nDevices++;
    }
  }

  //Print the addresses to the OLED screen
  display.setCursor(0, 0);
  display.println("I2C devices found:");
  for (int i = 0; i < nDevices; i++) {
    display.print("Device ");
    display.print(i);
    display.print(":0x");
    if (address[i] < 16) {
      display.print("0");
    }
    display.println(address[i], HEX);
  }

  //Display the data to the OLED.
  display.display();

  delay(5000);
}



/***********************************Menu******************************************/
/// @brief This function will display a menu on the OLED screen and navigate through it using the rotary encoder.
void displayMenu() {
  int menuIndex = 0;
  int lastMenuIndex = -1;

  while (true) {
    // Read the rotary encoder
    int newMenuIndex = menuIndex; // Replace this with actual rotary encoder reading logic

    if (newMenuIndex != lastMenuIndex) {
      lastMenuIndex = newMenuIndex;

      display.clearDisplay();
      display.setCursor(0, 0);

      switch (newMenuIndex) {
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
