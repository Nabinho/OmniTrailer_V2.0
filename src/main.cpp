// Arduino Framework Library
#include <Arduino.h>

/***********************************************************************************************************************
 *
 * Written by Nabinho - 2023
 *
 *  Mecanum Wheel Reference - https://en.wikipedia.org/wiki/Omni_wheel
 *
 *  Hardware Reference:
 *    - RoboCore Vespa - https://www.robocore.net/placa-robocore/vespa
 *
 **********************************************************************************************************************/

// Libraries
#include <SPI.h>
#include "RF24.h"
#include "printf.h"
#include <RoboCore_Vespa.h>
#include <Adafruit_NeoPixel.h>

// Vespa Control Objects
VespaMotors motors;
VespaBattery battery;

// ESP32 SPI Pins
const uint8_t MY_MISO = 19;
const uint8_t MY_MOSI = 23;
const uint8_t MY_SCLK = 18;
const uint8_t MY_SS = 5;

// Radio Controller Object
RF24 radio(16, 5);
SPIClass *hspi = nullptr;

// Radios Addresses
uint8_t address[][6] = {"Ctrlr", "Robot"};

// Radio Number
const bool radio_number = 1;

// Variables structure
typedef struct
{
  uint8_t button1_reading;
  uint8_t button2_reading;
  uint8_t button3_reading;
  uint8_t button4_reading;
  uint8_t button5_reading;
  uint8_t button6_reading;
  uint16_t X1axis_reading;
  uint16_t Y1axis_reading;
  uint16_t X2axis_reading;
  uint16_t Y2axis_reading;
  uint16_t slider1_reading;
  uint16_t slider2_reading;
} controller_variables;
controller_variables controller;

// Variables for Message Receptions
uint8_t channel;
uint8_t bytes;

// WS2812B LEDs Module Control Variables
const uint8_t PIN_LED1 = 25;
const uint8_t PIN_LED2 = 26;
const uint8_t NUMBER_LED = 4;

// LEDs Modules Control Objects
Adafruit_NeoPixel LED_LEFT(NUMBER_LED, PIN_LED1, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel LED_RIGHT(NUMBER_LED, PIN_LED2, NEO_GRB + NEO_KHZ800);

// Variables For Failsafe
unsigned long last_message = 0;
const uint16_t FAILSAFE_INTERVAL = 2000;

// Button Reading Variables
bool reading_button1;
bool button1_state;
bool last_button1_state = 0;
unsigned long last_debounce_time1;

// Button Reading Variables
bool reading_button2;
bool button2_state;
bool last_button2_state = 0;
unsigned long last_debounce_time2;

// Button Reading Variables
bool reading_button3;
bool button3_state;
bool last_button3_state = 0;
unsigned long last_debounce_time3;

// Button Reading Variables
bool reading_button4;
bool button4_state;
bool last_button4_state = 0;
unsigned long last_debounce_time4;

// Button Reading Variables
bool reading_button5;
bool button5_state;
bool last_button5_state = 0;
unsigned long last_debounce_time5;

// Button Reading Variables
bool reading_button6;
bool button6_state;
bool last_button6_state = 0;
unsigned long last_debounce_time6;

// Button Debounce
const uint8_t DEBOUNCE_TIME = 100;
uint16_t speed_max = 0;
const uint16_t speed_min = 65;
uint16_t horizontal = 0;

// Lights Control Variables
bool mode = true;
bool back_light = false;
bool enable_blink = false;
bool blink_right = false;
bool blink_left = false;

// Light Blink Variables
bool blink = true;
unsigned long blink_time = 0;
const uint16_t BLINK_INTERVAL = 500;

// Lights Colors Variables
const uint16_t RED_LIGHT[3] = {255, 0, 0};
const uint16_t ORANGE_LIGHT[3] = {255, 175, 0};

// LED_BUILTIN pin
const uint8_t LED_PIN = 15;

// Bluetooth message variable
char BT_message;

// Battery reading variables
unsigned long low_battery_time = 0;
const uint16_t BATTERY_FAILSAFE = 500;

//----------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------

// Function to Control the Robot Lights
void handle_lights(bool back, bool right, bool left)
{

  // Clear LEDs
  LED_RIGHT.clear();
  LED_LEFT.clear();

  // Blink Counter
  if (right || left)
  {
    if ((millis() - blink_time) > BLINK_INTERVAL)
    {
      blink = !blink;
      blink_time = millis();
    }
  }

  //********************************************************************************************************************
  // Control Back LEDs
  if (back)
  {
    for (uint8_t i = 0; i < NUMBER_LED; i++)
    {
      LED_LEFT.setPixelColor(i, LED_LEFT.Color(RED_LIGHT[0], RED_LIGHT[1], RED_LIGHT[2]));
      LED_RIGHT.setPixelColor(i, LED_RIGHT.Color(RED_LIGHT[0], RED_LIGHT[1], RED_LIGHT[2]));
    }
    if (right)
    {
      if (blink)
      {
        for (uint8_t i = 0; i < NUMBER_LED; i++)
        {
          LED_RIGHT.setPixelColor(i, LED_RIGHT.Color(ORANGE_LIGHT[0], ORANGE_LIGHT[1], ORANGE_LIGHT[2]));
        }
      }
      else
      {
        for (uint8_t i = 0; i < NUMBER_LED; i++)
        {
          LED_RIGHT.setPixelColor(i, LED_RIGHT.Color(0, 0, 0));
        }
      }
    }
    if (left)
    {
      if (blink)
      {
        for (uint8_t i = 0; i < NUMBER_LED; i++)
        {
          LED_LEFT.setPixelColor(i, LED_LEFT.Color(ORANGE_LIGHT[0], ORANGE_LIGHT[1], ORANGE_LIGHT[2]));
        }
      }
      else
      {
        for (uint8_t i = 0; i < NUMBER_LED; i++)
        {
          LED_LEFT.setPixelColor(i, LED_LEFT.Color(0, 0, 0));
        }
      }
    }
  }
  if (right)
  {
    if (blink)
    {
      for (uint8_t i = 0; i < NUMBER_LED; i++)
      {
        LED_RIGHT.setPixelColor(i, LED_RIGHT.Color(ORANGE_LIGHT[0], ORANGE_LIGHT[1], ORANGE_LIGHT[2]));
      }
    }
    else
    {
      for (uint8_t i = 0; i < NUMBER_LED; i++)
      {
        LED_RIGHT.setPixelColor(i, LED_RIGHT.Color(0, 0, 0));
      }
    }
  }
  if (left)
  {
    if (blink)
    {
      for (uint8_t i = 0; i < NUMBER_LED; i++)
      {
        LED_LEFT.setPixelColor(i, LED_LEFT.Color(ORANGE_LIGHT[0], ORANGE_LIGHT[1], ORANGE_LIGHT[2]));
      }
    }
    else
    {
      for (uint8_t i = 0; i < NUMBER_LED; i++)
      {
        LED_LEFT.setPixelColor(i, LED_LEFT.Color(0, 0, 0));
      }
    }
  }

  // Updates LEDs Control
  LED_RIGHT.show();
  LED_LEFT.show();
}

//----------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------

void setup()
{

  // Serial monitor initialization
  Serial.begin(115200);

  // LED_BUILTIN initialization
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  LED_LEFT.begin();
  LED_LEFT.clear();
  LED_LEFT.show();
  LED_RIGHT.begin();
  LED_RIGHT.clear();
  LED_RIGHT.show();

  motors.stop();

  // SPI bus configuration
  hspi = new SPIClass(HSPI);
  hspi->begin(MY_SCLK, MY_MISO, MY_MOSI, MY_SS);

  // Radio Initialization
  if (!radio.begin(hspi))
  {
    Serial.println("Radio Initialization Failed!");
    while (!radio.begin(hspi))
    {
      Serial.print(F("."));
    }
  }

  // Configure Radio for Maximum Power
  radio.setPALevel(RF24_PA_MAX);

  // Configure Radio Payload Size
  radio.setPayloadSize(sizeof(controller));

  // Configure Radio Listening Pipe
  radio.openWritingPipe(address[radio_number]);

  // Configure Radio Channel Number
  radio.openReadingPipe(1, address[!radio_number]);

  // Configure Radio to Listen for Incoming Data
  radio.startListening();
}

//----------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------

void loop()
{

  if (battery.readVoltage() > 7000)
  {

    // Updates Battery Timeout
    low_battery_time = millis();

    // Checks If New Reading Available
    if (radio.available(&channel))
    {
      // Reads Messages From Controller
      bytes = radio.getPayloadSize();
      radio.read(&controller, bytes);

      // Updates Lest Message Time
      last_message = millis();
      digitalWrite(LED_PIN, HIGH);

      //********************************************************************************************************************
      // Mode Control Changer
      reading_button1 = controller.button1_reading;
      if (reading_button1 != last_button1_state)
      {
        last_debounce_time1 = millis();
      }
      if ((millis() - last_debounce_time1) > DEBOUNCE_TIME)
      {
        if (reading_button1 != button1_state)
        {
          button1_state = reading_button1;
          if (button1_state == 1)
          {
            mode = false;
          }
          else
          {
            mode = true;
          }
        }
      }
      last_button1_state = reading_button1;

      //********************************************************************************************************************
      // Light Blink Enabling Changer
      reading_button2 = controller.button2_reading;
      if (reading_button2 != last_button2_state)
      {
        last_debounce_time2 = millis();
      }
      if ((millis() - last_debounce_time2) > DEBOUNCE_TIME)
      {
        if (reading_button2 != button2_state)
        {
          button2_state = reading_button2;
          if (button2_state == 1)
          {
            enable_blink = true;
          }
          else
          {
            enable_blink = false;
          }
        }
      }
      last_button2_state = reading_button2;

      //********************************************************************************************************************
      // Control the Front Light
      reading_button3 = controller.button3_reading;
      if (reading_button3 != last_button3_state)
      {
        last_debounce_time3 = millis();
      }
      if ((millis() - last_debounce_time3) > DEBOUNCE_TIME)
      {
        if (reading_button3 != button3_state)
        {
          button3_state = reading_button3;
          if (button3_state == 1)
          {
            back_light = true;
          }
          else
          {
            back_light = false;
          }
        }
      }
      last_button3_state = reading_button3;

      //********************************************************************************************************************
      // Speed Max Adjustment
      speed_max = map(((controller.slider1_reading + controller.slider2_reading) / 2), 1023, 0, speed_min, 100);

      //********************************************************************************************************************
      // Handle the Robot Control when the Joysticks Heads Forward
      if (controller.Y2axis_reading > 550)
      {
        if (controller.X1axis_reading > 550)
        {
          if (enable_blink)
          {
            blink_right = false;
            blink_left = true;
          }
          horizontal = map(controller.X1axis_reading, 550, 1023, speed_min, speed_max);
          if (mode)
          {
            motors.setSpeedRight(horizontal);
          }
          else
          {
            motors.setSpeedRight(-horizontal);
          }
        }
        // Handle the Control when the Joysticks Heads Left
        else if (controller.X1axis_reading < 500)
        {
          if (enable_blink)
          {
            blink_right = true;
            blink_left = false;
          }
          horizontal = map(controller.X1axis_reading, 500, 0, speed_min, speed_max);
          if (mode)
          {
            motors.setSpeedRight(-horizontal);
          }
          else
          {
            motors.setSpeedRight(horizontal);
          }
        }
        else
        {
          blink_right = false;
          blink_left = false;
          motors.stop();
        }
      }

      //********************************************************************************************************************
      // Handle the Robot Control when the Joysticks Heads Backward
      if (controller.Y2axis_reading < 500)
      {
        if (controller.X1axis_reading > 550)
        {
          if (enable_blink)
          {
            blink_right = false;
            blink_left = true;
          }
          horizontal = map(controller.X1axis_reading, 550, 1023, speed_min, speed_max);
          if (mode)
          {
            motors.setSpeedRight(-horizontal);
          }
          else
          {
            motors.setSpeedRight(horizontal);
          }
        }
        else if (controller.X1axis_reading < 500)
        {
          if (enable_blink)
          {
            blink_right = true;
            blink_left = false;
          }
          horizontal = map(controller.X1axis_reading, 500, 0, speed_min, speed_max);
          if (mode)
          {
            motors.setSpeedRight(horizontal);
          }
          else
          {
            motors.setSpeedRight(-horizontal);
          }
        }
        else
        {
          blink_right = false;
          blink_left = false;
          motors.stop();
        }
      }

      //********************************************************************************************************************
      // Handle the Control when the Joysticks Heads Left
      else if (controller.X1axis_reading > 550)
      {
        if (enable_blink)
        {
          blink_right = false;
          blink_left = true;
        }
        horizontal = map(controller.X1axis_reading, 550, 1023, speed_min, speed_max);
        if (mode)
        {
          motors.setSpeedRight(horizontal);
        }
        else
        {
          motors.setSpeedRight(-horizontal);
        }
      }

      //********************************************************************************************************************
      // Handle the Control when the Joysticks Heads Left
      else if (controller.X1axis_reading < 500)
      {
        if (enable_blink)
        {
          blink_right = true;
          blink_left = false;
        }
        horizontal = map(controller.X1axis_reading, 500, 0, speed_min, speed_max);
        if (mode)
        {
          motors.setSpeedRight(-horizontal);
        }
        else
        {
          motors.setSpeedRight(horizontal);
        }
      }

      //********************************************************************************************************************
      else
      {
        blink_right = false;
        blink_left = false;
        motors.stop();
      }

      // Function to Handle the Lights Control
      handle_lights(back_light, blink_right, blink_left);
    }

    //********************************************************************************************************************
    // Handle Failsafe
    else if ((millis() - last_message) > FAILSAFE_INTERVAL)
    {
      digitalWrite(LED_PIN, LOW);
      enable_blink = false;
      back_light = false;
      handle_lights(back_light, blink_right, blink_left);
      motors.stop();
    }
  }
  //********************************************************************************************************************
  // Handle Low Battery
  else if((millis() - low_battery_time) > BATTERY_FAILSAFE)
  {
    digitalWrite(LED_PIN, LOW);
    enable_blink = false;
    back_light = false;
    handle_lights(back_light, blink_right, blink_left);
    motors.stop();
  }
}

//----------------------------------------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------------------------------------