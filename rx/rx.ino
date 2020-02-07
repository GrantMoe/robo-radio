/*********************************************************************
 This is an example for our nRF51822 based Bluefruit LE modules

 Pick one up today in the adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

#include <string.h>
#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include <Servo.h>

#include "BluefruitConfig.h"

#if SOFTWARE_SERIAL_AVAILABLE
  #include <SoftwareSerial.h>
#endif

/*=========================================================================
    APPLICATION SETTINGS

    FACTORYRESET_ENABLE       Perform a factory reset when running this sketch
   
                              Enabling this will put your Bluefruit LE module
                              in a 'known good' state and clear any config
                              data set in previous sketches or projects, so
                              running this at least once is a good idea.
   
                              When deploying your project, however, you will
                              want to disable factory reset by setting this
                              value to 0.  If you are making changes to your
                              Bluefruit LE device via AT commands, and those
                              changes aren't persisting across resets, this
                              is the reason why.  Factory reset will erase
                              the non-volatile memory where config data is
                              stored, setting it back to factory default
                              values.
       
                              Some sketches that require you to bond to a
                              central device (HID mouse, keyboard, etc.)
                              won't work at all with this feature enabled
                              since the factory reset will clear all of the
                              bonding data stored on the chip, meaning the
                              central device won't be able to reconnect.
    MINIMUM_FIRMWARE_VERSION  Minimum firmware version to have some new features
    MODE_LED_BEHAVIOUR        LED activity, valid options are
                              "DISABLE" or "MODE" or "BLEUART" or
                              "HWUART"  or "SPI"  or "MANUAL"
    -----------------------------------------------------------------------*/
    #define FACTORYRESET_ENABLE         1
    #define MINIMUM_FIRMWARE_VERSION    "0.6.6"
    #define MODE_LED_BEHAVIOUR          "MODE"
/*=========================================================================*/

// Create the bluefruit object, either software serial...uncomment these lines
/*
SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                      BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);
*/

/* ...or hardware serial, which does not need the RTS/CTS pins. Uncomment this line */
// Adafruit_BluefruitLE_UART ble(BLUEFRUIT_HWSERIAL_NAME, BLUEFRUIT_UART_MODE_PIN);

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

/* ...software SPI, using SCK/MOSI/MISO user-defined SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO,
//                             BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS,
//                             BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);


// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

// function prototypes over in packetparser.cpp
uint8_t readPacket(Adafruit_BLE *ble, uint16_t timeout);
float parsefloat(uint8_t *buffer);
void printHex(const uint8_t * data, const uint32_t numBytes);

// the packet buffer
extern uint8_t packetbuffer[];

// PWM demo
bool moving_forward = true;
bool moving_reverse = false;

// 2 is forward
// 4 is backward
const int BUTTON_1 = 1;
const int BUTTON_FORWARD = 2;
const int BUTTON_3 = 3;
const int BUTTON_REVERSE = 4;
const int ARROW_UP = 5;
const int ARROW_DN = 6;
const int ARROW_LT = 7;
const int ARROW_RT = 8;
//
//const int THROTTLE_STOP = 1500;
//const int THROTTLE_FORWARD = 2000;
//const int THROTTLE_REVERSE = 1000;
//
//const int STEERING_MID = 1500;
//const int STEERING_LEFT = 1000;
//const int STEERING_RIGHT = 2000;
//
//const int DEFAULT_THROTTLE = 1500;
//const int DEFAULT_STEERING = 1500;
//
//const int PWM_MIN = 1000;
//const int PWM_MID = 1500;
//const int PWM_MAX = 2000;
//
//int pwm_throttle = 1500;
//int pwm_steering = 1500;
//int pwm_switch_1 = 1500;
//int pwm_switch_2 = 1500;

//int throttle_percent = 0;
//bool forward = true;
//int steering_percent = 0;

int throttle_pwm = 1500;
int steering_pwm = 1500;

Servo servo_esc;
Servo servo_front;
Servo servo_rear;

const int pin_esc = 11;
const int pin_front = 10;
const int pin_rear = 9;

// button pressed boolean
//int pressed[8] = {0, 0, 0, 0, 0, 0, 0, 0};
bool isPressed[9] = {false, false, false, false, false, false, false, false, false};

uint8_t len = 0;

/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/
void setup(void)
{
//  while (!Serial);  // required for Flora & Micro
//  delay(500);

  Serial.begin(115200);
  Serial.println(F("Adafruit Bluefruit App Controller Example"));
  Serial.println(F("-----------------------------------------"));

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ){
      error(F("Couldn't factory reset"));
    }
  }


  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  Serial.println(F("Please use Adafruit Bluefruit LE app to connect in Controller mode"));
  Serial.println(F("Then activate/use the sensors, color picker, game controller, etc!"));
  Serial.println();

  ble.verbose(false);  // debug info is a little annoying after this point!

  /* Wait for connection */
  while (! ble.isConnected()) {
      delay(500);
  }

  Serial.println(F("******************************"));

  // LED Activity command is only supported from 0.6.6
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    // Change Mode LED Activity
    Serial.println(F("Change LED activity to " MODE_LED_BEHAVIOUR));
    ble.sendCommandCheckOK("AT+HWModeLED=" MODE_LED_BEHAVIOUR);
  }

  // Set Bluefruit to DATA mode
  Serial.println( F("Switching to DATA mode!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);

  Serial.println(F("******************************"));

  // Servos etc.
  servo_esc.attach(pin_esc);
  servo_esc.write(90);
  servo_front.attach(pin_front);
  servo_front.write(90);
  servo_rear.attach(pin_rear);
  servo_rear.write(90);

}

/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
void loop(void)
{
  if (ble.available()) {
    Serial.println("Available!");
  
  /* Wait for new data to arrive */
    len = readPacket(&ble, BLE_READPACKET_TIMEOUT);
  //  if (len == 0) return;
    if (len > 0) {
        // Buttons
      if (packetbuffer[1] == 'B') {
        uint8_t buttnum = packetbuffer[2] - '0';
        boolean pressed = packetbuffer[3] - '0';
        Serial.print ("Button "); Serial.print(buttnum);
        if (pressed) {
          isPressed[buttnum] = true;
          Serial.println(" pressed");
        } else {
          isPressed[buttnum] = false;
          Serial.println(" released");
        }
      }
    }
  }

  /* Got a packet! */
//   printHex(packetbuffer, len);


  if (isPressed[BUTTON_1] == true) {
    Serial.println("Steering centered");
//    steering_percent = 0;
    steering_pwm = 1500;
    isPressed[BUTTON_1] = false;
  }
  
  if (isPressed[BUTTON_FORWARD] == true) { // aka button 2
    Serial.println("Forward!");
    if (throttle_pwm < 1500) {
      throttle_pwm = 1500;
      isPressed[BUTTON_FORWARD] = false;
    } else {
      if (throttle_pwm == 1500) {
        throttle_pwm = 1750;
      }
    }
//    
//    if (moving_reverse) {
//      moving_reverse = false;
//      throttle_percent = 0;
//    } else {
//      moving_forward = true;
//      if (throttle_percent == 0) {
//        throttle_percent = 50;
//      }
//    }
  }
  
  if (isPressed[BUTTON_3] == true) {
  }

  if (isPressed[BUTTON_REVERSE] == true) { // button 4
    Serial.println("Reverse!");
    if (throttle_pwm > 1500) {
      throttle_pwm = 1500;
      isPressed[BUTTON_REVERSE] = false;
//    if (moving_forward) {
//      moving_forward = false;
//      throttle_percent = 0;
    } else {
      if (throttle_pwm == 0) {
        throttle_pwm = 1250;
      }
//      moving_reverse = true;
//      if (throttle_percent == 0) {
//        throttle_percent = -50;
//      }
    }
  }

  if (isPressed[ARROW_UP] == true) { // button 5
    Serial.println("Throttling up!");
    if (throttle_pwm >= 1500) {
      throttle_pwm += 1;
      if (throttle_pwm > 2000) {
        throttle_pwm = 2000;
      }
    } else {
      throttle_pwm -= 1;
      if (throttle_pwm < 1000) {
        throttle_pwm = 1000;
      }
    }

//    
//    if (throttle_percent >= 0) {
//      throttle_percent += 10;
//      if (throttle_percent > 100) {
//        throttle_percent = 100;
//      }
//    } else {
//      throttle_percent -= 10;
//      if (throttle_percent < -100) {
//        throttle_percent = -100;
//      }
//    }
  }

  if (isPressed[ARROW_DN] == true) { // button 6
    Serial.println("Following down!");
    if (throttle_pwm >= 1500) {
      throttle_pwm -= 1;
      if (throttle_pwm < 1550) {
        throttle_pwm = 1550; 
      } else {
        throttle_pwm += 1;
        if (throttle_pwm > 1450) {
          throttle_pwm = 1450;
        }
      }
    }
//    if (throttle_percent >= 0) {
//      throttle_percent -= 10;
//      if (throttle_percent < 10) {
//        throttle_percent = 10;
//      }
//    } else {
//      throttle_percent += 10;
//      if (throttle_percent > -10) {
//        throttle_percent = -10;
//      }
//    }
  }

  if (isPressed[ARROW_LT] == true) {    // button 7
    Serial.println("TURNING LEFT");
    steering_pwm -= 1;
    if (steering_pwm < 1000) {
      steering_pwm = 1000;
    }
//    steering_percent -= 1;
//    if (steering_percent < -100) {
//      steering_percent = -100;
//    }
  }

  if (isPressed[ARROW_RT] == true) {    // button 8
    Serial.println("TURNING RIGHT");
    steering_pwm += 1;
    if (steering_pwm > 2000) {
      steering_pwm = 2000;
    }
//    steering_percent += 1;
//    if (steering_percent > 100) {
//      steering_percent = 100;
//    }
  }
  

  // map(value, fromLow, fromHigh, toLow, toHigh)
//  servo_esc.write(map(throttle_percent, -100, 100, 0, 180));
//  servo_front.write(map(steering_percent, -100, 100, 0, 180));
//  servo_rear.write(map(steering_percent, -100, 100, 180, 0));

//  servo_esc.writeMicroseconds(map(throttle_percent, -100, 100, 1000, 2000));
//  servo_front.writeMicroseconds(map(steering_percent, -100, 100, 1000, 2000));
//  servo_rear.writeMicroseconds(map(steering_percent, -100, 100, 2000, 1000));

  servo_esc.writeMicroseconds(throttle_pwm);
  servo_front.writeMicroseconds(steering_pwm);
  servo_rear.writeMicroseconds(map(steering_pwm, 1000, 2000, 2000, 1000));

  
}

int percentToPWM(int percent) {
  return 1500 + (percent * 5);  
}
