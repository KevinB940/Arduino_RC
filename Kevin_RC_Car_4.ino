/*
   This is a modified version of the PS3BT.ino example sketch by Kristian Lauszus
  For more information visit his blog: http://blog.tkjelectronics.dk/ or
  send him an e-mail:  kristianl@tkjelectronics.com
*/

#include <PS3BT.h> // Include the PS3BT library: https://github.com/felis/USB_Host_Shield_2.0
#include <EEPROM.h> // Include the EEPROM library
#include <Servo.h>

USB Usb;
BTD Btd(&Usb);
PS3BT PS3(&Btd);

Servo servo1; // Create instances of type Servo. servo1 is the steering servo on  digital pin (5).
Servo servo2; // Create instances of type Servo. servo2 is the ESC on digital pin (4).

#define ESC_MIN 1000
#define ESC_MAX 2000
#define ESC_MID (ESC_MAX - ESC_MIN)/2 + ESC_MIN

#define SERVO_MIN 800
#define SERVO_MAX 2200
#define SERVO_MID (SERVO_MAX - SERVO_MIN)/2 + SERVO_MIN

#define MAGIC_VALUE 0xAA

static float sensitivity = 1.0f; // Used to adjust the sensitivity of the throttle
static int8_t trim; // Used to trim the steering wheel
static uint32_t timer;


void setup() {
  Serial.begin(57600);

  servo1.attach(5); // Steering servo on digital pin 5
  servo2.attach(4); // ESC on signal pin 4
  
  if (Usb.Init() == -1) {
    Serial.println(F("\r\nOSC did not start"));
    while (1); // Halt
  }
  uint8_t magicValue;
  EEPROM.get(0, magicValue);
  if (magicValue = MAGIC_VALUE)
    EEPROM.get(1, trim); // Get trim value from the EEPROM
  else {
    magicValue = MAGIC_VALUE;
    trim = 0;
    EEPROM.put(0, magicValue);
    EEPROM.put(1, trim);
  }
  PS3.attachOnInit(updateLeds); // Set LEDs according to current sensitivity value upon a new connection
  Serial.println(F("\r\nSender started"));
  timer = millis();
}

// 0 to 1/7 all will be off
// 1/7 to 2/7 LED1 will be on
// 2/7 to 3/7 LED1 and LED2 will be on
// 3/7 to 4/7 LED2 will be on
// 4/7 to 5/7 LED2 and LED3 will be on
// 5/7 to 6/7 LED3 will be on
// 6/7 to 7/7 LED3 and LED4 will be on
// At exactly 7/7=1 only LED4 will be on
void updateLeds() {
  //Serial.println(sensitivity, 3);
  LEDEnum led1, led2;
  if (sensitivity < .14f) { // ~1/7
    led1 = OFF;
    led2 = OFF;
  } else if (sensitivity < .28f) { // ~2/7
    led1 = LED1;
    led2 = OFF;
  } else if (sensitivity < .42f) { // ~3/7
    led1 = LED1;
    led2 = LED2;
  } else if (sensitivity < .57f) { // ~4/7
    led1 = LED2;
    led2 = OFF;
  } else if (sensitivity < .71f) { // ~5/7
    led1 = LED2;
    led2 = LED3;
  } else if (sensitivity < .85f) { // ~6/7
    led1 = LED3;
    led2 = OFF;
  } else if (sensitivity < 1.0f) { // ~7/7
    led1 = LED3;
    led2 = LED4;
  } else { // Equal to 1
    led1 = LED4;
    led2 = OFF;
  }
  PS3.setLedRaw(pgm_read_byte(&PS3_LEDS[led1]) | pgm_read_byte(&PS3_LEDS[led2]));
}









void loop()
{
  Usb.Task();

  if (PS3.PS3Connected && millis() - PS3.getLastMessageTime() < 100)
  {

    
      if (PS3.getButtonClick(UP) && sensitivity < 1) {
        sensitivity += 1.0f / 7.0f; // Add 1/7
        sensitivity = constrain(sensitivity, 0.0f, 1.0f); // Due to the nature of floating point which might just be very close to 1
                                                          // this small fix is needed to make sure that it does not exceed 1
        updateLeds();
        
     }  else if (PS3.getButtonClick(DOWN) && sensitivity > 0) {
        sensitivity -= 1.0f / 7.0f; // Subtract 1/7
        sensitivity = constrain(sensitivity, 0.0f, 1.0f); // Similar reason as above, but just make sure that is it not negative
        updateLeds();
      }

    if (PS3.getButtonClick(RIGHT) && trim < 90) {
        trim++;
        EEPROM.put(1, trim); // Write value to EEPROM
      }
      
    else if (PS3.getButtonClick(LEFT) && trim > -90) {
        trim--;
        EEPROM.put(1, trim); // Write value to EEPROM
      } 
      

    else if (PS3.getButtonClick(SELECT)) {
        trim = 0; // Reset trim value
        EEPROM.put(1, trim); // Write value to EEPROM
      }












    
    uint16_t steering = map(PS3.getAnalogHat(RightHatX), 0, 255, SERVO_MIN, SERVO_MAX);
    steering = constrain(steering + trim * (SERVO_MAX - SERVO_MIN) / 180, SERVO_MIN, SERVO_MAX); // Apply trim value





      servo1.writeMicroseconds(steering);
      servo2.writeMicroseconds(map((PS3.getAnalogHat(LeftHatY) - 127) * sensitivity, -127, 128, ESC_MAX, ESC_MIN));



    
        
    if (PS3.getButtonClick(PS)) {
        servo1.writeMicroseconds(SERVO_MID); // Center steering servo
        servo2.writeMicroseconds(ESC_MID);  // Return ESC to Netrual postion
        PS3.disconnect();
      }


  
      }
    else
      {
    servo1.writeMicroseconds(SERVO_MID); // Center steering servo
    servo2.writeMicroseconds(ESC_MID);  // Return ESC to Netrual postion
    }
 }






























































