/*
  This is a modified version of the PS3BT.ino example sketch by Kristian Lauszus
  For more information visit his blog: http://blog.tkjelectronics.dk/ or
  send him an e-mail:  kristianl@tkjelectronics.com
*/

#include <PS3BT.h> // Include the PS3BT library: https://github.com/felis/USB_Host_Shield_2.0
#include <EEPROM.h> // Include the EEPROM library
#include <SPI.h> // Include the SPI library needed by the USB Host Shield library

#define N_CHANNELS 4 // Set the number of PPM channels here - I set it to four as this is the minimum my RC transmitter can decode
#define PPM_FRAME_LEN (N_CHANNELS * 2000 + 6500) // The PPM frame length in us, seems to be the standard equation for calculating it
#define PPM_PULSE_LEN 300 // The PPM positive pulse length in us

USB Usb;
BTD Btd(&Usb);
PS3BT PS3(&Btd);

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


USB Usb;
PS3BT srw1(&Usb);

static const uint8_t signalPin = 5; // Set PPM signal output pin
//static int8_t trim; // Used to trim the steering wheel
static volatile uint16_t ppm[N_CHANNELS]; // This array holds the servo values for the PPM signal
static uint8_t counterToUsScaleFactor; // Used to convert us values into values used for the counter


// These are used to read and write to the port registers - see http://www.arduino.cc/en/Reference/PortManipulation
// I do this to save processing power - see this page for more information: http://www.billporter.info/ready-set-oscillate-the-fastest-way-to-change-arduino-pins/
static volatile uint8_t pinBitMask, *pinOutPort;







void setup() {
  Serial.begin(57600);
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

void sendCommand(uint16_t steering, uint16_t forwardBackward) {
  Serial.write('C'); // Send header
  Serial.print(steering);
  Serial.write(',');
  Serial.print(forwardBackward);
  Serial.write(',');
  Serial.print((uint16_t)(steering ^ forwardBackward)); // Send a simple checksum
  Serial.write(';');
  Serial.flush(); // Wait until data is sent
  /*Serial.print(steering);
    Serial.write('\t');
    Serial.println(forwardBackward);*/
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

void loop() {
  Usb.Task();

  if (millis() - timer > 10) { // Limit the serial output frequency to 10 ms
    timer = millis();
    if (PS3.PS3Connected && millis() - PS3.getLastMessageTime() < 100) {
      if (PS3.getButtonClick(UP) && sensitivity < 1) {
        sensitivity += 1.0f / 7.0f; // Add 1/7
        sensitivity = constrain(sensitivity, 0.0f, 1.0f); // Due to the nature of floating point which might just be very close to 1
                                                          // this small fix is needed to make sure that it does not exceed 1
        updateLeds();
      } else if (PS3.getButtonClick(DOWN) && sensitivity > 0) {
        sensitivity -= 1.0f / 7.0f; // Subtract 1/7
        sensitivity = constrain(sensitivity, 0.0f, 1.0f); // Similar reason as above, but just make sure that is it not negative
        updateLeds();
      }

      if (PS3.getButtonClick(LEFT) && trim < 90) {
        trim++;
        EEPROM.put(1, trim); // Write value to EEPROM
      } else if (PS3.getButtonClick(RIGHT) && trim > -90) {
        trim--;
        EEPROM.put(1, trim); // Write value to EEPROM
      } else if (PS3.getButtonClick(SELECT)) {
        trim = 0; // Reset trim value
        EEPROM.put(1, trim); // Write value to EEPROM
      }

      uint16_t steering = map(PS3.getAnalogHat(RightHatX), 0, 255, SERVO_MAX, SERVO_MIN);
      steering = constrain(steering + trim * (SERVO_MAX - SERVO_MIN) / 180, SERVO_MIN, SERVO_MAX); // Apply trim value

      uint16_t forwardBackward = map((PS3.getAnalogHat(LeftHatY) - 127) * sensitivity, -127, 128, ESC_MAX, ESC_MIN);

      sendCommand(steering, forwardBackward);

      if (PS3.getButtonClick(PS)) {
        sendCommand(SERVO_MID, ESC_MID); // Center steering servo and stop ESC
        PS3.disconnect();
      }
    } else
      sendCommand(SERVO_MID, ESC_MID); // Center steering servo and send stop ESC if data has not been received from the controller in the last 100 ms
  }
}
