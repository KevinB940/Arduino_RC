/*
  This is a modified version of the PS3BT.ino example sketch by Kristian Lauszus
  For more information visit his blog: http://blog.tkjelectronics.dk/ or
  send him an e-mail:  kristianl@tkjelectronics.com
*/

#include <PS3BT.h> // Include the PS3BT library: https://github.com/felis/USB_Host_Shield_2.0

USB Usb;
BTD Btd(&Usb);
PS3BT PS3(&Btd);

#define ESC_MIN 1000
#define ESC_MID 1500
#define ESC_MAX 2000

uint32_t timer;

void setup() {
  Serial.begin(57600);
  if (Usb.Init() == -1) {
    Serial.println(F("\r\nOSC did not start"));
    while (1); // Halt
  }
  Serial.println(F("\r\nSender started"));
  timer = millis();
}

void sendCommand(uint8_t steering, uint16_t forwardBackward) {
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

void loop() {
  Usb.Task();

  if (millis() - timer > 10) { // Limit the serial output frequency to 10 ms
    timer = millis();
    if (PS3.PS3Connected && millis() - PS3.getLastMessageTime() < 100) {
      static float sensitivity = 1.0f; // Used to adjust the sensitivity of the throttle
      if (PS3.getButtonClick(UP) && sensitivity < 1)
        sensitivity += 0.1f;
      else if (PS3.getButtonClick(DOWN) && sensitivity > 0)
        sensitivity -= 0.1f;

      uint8_t steering = map(PS3.getAnalogHat(RightHatX), 0, 255, 180, 0);
      uint16_t forwardBackward = map((PS3.getAnalogHat(LeftHatY) - 127) * sensitivity, -127, 128, ESC_MAX, ESC_MIN);

      sendCommand(steering, forwardBackward);

      if (PS3.getButtonClick(PS)) {
        sendCommand(90, ESC_MID); // Center steering servo and stop ESC
        PS3.disconnect();
      }
    } else
      sendCommand(90, ESC_MID); // Center steering servo and send stop ESC if data has not been received from the controller in the last 100 ms
  }
}
