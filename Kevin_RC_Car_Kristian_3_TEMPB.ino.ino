/*
   This is a modified version of the PS3BT.ino example sketch by Kristian Lauszus
  For more information visit his blog: http://blog.tkjelectronics.dk/ or
  send him an e-mail:  kristianl@tkjelectronics.com
*/

#include <PS3BT.h>                                                    //Include the necessary libraries.
#include <Servo.h>

USB Usb;
BTD Btd(&Usb);
PS3BT PS3(&Btd);

Servo servo1;                                                         //Create instances of type Servo. servo1 is the steering servo and servo2 is the ESC.
Servo servo2;

void setup()
{
  Serial.begin(115200);
  if (Usb.Init() == -1)
  {
    Serial.print(F("\r\nOSC did not start"));
    while (1); //halt
  }
  Serial.print(F("\r\nPS3 Bluetooth Library Started"));
  servo1.attach(5);                                                  //Steering servo on digital pin 5
  servo2.attach(3);                                                  //ESC on sigital pin 3
}

void loop()
{
  Usb.Task();

  if (PS3.PS3Connected && millis() - PS3.getLastMessageTime() < 100)
  {
    static float scaling = 1.0f;
    if (PS3.getButtonClick(UP))
      scaling += 0.1f;
    else if (PS3.getButtonClick(DOWN))
      scaling -= 0.1f;

    if (scaling < 0)
      scaling = 0;
    else if (scaling > 1)
      scaling = 1;

    servo1.writeMicroseconds(map(PS3.getAnalogHat(RightHatX), 0, 255, 2000, 1000));
    servo2.writeMicroseconds(map((PS3.getAnalogHat(LeftHatY) - 127) * scaling, -127, 128, 2000, 1000));
    //Serial.println(PS3.getAnalogHat(LeftHatY));
    
  if (PS3.getButtonClick(PS))
  {
    PS3.disconnect();
    servo2.writeMicroseconds(1500); //If PS3 controller is not connected or gets disconnected from going out of range stop motor from spinning
  }
  }
  else
  {
    servo1.writeMicroseconds(1500);
    servo2.writeMicroseconds(1500);
  }
 }


