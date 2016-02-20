/*
  This is a modified version of the PS3BT.ino example sketch by Kristian Lauszus
  For more information visit his blog: http://blog.tkjelectronics.dk/ or
  send him an e-mail:  kristianl@tkjelectronics.com
*/

#include <Servo.h> // Include the Servo library: https://www.arduino.cc/en/reference/servo

Servo servo1, servo2; // Create instances of type Servo. servo1 is the steering servo and servo2 is the ESC.

#define ESC_MIN 1000
#define ESC_MID 1500
#define ESC_MAX 2000

uint32_t timer;

void setup() {
  Serial.begin(57600);
  servo1.attach(5); // Steering servo on digital pin 5
  servo2.attach(4); // ESC on signal pin 4
  Serial.println(F("\r\nReceiver started"));
  timer = millis();
}

void loop() {
  if (millis() - timer > 100) { // Stop car if it has been more than 100 ms since last message
    //servo1.write(90);
    servo1.writeMicroseconds(ESC_MID);
    servo2.writeMicroseconds(ESC_MID);
  }

  if (Serial.available()) {
    if (Serial.read() != 'C') // Make sure first character is 'C'
      return;
    String input = Serial.readStringUntil(';');
    //Serial.println(input);

    uint16_t steering = atoi(strtok((char*)input.c_str(), ",")); // Read upto comma and convert string to int
    uint16_t forwardBackward = atoi(strtok(NULL, ","));  // Read upto comma and convert string to int
    uint16_t checkSum = atoi(strtok(NULL, "\0"));  // Read upto null-character and convert string to int

    if (steering ^ forwardBackward == checkSum) { // Send output to servos if chechsum match
      if (steering >= 800 && steering <= 2200 && forwardBackward >= ESC_MIN && forwardBackward <= ESC_MAX) {
        timer = millis();
        //servo1.write(steering);
        servo1.writeMicroseconds(steering);
        servo2.writeMicroseconds(forwardBackward);
        /*Serial.print(steering);
        Serial.write('\t');
        Serial.println(forwardBackward);*/
      } else
        Serial.println("Values out of range!");
    } else
      Serial.println("CHECKSUM ERROR!");
  }
}
