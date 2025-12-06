#include <Alfredo_NoU3.h>
#include <PestoLink-Receive.h>

NoU_Motor myMotor(1);

NoU_Servo myServo(1);

void setup() {
  Serial.begin(115200);
  PestoLink.begin("State Machine Example");
  NoU3.begin();
}

void loop() {
  if (PestoLink.buttonHeld("<Desired Button Here>")) {
    // L1
    myMotor.set(0.25);
    myServo.write(45);
  } else if (PestoLink.buttonHeld("<Desired Button Here>")) {
    // L2
    myMotor.set(0.5);
    myServo.write(90);
  } else if (PestoLink.buttonHeld("<Desired Button Here>")) {
    // L3
    myMotor.set(.75);
    myServo.write(135);
  } else if (PestoLink.buttonHeld("<Desired Button Here>")) {
    // L4
    myMotor.set(1);
    myServo.write(180);
  }
}
