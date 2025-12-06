/*
Program: Field Relative Holonomic Drive Code
Creation: November 18th, 2025
Contributors: Daniel Principe, Owen King
Use: Field relative holonomic drive
*/

#include <PestoLink-Receive.h>
#include <Alfredo_NoU3.h>
#include <stdio.h>

// Drivetrain Init
NoU_Motor frontLeftMotor(1);
NoU_Motor frontRightMotor(4);
NoU_Motor rearLeftMotor(8);
NoU_Motor rearRightMotor(5);

NoU_Drivetrain drivetrain(&frontLeftMotor, &frontRightMotor, &rearLeftMotor, &rearRightMotor);

float measured_angle = 31.416;
float angular_scale = (5.0*2.0*PI) / measured_angle;

void setup() {
  PestoLink.begin("Field Relative Holonomic Drive Test");
  Serial.begin(115200);

  NoU3.begin();
  NoU3.calibrateIMUs(); // this takes exactly one second. Do not move the robot during calibration.

  // Inversion Parts
  frontLeftMotor.setInverted(true);
  frontRightMotor.setInverted(true);
  rearLeftMotor.setInverted(true);
  rearRightMotor.setInverted(true);
}

void loop() {
  // Ima be real gangalang, I have no idea what is this
  static unsigned long lastPrintTime = 0;
  if (lastPrintTime + 100 < millis()){
      Serial.printf("gyro yaw (radians): %.3f\r\n",  NoU3.yaw * angular_scale );
      lastPrintTime = millis();
  }

  // This measures your batteries voltage and sends it to PestoLink
  float batteryVoltage = NoU3.getBatteryVoltage();
  PestoLink.printBatteryVoltage(batteryVoltage);

  if (PestoLink.isConnected()) {
    // --- Drivetrain Code ---
    // Sets Axes
    float fieldPowerX = PestoLink.getAxis(0);
    float fieldPowerY = -1 * PestoLink.getAxis(1);
    float rotationPower = -1 * PestoLink.getAxis(2);

    // Get robot heading (in radians) from the gyro
    float heading = NoU3.yaw * angular_scale;

    // Rotate joystick vector to be robot-centric
    float cosA = cos(heading);
    float sinA = sin(heading);

    float robotPowerX = fieldPowerX * cosA + fieldPowerY * sinA;
    float robotPowerY = -fieldPowerX * sinA + fieldPowerY * cosA;

    //set motor power
    drivetrain.holonomicDrive(robotPowerX, robotPowerY, rotationPower);
    NoU3.setServiceLight(LIGHT_ENABLED);
  } else {
    drivetrain.holonomicDrive(0, 0, 0); // lmao the example def took from us
    NoU3.setServiceLight(LIGHT_DISABLED);
  }
}
