#include <MeAuriga.h>
#include <Wire.h>

MeEncoderOnBoard Encoder_1(SLOT1);
MeEncoderOnBoard Encoder_2(SLOT2);
MeLineFollower lineFinder(PORT_8);


int16_t moveSpeed = 200;
// int16_t lastHit = 2;

// Define constants for sensor states
#define S1_IN_S2_IN 0b11    // Both sensors on the line
#define S1_IN_S2_OUT 0b10   // Left sensor on the line, right sensor off the line
#define S1_OUT_S2_IN 0b01   // Left sensor off the line, right sensor on the line
#define S1_OUT_S2_OUT 0b00  // Both sensors off the line

void Forward(int speed) {
  Encoder_1.setMotorPwm(-speed);
  Encoder_2.setMotorPwm(speed);
}

void CurveLeft(int speed, int dif) {
  Encoder_1.setMotorPwm(-speed);
  Encoder_2.setMotorPwm(speed / dif);
}

void CurveRight(int speed, int dif) {
  Encoder_1.setMotorPwm(-speed / dif);
  Encoder_2.setMotorPwm(speed);
}

void drive(int left, int right) {
  Encoder_1.setMotorPwm(-left);
  Encoder_2.setMotorPwm(right);
}

// void CurveLeft(void) {
//   Encoder_1.setMotorPwm(-moveSpeed);
//   Encoder_2.setMotorPwm(0);
//   // Encoder_2.setMotorPwm(moveSpeed / 2);
// }

// void CurveRight() {
//   Encoder_1.setMotorPwm(0);
//   // Encoder_1.setMotorPwm(-moveSpeed / 2);
//   Encoder_2.setMotorPwm(moveSpeed);
// }

void Forward(void) {
  Encoder_1.setMotorPwm(-moveSpeed);  // setMotorPwm writes to the encoder controller
  Encoder_2.setMotorPwm(moveSpeed);   // so setting the speed change instantly
}

// void TurnLeft1(void) {
//   Encoder_1.setMotorPwm(-moveSpeed);
//   Encoder_2.setMotorPwm(-moveSpeed);
// }

// void TurnRight1(void) {
//   Encoder_1.setMotorPwm(moveSpeed);
//   Encoder_2.setMotorPwm(moveSpeed);
// }

void ChangeSpeed(int16_t spd) {
  moveSpeed = spd;
}

void setup() {
  Serial.begin(115200);
  //Set PWM 8KHz
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);
  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);
}

void loop() {
  followWhiteBackground();
}

void followWhiteBackground() {
  while (1) {
    int sensorState = lineFinder.readSensors();
    switch (sensorState) {
      case S1_OUT_S2_OUT:
        drive(moveSpeed, moveSpeed);
        break;
      case S1_IN_S2_OUT:
      //left
        drive(moveSpeed / 2, moveSpeed);
        break;
      case S1_OUT_S2_IN:
      //right
        drive(moveSpeed, moveSpeed / 5);
        break;
      case S1_IN_S2_IN:
        drive(moveSpeed, 0);
        break;
      default:
        break;
    }
  }
}

// void followWhiteBackground() {
//   int sensorState = lineFinder.readSensors();
//   switch (sensorState) {
//     case S1_OUT_S2_OUT:
//       Forward();
//       break;
//     case S1_IN_S2_OUT:
//       CurveRight();
//       break;
//     case S1_OUT_S2_IN:
//       CurveLeft();
//       break;
//     case S1_IN_S2_IN:
//       break;
//     default:
//       break;
//   }
// }
