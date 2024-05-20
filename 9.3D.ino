#include <MeAuriga.h>
#include <Wire.h>

#define ALLLEDS 0

// Auriga on-board light ring has 12 LEDs
#define LEDNUM 12

// on-board LED ring, at PORT0 (onboard)
MeRGBLed led(0, LEDNUM);

MeEncoderOnBoard Encoder_1(SLOT1);
MeEncoderOnBoard Encoder_2(SLOT2);
MeLineFollower lineFinder(PORT_8);

// Setting up sensors
MeUltrasonicSensor ultraSensorLeft(PORT_9);    /* Ultrasonic module can ONLY be connected to port 3, 4, 6, 7, 8 of base shield. */
MeUltrasonicSensor ultraSensorFront(PORT_10);  // /* Ultrasonic module can ONLY be connected to port 3, 4, 6, 7, 8 of base shield. */

int16_t moveSpeed = 50; // previously 30

// MeGyro gyro(1, 0x69);

// Define constants for sensor states
#define S1_IN_S2_IN 0b11    // Both sensors on the line
#define S1_IN_S2_OUT 0b10   // Left sensor on the line, right sensor off the line
#define S1_OUT_S2_IN 0b01   // Left sensor off the line, right sensor on the line
#define S1_OUT_S2_OUT 0b00  // Both sensors off the line

// Flag representing what type of line is being followed
bool followingBlack = true;


void CurveRight() {
  Encoder_1.setMotorPwm(0);
  Encoder_2.setMotorPwm(moveSpeed);
}

void CurveLeft(void) {
  Encoder_1.setMotorPwm(-moveSpeed);
  Encoder_2.setMotorPwm(0);
}

void Forward(void) {
  Encoder_1.setMotorPwm(-moveSpeed);  // setMotorPwm writes to the encoder controller
  Encoder_2.setMotorPwm(moveSpeed);   // so setting the speed change instantly
}
void Backward(void) {
  Encoder_1.setMotorPwm(moveSpeed);
  Encoder_2.setMotorPwm(-moveSpeed);
}

void BackwardAndTurnLeft(void) {
  Encoder_1.setMotorPwm(moveSpeed / 4);
  Encoder_2.setMotorPwm(-moveSpeed);
}

void BackwardAndTurnRight(void) {
  Encoder_1.setMotorPwm(moveSpeed);
  Encoder_2.setMotorPwm(-moveSpeed / 4);
}

void TurnLeft1(void) {
  Encoder_1.setMotorPwm(-moveSpeed);
  Encoder_2.setMotorPwm(-moveSpeed);
}

void TurnRight1(void) {
  Encoder_1.setMotorPwm(moveSpeed);
  Encoder_2.setMotorPwm(moveSpeed);
}

void Stop(void) {
  Encoder_1.setMotorPwm(0);
  Encoder_2.setMotorPwm(0);
}

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

  // Initalise LED
  led.setpin(44);
}

void loop() {
  if (followingBlack) {
    followBlackBackground();
  } else {
    followWhiteBackground();
  }
}

void followBlackBackground() {
  int sensorState = lineFinder.readSensors();
    led.setColorAt(2, 0, 0, 0);
    led.show();
  switch (sensorState) {
    case S1_IN_S2_IN: // both sensors see white
      // led.setColorAt(2, 35, 0, 0);
      // led.show();
      Forward();
      break;
    case S1_OUT_S2_IN: // Left sensor sees black and right sensor sees white 
      // led.setColorAt(2, 0, 35, 0);
      // led.show();
      CurveRight();
      // delay(120);
      break;
    case S1_IN_S2_OUT: // Left sensor sees white and right sensor sees black
      // led.setColorAt(2, 0, 0, 35);
      // led.show();
      CurveLeft();
      // delay();
      break;
    case S1_OUT_S2_OUT: // Both sensors see black 
      followingBlack = false;
      break;
    default:
      break;
  }
  delay(115); 
}
void followWhiteBackground() {
  int sensorState = lineFinder.readSensors();
    led.setColorAt(2, 30, 30, 30);
    led.show();
  switch (sensorState) {
    case S1_OUT_S2_OUT:
      // led.setColorAt(2, 35, 0, 0);
      // led.show();
      Forward();
      break;
    case S1_IN_S2_OUT:
      // led.setColorAt(2, 0, 35, 0);
      // led.show();
      CurveRight();
      // delay(120);
      // TurnRight1();
      // delay(200);
      // Forward();
      break;
    case S1_OUT_S2_IN:
      // led.setColorAt(2, 0, 0, 35);
      // led.show();
      CurveLeft();
      // delay(120);
      // TurnLeft1();
      // delay(200);
      // Forward();
      break;
    case S1_IN_S2_IN:
      followingBlack = true;
      break;
    default:
      break;
  }
  delay(115); 
}
