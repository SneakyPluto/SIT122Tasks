#include <MeAuriga.h>
#include <Wire.h>

#define BUZZER_PORT 45
#define ALLLEDS 0

// Auriga on-board light ring has 12 LEDs
#define LEDNUM 12

// on-board LED ring, at PORT0 (onboard)
MeRGBLed led(0, LEDNUM);


MeEncoderOnBoard Encoder_1(SLOT1);
MeEncoderOnBoard Encoder_2(SLOT2);

//Sound sensor object
MeSoundSensor mySound(14);

// Setting up sensors
MeUltrasonicSensor ultraSensorRight(PORT_10); /* Ultrasonic module can ONLY be connected to port 3, 4, 6, 7, 8 of base shield. */

MeUltrasonicSensor ultraSensorLeft(PORT_9);

MeGyro gyro(1, 0x69);

// Sets the move speed of the motors
int16_t moveSpeed = 80;
int16_t SPEED = 130;  
float angle;


float cur_angle;

MeBuzzer buzzer;
const int buzzerDuration = 2000;

void Forward(void) {
  Encoder_1.setMotorPwm(-moveSpeed);  // setMotorPwm writes to the encoder controller
  Encoder_2.setMotorPwm(moveSpeed);   // so setting the speed change instantly
}
void Backward(void) {
  Encoder_1.setMotorPwm(moveSpeed);
  Encoder_2.setMotorPwm(-moveSpeed);
}

void curveRight() {
  Encoder_1.setMotorPwm(-SPEED / 2);
  Encoder_2.setMotorPwm(SPEED);
}

void curveLeft() {
  Encoder_1.setMotorPwm(-SPEED);
  Encoder_2.setMotorPwm(SPEED / 2);
}

void driveStraight() {
  Encoder_1.setMotorPwm(-SPEED);
  Encoder_2.setMotorPwm(SPEED);
}

//Turning left whilst moving forward
void TurnLeft() {
  Encoder_1.setMotorPwm(-moveSpeed);
  Encoder_2.setMotorPwm(moveSpeed / 2);
}

void TurnRight() {
  Encoder_1.setMotorPwm(-moveSpeed / 2);
  Encoder_2.setMotorPwm(moveSpeed);
}

void CurveLeft() {
  Encoder_1.setMotorPwm(-moveSpeed);
  Encoder_2.setMotorPwm(0);
}

void CurveRight() {
  Encoder_1.setMotorPwm(0);
  Encoder_2.setMotorPwm(moveSpeed);
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
  //Serial.begin(9600);
  Serial.begin(115200);

  //Set PWM 8KHz
  TCCR1A = _BV(WGM10);
  TCCR1B = _BV(CS11) | _BV(WGM12);
  TCCR2A = _BV(WGM21) | _BV(WGM20);
  TCCR2B = _BV(CS21);

  buzzer.setpin(BUZZER_PORT);

  // Initalise LED
  led.setpin(44);
  gyro.begin();
}


void loop() {
  Patrol();
}

float getAngle(float delta) {
  float out = delta * 1.5f;

  // limit the angle to 15 degrees
  if (out > 15.0f) return 15.0f;
  if (out < -15.0f) return -15.0f;
  return out;
}

void Patrol() {
  while (1) {
    gyro.begin();
    long start_time = millis();
    double cur_angle = 0.00;
    float d_angle = 0.00f;
    while (millis() - start_time < 5000) {
      // get all the current values
      gyro.update();
      cur_angle = gyro.getAngleZ();
      float distanceLeft = ultraSensorLeft.distanceCm();
      float distanceRight = ultraSensorRight.distanceCm();

      // check if doorway is left or right
      if (distanceLeft >= 200 || distanceRight >= 200) {
        // if doorway is detected, drive straight
        d_angle = 0.00f;
        driveStraight();
      } else {
        // calculate the angle to turn
        d_angle = getAngle(distanceRight - distanceLeft);
      }
      if (d_angle > cur_angle) {
        TurnRight();
      } else if (d_angle < cur_angle) {
        TurnLeft();
      } else {
        driveStraight();
      }
    }
    // turn around
    Encoder_1.setMotorPwm(moveSpeed);
    Encoder_2.setMotorPwm(moveSpeed);
    do {
      gyro.update();
      cur_angle = gyro.getAngleZ();
    } while (cur_angle < 168.00);
    Stop();
  }
}

void turnRight(double degree) {
  gyro.update();
  Serial.println(gyro.getAngleZ());
  double startAngle = gyro.getAngleZ() + 180.00;
  double delta;

  //Turn right 1
  Encoder_1.setMotorPwm(moveSpeed);
  Encoder_2.setMotorPwm(moveSpeed);

  TurnRight();
  while (degree > 0) {
    gyro.update();
    delta = gyro.getAngleZ() + 180.00 - startAngle;
    if (delta < -180.00) {
      delta += 360.00;
    } else if (delta > 180) {
      delta -= 360;
    }
    degree -= delta;
    startAngle = gyro.getAngleZ() + 180.00;
  }
  gyro.update();
  Serial.println(gyro.getAngleZ());
  Stop();
}

void turnLeft(double degree) {
  gyro.update();
  Serial.println(gyro.getAngleZ());
  double startAngle = gyro.getAngleZ() + 180.00;
  double delta;

  //Turn left1 code
  Encoder_1.setMotorPwm(-moveSpeed);
  Encoder_2.setMotorPwm(-moveSpeed);
  while (degree > 0) {
    gyro.update();
    delta = gyro.getAngleZ() + 180.00 - startAngle;
    if (delta < -180.00) {
      delta += 360.00;
    } else if (delta > 180) {
      delta -= 360;
    }
    degree += delta;
    startAngle = gyro.getAngleZ() + 180.00;
  }
  gyro.update();
  Serial.println(gyro.getAngleZ());
  Stop();
}
