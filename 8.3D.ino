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

// Sets the move speed of the motors
int16_t moveSpeed = 75;

//Sound sensor object
MeSoundSensor mySound(14);

// Setting up sensors
MeUltrasonicSensor ultraSensorLeft(PORT_9); /* Ultrasonic module can ONLY be connected to port 3, 4, 6, 7, 8 of base shield. */


MeUltrasonicSensor ultraSensorFront(PORT_10);  // /* Ultrasonic module can ONLY be connected to port 3, 4, 6, 7, 8 of base shield. */

MeGyro gyro(1, 0x69);

double gyroRotation = 0.0;

float distanceClose = 20;
float distanceFar = 30;

int threshold = 570;
bool isClapped = false;


float target_distance = 20.0f;
int16_t SPEED = 75;  //TODO
float angle;

float cur_distance;
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

void BackwardAndTurnLeft(void) {
  Encoder_1.setMotorPwm(moveSpeed / 4);
  Encoder_2.setMotorPwm(-moveSpeed);
}

void BackwardAndTurnRight(void) {
  Encoder_1.setMotorPwm(moveSpeed);
  Encoder_2.setMotorPwm(-moveSpeed / 4);
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
void TurnLeft(void) {
  Encoder_1.setMotorPwm(-moveSpeed);
  Encoder_2.setMotorPwm(moveSpeed / 2);
}

void TurnRight() {
  Encoder_1.setMotorPwm(-moveSpeed / 2);
  Encoder_2.setMotorPwm(moveSpeed);
}

void CurveLeft(void) {
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
  gyro.update();
  Serial.read();
  cur_angle = gyro.getAngleZ();
  cur_distance = ultraSensorLeft.distanceCm();
  float d_angle = getAngle(target_distance - cur_distance);

  if (d_angle > cur_angle) {
    curveRight();
  } else if (d_angle < cur_angle) {
    curveLeft();
  } else {
    driveStraight();
  }

  if (mySound.strength() > threshold) {
    switchState();
  }

  CheckObstacle();
}

float getAngle(float delta) {
  float out = delta * 1.5f;

  if (out > 15.0f) return 15.0f;
  if (out < -15.0f) return -15.0f;
  return out;
}


void CheckObstacle() {
  double sensorFront = ultraSensorFront.distanceCm();
  if (sensorFront < 22.0) {
    // led.setColorAt(2, 0, 35, 35);
    // led.show();
    Overtake();
  }
}

void Overtake() {
  if (ultraSensorFront.distanceCm() < 20) {
    while (ultraSensorLeft.distanceCm() > 20) {
    TurnRight1();  
    }
    while (ultraSensorLeft.distanceCm() < 20) {
      if (ultraSensorLeft.distanceCm() < 20) {
        CurveLeft();
      } else {
        TurnRight1();
      }
    } 
    CurveRight();
    delay(200);
  }
}

void MaintainDistance() {
  if ((ultraSensorLeft.distanceCm() <= distanceClose + 5) && (ultraSensorLeft.distanceCm() >= distanceClose - 5)) {
    Forward();
  } else if (distanceClose > ultraSensorLeft.distanceCm()) {
    turnRight(5.00);
    Forward();
    delay(250);
    turnLeft(5.00);
  } else if (distanceClose < ultraSensorLeft.distanceCm()) {
    turnLeft(5.00);
    Forward();
    delay(250);
    turnRight(5.00);
  }
}


void switchState() {
  if (isClapped == true) {
    isClapped = false;
    target_distance = 20.0f;
    led.setColorAt(2, 35, 0, 0);
    led.show();
  } else {
    isClapped = true;
    target_distance = 35.0f;
    led.setColorAt(2, 0, 0, 35);
    led.show();
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

void MovementLeftGyro() {
  if (gyroRotation >= -15) {
    TurnLeft();
  }
  Forward();
}


// void switchState() {
//   if (isClapped == true) {
//     isClapped = false;
//     // distanceFar -= 10;
//     distanceClose -= 10;
//     led.setColorAt(2, 35, 0, 0);
//     led.show();
//   } else {
//     isClapped = true;
//     // distanceFar += 10;
//     distanceClose += 10;
//     led.setColorAt(2, 0, 0, 35);
//     led.show();
//   }
// }


//Add criteria to straighten up
//Potential movement straighten method?  Or add to Current movement method?
// void Movement() {
//   if (isClapped == true) {
//     if (distanceClose > ultraSensorSide.distanceCm() || distanceFar >  ultraSensorSide.distanceCm()) {
//       TurnLeft();
//       delay(300);
//     } else if (distanceClose < ultraSensorSide.distanceCm()) {
//       TurnRight();
//       delay(300);
//     }
//   }
// }

// void MovementClapTrue() {
//   if (isClapped == true) {
//     if (distanceFar > ultraSensorSide.distanceCm()) {
//       TurnRight();
//     } else if (distanceFar <= ultraSensorSide.distanceCm()) {
//       TurnLeft();
//     }
//   }
// }

// void MovementClapFalse() {
//   if (isClapped == false) {
//     if (distanceClose > ultraSensorSide.distanceCm()) {
//       TurnRight();
//     } else if (distanceClose <= ultraSensorSide.distanceCm()) {
//       TurnLeft();
//     }
//   }
// }

// void StraightenUp() {
//   if (ultraSensorSide.distanceCm() > distanceClose && ultraSensorSide.distanceCm() < distanceFar) {

//   }
// }
// } else {
//   TurnRight();
//   delay(1000);
// }

// void Switch () {
//   int switch = 0;
//   while (1) {
//     if  (switch == 0) {
//       // Call Near
//     }
//     else if (switch == 1) {
//       // Call Far
//     }
//   }
// }

// Can also create proportional gain function if needed

// Standard hysteresis function/algo
// void Hysteresis(double distance_far, double distance_short) {
//   while (1) {
//     if (ultraSensorSide.distanceCm() > distance_far) {
//       Forward();
//     } else if (ultraSensorSide.distanceCm() < distance_short) {
//       Backward();
//     } else {
//       Stop();
//     }
//   }
// }

// //
// void BangBang(double desired_distance) {
//   while (1) {
//     if (desired_distance < ultraSensorSide.distanceCm()) {
//       Forward();
//     } else {
//       Stop();
//     }
//   }
// }


// gyroRotation = gyro.getAngleZ();
// Serial.print("Z: ");
// Serial.println(gyroRotation);

// Serial.print("value= ");
// Serial.println(mySound.strength());
// delay(100);
// MaintainDistance();
// Onboard ultrasonic sensor measurement readings
// Serial.print("Distance : ");
// Serial.print(ultraSensorSide.distanceCm() );
// Serial.println(" cm");
// delay(100); /* the minimal measure interval is 100 milliseconds */

//Prints out the measured sound value
//BangBang(40.00);
// Hysteresis(40.00, 20.00);
