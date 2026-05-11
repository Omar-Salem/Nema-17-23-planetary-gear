#include <AccelStepper.h>

#define RXD2 16
#define TXD2 17

#define STEP_PIN 14
#define DIR_PIN 27

AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// --------------------
// MOTOR PARAMETERS
// --------------------

const bool isNema23 = false;

const double FULL_CIRCLE_STEPS =
  isNema23 ? 400.0 : 200.0;

const double FULL_CIRCLE_RADIANS = 6.28318530718;

const double REDUCTION = 36.0;

const int RPM = 30;

const double RADIANS_TO_STEPS =
  FULL_CIRCLE_STEPS / FULL_CIRCLE_RADIANS;

const double STEPS_TO_RADIANS =
  FULL_CIRCLE_RADIANS / FULL_CIRCLE_STEPS;

const double STEPS_PER_SECOND =
  (RPM * FULL_CIRCLE_STEPS) / 60.0;

// --------------------
// SWEEP PARAMETERS
// --------------------

const double SWEEP_ANGLE_RAD = 1.5708;

long targetA;
long targetB;

bool movingToA = false;

// --------------------
// ERROR MONITORING
// --------------------

const byte readErrorCmd[] = { 0xE0, 0x39, 0x19 };

unsigned long lastErrorRequest = 0;
const int requestInterval = 100;

double maxErrorInDegrees = 0;

// --------------------

void setup() {

  Serial.begin(115200);

  Serial2.begin(
    38400,
    SERIAL_8N1,
    RXD2,
    TXD2);

  targetA = 0;

  targetB =
    SWEEP_ANGLE_RAD * REDUCTION * RADIANS_TO_STEPS;

  Serial.print("Target B steps: ");
  Serial.println(targetB);

  Serial.print("Steps/sec: ");
  Serial.println(STEPS_PER_SECOND);

  stepper.setMaxSpeed(STEPS_PER_SECOND);

  stepper.setAcceleration(
    STEPS_PER_SECOND * 2);

  stepper.moveTo(targetB);
}

void loop() {

  // Continuous motion handling
  stepper.run();

  // When target reached, reverse direction
  if (stepper.distanceToGo() == 0) {

    movingToA = !movingToA;

    long target =
      movingToA ? targetA : targetB;


    stepper.moveTo(target);
  }

  // Read angle error periodically
  if (
    millis() - lastErrorRequest
    >= requestInterval) {

    requestMotorError();

    lastErrorRequest = millis();

    Serial.print("Max Error: ");
    Serial.print(maxErrorInDegrees, 4);
    Serial.println(" deg");
  }
}

// =====================================================

void requestMotorError() {

  Serial2.write(readErrorCmd, 3);

  delay(2);

  while (Serial2.available() >= 4) {

    if (Serial2.peek() != 0xE0) {
      Serial2.read();
      continue;
    }

    byte rx[4];

    int n =
      Serial2.readBytes(rx, 4);

    if (n != 4)
      return;

    // Ignore echo packet
    if (
      rx[0] == 0xE0 && rx[1] == 0x39 && rx[2] == 0x19 && rx[3] == 0xE0) {
      continue;
    }

    byte crc =
      (rx[0] + rx[1] + rx[2]) & 0xFF;

    if (crc != rx[3]) {
      continue;
    }

    int16_t raw =
      (rx[1] << 8) | rx[2];

    float angle =
      raw * 360.0 / 65536.0;
    if (abs(angle) > 10) { continue; }
    if (abs(angle) > maxErrorInDegrees) {
      maxErrorInDegrees = abs(angle);
    }
  }
}