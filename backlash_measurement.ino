#include <Adafruit_AS5600.h>

Adafruit_AS5600 as5600;

uint16_t zeroOffset = 0;

void setup() {
  Serial.begin(115200);

  Serial.println("AS5600 Zero Calibration Starting...");

  if (!as5600.begin()) {
    Serial.println("Could not find AS5600 sensor, check wiring!");
    while (1) delay(10);
  }

  // Wait for magnet
  while (!as5600.isMagnetDetected()) {
    Serial.println("Waiting for magnet...");
    delay(200);
  }

  // --- CAPTURE ZERO POSITION ---
  zeroOffset = as5600.getAngle();
  Serial.print("Zero calibrated at raw angle: ");
  Serial.println(zeroOffset);

  Serial.println("AS5600 ready!");

  as5600.enableWatchdog(false);
  as5600.setPowerMode(AS5600_POWER_MODE_NOM);
  as5600.setHysteresis(AS5600_HYSTERESIS_OFF);

  as5600.setOutputStage(AS5600_OUTPUT_STAGE_ANALOG_FULL);

  as5600.setSlowFilter(AS5600_SLOW_FILTER_16X);
  as5600.setFastFilterThresh(AS5600_FAST_FILTER_THRESH_SLOW_ONLY);

  as5600.setZPosition(0);
  as5600.setMPosition(4095);
  as5600.setMaxAngle(4095);
}

void loop() {
  if (!as5600.isMagnetDetected()) {
    Serial.println("No magnet found!");
    delay(200);
    return;
  }
  if (as5600.isAGCminGainOverflow()) {
    Serial.println(" | MH: magnet too strong");
    return;
  }
  if (as5600.isAGCmaxGainOverflow()) {
    Serial.println(" | ML: magnet too weak");
    return;
  }

  uint16_t rawAngle = as5600.getAngle();

  // --- APPLY ZERO OFFSET WITH WRAPAROUND ---
  int32_t corrected = rawAngle - zeroOffset;
  if (corrected < 0) corrected += 4096;

  // Convert to degrees
  float degrees = corrected * 360.0 / 4096.0;

  Serial.print("Degrees: ");
  Serial.print(degrees, 2);
  Serial.print("°");

  Serial.println();

  delay(50);
}