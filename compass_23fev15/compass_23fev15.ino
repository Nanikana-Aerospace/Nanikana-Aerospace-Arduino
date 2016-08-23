#include <Wire.h>
#include <HMC5883L.h>

HMC5883L compass;
     float declinationAngle = 0.212;    // Radians

void setup() {
  Serial.begin(57600);
  Wire.begin();
  compass = HMC5883L();
  // Set scale to +/- 1.3 Ga
  int error = compass.SetScale(1.3);
  if (error != 0) // If there is an error, print it out.
    Serial.println(compass.GetErrorText(error));
  // Set measurement mode to continous
  error = compass.SetMeasurementMode(Measurement_Continuous);
  if (error != 0) // If there is an error, print it out.
    Serial.println(compass.GetErrorText(error));
}

void loop() {
  Serial.print("Heading:\t");
  Serial.print(getDegrees());
  Serial.println(" Degrees   \t");
  delay(100);
}

int getDegrees () {
  // Retrived the scaled values from the compass (scaled to the configured scale).
  MagnetometerScaled scaled = compass.ReadScaledAxis();
   // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float heading = atan2(scaled.YAxis, scaled.XAxis) + declinationAngle;
  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2 * PI;
  // Check for wrap due to addition of declination.
  if(heading > 2 * PI)
    heading -= 2 * PI;
  // Convert radians to degrees for readability.
  return (int)  (heading * 180 / M_PI); 
}
