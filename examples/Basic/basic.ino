/******************************************************************************
MMC5883_compass Arduino
Leonardo Bispo
Sep, 2022
https://github.com/ldab/MMC5883_compass
Resources:
Uses Wire.h for i2c operation

Distributed as-is; no warranty is given.
******************************************************************************/

// Enable Serial debbug on Serial UART to see registers wrote
#define MMC5883_DEBUG Serial

#include "Wire.h"
#include "mmc5883.h"

MMC5883 myComp(0x30);

void setup()
{
  // put your setup code here, to run once:

  Serial.begin(115200);
  delay(1000); // wait until serial is open...

  if (myComp.begin(ODR_100HZ, CONTINUOUS_14HZ) != 0) {
    Serial.print("Failed to initialize Sensor.\n");
  } else {
    Serial.print("Sensor initialized.\n");
  }

  uint8_t readData = 0;

  // Get the ID:
  myComp.readRegister(&readData, MMC5883_WHO_AM_I);
  Serial.print("Who am I? 0x");
  Serial.println(readData, HEX);
}

void loop()
{
  mmc5883_axis_t axis = {0};
  myComp.getAxisMag(axis);

  // Read accelerometer data in mg as Float
  Serial.print(" Acceleration X float = ");
  Serial.println(axis.x);

  // Read accelerometer data in mg as Float
  Serial.print(" Acceleration Y float = ");
  Serial.println(axis.y);

  // Read accelerometer data in mg as Float
  Serial.print(" Acceleration Z float = ");
  Serial.println(axis.z);

  float headingRad = myComp.calcHeading(axis);
  float headingDeg = headingRad * 180 / PI;

  Serial.print(" Heading in degrees = ");
  Serial.println(headingDeg);

  delay(1000);
}