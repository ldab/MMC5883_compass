/******************************************************************************
mmc5883.cpp
mmc5883_compass Arduino
Leonardo Bispo
Sep, 2022
https://github.com/ldab/mmc5883_compass
Resources:
Uses Wire.h for i2c operation

Distributed as-is; no warranty is given.
******************************************************************************/

#include "mmc5883.h"
#include "stdint.h"

#include "Wire.h"

//****************************************************************************//
//
//  Default construction is I2C mode, address 0x30.
//
//****************************************************************************//
MMC5883::MMC5883(uint8_t inputArg = MMC5883_I2C_ADDRESS)
{
  I2CAddress = inputArg;
}

mmc5883_status_t MMC5883::begin(mmc5883_odr_t odr, mmc5883_cm_t cm)
{

  _DEBBUG("Configuring Sensor");

  mmc5883_status_t returnError = MAG_GENERIC_ERROR;

  Wire.begin();

  // Check the ID register to determine if the operation was a success.
  uint8_t _whoAmI;

  readRegister(&_whoAmI, MMC5883_WHO_AM_I);

  if (_whoAmI != 0b1100) {
    returnError = MAG_GENERIC_ERROR;
  }

  _DEBBUG("Apply settings");

  returnError = writeRegister(MMC5883_INTERNAL_CTRL1, odr);
  returnError = writeRegister(MMC5883_INTERNAL_CTRL2, cm);

  return returnError;
}

//****************************************************************************//
//  ReadRegisterRegion
//
//  Parameters:
//    *outputPointer -- Pass &variable (base address of) to save read data to
//    offset -- register to read
//    length -- number of bytes to read
//****************************************************************************//
mmc5883_status_t MMC5883::readRegisterRegion(uint8_t *outputPointer,
                                             uint8_t offset, uint8_t length)
{
  mmc5883_status_t returnError = MAG_SUCCESS;

  // define pointer that will point to the external space
  uint8_t i                    = 0;
  uint8_t c                    = 0;

  Wire.beginTransmission(I2CAddress);
  offset |= 0x80; // turn auto-increment bit on, bit 7 for I2C
  Wire.write(offset);
  if (Wire.endTransmission() != 0) {
    returnError = MAG_HW_ERROR;
  } else // OK, all worked, keep going
  {
    // request 6 bytes from slave device
    Wire.requestFrom(I2CAddress, length);
    while ((Wire.available()) &&
           (i < length)) // slave may send less than requested
    {
      c              = Wire.read(); // receive a byte as character
      *outputPointer = c;
      outputPointer++;
      i++;
    }
  }

  return returnError;
}

//****************************************************************************//
//  ReadRegister
//
//  Parameters:
//    *outputPointer -- Pass &variable (address of) to save read data to
//    offset -- register to read
//****************************************************************************//
mmc5883_status_t MMC5883::readRegister(uint8_t *outputPointer, uint8_t offset)
{
  // Return value
  uint8_t result               = 0;
  uint8_t numBytes             = 1;
  mmc5883_status_t returnError = MAG_SUCCESS;

  Wire.beginTransmission(I2CAddress);
  Wire.write(offset);

  if (Wire.endTransmission() != 0) {
    returnError = MAG_HW_ERROR;
  }

  Wire.requestFrom(I2CAddress, numBytes);

  while (Wire.available()) // slave may send less than requested
  {
    result = Wire.read(); // receive a byte as a proper uint8_t
  }

  _DEBBUG("Read register 0x", offset, " = ", result);

  *outputPointer = result;
  return returnError;
}

//****************************************************************************//
//  readRegisterInt16
//
//  Parameters:
//    *outputPointer -- Pass &variable (base address of) to save read data to
//    offset -- register to read
//****************************************************************************//
mmc5883_status_t MMC5883::readRegisterInt16(int16_t *outputPointer,
                                            uint8_t offset)
{
  // offset |= 0x80; //turn auto-increment bit on
  uint8_t myBuffer[2];
  mmc5883_status_t returnError =
      readRegisterRegion(myBuffer, offset, 2); // Does memory transfer
  int16_t output = (int16_t)myBuffer[0] | int16_t(myBuffer[1] << 8);

  _DEBBUG("12 bit from 0x", offset, " = ", output);
  *outputPointer = output;
  return returnError;
}

//****************************************************************************//
//  writeRegister
//
//  Parameters:
//    offset -- register to write
//    dataToWrite -- 8 bit data to write to register
//****************************************************************************//
mmc5883_status_t MMC5883::writeRegister(uint8_t offset, uint8_t dataToWrite)
{
  mmc5883_status_t returnError = MAG_SUCCESS;

  // Write the byte
  Wire.beginTransmission(I2CAddress);
  Wire.write(offset);
  Wire.write(dataToWrite);
  if (Wire.endTransmission() != 0) {
    returnError = MAG_HW_ERROR;
  }

  return returnError;
}

// Read axis acceleration as Float
mmc5883_status_t MMC5883::getAxisMag(mmc5883_axis_t &_axis)
{
  mmc5883_status_t err;
  uint8_t buffer[6];
  uint8_t status;

  // Trigger measurement
  err = writeRegister(MMC5883_INTERNAL_CTRL0, 0b1);

  // wait for the measuement ready bit
  do {
    err = readRegister(&status, MMC5883_STATUS);
    if (err != MAG_SUCCESS) {
      return err;
    }
    delay(1);
  } while (!(status & 0b1));

  err        = readRegisterRegion(buffer, MMC5883_OUT_X_L, 6);

  uint16_t x = (uint16_t)buffer[0] | (uint16_t)buffer[1] << 8;
  uint16_t y = (uint16_t)buffer[2] | (uint16_t)buffer[3] << 8;
  uint16_t z = (uint16_t)buffer[4] | (uint16_t)buffer[5] << 8;

  // 0.25mG per LSB -> 1G = 100uT
  _axis.x    = (x - 32768) * 0.25 / 100.0F;
  _axis.y    = (y - 32768) * 0.25 / 100.0F;
  _axis.z    = (z - 32768) * 0.25 / 100.0F;

  return err;
}

// Read internal temperature sensor as Float
float MMC5883::getTemp(void)
{
  uint8_t outRAW;
  uint8_t status;

  // trigger temperature measurement
  mmc5883_status_t err = writeRegister(MMC5883_INTERNAL_CTRL0, 0b10);

  // wait for the measuement ready bit
  do {
    err = readRegister(&status, MMC5883_STATUS);
    if (err != MAG_SUCCESS) {
      return err;
    }
    delay(1);
  } while (!(status & 0b10));

  readRegister(&outRAW, MMC5883_OUT_TEMP);

  float temp = (outRAW - 75) * 0.7;

  _DEBBUG("T: %.01fdegC", temp);

  return temp;
}

mmc5883_status_t MMC5883::calibrate(float &head_offset)
{
  mmc5883_status_t returnError;
  mmc5883_axis_t axis = {0};
  returnError         = reset();

  delay(5);

  // SET
  returnError = writeRegister(MMC5883_INTERNAL_CTRL0, 0b1000);
  delay(1);
  // Trigger measurement
  returnError   = writeRegister(MMC5883_INTERNAL_CTRL0, 0b1);
  // Get measurement
  returnError   = getAxisMag(axis);
  float measOne = calcHeading(axis, 0);

  // RESET
  returnError   = writeRegister(MMC5883_INTERNAL_CTRL0, 0b10000);
  // Trigger measurement
  returnError   = writeRegister(MMC5883_INTERNAL_CTRL0, 0b1);
  // Get measurement
  returnError   = getAxisMag(axis);
  float measTwo = calcHeading(axis, 0);

  head_offset   = (float)(measOne + measTwo) / 2.0F;

  return returnError;
}

float MMC5883::calcHeading(mmc5883_axis_t _axis, float h_offset)
{
  float x       = _axis.x + h_offset;
  float y       = _axis.y + h_offset;
  float heading = atan2(y, x);

  if (heading < 0)
    heading += 2 * PI;

  return heading;
}

mmc5883_status_t MMC5883::reset(void)
{
  return writeRegister(MMC5883_INTERNAL_CTRL1, 0b10000000);
}