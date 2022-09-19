/******************************************************************************
MMC5883.h
MMC5883_compass Arduino
Leonardo Bispo
Sep, 2022
https://github.com/ldab/MMC5883-1057
Resources:
Uses Wire.h for i2c operation

Distributed as-is; no warranty is given.
******************************************************************************/

#ifndef __MMC5883_MAG_H__
#define __MMC5883_MAG_H__

#include "stdint.h"

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#ifdef MMC5883_DEBUG
namespace
{
  template <typename T> static void _DEBBUG(T last) { Serial.println(last); }

  template <typename T, typename... Args>
  static void _DEBBUG(T head, Args... tail)
  {
    Serial.print(head);
    Serial.print(' ');
    _DEBBUG(tail...);
  }
}
#else
#define _DEBBUG(...)
#endif

// Device Registers
#define MMC5883_I2C_ADDRESS    0x30
#define MMC5883_WHO_AM_I       0x2F

#define MMC5883_OUT_X_L        0x00
#define MMC5883_OUT_X_H        0x01
#define MMC5883_OUT_Y_L        0x02
#define MMC5883_OUT_Y_H        0x03
#define MMC5883_OUT_Z_L        0x04
#define MMC5883_OUT_Z_H        0x05

#define MMC5883_OUT_TEMP       0x06

#define MMC5883_STATUS         0x07
#define MMC5883_INTERNAL_CTRL0 0x08
#define MMC5883_INTERNAL_CTRL1 0x09
#define MMC5883_INTERNAL_CTRL2 0x0A

#define MMC5883_X_THRSD        0x0B
#define MMC5883_Y_THRSD        0x0C
#define MMC5883_Z_THRSD        0x0D

typedef enum {
  MAG_SUCCESS,
  MAG_HW_ERROR,
  MAG_NOT_SUPPORTED,
  MAG_GENERIC_ERROR,
  //...
} mmc5883_status_t;

typedef enum {
  ODR_100HZ,
  ODR_200HZ,
  ODR_400HZ,
  ODR_600HZ,
  //...
} mmc5883_odr_t;

typedef enum {
  CONTINUOUS_OFF,
  CONTINUOUS_14HZ,
  CONTINUOUS_5HZ,
  CONTINUOUS_2_2HZ,
  CONTINUOUS_1HZ,
  CONTINUOUS_0_5HZ,
  CONTINUOUS_0_25HZ,
  CONTINUOUS_0_125HZ,
  CONTINUOUS_0_0625HZ,
  CONTINUOUS_0_03125HZ,
  CONTINUOUS_0_015625HZ,
  //...
} mmc5883_cm_t;

typedef struct mmc5883_axis_t {
  float x;
  float y;
  float z;
}mmc5883_axis;

class MMC5883
{
  public:
  MMC5883(uint8_t);

  // Output resolution and Continous measuremet rate
  mmc5883_status_t begin(mmc5883_odr_t odr = ODR_100HZ,
                         mmc5883_cm_t cm   = CONTINUOUS_OFF);

  // readRegister reads one 8-bit register
  mmc5883_status_t readRegister(uint8_t *outputPointer, uint8_t offset);

  // Reads two 8-bit regs, LSByte then MSByte order, and concatenates them.
  // Acts as a 16-bit read operation
  mmc5883_status_t readRegisterInt16(int16_t *, uint8_t offset);

  // Writes an 8-bit byte;
  mmc5883_status_t writeRegister(uint8_t, uint8_t);

  // Configure Interrupts
  // @Threshold from 1 to 4095 counts
  // @moveDur   from 1 to 255 counts
  // @naDur			from 1 to 255 counts
  // Threshold (g) = threshold (counts) / 256(counts/g)
  // timeDur (sec) = WAKEUP_COUNTER (counts) / Wake-Up Function ODR(Hz)
  // Non-ActivityTime (sec) = NA_COUNTER (counts) / Wake-Up Function ODR(Hz)
  mmc5883_status_t intConf(uint16_t threshold, uint8_t moveDur, uint8_t naDur,
                           bool polarity = HIGH);

  // Read axis magnetic field as Float
  mmc5883_status_t getAxisMag(mmc5883_axis_t &_axis);

  // Return internal temperature as Float
  float getTemp(void);

  // Calibrate
  mmc5883_status_t calibrate(float &head_offset);

  // Calculate heading based on the axis magnetic field
  float calcHeading(mmc5883_axis_t _axis, float h_offset = 0);

  // SW reset
  mmc5883_status_t reset(void);

  private:
  uint8_t I2CAddress;

  // ReadRegisterRegion takes a uint8 array address as input and reads
  //   a chunk of memory into that array.
  mmc5883_status_t readRegisterRegion(uint8_t *, uint8_t, uint8_t);
};

#endif // End of __MMC5883_MAG_H__ definition check