// Copyright (C) 2021 Toitware ApS. All rights reserved.
// Use of this source code is governed by a MIT-style license that can be found
// in the LICENSE file.

// BNO055 data sheet: https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf

import binary
import serial.device as serial
import serial.registers as serial

import math

I2C-ADDRESS         ::= 0x28
I2C-ADDRESS-ALT     ::= 0x29

/**
Driver for the Bosch BNO055 sensor, using either I2C or SPI.
*/
class Driver:
  //Device registers

  static CHIP-ID_              ::= 0xA0
  static CHIP-ID-REG_          ::= 0x00

  static NDOF-MODE_            ::= 0x0C
  static UNIT-SEL_             ::= 0x3B

  static EUL-DATA-X-LSB_       ::= 0x1A
  static EUL-DATA-X-MSB_       ::= 0x1B
  static EUL-DATA-Y-LSB_       ::= 0x1C
  static EUL-DATA-Y-MSB_       ::= 0x1D
  static EUL-DATA-Z-LSB_       ::= 0x1E
  static EUL-DATA-Z-MSB_       ::= 0x1F

  static QUA-DATA-W-LSB_       ::= 0x20
  static QUA-DATA-W-MSB_       ::= 0x21
  static QUA-DATA-X-LSB_       ::= 0x22
  static QUA-DATA-X-MSB_       ::= 0x23
  static QUA-DATA-Y-LSB_       ::= 0x24
  static QUA-DATA-Y-MSB_       ::= 0x25
  static QUA-DATA-Z-LSB_       ::= 0x26
  static QUA-DATA-Z-MSB_       ::= 0x27

  static GYR-DATA-X-LSB_       ::= 0x14
  static GYR-DATA-X-MSB_       ::= 0x15
  static GYR-DATA-Y-LSB_       ::= 0x16
  static GYR-DATA-Y-MSB_       ::= 0x17
  static GYR-DATA-Z-LSB_       ::= 0x18
  static GYR-DATA-Z-MSB_       ::= 0x19

  static LIA-DATA-X-LSB_       ::= 0x28
  static LIA-DATA-X-MSB_       ::= 0x29
  static LIA-DATA-Y-LSB_       ::= 0x2A
  static LIA-DATA-Y-MSB_       ::= 0x2B
  static LIA-DATA-Z-LSB_       ::= 0x2C
  static LIA-DATA-Z-MSB_       ::= 0x2D

  static GRV-DATA-X-LSB_       ::= 0x2E
  static GRV-DATA-X-MSB_       ::= 0x2F
  static GRV-DATA-Y-LSB_       ::= 0x30
  static GRV-DATA-Y-MSB_       ::= 0x31
  static GRV-DATA-Z-LSB_       ::= 0x32
  static GRV-DATA-Z-MSB_       ::= 0x33

  static MAG-DATA-X-LSB_       ::= 0x0E
  static MAG-DATA-X-MSB_       ::= 0x0F
  static MAG-DATA-Y-LSB_       ::= 0x10
  static MAG-DATA-Y-MSB_       ::= 0x11
  static MAG-DATA-Z-LSB_       ::= 0x12
  static MAG-DATA-Z-MSB_       ::= 0x13

  static MODE-REGISTER_        ::= 0x3D
  static TRIGGER-REGISTER_     ::= 0x3F
  static CALIBRATION-REGISTER_ ::= 0x35

  reg_/serial.Registers ::= ?

  constructor dev/serial.Device:
    reg_ = dev.registers

    sleep --ms=700         // Sensor startup time according to datasheet 650ms, table 0-2
    // Check chip ID
    if (reg_.read-u8 CHIP-ID-REG_) != CHIP-ID_:
      throw "INVALID_CHIP"
    else:
      print_ "BNO055 sensor detected"
      reset  // The sensor is in config mode after POR. Set mode NDOF and wait at least 7ms (Data sheet, page 22)
      reg_.write-u8 MODE-REGISTER_ NDOF-MODE_
      sleep --ms=7

  /**
  Does a full reset of the sensor, including a 700ms sleep to allow for sensor startup time.
  */
  reset -> none:
      reg_.write-u8 TRIGGER-REGISTER_ 0x20
      sleep --ms=700         // Sensor startup time according to datasheet 650ms, table 0-2

  /**
  Returns heading, roll, and pitch (in degrees) in a three element list.
  */
  read-euler -> List:
    heading := reg_.read-i16-le EUL-DATA-X-LSB_
    roll    := reg_.read-i16-le EUL-DATA-Y-LSB_
    pitch   := reg_.read-i16-le EUL-DATA-Z-LSB_
    // Section 3.6.5.4, Table 3-29.
    // 1 degree = 16 LSB
    scale := 1 / 16.0
    return [ heading * scale, roll * scale, pitch * scale ]

  /**
  Returns quaternions in a four element list.
  */
  read-quaternion -> List:
    w := reg_.read-i16-le QUA-DATA-W-LSB_
    x := reg_.read-i16-le QUA-DATA-X-LSB_
    y := reg_.read-i16-le QUA-DATA-Y-LSB_
    z := reg_.read-i16-le QUA-DATA-Z-LSB_
    // Section 3.6.5.5, Table 3-31.
    // 1 Quaternion (unit less) = 2^14 LSB
    scale := 1.0 / (1 << 14)
    return [ w * scale, x * scale, y * scale, z * scale ]

  /**
  Returns the gyro readings (in Dps, degrees per second) for the x, y, and z-plane
    in a three element list.
  */
  read-gyro -> List:
    x := reg_.read-i16-le GYR-DATA-X-LSB_
    y := reg_.read-i16-le GYR-DATA-Y-LSB_
    z := reg_.read-i16-le GYR-DATA-Z-LSB_
    // Section 3.6.4.3, Table 3-22.
    // 1 Dps = 16 LSB.
    scale := 1 / 16.0
    return [ x * scale, y * scale, z * scale ]

  /**
  Returns the linear acceleration (in m/s^2) for the x, y, and z-plane in a three
    element list.
  */
  read-acceleration -> List:
    x := reg_.read-i16-le LIA-DATA-X-LSB_
    y := reg_.read-i16-le LIA-DATA-Y-LSB_
    z := reg_.read-i16-le LIA-DATA-Z-LSB_
    // Section 3.6.4.1, Table 3-17.
    // 1 m/s2 = 100 LSB.
    scale := 1 / 100.0
    return [ x * scale, y * scale, z * scale ]

  /**
  Deprecated. Use $read-acceleration instead.
  */
  read-linear-acceleration -> List:
    return read-acceleration

  /**
  Returns the gravity reading (in m/s^2) for the x, y, and z-directions in
    a three element list.
  */
  read-gravity -> List:
    x := reg_.read-i16-le GRV-DATA-X-LSB_
    y := reg_.read-i16-le GRV-DATA-Y-LSB_
    z := reg_.read-i16-le GRV-DATA-Z-LSB_
    // Section 3.6.5.7, Table 3-35.
    // 1 m/s2 = 100 LSB.
    scale := 1 / 100.0
    return [ x * scale, y * scale, z * scale ]

  /**
  Returns the magnetometer reading (in Gauss) for the x, y, and z-plane in a three element list.
  */
  read-magnetometer -> List:
    x := reg_.read-i16-le MAG-DATA-X-LSB_
    y := reg_.read-i16-le MAG-DATA-Y-LSB_
    z := reg_.read-i16-le MAG-DATA-Z-LSB_
    // Section 3.6.1.
    // Magnetic Field Strength in Micro Tesla.
    // 1 microtesla = 0.01 Gauss.
    scale := 1 / 100.0
    return [ x * scale, y * scale, z * scale]

  /**
  Returns calibration status for Magnetometer, Accelerometer, Gyroscope, and full system, in
    a four element list.

  The return values may be 0-3 where '0' indicates no calibration, and '3' indicates a fully calibrated reading.
  */
  read-calibration -> List:
    calibration := reg_.read-u8 CALIBRATION-REGISTER_
    mag :=  calibration & 0b11
    acc := (calibration >> 2) & 0b11
    gyr := (calibration >> 4) & 0b11
    sys := (calibration >> 6) & 0b11
    return [ mag, acc, gyr, sys ]

  /**
  Returns selected units for measurements in a 5 element list:
  1. Acceleration unit: 0 -> m/s^2, 1 -> mg
  2. Gyroscope unit: 0 -> Degrees/s, 1 -> Radians/s
  3. Euler unit: 0 -> Degrees, 1 -> Radians
  4. Temperature unit: 0 -> Celsius, 1 -> Fahrenheit
  5. Windows/Android Orientation: 0 -> Windows Orientation (Pitch turning clockwise increase values), 1 -> Android Orientation (Pitch turning clockwise decreases values)
  */
  read-units -> List:
    // Section 3.6.1.
    // Table 3-11 and Table 3-12.
    units := reg_.read-u8 UNIT-SEL_
    acc := units & 0b1
    gyr := (units >> 1) & 0b1
    eul := (units >> 2) & 0b1
    tmp := (units >> 4) & 0b1
    win-android-orient := (units >> 7) & 0b1
    return [ acc, gyr, eul, tmp, win-android-orient ]
