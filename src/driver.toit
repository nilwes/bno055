// Copyright (C) 2021 Toitware ApS. All rights reserved.
// Use of this source code is governed by a MIT-style license that can be found
// in the LICENSE file.

// BNO055 data sheet: https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf

import binary
import serial.device as serial
import serial.registers as serial

import math

I2C_ADDRESS         ::= 0x28
I2C_ADDRESS_ALT     ::= 0x29

/**
Driver for the Bosch BNO055 sensor, using either I2C or SPI.
*/
class Driver:
  //Device registers

  static CHIP_ID_              ::= 0xA0
  static CHIP_ID_REG_          ::= 0x00

  static NDOF_MODE_            ::= 0x0C
  static UNIT_SEL_             ::= 0x3B

  static EUL_DATA_X_LSB_       ::= 0x1A
  static EUL_DATA_X_MSB_       ::= 0x1B
  static EUL_DATA_Y_LSB_       ::= 0x1C
  static EUL_DATA_Y_MSB_       ::= 0x1D
  static EUL_DATA_Z_LSB_       ::= 0x1E
  static EUL_DATA_Z_MSB_       ::= 0x1F

  static QUA_DATA_W_LSB_       ::= 0x20
  static QUA_DATA_W_MSB_       ::= 0x21
  static QUA_DATA_X_LSB_       ::= 0x22
  static QUA_DATA_X_MSB_       ::= 0x23
  static QUA_DATA_Y_LSB_       ::= 0x24
  static QUA_DATA_Y_MSB_       ::= 0x25
  static QUA_DATA_Z_LSB_       ::= 0x26
  static QUA_DATA_Z_MSB_       ::= 0x27

  static GYR_DATA_X_LSB_       ::= 0x14
  static GYR_DATA_X_MSB_       ::= 0x15
  static GYR_DATA_Y_LSB_       ::= 0x16
  static GYR_DATA_Y_MSB_       ::= 0x17
  static GYR_DATA_Z_LSB_       ::= 0x18
  static GYR_DATA_Z_MSB_       ::= 0x19

  static LIA_DATA_X_LSB_       ::= 0x28
  static LIA_DATA_X_MSB_       ::= 0x29
  static LIA_DATA_Y_LSB_       ::= 0x2A
  static LIA_DATA_Y_MSB_       ::= 0x2B
  static LIA_DATA_Z_LSB_       ::= 0x2C
  static LIA_DATA_Z_MSB_       ::= 0x2D

  static GRV_DATA_X_LSB_       ::= 0x2E
  static GRV_DATA_X_MSB_       ::= 0x2F
  static GRV_DATA_Y_LSB_       ::= 0x30
  static GRV_DATA_Y_MSB_       ::= 0x31
  static GRV_DATA_Z_LSB_       ::= 0x32
  static GRV_DATA_Z_MSB_       ::= 0x33

  static MAG_DATA_X_LSB_       ::= 0x0E
  static MAG_DATA_X_MSB_       ::= 0x0F
  static MAG_DATA_Y_LSB_       ::= 0x10
  static MAG_DATA_Y_MSB_       ::= 0x11
  static MAG_DATA_Z_LSB_       ::= 0x12
  static MAG_DATA_Z_MSB_       ::= 0x13

  static MODE_REGISTER_        ::= 0x3D
  static TRIGGER_REGISTER_     ::= 0x3F
  static CALIBRATION_REGISTER_ ::= 0x35

  reg_/serial.Registers ::= ?

  constructor dev/serial.Device:
    reg_ = dev.registers

    sleep --ms=700         // Sensor startup time according to datasheet 650ms, table 0-2
    // Check chip ID
    if (reg_.read_u8 CHIP_ID_REG_) != CHIP_ID_:
      throw "INVALID_CHIP"
    else:
      print_ "BNO055 sensor detected"
      reset  // The sensor is in config mode after POR. Set mode NDOF and wait at least 7ms (Data sheet, page 22)
      reg_.write_u8 MODE_REGISTER_ NDOF_MODE_
      sleep --ms=7

  /**
  Does a full reset of the sensor, including a 700ms sleep to allow for sensor startup time.
  */
  reset -> none:
      reg_.write_u8 TRIGGER_REGISTER_ 0x20
      sleep --ms=700         // Sensor startup time according to datasheet 650ms, table 0-2

  /**
  Returns heading, roll, and pitch (in degrees) in a three element list.
  */
  read_euler -> List:
    heading := reg_.read_i16_le EUL_DATA_X_LSB_
    roll    := reg_.read_i16_le EUL_DATA_Y_LSB_
    pitch   := reg_.read_i16_le EUL_DATA_Z_LSB_
    // Section 3.6.5.4, Table 3-29.
    // 1 degree = 16 LSB
    scale := 1 / 16.0
    return [ heading * scale, roll * scale, pitch * scale ]

  /**
  Returns quaternions in a four element list.
  */
  read_quaternion -> List:
    w := reg_.read_i16_le QUA_DATA_W_LSB_
    x := reg_.read_i16_le QUA_DATA_X_LSB_
    y := reg_.read_i16_le QUA_DATA_Y_LSB_
    z := reg_.read_i16_le QUA_DATA_Z_LSB_
    // Section 3.6.5.5, Table 3-31.
    // 1 Quaternion (unit less) = 2^14 LSB
    scale := 1.0 / (1 << 14)
    return [ w * scale, x * scale, y * scale, z * scale ]

  /**
  Returns the gyro readings (in Dps, degrees per second) for the x, y, and z-plane
    in a three element list.
  */
  read_gyro -> List:
    x := reg_.read_i16_le GYR_DATA_X_LSB_
    y := reg_.read_i16_le GYR_DATA_Y_LSB_
    z := reg_.read_i16_le GYR_DATA_Z_LSB_
    // Section 3.6.4.3, Table 3-22.
    // 1 Dps = 16 LSB.
    scale := 1 / 16.0
    return [ x * scale, y * scale, z * scale ]

  /**
  Returns the linear acceleration (in m/s^2) for the x, y, and z-plane in a three
    element list.
  */
  read_acceleration -> List:
    x := reg_.read_i16_le LIA_DATA_X_LSB_
    y := reg_.read_i16_le LIA_DATA_Y_LSB_
    z := reg_.read_i16_le LIA_DATA_Z_LSB_
    // Section 3.6.4.1, Table 3-17.
    // 1 m/s2 = 100 LSB.
    scale := 1 / 100.0
    return [ x * scale, y * scale, z * scale ]

  /**
  Deprecated. Use $read_acceleration instead.
  */
  read_linear_acceleration -> List:
    return read_acceleration

  /**
  Returns the gravity reading (in m/s^2) for the x, y, and z-plane in
    a three element list.
  */
  read_gravity -> List:
    x := reg_.read_i16_le GRV_DATA_X_LSB_
    y := reg_.read_i16_le GRV_DATA_Y_LSB_
    z := reg_.read_i16_le GRV_DATA_Z_LSB_
    // Section 3.6.5.7, Table 3-35.
    // 1 m/s2 = 100 LSB.
    scale := 1 / 100.0
    return [ x * scale, y * scale, z * scale ]

  /**
  Returns the magnetometer reading (in Gauss) for the x, y, and z-plane in a three element list.
  */
  read_magnetometer -> List:
    x := reg_.read_i16_le MAG_DATA_X_LSB_
    y := reg_.read_i16_le MAG_DATA_Y_LSB_
    z := reg_.read_i16_le MAG_DATA_Z_LSB_
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
  read_calibration -> List:
    calibration := reg_.read_u8 CALIBRATION_REGISTER_
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
  read_units -> List:
    // Section 3.6.1.
    // Table 3-11 and Table 3-12.
    units := reg_.read_u8 UNIT_SEL_
    acc := units & 0b1
    gyr := (units >> 1) & 0b1
    eul := (units >> 2) & 0b1
    tmp := (units >> 4) & 0b1
    win_android_orient := (units >> 7) & 0b1
    return [ acc, gyr, eul, tmp, win_android_orient ]
