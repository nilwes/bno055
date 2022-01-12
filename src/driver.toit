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

  static CHIP_ID_                ::= 0xA0
  static CHIP_ID_REG_            ::= 0x00

  static CONFIG_MODE_            ::= 0x00
  static ACCONLY_MODE_           ::= 0x01
  static MAGONLY_MODE_           ::= 0x02
  static GYRONLY_MODE_           ::= 0x03
  static ACCMAG_MODE_            ::= 0x04
  static ACCGYRO_MODE_           ::= 0x05
  static MAGGYRO_MODE_           ::= 0x06
  static AMG_MODE_               ::= 0x07
  static IMUPLUS_MODE_           ::= 0x08
  static COMPASS_MODE_           ::= 0x09
  static M4G_MODE_               ::= 0x0A
  static NDOF_FMC_OFF_MODE_      ::= 0x0B
  static NDOF_MODE_              ::= 0x0C
  static UNIT_SEL_               ::= 0x3B

  static ACCEL_2G_               ::= 0x00  // For accel_range property
  static ACCEL_4G_               ::= 0x01  // Default
  static ACCEL_8G_               ::= 0x02
  static ACCEL_16G_              ::= 0x03
  static ACCEL_7_81HZ_           ::= 0x00  // For accel_bandwidth property
  static ACCEL_15_63HZ_          ::= 0x04
  static ACCEL_31_25HZ_          ::= 0x08
  static ACCEL_62_5HZ_           ::= 0x0C  // Default
  static ACCEL_125HZ_            ::= 0x10
  static ACCEL_250HZ_            ::= 0x14
  static ACCEL_500HZ_            ::= 0x18
  static ACCEL_1000HZ_           ::= 0x1C
  static ACCEL_NORMAL_MODE_      ::= 0x00  // Default. For accel_MODE_ property
  static ACCEL_SUSPEND_MODE_     ::= 0x20
  static ACCEL_LOWPOWER1_MODE_   ::= 0x40
  static ACCEL_STANDBY_MODE_     ::= 0x60
  static ACCEL_LOWPOWER2_MODE_   ::= 0x80
  static ACCEL_DEEPSUSPEND_MODE_ ::= 0xA0

  static GYRO_2000_DPS_          ::= 0x00  // Default. For gyro_range property
  static GYRO_1000_DPS_          ::= 0x01
  static GYRO_500_DPS_           ::= 0x02
  static GYRO_250_DPS_           ::= 0x03
  static GYRO_125_DPS_           ::= 0x04
  static GYRO_523HZ_             ::= 0x00  // For gyro_bandwidth property
  static GYRO_230HZ_             ::= 0x08
  static GYRO_116HZ_             ::= 0x10
  static GYRO_47HZ_              ::= 0x18
  static GYRO_23HZ_              ::= 0x20
  static GYRO_12HZ_              ::= 0x28
  static GYRO_64HZ_              ::= 0x30
  static GYRO_32HZ_              ::= 0x38  // Default
  static GYRO_NORMAL_MODE_       ::= 0x00  // Default. For gyro_MODE_ property
  static GYRO_FASTPOWERUP_MODE_  ::= 0x01
  static GYRO_DEEPSUSPEND_MODE_  ::= 0x02
  static GYRO_SUSPEND_MODE_      ::= 0x03
  static GYRO_ADV_PWRSAVE_MODE_  ::= 0x04

  static MAGNET_2HZ_             ::= 0x00  // For magnet_rate property
  static MAGNET_6HZ_             ::= 0x01
  static MAGNET_8HZ_             ::= 0x02
  static MAGNET_10HZ_            ::= 0x03
  static MAGNET_15HZ_            ::= 0x04
  static MAGNET_20HZ_            ::= 0x05  // Default
  static MAGNET_25HZ_            ::= 0x06
  static MAGNET_30HZ_            ::= 0x07
  static MAGNET_LOWPOWER_MODE_   ::= 0x00  // For magnet_operation_MODE_ property
  static MAGNET_REGULAR_MODE_    ::= 0x08  // Default
  static MAGNET_ENH_REG_MODE_    ::= 0x10
  static MAGNET_ACCURACY_MODE_   ::= 0x18
  static MAGNET_NORMAL_MODE_     ::= 0x00  // for magnet_power_MODE_ property
  static MAGNET_SLEEP_MODE_      ::= 0x20
  static MAGNET_SUSPEND_MODE_    ::= 0x40
  static MAGNET_FORCEMODE__MODE_ ::= 0x60  // Default

  static POWER_NORMAL_           ::= 0x00
  static POWER_LOW_              ::= 0x01
  static POWER_SUSPEND_          ::= 0x02
  static MODE_REGISTER_          ::= 0x3D
  static PAGE_REGISTER_          ::= 0x07
  static ACCEL_CONFIG_REGISTER_  ::= 0x08
  static MAGNET_CONFIG_REGISTER_ ::= 0x09
  static GYRO_CONFIG_0_REGISTER_ ::= 0x0A
  static GYRO_CONFIG_1_REGISTER_ ::= 0x0B
  static CALIBRATION_REGISTER_   ::= 0x35
  static OFFSET_ACCEL_REGISTER_  ::= 0x55
  static OFFSET_MAGNET_REGISTER_ ::= 0x5B
  static OFFSET_GYRO_REGISTER_   ::= 0x61
  static RADIUS_ACCEL_REGISTER_  ::= 0x67
  static RADIUS_MAGNET_REGISTER_ ::= 0x69
  static TRIGGER_REGISTER_       ::= 0x3F
  static POWER_REGISTER_         ::= 0x3E
  static ID_REGISTER_            ::= 0x00

  // Axis remap registers and values
  static AXIS_MAP_CONFIG_REG_    ::= 0x41
  static AXIS_MAP_SIGN_REG_      ::= 0x42
  static AXIS_REMAP_X_           ::= 0x00
  static AXIS_REMAP_Y_           ::= 0x01
  static AXIS_REMAP_Z_           ::= 0x02
  static AXIS_REMAP_POSITIVE_    ::= 0x00
  static AXIS_REMAP_NEGATIVE_    ::= 0x01 

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
  Returns heading, roll, and pitch in a three element list.
  */
  read_euler -> List:
    euler_heading := (reg_.read_i16_le 0x1A).to_float / 16 //Convert the value to an appropriate range (section 3.6.4)
    euler_roll    := (reg_.read_i16_le 0x1C).to_float / 16  
    euler_pitch   := (reg_.read_i16_le 0x1E).to_float / 16
    return [euler_heading, euler_roll, euler_pitch]

  /**
  Returns quaternions in a four element list
  */
  read_quaternion -> List:
    quaternion_w := (reg_.read_i16_le 0x20).to_float
    quaternion_x := (reg_.read_i16_le 0x22).to_float
    quaternion_y := (reg_.read_i16_le 0x24).to_float
    quaternion_z := (reg_.read_i16_le 0x26).to_float
    scale := (1.0 / (1 << 14))

    return [scale * quaternion_w, scale * quaternion_x, scale * quaternion_y, scale * quaternion_z]
  
  /**
  Returns the gyro readings for the x, y, and z-plane in a three element list.
  */
  read_gyro -> List:
    gyro_x := (reg_.read_i16_le 0x14).to_float / 16
    gyro_y := (reg_.read_i16_le 0x16).to_float / 16
    gyro_z := (reg_.read_i16_le 0x18).to_float / 16
    return [gyro_x, gyro_y, gyro_z]

  /**
  Returns the linear acceleration for the x, y, and z-plane in a three element list.
  */
  read_linear_acceleration -> List:
    lin_acc_x := (reg_.read_i16_le 0x28).to_float / 100
    lin_acc_y := (reg_.read_i16_le 0x2A).to_float / 100
    lin_acc_z := (reg_.read_i16_le 0x2C).to_float / 100
    return [lin_acc_x, lin_acc_y, lin_acc_z]
    
  /**
  Returns the gravity reading for the x, y, and z-plane in a three element list.
  */
  read_gravity -> List:
    gravity_x := (reg_.read_i16_le 0x2E).to_float / 100
    gravity_y := (reg_.read_i16_le 0x30).to_float / 100
    gravity_z := (reg_.read_i16_le 0x32).to_float / 100
    return [gravity_x, gravity_y, gravity_z]

  /**
  Returns the magnetometer reading for the x, y, and z-plane in a three element list.
  */
  read_magnetometer -> List:
    mag_x := (reg_.read_i16_le 0x0E).to_float / 100
    mag_y := (reg_.read_i16_le 0x10).to_float / 100
    mag_z := (reg_.read_i16_le 0x12).to_float / 100
    return [mag_x, mag_y, mag_z]

  /**
  Returns calibration status for Magnetometer, Accelerometer, Gyroscope, and full system, in a four element list. The return values may be 0-3 where '0' indicates no calibration, and '3' indicates a fully calibrated reading.
  */
  read_calibration -> List:
    calibration := reg_.read_u8 CALIBRATION_REGISTER_
    mag_cal :=  calibration & 0b00_00_00_11
    acc_cal := (calibration & 0b00_00_11_00) >> 2
    gyr_cal := (calibration & 0b00_11_00_00) >> 4
    sys_cal := (calibration & 0b11_00_00_00) >> 6
    return [mag_cal, acc_cal, gyr_cal, sys_cal]

  /**
  Returns selected units for measurements in a 5 element list:
  1. Acceleration unit: 0 -> m/s^2, 1 -> mg
  2. Gyroscope unit: 0 -> Degrees/s, 1 -> Radians/s
  3. Euler unit: 0 -> Degrees, 1 -> Radians
  4. Temperature unit: 0 -> Celsius, 1 -> Farenheit
  5. Windows/Android Orientation: 0 -> Windows Orientation (Pitch turning clockwise increase values), 1 -> Android Orientation (Pitch turning clockwise decreases values)
  */
  read_units -> List:
    units := reg_.read_u8 UNIT_SEL_
    acc_unit := (units & 0b00000001) 
    gyr_unit := (units & 0b00000010) >> 1
    eul_unit := (units & 0b00000100) >> 2
    tmp_unit := (units & 0b00010000) >> 4
    win_android_orient := (units & 0b10000000) >> 7
    return [acc_unit, gyr_unit, eul_unit, tmp_unit, win_android_orient]
