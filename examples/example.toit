// Copyright (C) 2021 Toitware ApS. All rights reserved.
// Use of this source code is governed by a MIT-style license that can be found
// in the LICENSE file.

import gpio
import i2c
import bno055

main:
  bus := i2c.Bus
    --sda=gpio.Pin 21
    --scl=gpio.Pin 22

  device := bus.device bno055.I2C_ADDRESS
  sensor := bno055.Driver device

  units := sensor.read_units
  print "Units: $units"

  while true:
    euler        := sensor.read_euler
    gyro         := sensor.read_gyro
    linacc       := sensor.read_acceleration
    gravity      := sensor.read_gravity
    quaternion   := sensor.read_quaternion
    magnetometer := sensor.read_magnetometer
    calibration  := sensor.read_calibration

    print_ "Heading: $(%6.1f euler[0]), Roll: $(%6.1f euler[1]), Pitch: $(%6.1f euler[2])"
    //print_ "Acc X:   $(%7f gyro[0]), Acc Y:  $(%7f gyro[1]), Acc Z:    $(%7f gyro[2])"
    //print_ "$(%5.2f quaternion[0]), $(%5.2f quaternion[1]), $(%5.2f quaternion[2]), $(%5.2f quaternion[3])"
    //print_ "X: $(%5.2f linacc[0]), Y: $(%5.2f linacc[1]), Z: $(%5.2f linacc[2])"
    //print_ "X: $(%5.2f gravity[0]), Y: $(%5.2f gravity[1]), Z: $(%5.2f gravity[2])"
    //print_ "X: $(%5.2f magnetometer[0]), Y: $(%5.2f magnetometer[1]), Z: $(%5.2f magnetometer[2])"
    sleep --ms=100
