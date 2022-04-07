# BNO055

A Toit driver for the Bosch BNO055 absolute orientation sensor.

The BNO055 is features an accelerometer, gyroscope, and a geomagnetic sensor. The sensor
  is packaged with an ARM Cortex-M0 based processor which digests all the data and
  provides easy-to-use quaternions, Euler angles or vectors.

## Usage

A simple usage example.

```
import gpio
import i2c
import bno055

main:
  euler := [0.0,0.0,0.0]

  bus := i2c.Bus
    --sda=gpio.Pin 21
    --scl=gpio.Pin 22

  device := bus.device bno055.I2C_ADDRESS
  sensor := bno055.Driver device

  euler = sensor.read_euler
  print "Heading: $(%6.1f euler[0]), Roll: $(%6.1f euler[1]), Pitch: $(%6.1f euler[2])"

```

See the `examples` folder for more examples.

## References

Datasheet: https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf

## Features and bugs

Please file feature requests and bugs at the [issue tracker][tracker].

[tracker]: https://github.com/nilwes/bno055/issues
