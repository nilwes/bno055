# BNO055 Toit Driver

A Toit driver for the BNO055 sensor

## Usage

A simple usage example.

```
import gpio
import i2c
import ..src.bno055 as bno055

main:
  euler        := [0.0,0.0,0.0]

  bus := i2c.Bus
    --sda=gpio.Pin 21
    --scl=gpio.Pin 22

  device := bus.device bno055.I2C_ADDRESS
  sensor := bno055.Driver device

  euler = sensor.read_euler
  print_ "Heading: $(%6.1f euler[0]), Roll: $(%6.1f euler[1]), Pitch: $(%6.1f euler[2])"

```

See the `examples` folder for more examples.

## Features and bugs

Please file feature requests and bugs at the [issue tracker][tracker].

[tracker]: https://github.com/nilwes/bno055/issues
