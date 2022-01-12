# BNO055 Toit Driver

A Toit driver for the Bosch BNO055 sensor. 

The smart sensor BNO055 is a System in Package (SiP) solution that integrates a triaxial 14-bit accelerometer, an accurate close-loop triaxial 16-bit gyroscope, a triaxial geomagnetic sensor and a 32-bit microcontroller running the BSX3.0 FusionLib software. This smart sensor is significantly smaller than comparable solutions. By integrating sensors and sensor fusion in a single device, the BNO055 makes integration easy, avoids complex multivendor solutions and thus simplifies innovations, e.g. novel applications such as IoT hardware. The BNO055 is the perfect choice for AR, immersive gaming, personal health and fitness, indoor navigation and any other application requiring context awareness. It is ideally suited for demanding applications such as augmented reality, navigation, gaming, robotics, or industrial applications.

## Usage

A simple usage example.

```
import gpio
import i2c
import ..src.bno055 as bno055

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

## Features and bugs

Please file feature requests and bugs at the [issue tracker][tracker].

[tracker]: https://github.com/nilwes/bno055/issues
