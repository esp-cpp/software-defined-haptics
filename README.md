# BLDC Motor Test Stand

This project contains the MCAD and source files for a BLDC motor test stand,
designed to enable running various experiments in openloop control, closed loop
control, and user interface / haptics using a small BLDC motor that's commonly
used in camera gimbals.

## Hardware

* TinyS3
* BLDC Motor (I used [these A and B motors](https://www.aliexpress.us/item/3256802907900422.html) from aliexpress)
* MT6701 Magnetic Encoder Breakout (I used [this purple one (color B)](https://www.aliexpress.us/item/3256804851103272.html) from aliexpress)
* TMC6300 BOB
* Mini solderless breadboard
* 3d printed test stand enclosure (.stl and source files in the [mcad](./mcad) directory)
* Benchtop power supply (currently running at 5V 1A so many things should work)

:warning:
> NOTE: you MUST make sure that you run this code with the
> `zero_electrical_offset` value set to 0 (or not provided) at least once
> otherwise the sample will not work and could potentially damage your motor.

### Build and Flash

Build the project and flash it to the board, then run monitor tool to view serial output:

```
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.

## Output

The haptics (detent configuration, click/buzz) can be configured dynamically at
run-time using the provided CLI, see screenshot below:

![CleanShot 2023-06-23 at 13 23 44](https://github.com/esp-cpp/bldc_test_stand/assets/213467/eb2a2f37-01d0-46e3-992a-48820401c0ab)

As you can see, the cli also allows you to start and stop the haptic engine
(default is off when the program starts) and allows you to query the position of
the motor based on the current detent config. The default detent config is the
unbounded_no_detents configuration.

For more information, see the documentation or the original PR:
https://github.com/esp-cpp/espp/pull/60

Some examples:

### coarse values strong detents (best with sound)

https://github.com/esp-cpp/espp/assets/213467/a256b401-6e45-4284-89c7-2dec9a49daa7

### magnetic detents (best with sound)

https://github.com/esp-cpp/espp/assets/213467/ab1ace5c-f967-4cfc-b304-7736fdb35bcb

### On / Off Strong Detents (best with sound)

https://github.com/esp-cpp/espp/assets/213467/038d79b1-7cd9-4af9-b7e8-1b4daf6a363a

### Multi-rev no detents

https://github.com/esp-cpp/espp/assets/213467/2af81edb-67b8-488b-ae7a-3549be36b8cc

## Troubleshooting

Make sure to run the code once with `zero_electrical_offset` set to 0 so that
the motor will go through a calibration / zero offset routine. At the end of
this startup routine it will print the measured zero electrical offset that you
can then provide within the code, at which point it will not need to run the
calibration routine.

You must run this calibration any time you change your hardware configuration
(such as by remounting your motor, magnet, encoder chip).

## Code Breakdown

This example is relatively complex, but builds complex haptic behavior using the
following components:

* `espp::Mt6701`
* `espp::BldcDriver`
* `espp::BldcMotor`
* `espp::BldcHaptics`
* ESP-IDF's `i2c` peripheral driver

You combine the `Mt6701` and `BldcDriver` together when creating the `BldcMotor`
and then simply pass the `BldcMotor` to the `BldcHaptics` component. At that
point, you only have to interface to the `BldcHaptics` to read the input
position or reconfigure the haptics.
