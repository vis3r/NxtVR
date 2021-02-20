## Hardware

#### Main
- STM32F103 (robotdyn)
- BMX055 (cjmcu-055-9dof)
- ST-Link (v2?)

#### Utility
- female-female jumper wires

## Setup

#### Wiring
- Power, connect both BMX<3V3, GND> to it's respective pins on STM32
- Serial wires, be wary of differences between STM32 boards (Blue-pill, black-pill, robotdyn black-color-but-like-bluepill)
  - Robotdyn STM32
    - BMX\<SCL> <----> STM32\<B6>
    - BMX\<SDA> <----> STM32\<B7>
  - Others?
- BMX\<PS> <----> 3V3 (HIGH for I2C, LOW for SPI, also seems that LOW is "default")


#### Software
- ST-Link driver [Windows](https://www.st.com/en/development-tools/st-link-v2.html#tools-software), Ubuntu apt package: `stlink-tools`
- Arduino IDE
  - Linux\Ubuntu: Works with repository package, v1.8.9
- Libraries:
  - See setup of [stm32duino](https://github.com/rogerclarkmelbourne/Arduino_STM32/wiki/Installation)
- Sketch specific libraries are currently inside src folder, for now there is no need to install additional libs thru ArduinoIDE 

## Notes
- If you are connecting (thus powering) your STM32 thru USB cable, you don't have to connect pin ST-Link\<3V3> to STM32<3V3>
- Check [this](https://github.com/vis3r/NxtVR-alpha-code/blob/master/sketch_usbserial_example/README.md) for our udev rules
