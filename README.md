# 3D Printed Self-Balancing Bike



## Table of Contents
- [Introduction](#introduction)
- [Components](#components)
- [3D Printing](#3d-printing)
- [Assembly](#assembly)
- [Arduino Programming](#arduino-programming)
- [Usage](#usage)
- [Contributing](#contributing)
- [License](#license)

## Introduction

This project is a 3D printed self-balancing bike that uses a reaction wheel and an Inertial Measurement Unit (IMU) to maintain its balance. The bike is programmed using an Arduino, making it an excellent project for those interested in robotics, electronics, and 3D printing.

## Components

- **3D Printed Parts**
  - Frame
  - Fork
  - Wheels
  - Reaction Wheel Mount
  - Miscellaneous connectors and brackets

- **Electronics**
  - Arduino Uno or similar
  - MPU-6050 IMU
  - Brushless DC motor and ESC (Electronic Speed Controller) for the reaction wheel
  - Li-Po battery
  - Motor driver (if needed)
  - Wires and connectors

- **Hardware**
  - Screws, nuts, and bolts
  - Bearings
  - Axles

## 3D Printing

All the parts required for the bike are designed to be 3D printed. The STL files can be found in the `3D_Models` directory.

### Recommended Print Settings

- **Material**: PLA or ABS
- **Layer Height**: 0.2mm
- **Infill**: 20% or higher for strength
- **Supports**: Yes, where necessary

## Assembly



### Components Connection

- **MPU-6050**: Connect the SDA and SCL pins to the corresponding pins on the Arduino.
- **Motor and ESC**: Connect the ESC to the motor and the control signal wire to a PWM-capable pin on the Arduino.
- **Power Supply**: Connect the Li-Po battery to the ESC and ensure the Arduino is powered either through the ESC's BEC or an external power source.

## Arduino Programming

### Code

The Arduino code for the self-balancing bike can be found in the `Code` directory. Upload the code to your Arduino board using the Arduino IDE.

### Libraries

Ensure you have the following libraries installed:
- `Wire.h`
- `MPU6050.h`

You can install these libraries via the Arduino Library Manager.

### Configuration

Adjust the PID controller parameters in the code to suit your bike's balance dynamics. This may require some experimentation to get right.

## Usage

Once calibrated, your self-balancing bike is ready to use. Place it on a flat surface and let it balance itself. You can further modify the code to add remote control capabilities or other features.

## Contributing

Contributions are welcome! Feel free to fork this repository, make improvements, and submit a pull request.

## License

This project is licensed under the MIT License. See the `LICENSE` file for more details.

---

Happy building and balancing! If you have any questions or need further assistance, feel free to open an issue in this repository.
