# 3D Printed Self-Balancing Bike

![IMG_5848](https://github.com/user-attachments/assets/9fa83c51-07a4-46f7-a9b4-701d071ffdea)

![IMG_5852](https://github.com/user-attachments/assets/1eedc93d-6ad5-421b-bca4-9a38b7f129d1)


## Table of Contents
- [Introduction](#introduction)
- [Components](#components)
- [3D Printing](#3d-printing)
- [Assembly](#assembly)
- [Arduino Programming](#arduino-programming)
- [Usage](#usage)
- [Contributing](#contributing)

## Introduction

This project is a **3D printed self-balancing bike** that uses a **reaction wheel** and an Inertial Measurement Unit **(IMU)** to maintain its balance. The bike is programmed using an **Arduino**, making it an excellent project for those interested in **robotics, electronics, and 3D printing.**

## Components

- **3D Printed Parts**
  - **1 frame**
  - **1 front_wheel_mount**
  - **1 left_rear_fork**
  - **1 right_rear_fork**
  - **1 reaction_wheel_mount**
  - **1 reaction_wheel**
  - **1 front_fork**
  - **2 wheel_rim**
  - **2 wheel_tyre**
  - **1 wheel_pulley**
  - **1 wheel_pulley_top**
  - **1 motor_pulley**
  - **1 motor_pulley_top**

- **Electronics**
  - **Arduino pro mini:** any atmega328p based arduino should work
  - **MPU-6050 IMU**
  - **DC-DC buck converter:** 24V -> 5V
  - **2 [Nidec 24H brushless motors](https://www.aliexpress.us/item/3256804723483727.html?gatewayAdapt=glo2usa4itemAdapt)**
  - **6s Li-Po battery:** max 1300mah, because of size
  - **MG995 servo**
  - **PWM receiver**

- **Hardware**
  - **80 M4 nuts:** you might need more
  - **21 M4 12mm screws:** you might need more
  - **4 M4 10mm screws**
  - **2 M4 15mm screws**
  - **4 M3 8mm screw**
  - **5 M3 nuts**
  - **5 M3 4-6mm grub screws:** the pulley screw has to be shorter than 6mm, the others 6mm
  - **3 M3 25mm screws**
  - **1 26mm & 1 37mm aluminium rod**
  - **1 240mm closed loop gt2 timing belt**
  - **4 608 bearings:** 8 X 22 X 7mm

## 3D Printing

Download the print files from the `self-balancing_bike_parts` directory or from my [Printables](https://www.printables.com/model/956136-self-balancing-bike)

### Recommended Print Settings

- **Material**: PLA or PETG for the tyres use Tpu
- **Layer Height**: 0.2mm
- **Infill**: 20% or higher for strength. Low % for the tyres
- **Supports**: Yes, where necessary

## Assembly

Use the assembly guide 'self_balancing

### Components Connection

- **MPU-6050**: Connect the SDA and SCL pins to the corresponding pins on the Arduino.
- **Motor and ESC**: Connect the ESC to the motor and the control signal wire to a PWM-capable pin on the Arduino.
- **Power Supply**: Connect the Li-Po battery to the ESC and ensure the Arduino is powered either through the ESC's BEC or an external power source.

## Arduino Programming

### Code

The Arduino code for the self-balancing bike can be found in the `self-balancing_bike_code` directory. Upload the code to your Arduino board using the Arduino IDE.

### Libraries

Ensure you have the `Wire.h` librarie installed.

You can install this librarie via the Arduino Library Manager.

### Configuration

Adjust the PID controller parameters in the code to suit your bike's balance dynamics. This may require some experimentation to get right.

## Usage

Once calibrated(don't move it the first seconds), your self-balancing bike is ready to use. Place it on a flat surface and let it balance itself.

## Contributing

Contributions are welcome! Feel free to fork this repository, make improvements, and submit a pull request.

---

Happy building and balancing! If you have any questions or need further assistance, feel free to open an issue in this repository.
