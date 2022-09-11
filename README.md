# CAN Bus MagnetSensor for XCVario

Read Magnetsensor via I2C and send X,Y,Z raw data of 3D Hall sensors via CAN bus protocol to XCVario


## How to use

Follow detailed instructions provided specifically for an ESP32-C3 example project:

Select the instructions depending on Espressif chip installed on your development board:

- [ESP32 Getting Started Guide](https://docs.espressif.com/projects/esp-idf/en/stable/get-started/index.html)
- [ESP32-C3 Getting Started Guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32c3/get-started/index.html)


## folder contents

The project **CanMagSens** contains one source files in C++ language. The files are located in folder [main](main).

ESP-IDF projects are built using CMake. The project build configuration is contained in `CMakeLists.txt` files that provide set of directives and instructions describing the project's source files and targets (executable, library, or both). 

Below is short explanation of remaining files in the project folder.

```
├── CMakeLists.txt
├── main
│   ├── CMakeLists.txt
│   ├── component.mk           Component make file
│   └── *.cpp
├── Makefile                   Makefile used by legacy GNU Make
└── README.md                  This is the file you are currently reading
```

The follwing procedure shall be okay to build the software when done from scratch:

1) Install esp-idf v4.3 for esp32c3 chip:
apt install python3.10-venv; mkdir -p ~/esp; cd ~/esp; git clone --recursive https://github.com/espressif/esp-idf.git; cd ~/esp/esp-idf; git checkout release/v4.3; ./install.sh esp32c3; git submodule update --init; alias get_idf='source ~/esp/esp-idf/export.sh';
2) Build software:
git clone git@github.com:iltis42/CanMagSens.git; cd CanMagSens/; get_idf; idf.py build

For more information on structure and contents of ESP-IDF projects, please refer to Section [Build System](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/build-system.html) of the ESP-IDF Programming Guide.

## Troubleshooting

* Program upload failure

    * Hardware connection is not correct: run `idf.py -p PORT monitor`, and reboot your board to see if there are any output logs.
    * The baud rate for downloading is too high: lower your baud rate in the `menuconfig` menu, and try again.

## Technical support and feedback

Please use the following feedback channels:

* For technical queries, go to the [esp32.com](https://esp32.com/) forum
* For a feature request or bug report, create a [GitHub issue](https://github.com/espressif/esp-idf/issues)

We will get back to you as soon as possible.
