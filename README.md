# PyMotionDriver

A DMP available python wrapper for MPU6050, binded from [MotionSensorExample ](https://github.com/rpicopter/MotionSensorExample). and [pybind11](https://github.com/pybind/pybind11) in using.

This project aims to provide easy-to-use Python library for MPU6050. You can make some changes for MPU6050/MPU6500/MPU9150/MPU9250  reading.

## Features

- DMP available
- able to setup MPU6050 with personal args




## Build 

1.clone this repro

```shell
git clone --recursive https://github.com/JokinYang/PyMotionDriver.git
# or
git clone https://github.com/JokinYang/PyMotionDriver.git
git submodule init
git submodule update
```

2.build on your own patform

use cmake

```shell
cd your/dir/to/PyMotionDriver
mkdir ./src/build
cd ./src/build
cmake .. && make
```

or use setup.py

```shell
python3 setup.py build
```

gen *.pyi file for type hint

```
pip3 install mypy
cd /the/path/to/your/*.so/file
stubgen -m PyMotionDriver
```

## TODO

- [ ] provide a  MPU6050 data server
- [ ] add command line to setup PyMotionDriver