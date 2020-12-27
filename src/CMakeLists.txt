project(PyMotionDriver)
cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 14)

SET(SRC_LIST
        pybind11_wrap.cpp
        helper_3dmath.h
        I2Cdev/I2Cdev.h
        I2Cdev/I2Cdev.c
        inv_mpu_lib/dmpmap.h
        inv_mpu_lib/dmpKey.h
        inv_mpu_lib/inv_mpu.c
        inv_mpu_lib/inv_mpu.h
        inv_mpu_lib/inv_mpu_dmp_motion_driver.c
        inv_mpu_lib/inv_mpu_dmp_motion_driver.h
        )

add_definitions(-D MPU6050)

add_subdirectory(pybind11)
pybind11_add_module(PyMotionDriver ${SRC_LIST})