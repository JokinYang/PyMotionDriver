#!/usr/bin/env python3

from setuptools import setup, find_packages
from pybind11.setup_helpers import Pybind11Extension, build_ext

__version__ = "0.0.1"

with open("README.md", "r", encoding="utf-8") as fh:
	long_description = fh.read()

sources = ['src/pybind11_wrap.cpp', 'src/I2Cdev/I2Cdev.c', 'src/inv_mpu_lib/inv_mpu.c',
		   'src/inv_mpu_lib/inv_mpu_dmp_motion_driver.c', ]

ext_modules = [
	Pybind11Extension("PyMotionDriver", sources=sources, define_macros=[('MPU6050', True)], ),
]

setup(
	name="PyMotionDriver",
	version=__version__,
	author="Jokin Yang",
	author_email="yangjun97@foxmail.com",
	url=" ",
	description="description",
	long_description=long_description,
	ext_modules=ext_modules,
	packages=find_packages(exclude=['__pycache__']),
	install_requires=[
		'pybind11'
	],
	classifiers=[
		"Programming Language :: Python :: 3",
		"License :: OSI Approved :: MIT License",
		"Operating System :: OS Independent",
	],
	cmdclass={"build_ext": build_ext},
	zip_safe=False,
	python_requires='>=3.0',
)
