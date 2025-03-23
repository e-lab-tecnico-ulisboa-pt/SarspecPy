# SarspecPy
Library to interface with Sarspec devices using Python

## Dependencies

This python library depends on the base Sarspec C++ device driver, on the Pybind11, libftdi1, and python-dev libraries.

## Compiling the module

Check the Pybind11 documentation for up-to-date instructions on how to compile the modules 

https://pybind11.readthedocs.io/en/stable/basics.html#compiling-the-test-cases

Currently, the library can be compiled with 

```
c++ -O3 -Wall -shared -std=c++11 -I/usr/include/python3.11 -fPIC $(python3 -m pybind11 --includes) sarspec-device.cpp sarspecpy.cpp  -o SarspecPy$(python3-config --extension-suffix) -lftdi1
```

## Using the module

Check the Pybind11 documentation for up-to-date instructions on how to use the compiled modules

https://pybind11.readthedocs.io/en/stable/compiling.html

Currently, the standard procedure to use the modules is to create a Python package, and then install it into a virtual environment. To use the module, activate the virtual environment and import the module. The same methods defined in the C++ library will then be available in Python
