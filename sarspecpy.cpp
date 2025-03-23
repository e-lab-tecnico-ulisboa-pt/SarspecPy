#include "sarspec-device.h"
#define PY_SSIZE_T_CLEAN
#include <Python.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>


namespace py = pybind11;

PYBIND11_MODULE(SarspecPy, m) {
    m.doc() = "Python module for Sarspec Devices";

    py::class_<sarspec_usb::SarspecResDevice>(m,"SarspecResDevice")
        .def(py::init())
        .def("connect", &sarspec_usb::SarspecResDevice::connect)
        .def("disconnect", &sarspec_usb::SarspecResDevice::disconnect)
        .def("setLed", &sarspec_usb::SarspecResDevice::setLed)
        .def("getXData", &sarspec_usb::SarspecResDevice::getXData)
        .def("getYData", &sarspec_usb::SarspecResDevice::getYData)
        .def("getYDataSequence", &sarspec_usb::SarspecResDevice::getYDataSequence)
        .def("setIntegrationTime", &sarspec_usb::SarspecResDevice::setIntegrationTime)
        .def("setTimeout", &sarspec_usb::SarspecResDevice::setTimeout)
        .def("readEEPROMPage", &sarspec_usb::SarspecResDevice::readEEPROMPage)
        .def("setDark", &sarspec_usb::SarspecResDevice::setDark)
        .def("setGain", &sarspec_usb::SarspecResDevice::setGain)
        .def("getCurrentEEPROMDarkGain", &sarspec_usb::SarspecResDevice::getCurrentEEPROMDarkGain)
        .def("setDeviceGain", &sarspec_usb::SarspecResDevice::setDeviceGain);
        //.def("LibTest", &sarspec_usb::SarspecResDevice::LibTest);
}
