# CPP API Sample

## Introduction

This repository contains a small example on how to use PLUX's CPP API to find, connect and adquire data, using either the internal bluetooth connection or using our [Fast USB Adaptor](https://plux.info/biosignalsplux-accessories/371-fast-usb-data-transfer-cable-for-biosignalsplux-820201514.html) (only for windows).

## Contents
In this repository you can find:
* The *.dlls* needed to comunicate with Fast USB Adaptor (windows only)
* plux.h - the header for plux's API
* simpleDev.cpp and .h - an example of a custom device implementation, which overload some of the base classes
* simpleApp.cpp - an example on how to find all PLUX's devices, connecting to one, configuring it, and start an acquisition

## Usage
To use this samples, simply clone this repository and link the correct *.lib/.a* for your operating system.
The API files can be found at this [link](https://downloads.plux.info/apis/PLUX-API-Cpp.zip), which support the following architectures:

* Win32/Win64 (VS2017)
* MacOS x86-64
* Linux x86-64

If you need the API compiled in other architectures, contact <support@plux.info>
