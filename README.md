# Acquire driver for Spinnaker

This is an Acquire driver that supports some Teledyne FLIR cameras using the Spinnaker SDK.

## Usage

The intended way to use this library is as part of the [acquire-imaging](https://github.com/acquire-project/acquire-python)
Python package, which also includes Acquire's runtime as well as other drivers.

That package can be installed using the following command in an environment with Python and `pip`.

```
python -m pip install acquire-imaging
```

### Prerequisites

Before using this driver you must install the [Spinnaker SDK](https://www.flir.com/products/spinnaker-sdk/?vertical=machine+vision&segment=iis).
This driver is built against version 3.1.0.79 of the SDK and you should install exactly the same version.

### Supported cameras

The following cameras have been tested using this driver.

- [Blackfly USB3: BFLY-U3-23S6M-C](https://www.flir.com/products/blackfly-usb3/?model=BFLY-U3-23S6M-C&vertical=machine+vision&segment=iis)
- [Oryx 10GigE: ORX-10GS-51S5M-C](https://www.flir.com/products/oryx-10gige/?model=ORX-10GS-51S5M-C&vertical=machine+vision&segment=iis)

### Supported operating systems

Windows is the primary usage target.
But both Windows and macOS are currently supported for development purposes.

## Development

We welcome contributors. The following will help you get started building the code.

### Environment

Requires

- CMake 3.13+ from its [download page](https://cmake.org/download/), or via
  [chocolatey](https://community.chocolatey.org/packages/cmake) or [homebrew](https://formulae.brew.sh/formula/cmake).
- A C++20 compiler such as [Microsoft Visual Studio](https://visualstudio.microsoft.com/downloads/) or [Clang](https://clang.llvm.org/).


### Configure

From the repository root, run the following commands to configure the CMake build files.

```
mkdir build
cd build
cmake ..
```

### Build

After configuration, run the following command to build the library and tests.

```
cmake --build .
```

### Test

After building the default debug configuration, run the following command to run all the tests using CTest.

```
ctest -C Debug
```

You will need all supported cameras connected to the device that you're testing on for all tests to pass.

If you only want to run a subset of the tests you can use CTest's `-R` option to specify a pattern to match against.
For example, if you only want to run the oryx tests, run the following command.

```
ctest -C Debug -R oryx
```
