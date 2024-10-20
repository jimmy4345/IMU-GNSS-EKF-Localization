# IMU-GNSS-EKF-Localization

**Still developing**

## Introduction

This is an Extended Kalman Filter(EKF)-based GNSS/INS integrated navigation system. It implements the fusion of GNSS positioning results and IMU data.


## Folder Structure
```bash
IMU-GNSS-EKF-Localization/
│  .gitignore
│  CMakeLists.txt
│  LICENSE
│  README.md
│
├─config
│      config.yaml
│
├─include
│      Angle.hpp
│      Earth.hpp
│      Global_defs.hpp
│      Rotation.hpp
│
├─sample_data
├─src
│      Angle.cpp
│      Earth.cpp
│      Rotation.cpp
│      test_function_valid.cpp
│
├─tests
└─Thirdparty
    ├─eigen-3.4.0
    └─yaml-cpp-0.7.0
```

## Program Dependencies

Dependencies are list in the thirdparty directory and this program requires these third party dependencies:

1. **Eigen3**: `Eigen` is a C++ template library for linear algebra.
2. **yaml-cpp**:  `yaml-cpp` is a [YAML](http://www.yaml.org/) parser and emitter in C++ matching the [YAML 1.2 spec](http://www.yaml.org/spec/1.2/spec.html).

TBD


## Compile the program

### 1. Compile under Windows

> [!IMPORTANT]  
> Since I use the MinGW64 in my Windows 10, I suggest to use MinGW64 as c++ compiler. See more information [here](https://blog.csdn.net/zhaotun123/article/details/100042073)

After preparing your own compilation environment, open a PowerShell or CMD terminal in the project directory:
```bash
mkdir build
cd build
cmake -G "MinGW Makefiles" ..
mingw32-make
```

### 2. Compile under Ubuntu
TBD



## Exccute the fusion program

### 1. Change the config.yaml setting
TBD

### 2. Execute the program
```bash
# Go inside the build directory
cd build/
# Execute the sample data
./EKF_Localization.exe
```



TBD

