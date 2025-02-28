# Robot_TrackingSystem_Calibration

This repository contains the library developed at the Surgical Robotics Lab in IST, Portugal to perform the spatial calibration between a surgical robot and the Polaris Spectra optical navigation system.


## Build and Dependencies

This project can be used on both Windows and Linux operative systems:

### Windows

Start by installing [vcpkg](https://vcpkg.io/en/) on your local machine.

Navigate to your vcpkg directory and execute the following commands to install the necessary libraries:

``` bash
.\vcpkg.exe install ceres:x64-windows
.\vcpkg.exe install nlohmann-json:x64-windows
```

You also need the Visual Studio compiler, so please install the required tools. Once you have the compiler, you can open the bash window "x64 Native Tools Command Prompt for VS 2022" and perform the following steps in the desired installation directory:

```bash
git clone https://github.com/Manuel-Lourenco-Carvalho/Robot_OpticalSystem_Calibration.git
cd Robot_OpticalSystem_Calibration
mkdir build
cd build
cmake .. -DCMAKE_TOOLCHAIN_FILE="~path_to_vcpkg_direcoty\scripts\buildsystems\vcpkg.cmake" -DVCPKG_TARGET_TRIPLET=x64-windows -DCMAKE_BUILD_TYPE=Release -G "Ninja"
cmake --build .
```

To use the library in C++ projects, link your project to the calibration_solver target in CMake.

### Linux

First you will need to install the necessary compilers and the CMake package. On Ubuntu or Debian-based systems, you can do this by running the following command:

```
sudo apt-get install build-essential cmake
```

Install [vcpkg](https://vcpkg.io/en/) on your local machine, navigate to your vcpkg directory and execute the following commands to install the necessary libraries:

``` bash
./vcpkg install ceres:x64-linux
./vcpkg install nlohmann-json:x64-linux
```

Now navigate to the desired installation directory and run the following commands in the terminal to clone and compile the project:


```bash
git clone https://github.com/Manuel-Lourenco-Carvalho/Robot_OpticalSystem_Calibration.git
cd Robot_OpticalSystem_Calibration
mkdir build
cd build
cmake .. -DCMAKE_TOOLCHAIN_FILE="~path_to_vcpkg_direcoty/scripts/buildsystems/vcpkg.cmake" -DVCPKG_TARGET_TRIPLET=x64-linux -DCMAKE_BUILD_TYPE=Release -G "Ninja"
cmake --build .
```

To use the library in C++ projects, link your project to the calibration_solver target in CMake.

### Integration with a proper IDE

Alternatively, you can compile the project directly using an IDE such as Visual Studio Code (VS Code) with the vcpkg manifest (vcpkg.json) provided to compile the required libraries. For VSCode, follow the following steps;

1 - First install Git in your system if you do not have it installed yet.

2 - Install [vcpkg](https://vcpkg.io/en/) on your local machine.

3 - Install the lattest version of Mycrosoft Visual Studio - Community Edition which integrates C++ compilers in your system (Windows only).

4 - Now you can install vscode by downloading it from the website https://code.visualstudio.com/

5 - Once the download is finished you can open the vscode IDE, go to the extensions tab, install the C++ extension, the cmake extension and also install the vcpkg extension. 

6 - Once this is done, you have to enable vcpkg. For this, go to the page of the vcpkg extension and enable it or simply type ">vcpkg: Enable vcpkg" in the search bar. This should create a folder in your project called .vscode with a file inside it called settings.json. Change this file to the following contents:

```
{
    "cmake.configureArgs": [
        "-DVCPKG_APPLOCAL_DEPS=ON",
        "-DX_VCPKG_APPLOCAL_DEPS_INSTALL=ON",
        "-DVCPKG_TARGET_TRIPLET=x64-xxxxxxx",
        "-DVCPKG_MANIFEST_MODE=ON"
    ],
    "vcpkg.general.enable": true,
    "vcpkg.target.hostTriplet": "x64-xxxxxxx",
    "vcpkg.target.defaultTriplet": "x64-xxxxxxx",
    "cmake.generator":"Ninja",
    "vcpkg.target.useStaticLib": false,
    "cmake.configureSettings": {
        "CMAKE_TOOLCHAIN_FILE": "~path_to_vcpkg_direcoty/scripts/buildsystems/vcpkg.cmake"
    },
    "vcpkg.target.installDependencies": true,
    "vcpkg.target.preferSystemLibs": false,
    "vcpkg.target.useManifest": true
}
```
where, "x64-xxxxxxx" should be replaced with the proper triplet ("x64-windows" or "x64-linux"). Notice that we are forcing the cmake extension to pass the arguments of where vcpkg is installed in the line "CMAKE_TOOLCHAIN_FILE: "path to your vcpkg instalation directory".

You can now build and run the project from within the IDE. Ensure that the correct CMake kit is selected for your environment, and use the CMake Tools extension to configure and build the project.

This method simplifies managing dependencies and can be an easier way to work with the project, especially if you're familiar with VSCode.


## Test

To verify if the compilation was successful, an example file named *observations.json* is provided. To test, locate the executable file named *CalibrationSolver.exe*, which should be inside the "build" folder. You can execute it via the terminal or by simply double-clicking on it. Upon running, a file named *results.txt* will be generated in the project's main folder. This file contains the optimization results.

The creation of the *results.txt* file indicates that the compilation was successful.

Please note that the executable is designed to always look for the data file (*observations.json*) in the project's main folder, which is the same directory as the CMakeLists.txt file. If you wish to specify a different location for the data file, you can modify the following line in the *CMakeLists.txt* file to the desired path, then recompile the project:

``` cmake
add_definitions(-DPROJECT_DIR="~desired/path/to/observations.json/file")
```

The *results.txt* file will always be saved in the same directory as the *observations.json* file.


## Matlab integration

To make this project more user-friendly, even for novice programmers, we have integrated it with MATLAB.

We have developed several scripts and functions to facilitate solving the optimization problem. These scripts can perform tasks such as creating the *observations.json* file, plotting results, generating simulation data, conducting pre- and post-processing, and more.

The executable file that performs the optimization can also be run directly from MATLAB with:
``` matlab
system("~project_main_path/build/CalibrationsSolver.exe");
```

All those scripts and functions can be found inside the *matlab_utilities* folder.


## Citation

This library was developed to showcase our work to the scientific community. If you find this work useful in your own research, please cite our paper.

@article{Carvalho2025,
  title = {Hand-Eye Calibration Method for Robotic Surgery with Optical Navigation System},
  author = {Manuel Carvalho and João Oliveira and Rui Coelho and Jorge Martins},
  journal = { },
  year = {2025},
  month = {}
}



