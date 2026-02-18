# Quadconn 

# Cloning With Submodules
This project uses git submodules to contain its dependencies within the repository. To 
clone the project and initialize the submodules run the following:
```bash
git clone --recursive https://github.com/Quadconn/quadconn.git 
```

# Installing Dependencies

## C++
Using your system package manager install
* c++ compiler
* rust compiler
* cmake
* [gstreamer](https://gstreamer.freedesktop.org/download/#linux)

All other dependencies can be installed by running the `external/boostrap.sh` script. Go to the 
external directory and run the script as such: `./bootstrap`.

## Python
After creating a python virtual environment use the `requirements.txt` file to install the 
dependencies as such:
```bash
pip install -r requirements.txt

```


# Building The Code
By default the build will create all executables. The commands to do this are:
```bash
cmake -B build      # Configure step (only needs to be ran once)
cmake --build build # Build step (run anytime you want to rebuild)
```

There are options that can be set during the build configuration to only build some of the
executables by passing arguments to cmake as such:
```bash
cmake -B build -DMOTOR=OFF -DPERCEPTION=OFF # -DCONTROL=OFF can be used to not build the control code
```
This will configure cmake to only build the control code each time `cmake --build build` is ran. 
Refer to the `CMakeLists.txt` file for more information about the build options.


# Running The Code
The CMakeFile currently creates individual executables for each node. Run them
once they are created in the build directory:

```bash
cmake -B build
cmake --build build
cd build
./Executable_of_choice
```
In the future, there will be run.sh scripts on the top directory to execute all code
at once, but running them individually allows each terminal to display debug info. 
