# Quadconn Motor Branch



# Installing Iceoryx2
Commands to build Cpp bindings:
```bash
git clone https://github.com/eclipse-iceoryx/iceoryx2.git
cd iceoryx2
cmake -S . -B target/release -DCMAKE_BUILD_TYPE=Release
cmake --build target/release
sudo cmake --install target/release --prefix /usr/local
```
To have Python bindings, run the following code:
```bash
pip install iceoryx2
```
Highly recommend creating in a virtual environment!

# How to run the code
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
