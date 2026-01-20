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
To have python bindings, run the following code:
```bash
pip install iceoryx2
```
highly reccomend creating in virtual environment!

# TODO:
1. Create separate executables in CMakeLists.txt for IPC
2. Create 