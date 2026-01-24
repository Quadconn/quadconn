# Quadconn

## Run With Visualization
```
gepetto-gui & python main.py
```

## Developer Setup
Install the following necessary conda packages into your preferred conda environment:
```
conda install -c conda-forge pinocchio
conda install -c conda-forge compilers
```

# Configuring and Building
From the project root run the following from within your conda environment:
```
mkdir build && cd build

cmake .. \
    -DCMAKE_PREFIX_PATH=$CONDA_PREFIX \
    -DCMAKE_CXX_COMPILER=$CONDA_PREFIX/bin/x86_64-conda-linux-gnu-g++
```
