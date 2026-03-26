# REE-TraPla: Runtime- and Energie-Efficient Dynamic Game-Theoretic Trajectory Planner
This repository uses 
- **vectorization** to acclerate a [trajectory planner](https://github.com/giovannilucente/dynamic_game_trajectory_planner.git) based on **Generalized Nash Equilibrium (GNE)**. 
- **nondimensionalization** to reduce the precision loss caused by reducing bit-width of data.


This repository is a modified implementation of the original https://github.com/AImotion-Bavaria/REE-TraPla - implementation of vectorization and nondimensionalization and a script to show the effect of the intersection scenario. A new posit implementation on the scalar version is done to understand the impact of 32 bit posit from the universal number library(https://github.com/stillwater-sc/universal) on the precision of the trajectory values against 32 bit IEEE floats. A python evaluation script calculates the error difference of 32 bit posit and 32 bit float against ground truth of 64 bit float.

Original Authors: Wenguang Xu, Giovanni Lucente, and Richard Membarth.
For further information please check the related paper accepted by IV 2026: 

```bibtex
@INPROCEEDINGS{REE_TraPla_Xu_2026,
  author={Xu, Wenguang and Lucente, Giovanni and Membarth, Richard.},
  booktitle={2026 IEEE Intelligent Vehicles Symposium (IV)},
  title={Energy- and Runtime-Efficient Trajectory Planning via SIMD Vectorization},
  year={2026},
  organization={IEEE}
}
```

## Installation and Setup:
To install and run this project locally, follow these steps:

### 1. Clone the Repository
First, clone the repository to your local machine:
```bash
git clone git@github.com:https:Sahniel07/REE-TraPla-posit.git
cd REE-TraPla-posit

```

### 2. Install C++ Dependencies
```bash
sudo apt update
sudo apt install libeigen3-dev cmake g++
```
For this implementation we do not need an ispc compiler as we work with the scalar version, if needed install from https://github.com/ispc/ispc.git. You may need also user's guide during customized code: https://ispc.github.io/ispc.html.


### 3. Build and Run the Project (CMake)
The main file is in /src/main.cpp. There you will find the definition of scenario for intersection. This scenario is designed to see the difference between with and without nondimensionalization. For the new posit implementation, we will test it without nondimensionalization.

#### Create build folder
```bash
mkdir build
cd build
cmake ..
```
There are different configurations in CMakeLists. A suggestion is to use ccmake to modify the configuration by
```bash
ccmake .
```
A package installation of ccmake may needed depend on your system.
There are five configurations: ENABLE_POSIT is a new implementation

1. **ENABLE_AARCH64**: decide if you are working on a arm or x86 processor. This is relevant to the ispc compilation. Turn OFF for new implementation.
2. **ENABLE_QUANTIZATION**: use or don't use nondimensionalization. Turn OFF for new implementation.
3. **REAL_BITS**: the bit-width of data. The default is 64. Set it to 32 for measurement.
4. **USE_RECORDER**: for debug-recording. See the submodule in Recorder4Cpp. Turn OFF for new implementation.
5. **ENABLE_POSIT**: for running posit version. Default is OFF, turn ON for posit implementation 

#### Build and run:
For our implementation to be evaluated we need three runs.

First run, we generate ground truth, Set REAL_BITS=64, let the rest of the configurations be OFF. 
```
cmake --build .
./dynamic_game_trajectory_planner
```
After running the program, the runtime information will be printed for the run() and integrate() function. Some information, including the trajectory points for each vehicle, are also printed in the terminal. This information is saved as a csv file.

Second run, we generate the trajectory with 32 bit IEEE float, Set REAL_BITS=32, let the rest of the configurations be OFF and build and run.

Third run, we generate the trajectory with 32 bit Posit,Leave REAL_BITS=32 and set ENABLE_POSIT=ON, let the rest of the configurations be OFF.
Here there is have the option of modifying the regime bit of posit, the
regime is a variable-length bit field that acts as an exponent of an exponent to provide a much larger dynamic range. Inside the dynamic_game_planner.h file in include folder, the regime bit can be modified.
```
using posit32 = sw::universal::posit<32, 1>; // Defines a 32 bit posit with 1 regime bit
```
Build and run.

#### Precision check
To check the precision error, a python script has been implemented. That calculates the Euclidean distance to measure the difference in precision. Run this script
```
python3 ../evaluate_precision.py
```
Four metrics of errors are considered:
1. Mean Error: Average of every single drift by the total number of drifts
2. Median Error: Middle value of the drift when all errors are sorted from small to large
3. Max Error: Largest deviation from ground truth
4. Min Error: Smallest deviation from ground truth
4. Std Dev: Standard deviation, measures how far apart the errors are

Through testing posit with regime values of 0,1,2 and 3. Results with overall lesser errors were found with regime bit 1.
