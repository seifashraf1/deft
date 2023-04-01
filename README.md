# GPU-IFDS
A GPU-based static analysis framework.

commands: 
cd /tools/constant-propagation <br />
clang++-9 -g -O3 main.cu `llvm-config-9 --cxxflags --ldflags --libs` --cuda-gpu-arch=sm_35 -std=c++17 -o main

For multithreaded version, compile with command:
g++ main.cpp `llvm-config-9 --cxxflags --libs --cflags` -std=c++17 -o main_multithreading -std=c++17 -pthread -lz -lstdc++