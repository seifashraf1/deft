Testing LLVM on Cuda.

A small experiment to initialize cuda kernel with a single LLVM instruction.

State:
FAILED. No LLVM code, even single instruction can be run on Cuda.

To reproduce the results from this directory, use the command:
clang++-9 -g -O3 main.cu `llvm-config-9 --cxxflags --ldflags --libs` --cuda-gpu-arch=sm_35 -std=c++17 -o main
