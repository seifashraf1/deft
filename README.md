# GPU-IFDS
A GPU-based static analysis framework.

command: clang++ -std=c++14 -g -O3 main.cu `llvm-config-9 --cxxflags --ldflags --libs` --cuda-gpu-arch=sm_35 -ferror-limit=30 -o main