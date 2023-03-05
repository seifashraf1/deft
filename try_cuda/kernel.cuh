#include <bitset>
#include <memory>
#include <string>
#include <cuda.h>
#include "llvm/IR/InstIterator.h"
#include "llvm/ADT/DenseSet.h"

__global__
void kernel(llvm::Instruction* i, int* b) {
    *b = i->hasName();

    return;
}


// __global__
// void kernel(llvm::Value* i) {
//     //llvm::DenseSet<llvm::Instruction> s;
//     i->takeName(i);
//     return;
// }