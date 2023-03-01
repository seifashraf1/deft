#include <bitset>
#include <memory>
#include <string>
#include <cuda.h>
#include "llvm/IR/InstIterator.h"
#include "llvm/ADT/DenseSet.h"

__global__
void kernel(llvm::Instruction& i) {
    llvm::DenseSet<llvm::Instruction> s;
    return;
}