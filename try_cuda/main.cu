
#include<iostream>
#include "llvm/ADT/APSInt.h"
#include "llvm/Analysis/ConstantFolding.h"
#include "llvm/IR/CallSite.h"
#include "llvm/IR/Constants.h"
#include "llvm/IR/DebugInfo.h"
#include "llvm/IR/LLVMContext.h"
#include "llvm/IR/Module.h"
#include "llvm/IRReader/IRReader.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/ManagedStatic.h"
#include "llvm/Support/PrettyStackTrace.h"
#include "llvm/Support/Signals.h"
#include "llvm/Support/SourceMgr.h"
#include "llvm/Support/raw_ostream.h"
#include "llvm/IR/InstIterator.h"
#include "/home/g1f/Documents/deft/try_cuda/kernel.cuh"

#include <bitset>
#include <memory>
#include <string>



using namespace llvm;
using std::string;
using std::unique_ptr;


static cl::OptionCategory propagationCategory{"constant propagation options"};

static cl::opt<string> inPath{cl::Positional,
                              cl::desc{"<Module to analyze>"},
                              cl::value_desc{"bitcode filename"},
                              cl::init(""),
                              cl::Required,
                              cl::cat{propagationCategory}};

using namespace std;


int
main(int argc, char** argv) {
  // This boilerplate provides convenient stack traces and clean LLVM exit
  // handling. It also initializes the built in support for convenient
  // command line option handling.
  sys::PrintStackTraceOnErrorSignal(argv[0]);
  llvm::PrettyStackTraceProgram X(argc, argv);
  llvm_shutdown_obj shutdown;
  cl::HideUnrelatedOptions(propagationCategory);
  cl::ParseCommandLineOptions(argc, argv);

  // Construct an IR file from the filename passed on the command line.
  SMDiagnostic err;
  LLVMContext context;
  unique_ptr<Module> module = parseIRFile(inPath.getValue(), err, context);

  if (!module.get()) {
    errs() << "Error reading bitcode file: " << inPath << "\n";
    err.print(argv[0], errs());
    return -1;
  }

  auto* mainFunction = module->getFunction("main");
  if (!mainFunction) {
    llvm::report_fatal_error("Unable to find main function.");
  }

  dim3 gridDim(1);
  dim3 blockDim(1);
  int* b;
  b=new int;
  *b = 1;
  // printf("%d\n", *b);


  int* d_b;llvm::Instruction* d_i;

  cudaMalloc((void **)&d_b, sizeof(int));
  cudaMalloc((void **)&d_i, sizeof(llvm::Instruction));

  for (auto& i : llvm::instructions(*mainFunction)) {
    cudaMemcpy(d_i, &i, sizeof(llvm::Instruction), cudaMemcpyHostToDevice);

    kernel<<<gridDim, blockDim>>>(d_i, d_b);

    cudaMemcpy(b, d_b, sizeof(int), cudaMemcpyDeviceToHost);
    // printf("%d\n", *b);
  }
  
  return 0;
}