#ifndef DATAFLOW_ANALYSIS_H
#define DATAFLOW_ANALYSIS_H

#include <algorithm>
#include <deque>
#include <numeric>

#include "llvm/ADT/DenseMap.h"
#include "llvm/ADT/DenseSet.h"
#include "llvm/ADT/PostOrderIterator.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/IR/CFG.h"
#include "llvm/IR/Function.h"
#include "llvm/IR/InstIterator.h"
#include <bitset>
#include <memory>
#include <string>
#include <cuda.h>


namespace analysis {
template<typename T>
class WorkList {
public:
  template<typename IterTy>
  WorkList(IterTy i, IterTy e)
    : inList{},
      work{i, e} {
    inList.insert(i,e);
  }

  WorkList()
    : inList{},
      work{}
      { }

  bool empty() const { return work.empty(); }

  bool contains(T elt) const { return inList.count(elt); }

  void
  add(T elt) {
    if (!inList.count(elt)) {
      work.push_back(elt);
    }
  }

  T
  take() {
    T front = work.front();
    work.pop_front();
    inList.erase(front);
    return front;
  }

private:
  llvm::DenseSet<T> inList;
  std::deque<T> work;
};

using BasicBlockWorklist = WorkList<llvm::BasicBlock*>;

template <typename AbstractValue>
using AbstractState = llvm::DenseMap<llvm::Value*,AbstractValue>;


template <typename AbstractValue>
using DataflowResult =  // Result of function
  llvm::DenseMap<llvm::Value*, AbstractState<AbstractValue>>;


template <typename AbstractValue>
bool
operator==(const AbstractState<AbstractValue>& s1,
           const AbstractState<AbstractValue>& s2) {
  if (s1.size() != s2.size()) {
    return false;
  }
  return std::all_of(s1.begin(), s1.end(),
    [&s2] (auto &kvPair) {
      auto found = s2.find(kvPair.first);
      return found != s2.end() && found->second == kvPair.second;
    });
}

// This class can be extended with a concrete implementation of the meet
// operator for two elements of the abstract domain. Implementing the
// `meetPair()` method in the subclass will enable it to be used within the
// general meet operator because of the curiously recurring template pattern.
template <typename AbstractValue, typename SubClass>
class Meet {
public:
  AbstractValue
  operator()(llvm::ArrayRef<AbstractValue> values) {
    return std::accumulate(values.begin(), values.end(),
      AbstractValue(),
      [this] (auto v1, auto v2) {
        return this->asSubClass().meetPair(v1, v2);
      });
  }

  AbstractValue
  meetPair(AbstractValue& v1, AbstractValue& v2) const {
    llvm_unreachable("unimplemented meet");
  }

  void print(llvm::raw_ostream& out, AbstractValue& value) { }
  void printState(llvm::raw_ostream& out, AbstractState<AbstractValue>& state) {
    out << "DUMP ";
    for (auto& kvPair : state) {
      this->asSubClass().print(out, kvPair.second);
    }
    out << "\n";
  }

private:
  SubClass& asSubClass() { return static_cast<SubClass&>(*this); };
};

class Forward {
public:
  static auto getInstructions(llvm::BasicBlock& bb) {
    return llvm::iterator_range<decltype(bb.begin())>(bb);
  };
  static auto getFunctionTraversal(llvm::Function& f) {
    return llvm::ReversePostOrderTraversal<llvm::Function*>(&f);
  }
  static auto* getEntryKey(llvm::BasicBlock& bb) {
    return &bb;
  }
  static auto* getExitKey(llvm::BasicBlock& bb) {
    return bb.getTerminator();
  }
  static llvm::Value* getFunctionValueKey(llvm::BasicBlock& bb) {
    if (auto* ret = llvm::dyn_cast<llvm::ReturnInst>(bb.getTerminator())) {
      return ret->getReturnValue();
    }
    return nullptr;
  }
  static auto getSuccessors(llvm::BasicBlock& bb) {
    return llvm::successors(&bb);
  }
  static auto getPredecessors(llvm::BasicBlock& bb) {
    return llvm::predecessors(&bb);
  }
  static bool shouldMeetPHI() { return true; }
  template <class State, class Transfer, class Meet>
  static bool prepareSummaryState(llvm::CallSite cs,
                                  llvm::Function* callee,
                                  State& state,
                                  State& summaryState,
                                  Transfer& transfer,
                                  Meet& meet) {
    unsigned index = 0;
    bool needsUpdate = false;
    for (auto& functionArg : callee->args()) {
      auto* passedConcrete = cs.getArgument(index);
      auto passedAbstract = state.find(passedConcrete);
      if (passedAbstract == state.end()) {
        transfer(*passedConcrete, state);
        passedAbstract = state.find(passedConcrete);
      }
      auto& arg     = summaryState[&functionArg];
      auto newState = meet({passedAbstract->second, arg});
      needsUpdate |= !(newState == arg);
      arg = newState;
      ++index;
    }
    return needsUpdate;
  }
};

//DataFlowAnalysis
const unsigned long ContextSize=2ul;

template <typename AbstractValue>
using State   = AbstractState<AbstractValue>;

using Context = llvm::ArrayRef<llvm::Instruction*>;

template <typename AbstractValue>
using FunctionResults = DataflowResult<AbstractValue>;

using ContextFunction = std::pair<Context, llvm::Function*>;

template <typename AbstractValue>
using ContextResults  = llvm::DenseMap<llvm::Function*, FunctionResults<AbstractValue>>;

using ContextWorklist = WorkList<ContextFunction>;

using ContextMapInfo = llvm::DenseMapInfo<llvm::ArrayRef<llvm::Instruction*>>;

template <typename AbstractValue>
using AllResults = llvm::DenseMap<Context, ContextResults<AbstractValue>, ContextMapInfo>;


template <typename AbstractValue, typename Transfer, typename Meet, typename Direction=Forward> 
__global__
void ComputeDataflowKernel(llvm::Function* f,  const Context* context,
                                              llvm::DenseSet<ContextFunction>* active, AllResults<AbstractValue>* allResults,
                                              llvm::DenseMap<ContextFunction, llvm::DenseSet<ContextFunction>>* callers)
{

  // active->insert({*context,f});
   
  // FunctionResults<AbstractValue> results = allResults->FindAndConstruct(*context).second.FindAndConstruct(f).second;
  // DenseMap<ArrayRef<llvm::Instruction *>, DenseMap<llvm::Function *, DenseMap<llvm::Value *, DenseMap<llvm::Value *, type-parameter-0-0> > >, DenseMapInfo<llvm::ArrayRef<llvm::Instruction *> > > *
  return;
}

template <typename AbstractValue>
using AllResults = llvm::DenseMap<Context, ContextResults<AbstractValue>, ContextMapInfo>;

template <typename AbstractValue, typename Transfer, typename Meet, typename Direction=Forward> 

AllResults<AbstractValue> ComputeDataflow(llvm::Module& m, llvm::ArrayRef<llvm::Function*> entryPoints) {
  AllResults<AbstractValue> allResults;
  ContextWorklist contextWork;  // worklist

  // Add entry point (typically main)
  for (auto* entry : entryPoints) {
    contextWork.add({Context{}, entry});
  }

  // Copy data to kernel

  llvm::Function* d_f;
  Context* d_context;
  llvm::DenseSet<ContextFunction>* d_active; 
  AllResults<AbstractValue>* d_allResults;
  llvm::DenseMap<ContextFunction,llvm::DenseSet<ContextFunction>>* d_callers;



  cudaMalloc((void **)&d_f, sizeof(llvm::Function));
  cudaMalloc((void **)&d_context, sizeof(Context ));
  cudaMalloc((void **)&d_active, sizeof(llvm::DenseSet<ContextFunction>)); //decide its max size baeden
  cudaMalloc((void **)&d_allResults, sizeof(AllResults<AbstractValue>));
  cudaMalloc((void **)&d_callers, sizeof(llvm::DenseMap<ContextFunction,llvm::DenseSet<ContextFunction>>));



  // Call kernel with all {context, function}
  auto front = contextWork.take();
  llvm::Function* cur_function = (front.second);
  Context* cur_context = &(front.first);
FunctionResults<AbstractValue> results = allResults.FindAndConstruct(*cur_context).second.FindAndConstruct(cur_function).second;
  cudaMemcpy(d_f, &cur_function, sizeof(llvm::Instruction), cudaMemcpyHostToDevice);
  cudaMemcpy(d_context, &cur_context, sizeof(llvm::Instruction), cudaMemcpyHostToDevice);

  //(TODO) Init blockDim with worklist size
  dim3 gridDim(1);
  dim3 blockDim(1);

  ComputeDataflowKernel<AbstractValue, Transfer, Meet><<<gridDim, blockDim>>>(d_f, d_context, d_active, d_allResults, d_callers);
  //copy data from device to host 

  // Merge worklist updates (new functions to be analyzed from callers) and results 

  return allResults;
}
}

#endif