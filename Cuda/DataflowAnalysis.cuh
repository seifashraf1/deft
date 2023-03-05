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


//DataFlowAnalysis
const unsigned long ContextSize=2ul;

template <typename AbstractValue>
using State   = AbstractState<AbstractValue>;

using Context = std::array<llvm::Instruction*, ContextSize>;

template <typename AbstractValue>
using FunctionResults = DataflowResult<AbstractValue>;

using ContextFunction = std::pair<Context, llvm::Function*>;

template <typename AbstractValue>
using ContextResults  = llvm::DenseMap<llvm::Function*, FunctionResults<AbstractValue>>;

using ContextWorklist = WorkList<ContextFunction>;

using ContextMapInfo = llvm::DenseMapInfo<std::array<llvm::Instruction*, ContextSize>>;

template <typename AbstractValue>
using AllResults = llvm::DenseMap<Context, ContextResults<AbstractValue>, ContextMapInfo>;

    template <typename AbstractValue, typename Transfer, typename Meet>
    AllResults<AbstractValue> ComputeDataflow(llvm::Module& m, llvm::Function* entryPoint) {

    }
}