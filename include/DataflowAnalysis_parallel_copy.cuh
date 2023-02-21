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

template <typename AbstractValue, typename Transfer, typename Meet, typename Direction=Forward>
__device__
AbstractValue meetOverPHI(State<AbstractValue>& state, const llvm::PHINode& phi) {
    auto phiValue = AbstractValue();
    Meet meet;
    Transfer transfer;
    
    for (auto& value : phi.incoming_values()) {
      auto found = state.find(value.get());
      if (state.end() == found) {
        transfer(*value.get(), state);
        found = state.find(value.get());
      }
      phiValue = meet({phiValue, found->second});
    }
    return phiValue;
}

template <typename AbstractValue, typename Transfer, typename Meet, typename Direction=Forward>
__device__
void applyTransfer(llvm::Instruction& i, State<AbstractValue>& state) {
    Transfer transfer;

    if (auto* phi = llvm::dyn_cast<llvm::PHINode>(&i);
        phi && Direction::shouldMeetPHI()) {
      // Phis can be explicit meet operations
      state[phi] = meetOverPHI<AbstractValue, Transfer, Meet>(state, *phi);
    } else {
      transfer(i, state);
    }
  }

template <typename AbstractValue, typename Transfer, typename Meet, typename Direction=Forward>
__device__
void mergeInState(State<AbstractValue>& destination, const State<AbstractValue>& toMerge) {
    for (auto& valueStatePair : toMerge) {
      // If an incoming Value has an AbstractValue in the already merged
      // state, meet it with the new one. Otherwise, copy the new value over,
      // implicitly meeting with bottom.
      Meet meet;
      auto [found, newlyAdded] = destination.insert(valueStatePair);
      if (!newlyAdded) {
        found->second = meet({found->second, valueStatePair.second});
      }
    }
}


template <typename AbstractValue, typename Transfer, typename Meet, typename Direction=Forward>
__device__
State<AbstractValue> mergeStateFromPredecessors(llvm::BasicBlock* bb, FunctionResults<AbstractValue>& results) {
  State<AbstractValue> mergedState = State{};
  mergeInState<AbstractValue, Transfer, Meet>(mergedState, results[bb]);
  for (auto* p : Direction::getPredecessors(*bb)) {
    auto predecessorFacts = results.find(Direction::getExitKey(*p));
    if (results.end() == predecessorFacts) {
      continue;
    }
    mergeInState<AbstractValue, Transfer, Meet>(mergedState, predecessorFacts->second);
  }
  return mergedState;
}

template <typename AbstractValue, typename Transfer, typename Meet, typename Direction=Forward> 
__device__
llvm::Function* getCalledFunction(llvm::CallSite cs) {
  auto* calledValue = cs.getCalledValue()->stripPointerCasts();
  return llvm::dyn_cast<llvm::Function>(calledValue);
}

template <typename AbstractValue, typename Transfer, typename Meet, typename Direction=Forward> 
__device__
bool isAnalyzableCall(llvm::CallSite cs) {
  if (!cs.getInstruction()) {
    return false;
  }
  auto* called = getCalledFunction<AbstractValue, Transfer, Meet>(cs);
  return called && !called->isDeclaration();
}

template <typename AbstractValue, typename Transfer, typename Meet, typename Direction=Forward> 
__device__
void analyzeCall(llvm::CallSite cs, State<AbstractValue> &state, const Context& context, llvm::DenseMap<ContextFunction, llvm::DenseSet<ContextFunction>>& callers) {
  Context newContext;
  Meet meet;
  Transfer transfer;
  
  if (newContext.size() > 0) {
    std::copy(context.begin() + 1, context.end(), newContext.begin());
    newContext.back() = cs.getInstruction();
  }

  auto* caller  = cs.getInstruction()->getFunction();
  auto* callee  = getCalledFunction<AbstractValue, Transfer, Meet>(cs);
  auto toCall   = std::make_pair(newContext, callee);
  auto toUpdate = std::make_pair(context, caller);

  auto& calledState  = allResults[newContext][callee];
  auto& summaryState = calledState[callee];
  bool needsUpdate   = summaryState.size() == 0;

  needsUpdate |= Direction::prepareSummaryState(cs, callee, state, summaryState, transfer, meet);

  if (!active.count(toCall) && needsUpdate) {
    //NOTE: might needs to be handled by CUDA (recursion!!)
    computeDataflow<AbstractValue, Transfer, Meet>(*callee, newContext);
  }

  state[cs.getInstruction()] = calledState[callee][callee];
  callers[toCall].insert(toUpdate);
}


template <typename AbstractValue, typename Transfer, typename Meet, typename Direction=Forward> 
__device__
DataflowResult<AbstractValue> ComputeDataflow(llvm::Function& f, const Context& context,
                                              llvm::DenseSet<ContextFunction>& active, AllResults<AbstractValue>& allResults,
                                              llvm::DenseMap<ContextFunction, llvm::DenseSet<ContextFunction>>& callers)
{
  active.insert({context, &f});
  // First compute the initial outgoing state of all instructions
  FunctionResults<AbstractValue> results = allResults.FindAndConstruct(context).second
                                      .FindAndConstruct(&f).second;
  if (results.find(getSummaryKey(f)) == results.end()) {
    for (auto& i : llvm::instructions(f)) {
      results.FindAndConstruct(&i);
    }
  }
  // Add all blocks to the worklist in topological order for efficiency
  auto traversal = Direction::getFunctionTraversal(f);
  BasicBlockWorklist work(traversal.begin(), traversal.end());

  while (!work.empty()) {
    auto* bb = work.take();
    // Save a copy of the outgoing abstract state to check for changes.
    const auto& oldEntryState = results[Direction::getEntryKey(*bb)];
    const auto oldExitState   = results[Direction::getExitKey(*bb)];

    // Merge the state coming in from all predecessors including the function
    // summary (which contains arguments, etc.)
    auto state = mergeStateFromPredecessors<AbstractValue, Transfer, Meet>(bb, results);
    mergeInState(state, results[getSummaryKey(f)]);

    // If we have already processed the block and no changes have been made to
    // the abstract input, we can skip processing the block. Otherwise, save
    // the new entry state and proceed processing this block.
    if (state == oldEntryState && !state.empty()) {
      continue;
    }
    results[bb] = state;

    // Propagate through all instructions in the block
    for (auto& i : Direction::getInstructions(*bb)) {
      llvm::CallSite cs(&i);
      if (isAnalyzableCall(cs)) {
        analyzeCall(cs, state, context, callers);
      } else {
        applyTransfer(i, state);
      }
      //meet.printState(llvm::outs(),state);
      results[&i] = state;
    }
  }
}


template <typename AbstractValue, typename Transfer, typename Meet, typename Direction=Forward> 
__global__ void ComputeDataflowKernel(llvm::Module& m, llvm::Function* entryPoint) {
  Meet meet;
  Transfer transfer;
  llvm::DenseSet<ContextFunction> active;
  AllResults<AbstractValue> allResults;
  ContextWorklist contextWork;
  llvm::DenseMap<ContextFunction, llvm::DenseSet<ContextFunction>> callers;
  
  contextWork.add({Context{}, entryPoint});

  while (!contextWork.empty()) {
    auto [context, function] = contextWork.take();
    computeDataflow<AbstractValue, Transfer, Meet>(*function, context, active. allResults, callers);
  }

}

}


#endif