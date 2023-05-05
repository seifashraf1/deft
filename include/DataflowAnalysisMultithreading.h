
#ifndef DATAFLOW_ANALYSIS_H
#define DATAFLOW_ANALYSIS_H

#include <algorithm>
#include <deque>
#include <numeric>
#include <chrono>
#include <pthread.h>
#include <mutex>
#include <vector>
#include<iostream>
#include<set>

#include "llvm/ADT/DenseMap.h"
#include "llvm/ADT/DenseSet.h"
#include "llvm/ADT/PostOrderIterator.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/IR/CFG.h"
#include "llvm/IR/Function.h"
#include "llvm/IR/InstIterator.h"


namespace llvm {


template<unsigned long Size>
struct DenseMapInfo<std::array<llvm::Instruction*, Size>> {
  using Context = std::array<llvm::Instruction*, Size>;
  static inline Context
  getEmptyKey() {
    Context c;
    std::fill(c.begin(), c.end(),
      llvm::DenseMapInfo<llvm::Instruction*>::getEmptyKey());
    return c;
  }
  static inline Context
  getTombstoneKey() {
    Context c;
    std::fill(c.begin(), c.end(),
      llvm::DenseMapInfo<llvm::Instruction*>::getTombstoneKey());
    return c;
  }
  static unsigned
  getHashValue(const Context& c) {
    return llvm::hash_combine_range(c.begin(), c.end());
  }
  static bool
  isEqual(const Context& lhs, const Context& rhs) {
    return lhs == rhs;
  }
};


}


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

  int size() const { return work.size(); }

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


// The dataflow analysis computes three different granularities of results.
// An AbstractValue represents information in the abstract domain for a single
// LLVM Value. An AbstractState is the abstract representation of all values
// relevent to a particular point of the analysis. A DataflowResult contains
// the abstract states before and after each instruction in a function. The
// incoming state for one instruction is the outgoing state from the previous
// instruction. For the first instruction in a BasicBlock, the incoming state is
// keyed upon the BasicBlock itself.
//
// Note: In all cases, the AbstractValue should have a no argument constructor
// that builds constructs the initial value within the abstract domain.

template <typename AbstractValue>
using AbstractState = llvm::DenseMap<llvm::Value*,AbstractValue>;


template <typename AbstractValue>
using DataflowResult =
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


template <typename AbstractValue>
AbstractState<AbstractValue>&
getIncomingState(DataflowResult<AbstractValue>& result, llvm::Instruction& i) {
  auto* bb = i.getParent();
  auto* key = (&bb->front() == &i)
    ? static_cast<llvm::Value*>(bb)
    : static_cast<llvm::Value*>(&*--llvm::BasicBlock::iterator{i});
  return result[key];
}


// NOTE: This class is not intended to be used. It is only intended to
// to document the structure of a Transfer policy object as used by the
// DataflowAnalysis class. For a specific analysis, you should implement
// a class with the same interface.
template <typename AbstractValue>
class Transfer {
public:
  void
  operator()(llvm::Value& v, AbstractState<AbstractValue>& s) {
    llvm_unreachable("unimplemented transfer");
  }
};


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


class Backward {
public:
  static auto getInstructions(llvm::BasicBlock& bb) {
    return llvm::reverse(bb);
  };
  static auto getFunctionTraversal(llvm::Function& f) {
    return llvm::post_order<llvm::Function*>(&f);
  }
  static auto* getEntryKey(llvm::BasicBlock& bb) {
    return &bb;
  }
  static auto* getExitKey(llvm::BasicBlock& bb) {
    return &*bb.begin();
  }
  static llvm::Value* getFunctionValueKey(llvm::BasicBlock& bb) {
    return (&bb == &bb.getParent()->getEntryBlock()) ? getExitKey(bb) : nullptr;
  }
  static auto getSuccessors(llvm::BasicBlock& bb) {
    return llvm::predecessors(&bb);
  }
  static auto getPredecessors(llvm::BasicBlock& bb) {
    return llvm::successors(&bb);
  }
  static bool shouldMeetPHI() { return false; }
  template <class State, class Transfer, class Meet>
  static bool prepareSummaryState(llvm::CallSite cs,
                                  llvm::Function* callee,
                                  State& state,
                                  State& summaryState,
                                  Transfer& transfer,
                                  Meet& meet) {
    // TODO: This would be better if it checked whether the new state at this
    // function different from the old.
    bool needsUpdate = false;
    auto* passedConcrete = cs.getInstruction();
    auto& passedAbstract = state.FindAndConstruct(passedConcrete);
    for (auto& bb : *callee) {
      if (auto* ret = llvm::dyn_cast<llvm::ReturnInst>(bb.getTerminator());
          ret && ret->getReturnValue()) {
        auto& retState = summaryState[ret->getReturnValue()];
        auto newState = meet({passedAbstract.second, retState});
        needsUpdate |= !(newState == retState);
        retState = newState;
      }
    }
    return needsUpdate;
  }
};

const unsigned long ContextSize_=2ul;
using Context = std::array<llvm::Instruction*, ContextSize_>;

struct ComputePair {
  llvm::Function& f;
  const Context& context;
  ComputePair(llvm::Function& f_, const Context&  context_) : f(f_), context(context_) {}
};


  std::mutex allResults_mtx;
  std::mutex contextWork_mtx;
  std::mutex callers_mtx;
  std::mutex active_mtx;

template <typename AbstractValue,
          typename Transfer,
          typename Meet,
          typename Direction=Forward,
          unsigned long ContextSize=2ul>
class DataflowAnalysis {
public:
  using State   = AbstractState<AbstractValue>;
  

  using FunctionResults = DataflowResult<AbstractValue>;
  using ContextFunction = std::pair<Context, llvm::Function*>;
  using ContextResults  = llvm::DenseMap<llvm::Function*, FunctionResults>;
  using ContextWorklist = WorkList<ContextFunction>;

  using ContextMapInfo =
    llvm::DenseMapInfo<std::array<llvm::Instruction*, ContextSize>>;
  using AllResults = llvm::DenseMap<Context, ContextResults, ContextMapInfo>;

  


  DataflowAnalysis(llvm::Module& m,
                          llvm::ArrayRef<llvm::Function*> entryPoints) {
    for (auto* entry : entryPoints) {
      contextWork.add({Context{}, entry});
    }
  }


  // computeDataflow collects the dataflow facts for all instructions
  // in the program reachable from the entryPoints passed to the constructor.
  static AllResults
  computeDataflow() {
    while (!contextWork.empty()) {
      int worklist_size = contextWork.size();
      std::cout<< "Here " << worklist_size <<"\n";
      pthread_t* tid;
      tid = new pthread_t[worklist_size];
      // ComputePair* cp[worklist_size];
      std::vector<ComputePair> cp;
      int context_idx = 0;
      llvm::DenseSet<llvm::Function*> current_set;
      //ContextWorklist temp_worklist;
      while(!contextWork.empty() && context_idx < 4){
        auto [context, function] = contextWork.take();
        // cp[context_idx] = new ComputePair(*function, context);
        if(current_set.count(function)) continue;
        current_set.insert(function);
        cp.push_back(ComputePair(*function, context));
        pthread_create(&(tid[context_idx]), NULL, computeDataflow, (void*) &cp[context_idx]);
        context_idx++;
      }
      current_set.clear();
      for(int i=0; i<context_idx; i++) pthread_join(tid[i], NULL);
    }

    return allResults;
  }

  // computeDataflow collects the dataflowfacts for all instructions
  // within Function f with the associated execution context. Functions whose
  // results are required for the analysis of f will be transitively analyzed.

  // Multithreaded compute dataflow
  static void *
  computeDataflow(void* v_cp) {
    struct ComputePair *cp = (struct ComputePair*)v_cp;
    llvm::Function& f = cp->f; const Context& context = cp->context;
    active_mtx.lock();
    active.insert({cp->context, &cp->f});
    active_mtx.unlock();

    // First compute the initial outgoing state of all instructions
    // We are sure that all {context, function} pairs in the worklist are unique from the set
    allResults_mtx.lock();
    FunctionResults results = allResults.FindAndConstruct(context).second
                                        .FindAndConstruct(&f).second;
    
    std::cout<< "Done declarations\n";
    if (results.find(getSummaryKey(f)) == results.end()) {
      for (auto& i : llvm::instructions(f)) {
        results.FindAndConstruct(&i);
      }
    }
    allResults_mtx.unlock();
    

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

      auto state = mergeStateFromPredecessors(bb, results);
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
          analyzeCall(cs, state, context);
        } else {
          applyTransfer(i, state);
        }
//meet.printState(llvm::outs(),state);
        results[&i] = state;
      }

      // If the abstract state for this block did not change, then we are done
      // with this block. Otherwise, we must update the abstract state and
      // consider changes to successors.
      if (state == oldExitState) {
        continue;
      }

      for (auto* s : Direction::getSuccessors(*bb)) {
        work.add(s);
      }

      if (auto* key = Direction::getFunctionValueKey(*bb)) {
        auto* summary = getSummaryKey(f);
        results[&f][summary] = meet({results[&f][summary], state[key]});
      }
    }

    // The overall results for the given function and context are updated if
    // necessary. Updating the results for this (function,context) means that
    // all callers must be updated as well.
    
    // allResults, contextWork, and active are shared datastructures so we need mutex here
    allResults_mtx.lock();
    auto& oldResults = allResults[context][&f];
    if (!(oldResults == results)) {
      oldResults = results;
      for (auto& caller : callers[{context, &f}]) {
        contextWork_mtx.lock();
        contextWork.add(caller);
        contextWork_mtx.unlock();
      }
    }
    allResults_mtx.unlock();

    active_mtx.lock();
    active.erase({context, &f});
    active_mtx.unlock();
    
  }


  // Original Compute dataflow
  static DataflowResult<AbstractValue>
computeDataflow(llvm::Function& f, const Context& context) {
  active_mtx.lock();
  active.insert({context, &f});
  active_mtx.unlock();

  // First compute the initial outgoing state of all instructions
  allResults_mtx.lock();
  FunctionResults results = allResults.FindAndConstruct(context).second
                                      .FindAndConstruct(&f).second;
  allResults_mtx.unlock();
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
    auto state = mergeStateFromPredecessors(bb, results);
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
        analyzeCall(cs, state, context);
      } else {
        applyTransfer(i, state);
      }
//meet.printState(llvm::outs(),state);
      results[&i] = state;
    }

    // If the abstract state for this block did not change, then we are done
    // with this block. Otherwise, we must update the abstract state and
    // consider changes to successors.
    if (state == oldExitState) {
      continue;
    }

    for (auto* s : Direction::getSuccessors(*bb)) {
      work.add(s);
    }

    if (auto* key = Direction::getFunctionValueKey(*bb)) {
      auto* summary = getSummaryKey(f);
      results[&f][summary] = meet({results[&f][summary], state[key]});
    }
  }

  // The overall results for the given function and context are updated if
  // necessary. Updating the results for this (function,context) means that
  // all callers must be updated as well.
  allResults_mtx.lock();
  auto& oldResults = allResults[context][&f];
  if (!(oldResults == results)) {
    oldResults = results;
    for (auto& caller : callers[{context, &f}]) {
      contextWork_mtx.lock();
      contextWork.add(caller);
      contextWork_mtx.unlock();
    }
  }
  allResults_mtx.unlock();
  active_mtx.lock();
  active.erase({context, &f});
  active_mtx.unlock();
  return results;
}

  static llvm::Function*
  getCalledFunction(llvm::CallSite cs) {
    auto* calledValue = cs.getCalledValue()->stripPointerCasts();
    return llvm::dyn_cast<llvm::Function>(calledValue);
  }

  static bool
  isAnalyzableCall(llvm::CallSite cs) {
    if (!cs.getInstruction()) {
      return false;
    }
    auto* called = getCalledFunction(cs);
    return called && !called->isDeclaration();
  }

  static void
  analyzeCall(llvm::CallSite cs, State &state, const Context& context) {
    Context newContext;
    if (newContext.size() > 0) {
      std::copy(context.begin() + 1, context.end(), newContext.begin());
      newContext.back() = cs.getInstruction();
    }

    auto* caller  = cs.getInstruction()->getFunction();
    auto* callee  = getCalledFunction(cs);
    auto toCall   = std::make_pair(newContext, callee);
    auto toUpdate = std::make_pair(context, caller);

    auto& calledState  = allResults[newContext][callee];
    auto& summaryState = calledState[callee];
    bool needsUpdate   = summaryState.size() == 0;

    needsUpdate |= Direction::prepareSummaryState(cs, callee, state, summaryState, transfer, meet);

    if (!active.count(toCall) && needsUpdate) {
      computeDataflow(*callee, newContext);
    }

    state[cs.getInstruction()] = calledState[callee][callee];
    callers_mtx.lock();
    callers[toCall].insert(toUpdate);
    callers_mtx.unlock();
  }

private:
  // These property objects determine the behavior of the dataflow analysis.
  // They should by replaced by concrete implementation classes on a per
  // analysis basis.
  inline static Meet meet;
  inline static Transfer transfer;

  inline static AllResults allResults;
  inline static ContextWorklist contextWork;
  inline static llvm::DenseMap<ContextFunction, llvm::DenseSet<ContextFunction>> callers;
  inline static llvm::DenseSet<ContextFunction> active;


  static llvm::Value*
  getSummaryKey(llvm::Function& f) {
    return &f;
  }

  static void
  mergeInState(State& destination, const State& toMerge) {
    for (auto& valueStatePair : toMerge) {
      // If an incoming Value has an AbstractValue in the already merged
      // state, meet it with the new one. Otherwise, copy the new value over,
      // implicitly meeting with bottom.
      auto [found, newlyAdded] = destination.insert(valueStatePair);
      if (!newlyAdded) {
        found->second = meet({found->second, valueStatePair.second});
      }
    }
  }

  static State
  mergeStateFromPredecessors(llvm::BasicBlock* bb, FunctionResults& results) {
    State mergedState = State{};
    mergeInState(mergedState, results[bb]);
    for (auto* p : Direction::getPredecessors(*bb)) {
      auto predecessorFacts = results.find(Direction::getExitKey(*p));
      if (results.end() == predecessorFacts) {
        continue;
      }
      mergeInState(mergedState, predecessorFacts->second);
    }
    return mergedState;
  }

  static AbstractValue
  meetOverPHI(State& state, const llvm::PHINode& phi) {
    auto phiValue = AbstractValue();
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

  static void
  applyTransfer(llvm::Instruction& i, State& state) {
    if (auto* phi = llvm::dyn_cast<llvm::PHINode>(&i);
        phi && Direction::shouldMeetPHI()) {
      // Phis can be explicit meet operations
      state[phi] = meetOverPHI(state, *phi);
    } else {
      transfer(i, state);
    }
  }
};


} // end namespace


#endif