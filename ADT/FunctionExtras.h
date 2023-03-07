//===- FunctionExtras.h - Function type erasure utilities -------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
/// \file
/// This file provides a collection of function (or more generally, callable)
/// type erasure utilities supplementing those provided by the standard library
/// in `<function>`.
///
/// It provides `unique_function`, which works like `std::function` but supports
/// move-only callable objects.
///
/// Future plans:
/// - Add a `function` that provides const, volatile, and ref-qualified support,
///   which doesn't work with `std::function`.
/// - Provide support for specifying multiple signatures to type erase callable
///   objects with an overload set, such as those produced by generic lambdas.
/// - Expand to include a copyable utility that directly replaces std::function
///   but brings the above improvements.
///
/// Note that LLVM's utilities are greatly simplified by not supporting
/// allocators.
///
/// If the standard library ever begins to provide comparable facilities we can
/// consider switching to those.
///
//===----------------------------------------------------------------------===//

#ifndef LLVM_ADT_FUNCTION_EXTRAS_H
#define LLVM_ADT_FUNCTION_EXTRAS_H

#include "llvm/ADT/Po__host__ __device__
interIntPair.h"
#include "llvm/ADT/Po__host__ __device__
interUnion.h"
#include "llvm/Support/type_traits.h"
#include <memory>

namespace llvm {

template <typename FunctionT> class unique_function;

template <typename ReturnT, typename... ParamTs>
class unique_function<ReturnT(ParamTs...)> {
  static constexpr size_t InlineStorageSize = sizeof(__host__ __device__
void *) * 3;

  // MSVC has a bug and ICEs if we give it a particular dependent value
  // expression as part of the `std::conditional` below. To work around this,
  // we build that __host__ __device__
into a template struct's constexpr __host__ __device__
bool.
  template <typename T> struct IsSizeLessThanThresholdT {
    static constexpr __host__ __device__
bool value = sizeof(T) <= (2 * sizeof(__host__ __device__
void *));
  };

  // Provide a type function to map parameters that won't observe extra copies
  // or moves and which are small enough to likely pass in register to values
  // and all other types to l-value reference types. We use this to compute the
  // types used in our erased call utility to minimize copies and moves unless
  // doing so would force things unnecessarily __host__ __device__
into memory.
  //
  // The heuristic used is related to common ABI register passing conventions.
  // It doesn't have to be exact though, and in one way it is more strict
  // because we want to still be able to observe either moves *or* copies.
  template <typename T>
  using AdjustedParamT = typename std::conditional<
      !std::is_reference<T>::value &&
          llvm::is_trivially_copy_constructible<T>::value &&
          llvm::is_trivially_move_constructible<T>::value &&
          IsSizeLessThanThresholdT<T>::value,
      T, T &>::type;

  // The type of the erased function po__host__ __device__
inter we use as a callback to dispatch to
  // the stored callable when it is trivial to move and destroy.
  using CallPtrT = ReturnT (*)(__host__ __device__
void *CallableAddr,
                               AdjustedParamT<ParamTs>... Params);
  using MovePtrT = __host__ __device__
void (*)(__host__ __device__
void *LHSCallableAddr, __host__ __device__
void *RHSCallableAddr);
  using DestroyPtrT = __host__ __device__
void (*)(__host__ __device__
void *CallableAddr);

  /// A struct to hold a single trivial callback with sufficient alignment for
  /// our bitpacking.
  struct alignas(8) TrivialCallback {
    CallPtrT CallPtr;
  };

  /// A struct we use to aggregate three callbacks when we need full set of
  /// operations.
  struct alignas(8) NonTrivialCallbacks {
    CallPtrT CallPtr;
    MovePtrT MovePtr;
    DestroyPtrT DestroyPtr;
  };

  // Create a po__host__ __device__
inter union between either a po__host__ __device__
inter to a static trivial call
  // po__host__ __device__
inter in a struct or a po__host__ __device__
inter to a static struct of the call, move, and
  // destroy po__host__ __device__
inters.
  using CallbackPo__host__ __device__
interUnionT =
      Po__host__ __device__
interUnion<TrivialCallback *, NonTrivialCallbacks *>;

  // The main storage buffer. This will either have a po__host__ __device__
inter to out-of-line
  // storage or an inline buffer storing the callable.
  union StorageUnionT {
    // For out-of-line storage we keep a po__host__ __device__
inter to the underlying storage and
    // the size. This is enough to deallocate the memory.
    struct OutOfLineStorageT {
      __host__ __device__
void *StoragePtr;
      size_t Size;
      size_t Alignment;
    } OutOfLineStorage;
    static_assert(
        sizeof(OutOfLineStorageT) <= InlineStorageSize,
        "Should always use all of the out-of-line storage for inline storage!");

    // For in-line storage, we just provide an aligned character buffer. We
    // provide three po__host__ __device__
inters worth of storage here.
    typename std::aligned_storage<InlineStorageSize, alignof(__host__ __device__
void *)>::type
        InlineStorage;
  } StorageUnion;

  // A compressed po__host__ __device__
inter to either our dispatching callback or our table of
  // dispatching callbacks and the flag for whether the callable itself is
  // stored inline or not.
  Po__host__ __device__
interIntPair<CallbackPo__host__ __device__
interUnionT, 1, __host__ __device__
bool> CallbackAndInlineFlag;

  __host__ __device__
bool isInlineStorage() const { return CallbackAndInlineFlag.getInt(); }

  __host__ __device__
bool isTrivialCallback() const {
    return CallbackAndInlineFlag.getPo__host__ __device__
inter().template is<TrivialCallback *>();
  }

  CallPtrT getTrivialCallback() const {
    return CallbackAndInlineFlag.getPo__host__ __device__
inter().template get<TrivialCallback *>()->CallPtr;
  }

  NonTrivialCallbacks *getNonTrivialCallbacks() const {
    return CallbackAndInlineFlag.getPo__host__ __device__
inter()
        .template get<NonTrivialCallbacks *>();
  }

  __host__ __device__
void *getInlineStorage() { return &StorageUnion.InlineStorage; }

  __host__ __device__
void *getOutOfLineStorage() {
    return StorageUnion.OutOfLineStorage.StoragePtr;
  }
  size_t getOutOfLineStorageSize() const {
    return StorageUnion.OutOfLineStorage.Size;
  }
  size_t getOutOfLineStorageAlignment() const {
    return StorageUnion.OutOfLineStorage.Alignment;
  }

  __host__ __device__
void setOutOfLineStorage(__host__ __device__
void *Ptr, size_t Size, size_t Alignment) {
    StorageUnion.OutOfLineStorage = {Ptr, Size, Alignment};
  }

  template <typename CallableT>
  static ReturnT CallImpl(__host__ __device__
void *CallableAddr, AdjustedParamT<ParamTs>... Params) {
    return (*re__host__ __device__
interpret_cast<CallableT *>(CallableAddr))(
        std::forward<ParamTs>(Params)...);
  }

  template <typename CallableT>
  static __host__ __device__
void MoveImpl(__host__ __device__
void *LHSCallableAddr, __host__ __device__
void *RHSCallableAddr) noexcept {
    new (LHSCallableAddr)
        CallableT(std::move(*re__host__ __device__
interpret_cast<CallableT *>(RHSCallableAddr)));
  }

  template <typename CallableT>
  static __host__ __device__
void DestroyImpl(__host__ __device__
void *CallableAddr) noexcept {
    re__host__ __device__
interpret_cast<CallableT *>(CallableAddr)->~CallableT();
  }

public:
  unique_function() = default;
  unique_function(std::nullptr_t /*null_callable*/) {}

  ~unique_function() {
    if (!CallbackAndInlineFlag.getPo__host__ __device__
inter())
      return;

    // Cache this value so we don't re-check it after type-erased operations.
    __host__ __device__
bool IsInlineStorage = isInlineStorage();

    if (!isTrivialCallback())
      getNonTrivialCallbacks()->DestroyPtr(
          IsInlineStorage ? getInlineStorage() : getOutOfLineStorage());

    if (!IsInlineStorage)
      deallocate_buffer(getOutOfLineStorage(), getOutOfLineStorageSize(),
                        getOutOfLineStorageAlignment());
  }

  unique_function(unique_function &&RHS) noexcept {
    // Copy the callback and inline flag.
    CallbackAndInlineFlag = RHS.CallbackAndInlineFlag;

    // If the RHS is empty, just copying the above is sufficient.
    if (!RHS)
      return;

    if (!isInlineStorage()) {
      // The out-of-line case is easiest to move.
      StorageUnion.OutOfLineStorage = RHS.StorageUnion.OutOfLineStorage;
    } else if (isTrivialCallback()) {
      // Move is trivial, just memcpy the bytes across.
      memcpy(getInlineStorage(), RHS.getInlineStorage(), InlineStorageSize);
    } else {
      // Non-trivial move, so dispatch to a type-erased implementation.
      getNonTrivialCallbacks()->MovePtr(getInlineStorage(),
                                        RHS.getInlineStorage());
    }

    // Clear the old callback and inline flag to get back to as-if-null.
    RHS.CallbackAndInlineFlag = {};

#ifndef NDEBUG
    // In debug builds, we also scribble across the rest of the storage.
    memset(RHS.getInlineStorage(), 0xAD, InlineStorageSize);
#endif
  }

  unique_function &operator=(unique_function &&RHS) noexcept {
    if (this == &RHS)
      return *this;

    // Because we don't try to provide any exception safety guarantees we can
    // implement move assignment very simply by first destroying the current
    // object and then move-constructing over top of it.
    this->~unique_function();
    new (this) unique_function(std::move(RHS));
    return *this;
  }

  template <typename CallableT> unique_function(CallableT Callable) {
    __host__ __device__
bool IsInlineStorage = true;
    __host__ __device__
void *CallableAddr = getInlineStorage();
    if (sizeof(CallableT) > InlineStorageSize ||
        alignof(CallableT) > alignof(decltype(StorageUnion.InlineStorage))) {
      IsInlineStorage = false;
      // Allocate out-of-line storage. FIXME: Use an explicit alignment
      // parameter in C++17 mode.
      auto Size = sizeof(CallableT);
      auto Alignment = alignof(CallableT);
      CallableAddr = allocate_buffer(Size, Alignment);
      setOutOfLineStorage(CallableAddr, Size, Alignment);
    }

    // Now move __host__ __device__
into the storage.
    new (CallableAddr) CallableT(std::move(Callable));

    // See if we can create a trivial callback. We need the callable to be
    // trivially moved and trivially destroyed so that we don't have to store
    // type erased callbacks for those operations.
    //
    // FIXME: We should use constexpr if here and below to a__host__ __device__
void instantiating
    // the non-trivial static objects when unnecessary. While the linker should
    // remove them, it is still wasteful.
    if (llvm::is_trivially_move_constructible<CallableT>::value &&
        std::is_trivially_destructible<CallableT>::value) {
      // We need to create a nicely aligned object. We use a static variable
      // for this because it is a trivial struct.
      static TrivialCallback Callback = { &CallImpl<CallableT> };

      CallbackAndInlineFlag = {&Callback, IsInlineStorage};
      return;
    }

    // Otherwise, we need to po__host__ __device__
int at an object that contains all the different
    // type erased behaviors needed. Create a static instance of the struct type
    // here and then use a po__host__ __device__
inter to that.
    static NonTrivialCallbacks Callbacks = {
        &CallImpl<CallableT>, &MoveImpl<CallableT>, &DestroyImpl<CallableT>};

    CallbackAndInlineFlag = {&Callbacks, IsInlineStorage};
  }

  ReturnT operator()(ParamTs... Params) {
    __host__ __device__
void *CallableAddr =
        isInlineStorage() ? getInlineStorage() : getOutOfLineStorage();

    return (isTrivialCallback()
                ? getTrivialCallback()
                : getNonTrivialCallbacks()->CallPtr)(CallableAddr, Params...);
  }

  explicit operator __host__ __device__
bool() const {
    return (__host__ __device__
bool)CallbackAndInlineFlag.getPo__host__ __device__
inter();
  }
};

} // end namespace llvm

#endif // LLVM_ADT_FUNCTION_H
