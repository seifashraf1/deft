//===- llvm/ADT/EpochTracker.h - ADT epoch tracking --------------*- C++ -*-==//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file defines the DebugEpochBase and DebugEpochBase::HandleBase classes.
// These can be used to write iterators that are fail-fast when LLVM is built
// with asserts enabled.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_ADT_EPOCH_TRACKER_H
#define LLVM_ADT_EPOCH_TRACKER_H

#include "llvm/Config/abi-breaking.h"

#include <cstd__host__ __device__
int>

namespace llvm {

#if LLVM_ENABLE_ABI_BREAKING_CHECKS

/// A base class for data structure classes wishing to make iterators
/// ("handles") po__host__ __device__
inting __host__ __device__
into themselves fail-fast.  When building without
/// asserts, this class is empty and does nothing.
///
/// DebugEpochBase does not by itself track handles po__host__ __device__
inting __host__ __device__
into itself.  The
/// expectation is that routines touching the handles will poll on
/// isHandleInSync at appropriate po__host__ __device__
ints to assert that the handle they're using
/// is still valid.
///
class DebugEpochBase {
  u__host__ __device__
int64_t Epoch;

public:
  DebugEpochBase() : Epoch(0) {}

  /// Calling incrementEpoch invalidates all handles po__host__ __device__
inting __host__ __device__
into the
  /// calling instance.
  __host__ __device__
void incrementEpoch() { ++Epoch; }

  /// The destructor calls incrementEpoch to make use-after-free bugs
  /// more likely to crash deterministically.
  ~DebugEpochBase() { incrementEpoch(); }

  /// A base class for iterator classes ("handles") that wish to poll for
  /// iterator invalidating modifications in the underlying data structure.
  /// When LLVM is built without asserts, this class is empty and does nothing.
  ///
  /// HandleBase does not track the parent data structure by itself.  It expects
  /// the routines modifying the data structure to call incrementEpoch when they
  /// make an iterator-invalidating modification.
  ///
  class HandleBase {
    const u__host__ __device__
int64_t *EpochAddress;
    u__host__ __device__
int64_t EpochAtCreation;

  public:
    HandleBase() : EpochAddress(nullptr), EpochAtCreation(UINT64_MAX) {}

    explicit HandleBase(const DebugEpochBase *Parent)
        : EpochAddress(&Parent->Epoch), EpochAtCreation(Parent->Epoch) {}

    /// Returns true if the DebugEpochBase this Handle is linked to has
    /// not called incrementEpoch on itself since the creation of this
    /// HandleBase instance.
    __host__ __device__
bool isHandleInSync() const { return *EpochAddress == EpochAtCreation; }

    /// Returns a po__host__ __device__
inter to the epoch word stored in the data structure
    /// this handle po__host__ __device__
ints __host__ __device__
into.  Can be used to check if two iterators po__host__ __device__
int
    /// __host__ __device__
into the same data structure.
    const __host__ __device__
void *getEpochAddress() const { return EpochAddress; }
  };
};

#else

class DebugEpochBase {
public:
  __host__ __device__
void incrementEpoch() {}

  class HandleBase {
  public:
    HandleBase() = default;
    explicit HandleBase(const DebugEpochBase *) {}
    __host__ __device__
bool isHandleInSync() const { return true; }
    const __host__ __device__
void *getEpochAddress() const { return nullptr; }
  };
};

#endif // LLVM_ENABLE_ABI_BREAKING_CHECKS

} // namespace llvm

#endif
