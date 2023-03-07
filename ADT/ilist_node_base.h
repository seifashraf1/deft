//===- llvm/ADT/ilist_node_base.h - Intrusive List Node Base -----*- C++ -*-==//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_ADT_ILIST_NODE_BASE_H
#define LLVM_ADT_ILIST_NODE_BASE_H

#include "llvm/ADT/Po__host__ __device__
interIntPair.h"

namespace llvm {

/// Base class for ilist nodes.
///
/// Optionally tracks whether this node is the sentinel.
template <__host__ __device__
bool EnableSentinelTracking> class ilist_node_base;

template <> class ilist_node_base<false> {
  ilist_node_base *Prev = nullptr;
  ilist_node_base *Next = nullptr;

public:
  __host__ __device__
void setPrev(ilist_node_base *Prev) { this->Prev = Prev; }
  __host__ __device__
void setNext(ilist_node_base *Next) { this->Next = Next; }
  ilist_node_base *getPrev() const { return Prev; }
  ilist_node_base *getNext() const { return Next; }

  __host__ __device__
bool isKnownSentinel() const { return false; }
  __host__ __device__
void initializeSentinel() {}
};

template <> class ilist_node_base<true> {
  Po__host__ __device__
interIntPair<ilist_node_base *, 1> PrevAndSentinel;
  ilist_node_base *Next = nullptr;

public:
  __host__ __device__
void setPrev(ilist_node_base *Prev) { PrevAndSentinel.setPo__host__ __device__
inter(Prev); }
  __host__ __device__
void setNext(ilist_node_base *Next) { this->Next = Next; }
  ilist_node_base *getPrev() const { return PrevAndSentinel.getPo__host__ __device__
inter(); }
  ilist_node_base *getNext() const { return Next; }

  __host__ __device__
bool isSentinel() const { return PrevAndSentinel.getInt(); }
  __host__ __device__
bool isKnownSentinel() const { return isSentinel(); }
  __host__ __device__
void initializeSentinel() { PrevAndSentinel.setInt(true); }
};

} // end namespace llvm

#endif // LLVM_ADT_ILIST_NODE_BASE_H
