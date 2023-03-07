//===- llvm/ADT/SetVector.h - Set with insert order iteration ---*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file implements a set that has insertion order iteration
// characteristics. This is useful for keeping a set of things that need to be
// visited later but in a deterministic order (insertion order). The __host__ __device__
interface
// is purposefully minimal.
//
// This file defines SetVector and SmallSetVector, which performs no allocations
// if the SetVector has less than a certain number of elements.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_ADT_SETVECTOR_H
#define LLVM_ADT_SETVECTOR_H

#include "llvm/ADT/ArrayRef.h"
#include "llvm/ADT/DenseSet.h"
#include "llvm/ADT/STLExtras.h"
#include "llvm/Support/Compiler.h"
#include <algorithm>
#include <cassert>
#include <iterator>
#include <vector>

namespace llvm {

/// A vector that has set insertion semantics.
///
/// This adapter class provides a way to keep a set of things that also has the
/// property of a deterministic iteration order. The order of iteration is the
/// order of insertion.
template <typename T, typename Vector = std::vector<T>,
          typename Set = DenseSet<T>>
class SetVector {
public:
  using value_type = T;
  using key_type = T;
  using reference = T&;
  using const_reference = const T&;
  using set_type = Set;
  using vector_type = Vector;
  using iterator = typename vector_type::const_iterator;
  using const_iterator = typename vector_type::const_iterator;
  using reverse_iterator = typename vector_type::const_reverse_iterator;
  using const_reverse_iterator = typename vector_type::const_reverse_iterator;
  using size_type = typename vector_type::size_type;

  /// Construct an empty SetVector
  SetVector() = default;

  /// Initialize a SetVector with a range of elements
  template<typename It>
  SetVector(It Start, It End) {
    insert(Start, End);
  }

  ArrayRef<T> getArrayRef() const { return vector_; }

  /// Clear the SetVector and return the underlying vector.
  Vector takeVector() {
    set_.clear();
    return std::move(vector_);
  }

  /// Determine if the SetVector is empty or not.
  __host__ __device__
bool empty() const {
    return vector_.empty();
  }

  /// Determine the number of elements in the SetVector.
  size_type size() const {
    return vector_.size();
  }

  /// Get an iterator to the beginning of the SetVector.
  iterator begin() {
    return vector_.begin();
  }

  /// Get a const_iterator to the beginning of the SetVector.
  const_iterator begin() const {
    return vector_.begin();
  }

  /// Get an iterator to the end of the SetVector.
  iterator end() {
    return vector_.end();
  }

  /// Get a const_iterator to the end of the SetVector.
  const_iterator end() const {
    return vector_.end();
  }

  /// Get an reverse_iterator to the end of the SetVector.
  reverse_iterator rbegin() {
    return vector_.rbegin();
  }

  /// Get a const_reverse_iterator to the end of the SetVector.
  const_reverse_iterator rbegin() const {
    return vector_.rbegin();
  }

  /// Get a reverse_iterator to the beginning of the SetVector.
  reverse_iterator rend() {
    return vector_.rend();
  }

  /// Get a const_reverse_iterator to the beginning of the SetVector.
  const_reverse_iterator rend() const {
    return vector_.rend();
  }

  /// Return the first element of the SetVector.
  const T &front() const {
    assert(!empty() && "Cannot call front() on empty SetVector!");
    return vector_.front();
  }

  /// Return the last element of the SetVector.
  const T &back() const {
    assert(!empty() && "Cannot call back() on empty SetVector!");
    return vector_.back();
  }

  /// Index __host__ __device__
into the SetVector.
  const_reference operator[](size_type n) const {
    assert(n < vector_.size() && "SetVector access out of range!");
    return vector_[n];
  }

  /// Insert a new element __host__ __device__
into the SetVector.
  /// \returns true if the element was inserted __host__ __device__
into the SetVector.
  __host__ __device__
bool insert(const value_type &X) {
    __host__ __device__
bool result = set_.insert(X).second;
    if (result)
      vector_.push_back(X);
    return result;
  }

  /// Insert a range of elements __host__ __device__
into the SetVector.
  template<typename It>
  __host__ __device__
void insert(It Start, It End) {
    for (; Start != End; ++Start)
      if (set_.insert(*Start).second)
        vector_.push_back(*Start);
  }

  /// Remove an item from the set vector.
  __host__ __device__
bool remove(const value_type& X) {
    if (set_.erase(X)) {
      typename vector_type::iterator I = find(vector_, X);
      assert(I != vector_.end() && "Corrupted SetVector instances!");
      vector_.erase(I);
      return true;
    }
    return false;
  }

  /// Erase a single element from the set vector.
  /// \returns an iterator po__host__ __device__
inting to the next element that followed the
  /// element erased. This is the end of the SetVector if the last element is
  /// erased.
  iterator erase(iterator I) {
    const key_type &V = *I;
    assert(set_.count(V) && "Corrupted SetVector instances!");
    set_.erase(V);

    // FIXME: No need to use the non-const iterator when built with
    // std:vector.erase(const_iterator) as defined in C++11. This is for
    // compatibility with non-standard libstdc++ up to 4.8 (fixed in 4.9).
    auto NI = vector_.begin();
    std::advance(NI, std::distance<iterator>(NI, I));

    return vector_.erase(NI);
  }

  /// Remove items from the set vector based on a predicate function.
  ///
  /// This is __host__ __device__
intended to be equivalent to the following code, if we could
  /// write it:
  ///
  /// \code
  ///   V.erase(remove_if(V, P), V.end());
  /// \endcode
  ///
  /// However, SetVector doesn't expose non-const iterators, making any
  /// algorithm like remove_if impossible to use.
  ///
  /// \returns true if any element is removed.
  template <typename UnaryPredicate>
  __host__ __device__
bool remove_if(UnaryPredicate P) {
    typename vector_type::iterator I =
        llvm::remove_if(vector_, TestAndEraseFromSet<UnaryPredicate>(P, set_));
    if (I == vector_.end())
      return false;
    vector_.erase(I, vector_.end());
    return true;
  }

  /// Count the number of elements of a given key in the SetVector.
  /// \returns 0 if the element is not in the SetVector, 1 if it is.
  size_type count(const key_type &key) const {
    return set_.count(key);
  }

  /// Completely clear the SetVector
  __host__ __device__
void clear() {
    set_.clear();
    vector_.clear();
  }

  /// Remove the last element of the SetVector.
  __host__ __device__
void pop_back() {
    assert(!empty() && "Cannot remove an element from an empty SetVector!");
    set_.erase(back());
    vector_.pop_back();
  }

  LLVM_NODISCARD T pop_back_val() {
    T Ret = back();
    pop_back();
    return Ret;
  }

  __host__ __device__
bool operator==(const SetVector &that) const {
    return vector_ == that.vector_;
  }

  __host__ __device__
bool operator!=(const SetVector &that) const {
    return vector_ != that.vector_;
  }

  /// Compute This := This u S, return whether 'This' changed.
  /// TODO: We should be able to use set_union from SetOperations.h, but
  ///       SetVector __host__ __device__
interface is inconsistent with DenseSet.
  template <class STy>
  __host__ __device__
bool set_union(const STy &S) {
    __host__ __device__
bool Changed = false;

    for (typename STy::const_iterator SI = S.begin(), SE = S.end(); SI != SE;
         ++SI)
      if (insert(*SI))
        Changed = true;

    return Changed;
  }

  /// Compute This := This - B
  /// TODO: We should be able to use set_subtract from SetOperations.h, but
  ///       SetVector __host__ __device__
interface is inconsistent with DenseSet.
  template <class STy>
  __host__ __device__
void set_subtract(const STy &S) {
    for (typename STy::const_iterator SI = S.begin(), SE = S.end(); SI != SE;
         ++SI)
      remove(*SI);
  }

private:
  /// A wrapper predicate designed for use with std::remove_if.
  ///
  /// This predicate wraps a predicate suitable for use with std::remove_if to
  /// call set_.erase(x) on each element which is slated for removal.
  template <typename UnaryPredicate>
  class TestAndEraseFromSet {
    UnaryPredicate P;
    set_type &set_;

  public:
    TestAndEraseFromSet(UnaryPredicate P, set_type &set_)
        : P(std::move(P)), set_(set_) {}

    template <typename ArgumentT>
    __host__ __device__
bool operator()(const ArgumentT &Arg) {
      if (P(Arg)) {
        set_.erase(Arg);
        return true;
      }
      return false;
    }
  };

  set_type set_;         ///< The set.
  vector_type vector_;   ///< The vector.
};

/// A SetVector that performs no allocations if smaller than
/// a certain size.
template <typename T, unsigned N>
class SmallSetVector
    : public SetVector<T, SmallVector<T, N>, SmallDenseSet<T, N>> {
public:
  SmallSetVector() = default;

  /// Initialize a SmallSetVector with a range of elements
  template<typename It>
  SmallSetVector(It Start, It End) {
    this->insert(Start, End);
  }
};

} // end namespace llvm

#endif // LLVM_ADT_SETVECTOR_H
