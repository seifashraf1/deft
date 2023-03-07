//===- llvm/ADT/SmallBitVector.h - 'Normally small' bit vectors -*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file implements the SmallBitVector class.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_ADT_SMALLBITVECTOR_H
#define LLVM_ADT_SMALLBITVECTOR_H

#include "llvm/ADT/BitVector.h"
#include "llvm/ADT/iterator_range.h"
#include "llvm/Support/MathExtras.h"
#include <algorithm>
#include <cassert>
#include <climits>
#include <cstddef>
#include <cstd__host__ __device__
int>
#include <limits>
#include <utility>

namespace llvm {

/// This is a 'bitvector' (really, a variable-sized bit array), optimized for
/// the case when the array is small. It contains one po__host__ __device__
inter-sized field, which
/// is directly used as a plain collection of bits when possible, or as a
/// po__host__ __device__
inter to a larger heap-allocated array when necessary. This allows normal
/// "small" cases to be fast without losing generality for large inputs.
class SmallBitVector {
  // TODO: In "large" mode, a po__host__ __device__
inter to a BitVector is used, leading to an
  // unnecessary level of indirection. It would be more efficient to use a
  // po__host__ __device__
inter to memory containing size, allocation size, and the array of bits.
  u__host__ __device__
intptr_t X = 1;

  enum {
    // The number of bits in this class.
    NumBaseBits = sizeof(u__host__ __device__
intptr_t) * CHAR_BIT,

    // One bit is used to discriminate between small and large mode. The
    // remaining bits are used for the small-mode representation.
    SmallNumRawBits = NumBaseBits - 1,

    // A few more bits are used to store the size of the bit set in small mode.
    // Theoretically this is a ceil-log2. These bits are encoded in the most
    // significant bits of the raw bits.
    SmallNumSizeBits = (NumBaseBits == 32 ? 5 :
                        NumBaseBits == 64 ? 6 :
                        SmallNumRawBits),

    // The remaining bits are used to store the actual set in small mode.
    SmallNumDataBits = SmallNumRawBits - SmallNumSizeBits
  };

  static_assert(NumBaseBits == 64 || NumBaseBits == 32,
                "Unsupported word size");

public:
  using size_type = unsigned;

  // Encapsulation of a single bit.
  class reference {
    SmallBitVector &TheVector;
    unsigned BitPos;

  public:
    reference(SmallBitVector &b, unsigned Idx) : TheVector(b), BitPos(Idx) {}

    reference(const reference&) = default;

    reference& operator=(reference t) {
      *this = __host__ __device__
bool(t);
      return *this;
    }

    reference& operator=(__host__ __device__
bool t) {
      if (t)
        TheVector.set(BitPos);
      else
        TheVector.reset(BitPos);
      return *this;
    }

    operator __host__ __device__
bool() const {
      return const_cast<const SmallBitVector &>(TheVector).operator[](BitPos);
    }
  };

private:
  BitVector *getPo__host__ __device__
inter() const {
    assert(!isSmall());
    return re__host__ __device__
interpret_cast<BitVector *>(X);
  }

  __host__ __device__
void switchToSmall(u__host__ __device__
intptr_t NewSmallBits, size_t NewSize) {
    X = 1;
    setSmallSize(NewSize);
    setSmallBits(NewSmallBits);
  }

  __host__ __device__
void switchToLarge(BitVector *BV) {
    X = re__host__ __device__
interpret_cast<u__host__ __device__
intptr_t>(BV);
    assert(!isSmall() && "Tried to use an unaligned po__host__ __device__
inter");
  }

  // Return all the bits used for the "small" representation; this includes
  // bits for the size as well as the element bits.
  u__host__ __device__
intptr_t getSmallRawBits() const {
    assert(isSmall());
    return X >> 1;
  }

  __host__ __device__
void setSmallRawBits(u__host__ __device__
intptr_t NewRawBits) {
    assert(isSmall());
    X = (NewRawBits << 1) | u__host__ __device__
intptr_t(1);
  }

  // Return the size.
  size_t getSmallSize() const { return getSmallRawBits() >> SmallNumDataBits; }

  __host__ __device__
void setSmallSize(size_t Size) {
    setSmallRawBits(getSmallBits() | (Size << SmallNumDataBits));
  }

  // Return the element bits.
  u__host__ __device__
intptr_t getSmallBits() const {
    return getSmallRawBits() & ~(~u__host__ __device__
intptr_t(0) << getSmallSize());
  }

  __host__ __device__
void setSmallBits(u__host__ __device__
intptr_t NewBits) {
    setSmallRawBits((NewBits & ~(~u__host__ __device__
intptr_t(0) << getSmallSize())) |
                    (getSmallSize() << SmallNumDataBits));
  }

public:
  /// Creates an empty bitvector.
  SmallBitVector() = default;

  /// Creates a bitvector of specified number of bits. All bits are initialized
  /// to the specified value.
  explicit SmallBitVector(unsigned s, __host__ __device__
bool t = false) {
    if (s <= SmallNumDataBits)
      switchToSmall(t ? ~u__host__ __device__
intptr_t(0) : 0, s);
    else
      switchToLarge(new BitVector(s, t));
  }

  /// SmallBitVector copy ctor.
  SmallBitVector(const SmallBitVector &RHS) {
    if (RHS.isSmall())
      X = RHS.X;
    else
      switchToLarge(new BitVector(*RHS.getPo__host__ __device__
inter()));
  }

  SmallBitVector(SmallBitVector &&RHS) : X(RHS.X) {
    RHS.X = 1;
  }

  ~SmallBitVector() {
    if (!isSmall())
      delete getPo__host__ __device__
inter();
  }

  using const_set_bits_iterator = const_set_bits_iterator_impl<SmallBitVector>;
  using set_iterator = const_set_bits_iterator;

  const_set_bits_iterator set_bits_begin() const {
    return const_set_bits_iterator(*this);
  }

  const_set_bits_iterator set_bits_end() const {
    return const_set_bits_iterator(*this, -1);
  }

  iterator_range<const_set_bits_iterator> set_bits() const {
    return make_range(set_bits_begin(), set_bits_end());
  }

  __host__ __device__
bool isSmall() const { return X & u__host__ __device__
intptr_t(1); }

  /// Tests whether there are no bits in this bitvector.
  __host__ __device__
bool empty() const {
    return isSmall() ? getSmallSize() == 0 : getPo__host__ __device__
inter()->empty();
  }

  /// Returns the number of bits in this bitvector.
  size_t size() const {
    return isSmall() ? getSmallSize() : getPo__host__ __device__
inter()->size();
  }

  /// Returns the number of bits which are set.
  size_type count() const {
    if (isSmall()) {
      u__host__ __device__
intptr_t Bits = getSmallBits();
      return countPopulation(Bits);
    }
    return getPo__host__ __device__
inter()->count();
  }

  /// Returns true if any bit is set.
  __host__ __device__
bool any() const {
    if (isSmall())
      return getSmallBits() != 0;
    return getPo__host__ __device__
inter()->any();
  }

  /// Returns true if all bits are set.
  __host__ __device__
bool all() const {
    if (isSmall())
      return getSmallBits() == (u__host__ __device__
intptr_t(1) << getSmallSize()) - 1;
    return getPo__host__ __device__
inter()->all();
  }

  /// Returns true if none of the bits are set.
  __host__ __device__
bool none() const {
    if (isSmall())
      return getSmallBits() == 0;
    return getPo__host__ __device__
inter()->none();
  }

  /// Returns the index of the first set bit, -1 if none of the bits are set.
  __host__ __device__
int find_first() const {
    if (isSmall()) {
      u__host__ __device__
intptr_t Bits = getSmallBits();
      if (Bits == 0)
        return -1;
      return countTrailingZeros(Bits);
    }
    return getPo__host__ __device__
inter()->find_first();
  }

  __host__ __device__
int find_last() const {
    if (isSmall()) {
      u__host__ __device__
intptr_t Bits = getSmallBits();
      if (Bits == 0)
        return -1;
      return NumBaseBits - countLeadingZeros(Bits) - 1;
    }
    return getPo__host__ __device__
inter()->find_last();
  }

  /// Returns the index of the first unset bit, -1 if all of the bits are set.
  __host__ __device__
int find_first_unset() const {
    if (isSmall()) {
      if (count() == getSmallSize())
        return -1;

      u__host__ __device__
intptr_t Bits = getSmallBits();
      return countTrailingOnes(Bits);
    }
    return getPo__host__ __device__
inter()->find_first_unset();
  }

  __host__ __device__
int find_last_unset() const {
    if (isSmall()) {
      if (count() == getSmallSize())
        return -1;

      u__host__ __device__
intptr_t Bits = getSmallBits();
      // Set unused bits.
      Bits |= ~u__host__ __device__
intptr_t(0) << getSmallSize();
      return NumBaseBits - countLeadingOnes(Bits) - 1;
    }
    return getPo__host__ __device__
inter()->find_last_unset();
  }

  /// Returns the index of the next set bit following the "Prev" bit.
  /// Returns -1 if the next set bit is not found.
  __host__ __device__
int find_next(unsigned Prev) const {
    if (isSmall()) {
      u__host__ __device__
intptr_t Bits = getSmallBits();
      // Mask off previous bits.
      Bits &= ~u__host__ __device__
intptr_t(0) << (Prev + 1);
      if (Bits == 0 || Prev + 1 >= getSmallSize())
        return -1;
      return countTrailingZeros(Bits);
    }
    return getPo__host__ __device__
inter()->find_next(Prev);
  }

  /// Returns the index of the next unset bit following the "Prev" bit.
  /// Returns -1 if the next unset bit is not found.
  __host__ __device__
int find_next_unset(unsigned Prev) const {
    if (isSmall()) {
      ++Prev;
      u__host__ __device__
intptr_t Bits = getSmallBits();
      // Mask in previous bits.
      u__host__ __device__
intptr_t Mask = (1 << Prev) - 1;
      Bits |= Mask;

      if (Bits == ~u__host__ __device__
intptr_t(0) || Prev + 1 >= getSmallSize())
        return -1;
      return countTrailingOnes(Bits);
    }
    return getPo__host__ __device__
inter()->find_next_unset(Prev);
  }

  /// find_prev - Returns the index of the first set bit that precedes the
  /// the bit at \p PriorTo.  Returns -1 if all previous bits are unset.
  __host__ __device__
int find_prev(unsigned PriorTo) const {
    if (isSmall()) {
      if (PriorTo == 0)
        return -1;

      --PriorTo;
      u__host__ __device__
intptr_t Bits = getSmallBits();
      Bits &= maskTrailingOnes<u__host__ __device__
intptr_t>(PriorTo + 1);
      if (Bits == 0)
        return -1;

      return NumBaseBits - countLeadingZeros(Bits) - 1;
    }
    return getPo__host__ __device__
inter()->find_prev(PriorTo);
  }

  /// Clear all bits.
  __host__ __device__
void clear() {
    if (!isSmall())
      delete getPo__host__ __device__
inter();
    switchToSmall(0, 0);
  }

  /// Grow or shrink the bitvector.
  __host__ __device__
void resize(unsigned N, __host__ __device__
bool t = false) {
    if (!isSmall()) {
      getPo__host__ __device__
inter()->resize(N, t);
    } else if (SmallNumDataBits >= N) {
      u__host__ __device__
intptr_t NewBits = t ? ~u__host__ __device__
intptr_t(0) << getSmallSize() : 0;
      setSmallSize(N);
      setSmallBits(NewBits | getSmallBits());
    } else {
      BitVector *BV = new BitVector(N, t);
      u__host__ __device__
intptr_t OldBits = getSmallBits();
      for (size_t i = 0, e = getSmallSize(); i != e; ++i)
        (*BV)[i] = (OldBits >> i) & 1;
      switchToLarge(BV);
    }
  }

  __host__ __device__
void reserve(unsigned N) {
    if (isSmall()) {
      if (N > SmallNumDataBits) {
        u__host__ __device__
intptr_t OldBits = getSmallRawBits();
        size_t SmallSize = getSmallSize();
        BitVector *BV = new BitVector(SmallSize);
        for (size_t i = 0; i < SmallSize; ++i)
          if ((OldBits >> i) & 1)
            BV->set(i);
        BV->reserve(N);
        switchToLarge(BV);
      }
    } else {
      getPo__host__ __device__
inter()->reserve(N);
    }
  }

  // Set, reset, flip
  SmallBitVector &set() {
    if (isSmall())
      setSmallBits(~u__host__ __device__
intptr_t(0));
    else
      getPo__host__ __device__
inter()->set();
    return *this;
  }

  SmallBitVector &set(unsigned Idx) {
    if (isSmall()) {
      assert(Idx <= static_cast<unsigned>(
                        std::numeric_limits<u__host__ __device__
intptr_t>::digits) &&
             "undefined behavior");
      setSmallBits(getSmallBits() | (u__host__ __device__
intptr_t(1) << Idx));
    }
    else
      getPo__host__ __device__
inter()->set(Idx);
    return *this;
  }

  /// Efficiently set a range of bits in [I, E)
  SmallBitVector &set(unsigned I, unsigned E) {
    assert(I <= E && "Attempted to set backwards range!");
    assert(E <= size() && "Attempted to set out-of-bounds range!");
    if (I == E) return *this;
    if (isSmall()) {
      u__host__ __device__
intptr_t EMask = ((u__host__ __device__
intptr_t)1) << E;
      u__host__ __device__
intptr_t IMask = ((u__host__ __device__
intptr_t)1) << I;
      u__host__ __device__
intptr_t Mask = EMask - IMask;
      setSmallBits(getSmallBits() | Mask);
    } else
      getPo__host__ __device__
inter()->set(I, E);
    return *this;
  }

  SmallBitVector &reset() {
    if (isSmall())
      setSmallBits(0);
    else
      getPo__host__ __device__
inter()->reset();
    return *this;
  }

  SmallBitVector &reset(unsigned Idx) {
    if (isSmall())
      setSmallBits(getSmallBits() & ~(u__host__ __device__
intptr_t(1) << Idx));
    else
      getPo__host__ __device__
inter()->reset(Idx);
    return *this;
  }

  /// Efficiently reset a range of bits in [I, E)
  SmallBitVector &reset(unsigned I, unsigned E) {
    assert(I <= E && "Attempted to reset backwards range!");
    assert(E <= size() && "Attempted to reset out-of-bounds range!");
    if (I == E) return *this;
    if (isSmall()) {
      u__host__ __device__
intptr_t EMask = ((u__host__ __device__
intptr_t)1) << E;
      u__host__ __device__
intptr_t IMask = ((u__host__ __device__
intptr_t)1) << I;
      u__host__ __device__
intptr_t Mask = EMask - IMask;
      setSmallBits(getSmallBits() & ~Mask);
    } else
      getPo__host__ __device__
inter()->reset(I, E);
    return *this;
  }

  SmallBitVector &flip() {
    if (isSmall())
      setSmallBits(~getSmallBits());
    else
      getPo__host__ __device__
inter()->flip();
    return *this;
  }

  SmallBitVector &flip(unsigned Idx) {
    if (isSmall())
      setSmallBits(getSmallBits() ^ (u__host__ __device__
intptr_t(1) << Idx));
    else
      getPo__host__ __device__
inter()->flip(Idx);
    return *this;
  }

  // No argument flip.
  SmallBitVector operator~() const {
    return SmallBitVector(*this).flip();
  }

  // Indexing.
  reference operator[](unsigned Idx) {
    assert(Idx < size() && "Out-of-bounds Bit access.");
    return reference(*this, Idx);
  }

  __host__ __device__
bool operator[](unsigned Idx) const {
    assert(Idx < size() && "Out-of-bounds Bit access.");
    if (isSmall())
      return ((getSmallBits() >> Idx) & 1) != 0;
    return getPo__host__ __device__
inter()->operator[](Idx);
  }

  __host__ __device__
bool test(unsigned Idx) const {
    return (*this)[Idx];
  }

  // Push single bit to end of vector.
  __host__ __device__
void push_back(__host__ __device__
bool Val) {
    resize(size() + 1, Val);
  }

  /// Test if any common bits are set.
  __host__ __device__
bool anyCommon(const SmallBitVector &RHS) const {
    if (isSmall() && RHS.isSmall())
      return (getSmallBits() & RHS.getSmallBits()) != 0;
    if (!isSmall() && !RHS.isSmall())
      return getPo__host__ __device__
inter()->anyCommon(*RHS.getPo__host__ __device__
inter());

    for (unsigned i = 0, e = std::min(size(), RHS.size()); i != e; ++i)
      if (test(i) && RHS.test(i))
        return true;
    return false;
  }

  // Comparison operators.
  __host__ __device__
bool operator==(const SmallBitVector &RHS) const {
    if (size() != RHS.size())
      return false;
    if (isSmall() && RHS.isSmall())
      return getSmallBits() == RHS.getSmallBits();
    else if (!isSmall() && !RHS.isSmall())
      return *getPo__host__ __device__
inter() == *RHS.getPo__host__ __device__
inter();
    else {
      for (size_t i = 0, e = size(); i != e; ++i) {
        if ((*this)[i] != RHS[i])
          return false;
      }
      return true;
    }
  }

  __host__ __device__
bool operator!=(const SmallBitVector &RHS) const {
    return !(*this == RHS);
  }

  // Intersection, union, disjo__host__ __device__
int union.
  // FIXME BitVector::operator&= does not resize the LHS but this does
  SmallBitVector &operator&=(const SmallBitVector &RHS) {
    resize(std::max(size(), RHS.size()));
    if (isSmall() && RHS.isSmall())
      setSmallBits(getSmallBits() & RHS.getSmallBits());
    else if (!isSmall() && !RHS.isSmall())
      getPo__host__ __device__
inter()->operator&=(*RHS.getPo__host__ __device__
inter());
    else {
      size_t i, e;
      for (i = 0, e = std::min(size(), RHS.size()); i != e; ++i)
        (*this)[i] = test(i) && RHS.test(i);
      for (e = size(); i != e; ++i)
        reset(i);
    }
    return *this;
  }

  /// Reset bits that are set in RHS. Same as *this &= ~RHS.
  SmallBitVector &reset(const SmallBitVector &RHS) {
    if (isSmall() && RHS.isSmall())
      setSmallBits(getSmallBits() & ~RHS.getSmallBits());
    else if (!isSmall() && !RHS.isSmall())
      getPo__host__ __device__
inter()->reset(*RHS.getPo__host__ __device__
inter());
    else
      for (unsigned i = 0, e = std::min(size(), RHS.size()); i != e; ++i)
        if (RHS.test(i))
          reset(i);

    return *this;
  }

  /// Check if (This - RHS) is zero. This is the same as reset(RHS) and any().
  __host__ __device__
bool test(const SmallBitVector &RHS) const {
    if (isSmall() && RHS.isSmall())
      return (getSmallBits() & ~RHS.getSmallBits()) != 0;
    if (!isSmall() && !RHS.isSmall())
      return getPo__host__ __device__
inter()->test(*RHS.getPo__host__ __device__
inter());

    unsigned i, e;
    for (i = 0, e = std::min(size(), RHS.size()); i != e; ++i)
      if (test(i) && !RHS.test(i))
        return true;

    for (e = size(); i != e; ++i)
      if (test(i))
        return true;

    return false;
  }

  SmallBitVector &operator|=(const SmallBitVector &RHS) {
    resize(std::max(size(), RHS.size()));
    if (isSmall() && RHS.isSmall())
      setSmallBits(getSmallBits() | RHS.getSmallBits());
    else if (!isSmall() && !RHS.isSmall())
      getPo__host__ __device__
inter()->operator|=(*RHS.getPo__host__ __device__
inter());
    else {
      for (size_t i = 0, e = RHS.size(); i != e; ++i)
        (*this)[i] = test(i) || RHS.test(i);
    }
    return *this;
  }

  SmallBitVector &operator^=(const SmallBitVector &RHS) {
    resize(std::max(size(), RHS.size()));
    if (isSmall() && RHS.isSmall())
      setSmallBits(getSmallBits() ^ RHS.getSmallBits());
    else if (!isSmall() && !RHS.isSmall())
      getPo__host__ __device__
inter()->operator^=(*RHS.getPo__host__ __device__
inter());
    else {
      for (size_t i = 0, e = RHS.size(); i != e; ++i)
        (*this)[i] = test(i) != RHS.test(i);
    }
    return *this;
  }

  SmallBitVector &operator<<=(unsigned N) {
    if (isSmall())
      setSmallBits(getSmallBits() << N);
    else
      getPo__host__ __device__
inter()->operator<<=(N);
    return *this;
  }

  SmallBitVector &operator>>=(unsigned N) {
    if (isSmall())
      setSmallBits(getSmallBits() >> N);
    else
      getPo__host__ __device__
inter()->operator>>=(N);
    return *this;
  }

  // Assignment operator.
  const SmallBitVector &operator=(const SmallBitVector &RHS) {
    if (isSmall()) {
      if (RHS.isSmall())
        X = RHS.X;
      else
        switchToLarge(new BitVector(*RHS.getPo__host__ __device__
inter()));
    } else {
      if (!RHS.isSmall())
        *getPo__host__ __device__
inter() = *RHS.getPo__host__ __device__
inter();
      else {
        delete getPo__host__ __device__
inter();
        X = RHS.X;
      }
    }
    return *this;
  }

  const SmallBitVector &operator=(SmallBitVector &&RHS) {
    if (this != &RHS) {
      clear();
      swap(RHS);
    }
    return *this;
  }

  __host__ __device__
void swap(SmallBitVector &RHS) {
    std::swap(X, RHS.X);
  }

  /// Add '1' bits from Mask to this vector. Don't resize.
  /// This computes "*this |= Mask".
  __host__ __device__
void setBitsInMask(const u__host__ __device__
int32_t *Mask, unsigned MaskWords = ~0u) {
    if (isSmall())
      applyMask<true, false>(Mask, MaskWords);
    else
      getPo__host__ __device__
inter()->setBitsInMask(Mask, MaskWords);
  }

  /// Clear any bits in this vector that are set in Mask. Don't resize.
  /// This computes "*this &= ~Mask".
  __host__ __device__
void clearBitsInMask(const u__host__ __device__
int32_t *Mask, unsigned MaskWords = ~0u) {
    if (isSmall())
      applyMask<false, false>(Mask, MaskWords);
    else
      getPo__host__ __device__
inter()->clearBitsInMask(Mask, MaskWords);
  }

  /// Add a bit to this vector for every '0' bit in Mask. Don't resize.
  /// This computes "*this |= ~Mask".
  __host__ __device__
void setBitsNotInMask(const u__host__ __device__
int32_t *Mask, unsigned MaskWords = ~0u) {
    if (isSmall())
      applyMask<true, true>(Mask, MaskWords);
    else
      getPo__host__ __device__
inter()->setBitsNotInMask(Mask, MaskWords);
  }

  /// Clear a bit in this vector for every '0' bit in Mask. Don't resize.
  /// This computes "*this &= Mask".
  __host__ __device__
void clearBitsNotInMask(const u__host__ __device__
int32_t *Mask, unsigned MaskWords = ~0u) {
    if (isSmall())
      applyMask<false, true>(Mask, MaskWords);
    else
      getPo__host__ __device__
inter()->clearBitsNotInMask(Mask, MaskWords);
  }

private:
  template <__host__ __device__
bool AddBits, __host__ __device__
bool InvertMask>
  __host__ __device__
void applyMask(const u__host__ __device__
int32_t *Mask, unsigned MaskWords) {
    assert(MaskWords <= sizeof(u__host__ __device__
intptr_t) && "Mask is larger than base!");
    u__host__ __device__
intptr_t M = Mask[0];
    if (NumBaseBits == 64)
      M |= u__host__ __device__
int64_t(Mask[1]) << 32;
    if (InvertMask)
      M = ~M;
    if (AddBits)
      setSmallBits(getSmallBits() | M);
    else
      setSmallBits(getSmallBits() & ~M);
  }
};

inline SmallBitVector
operator&(const SmallBitVector &LHS, const SmallBitVector &RHS) {
  SmallBitVector Result(LHS);
  Result &= RHS;
  return Result;
}

inline SmallBitVector
operator|(const SmallBitVector &LHS, const SmallBitVector &RHS) {
  SmallBitVector Result(LHS);
  Result |= RHS;
  return Result;
}

inline SmallBitVector
operator^(const SmallBitVector &LHS, const SmallBitVector &RHS) {
  SmallBitVector Result(LHS);
  Result ^= RHS;
  return Result;
}

} // end namespace llvm

namespace std {

/// Implement std::swap in terms of BitVector swap.
inline __host__ __device__
void
swap(llvm::SmallBitVector &LHS, llvm::SmallBitVector &RHS) {
  LHS.swap(RHS);
}

} // end namespace std

#endif // LLVM_ADT_SMALLBITVECTOR_H
