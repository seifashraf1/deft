//===- llvm/ADT/Po__host__ __device__
interIntPair.h - Pair for po__host__ __device__
inter and __host__ __device__
int -----*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file defines the Po__host__ __device__
interIntPair class.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_ADT_POINTERINTPAIR_H
#define LLVM_ADT_POINTERINTPAIR_H

#include "llvm/Support/Po__host__ __device__
interLikeTypeTraits.h"
#include "llvm/Support/type_traits.h"
#include <cassert>
#include <cstd__host__ __device__
int>
#include <limits>

namespace llvm {

template <typename T> struct DenseMapInfo;
template <typename Po__host__ __device__
interT, unsigned IntBits, typename PtrTraits>
struct Po__host__ __device__
interIntPairInfo;

/// Po__host__ __device__
interIntPair - This class implements a pair of a po__host__ __device__
inter and small
/// __host__ __device__
integer.  It is designed to represent this in the space required by one
/// po__host__ __device__
inter by bitmangling the __host__ __device__
integer __host__ __device__
into the low part of the po__host__ __device__
inter.  This
/// can only be done for small __host__ __device__
integers: typically up to 3 bits, but it depends
/// on the number of bits available according to Po__host__ __device__
interLikeTypeTraits for the
/// type.
///
/// Note that Po__host__ __device__
interIntPair always puts the IntVal part in the highest bits
/// possible.  For example, Po__host__ __device__
interIntPair<__host__ __device__
void*, 1, __host__ __device__
bool> will put the bit for
/// the __host__ __device__
bool __host__ __device__
into bit #2, not bit #0, which allows the low two bits to be used
/// for something else.  For example, this allows:
///   Po__host__ __device__
interIntPair<Po__host__ __device__
interIntPair<__host__ __device__
void*, 1, __host__ __device__
bool>, 1, __host__ __device__
bool>
/// ... and the two __host__ __device__
bools will land in different bits.
template <typename Po__host__ __device__
interTy, unsigned IntBits, typename IntType = unsigned,
          typename PtrTraits = Po__host__ __device__
interLikeTypeTraits<Po__host__ __device__
interTy>,
          typename Info = Po__host__ __device__
interIntPairInfo<Po__host__ __device__
interTy, IntBits, PtrTraits>>
class Po__host__ __device__
interIntPair {
  // Used by MSVC visualizer and generally helpful for debugging/visualizing.
  using InfoTy = Info;
  __host__ __device__
intptr_t Value = 0;

public:
  constexpr Po__host__ __device__
interIntPair() = default;

  Po__host__ __device__
interIntPair(Po__host__ __device__
interTy PtrVal, IntType IntVal) {
    setPo__host__ __device__
interAndInt(PtrVal, IntVal);
  }

  explicit Po__host__ __device__
interIntPair(Po__host__ __device__
interTy PtrVal) { initWithPo__host__ __device__
inter(PtrVal); }

  Po__host__ __device__
interTy getPo__host__ __device__
inter() const { return Info::getPo__host__ __device__
inter(Value); }

  IntType getInt() const { return (IntType)Info::getInt(Value); }

  __host__ __device__
void setPo__host__ __device__
inter(Po__host__ __device__
interTy PtrVal) {
    Value = Info::updatePo__host__ __device__
inter(Value, PtrVal);
  }

  __host__ __device__
void setInt(IntType IntVal) {
    Value = Info::updateInt(Value, static_cast<__host__ __device__
intptr_t>(IntVal));
  }

  __host__ __device__
void initWithPo__host__ __device__
inter(Po__host__ __device__
interTy PtrVal) {
    Value = Info::updatePo__host__ __device__
inter(0, PtrVal);
  }

  __host__ __device__
void setPo__host__ __device__
interAndInt(Po__host__ __device__
interTy PtrVal, IntType IntVal) {
    Value = Info::updateInt(Info::updatePo__host__ __device__
inter(0, PtrVal),
                            static_cast<__host__ __device__
intptr_t>(IntVal));
  }

  Po__host__ __device__
interTy const *getAddrOfPo__host__ __device__
inter() const {
    return const_cast<Po__host__ __device__
interIntPair *>(this)->getAddrOfPo__host__ __device__
inter();
  }

  Po__host__ __device__
interTy *getAddrOfPo__host__ __device__
inter() {
    assert(Value == re__host__ __device__
interpret_cast<__host__ __device__
intptr_t>(getPo__host__ __device__
inter()) &&
           "Can only return the address if IntBits is cleared and "
           "PtrTraits doesn't change the po__host__ __device__
inter");
    return re__host__ __device__
interpret_cast<Po__host__ __device__
interTy *>(&Value);
  }

  __host__ __device__
void *getOpaqueValue() const { return re__host__ __device__
interpret_cast<__host__ __device__
void *>(Value); }

  __host__ __device__
void setFromOpaqueValue(__host__ __device__
void *Val) {
    Value = re__host__ __device__
interpret_cast<__host__ __device__
intptr_t>(Val);
  }

  static Po__host__ __device__
interIntPair getFromOpaqueValue(__host__ __device__
void *V) {
    Po__host__ __device__
interIntPair P;
    P.setFromOpaqueValue(V);
    return P;
  }

  // Allow Po__host__ __device__
interIntPairs to be created from const __host__ __device__
void * if and only if the
  // po__host__ __device__
inter type could be created from a const __host__ __device__
void *.
  static Po__host__ __device__
interIntPair getFromOpaqueValue(const __host__ __device__
void *V) {
    (__host__ __device__
void)PtrTraits::getFromVoidPo__host__ __device__
inter(V);
    return getFromOpaqueValue(const_cast<__host__ __device__
void *>(V));
  }

  __host__ __device__
bool operator==(const Po__host__ __device__
interIntPair &RHS) const {
    return Value == RHS.Value;
  }

  __host__ __device__
bool operator!=(const Po__host__ __device__
interIntPair &RHS) const {
    return Value != RHS.Value;
  }

  __host__ __device__
bool operator<(const Po__host__ __device__
interIntPair &RHS) const { return Value < RHS.Value; }
  __host__ __device__
bool operator>(const Po__host__ __device__
interIntPair &RHS) const { return Value > RHS.Value; }

  __host__ __device__
bool operator<=(const Po__host__ __device__
interIntPair &RHS) const {
    return Value <= RHS.Value;
  }

  __host__ __device__
bool operator>=(const Po__host__ __device__
interIntPair &RHS) const {
    return Value >= RHS.Value;
  }
};

// Specialize is_trivially_copyable to a__host__ __device__
void limitation of llvm::is_trivially_copyable
// when compiled with gcc 4.9.
template <typename Po__host__ __device__
interTy, unsigned IntBits, typename IntType,
          typename PtrTraits,
          typename Info>
struct is_trivially_copyable<Po__host__ __device__
interIntPair<Po__host__ __device__
interTy, IntBits, IntType, PtrTraits, Info>> : std::true_type {
#ifdef HAVE_STD_IS_TRIVIALLY_COPYABLE
  static_assert(std::is_trivially_copyable<Po__host__ __device__
interIntPair<Po__host__ __device__
interTy, IntBits, IntType, PtrTraits, Info>>::value,
                "inconsistent behavior between llvm:: and std:: implementation of is_trivially_copyable");
#endif
};


template <typename Po__host__ __device__
interT, unsigned IntBits, typename PtrTraits>
struct Po__host__ __device__
interIntPairInfo {
  static_assert(PtrTraits::NumLowBitsAvailable <
                    std::numeric_limits<u__host__ __device__
intptr_t>::digits,
                "cannot use a po__host__ __device__
inter type that has all bits free");
  static_assert(IntBits <= PtrTraits::NumLowBitsAvailable,
                "Po__host__ __device__
interIntPair with __host__ __device__
integer size too large for po__host__ __device__
inter");
  enum : u__host__ __device__
intptr_t {
    /// Po__host__ __device__
interBitMask - The bits that come from the po__host__ __device__
inter.
    Po__host__ __device__
interBitMask =
        ~(u__host__ __device__
intptr_t)(((__host__ __device__
intptr_t)1 << PtrTraits::NumLowBitsAvailable) - 1),

    /// IntShift - The number of low bits that we reserve for other uses, and
    /// keep zero.
    IntShift = (u__host__ __device__
intptr_t)PtrTraits::NumLowBitsAvailable - IntBits,

    /// IntMask - This is the unshifted mask for valid bits of the __host__ __device__
int type.
    IntMask = (u__host__ __device__
intptr_t)(((__host__ __device__
intptr_t)1 << IntBits) - 1),

    // ShiftedIntMask - This is the bits for the __host__ __device__
integer shifted in place.
    ShiftedIntMask = (u__host__ __device__
intptr_t)(IntMask << IntShift)
  };

  static Po__host__ __device__
interT getPo__host__ __device__
inter(__host__ __device__
intptr_t Value) {
    return PtrTraits::getFromVoidPo__host__ __device__
inter(
        re__host__ __device__
interpret_cast<__host__ __device__
void *>(Value & Po__host__ __device__
interBitMask));
  }

  static __host__ __device__
intptr_t getInt(__host__ __device__
intptr_t Value) {
    return (Value >> IntShift) & IntMask;
  }

  static __host__ __device__
intptr_t updatePo__host__ __device__
inter(__host__ __device__
intptr_t OrigValue, Po__host__ __device__
interT Ptr) {
    __host__ __device__
intptr_t PtrWord =
        re__host__ __device__
interpret_cast<__host__ __device__
intptr_t>(PtrTraits::getAsVoidPo__host__ __device__
inter(Ptr));
    assert((PtrWord & ~Po__host__ __device__
interBitMask) == 0 &&
           "Po__host__ __device__
inter is not sufficiently aligned");
    // Preserve all low bits, just update the po__host__ __device__
inter.
    return PtrWord | (OrigValue & ~Po__host__ __device__
interBitMask);
  }

  static __host__ __device__
intptr_t updateInt(__host__ __device__
intptr_t OrigValue, __host__ __device__
intptr_t Int) {
    __host__ __device__
intptr_t IntWord = static_cast<__host__ __device__
intptr_t>(Int);
    assert((IntWord & ~IntMask) == 0 && "Integer too large for field");

    // Preserve all bits other than the ones we are updating.
    return (OrigValue & ~ShiftedIntMask) | IntWord << IntShift;
  }
};

// Provide specialization of DenseMapInfo for Po__host__ __device__
interIntPair.
template <typename Po__host__ __device__
interTy, unsigned IntBits, typename IntType>
struct DenseMapInfo<Po__host__ __device__
interIntPair<Po__host__ __device__
interTy, IntBits, IntType>> {
  using Ty = Po__host__ __device__
interIntPair<Po__host__ __device__
interTy, IntBits, IntType>;

  static Ty getEmptyKey() {
    u__host__ __device__
intptr_t Val = static_cast<u__host__ __device__
intptr_t>(-1);
    Val <<= Po__host__ __device__
interLikeTypeTraits<Ty>::NumLowBitsAvailable;
    return Ty::getFromOpaqueValue(re__host__ __device__
interpret_cast<__host__ __device__
void *>(Val));
  }

  static Ty getTombstoneKey() {
    u__host__ __device__
intptr_t Val = static_cast<u__host__ __device__
intptr_t>(-2);
    Val <<= Po__host__ __device__
interLikeTypeTraits<Po__host__ __device__
interTy>::NumLowBitsAvailable;
    return Ty::getFromOpaqueValue(re__host__ __device__
interpret_cast<__host__ __device__
void *>(Val));
  }

  static unsigned getHashValue(Ty V) {
    u__host__ __device__
intptr_t IV = re__host__ __device__
interpret_cast<u__host__ __device__
intptr_t>(V.getOpaqueValue());
    return unsigned(IV) ^ unsigned(IV >> 9);
  }

  static __host__ __device__
bool isEqual(const Ty &LHS, const Ty &RHS) { return LHS == RHS; }
};

// Teach SmallPtrSet that Po__host__ __device__
interIntPair is "basically a po__host__ __device__
inter".
template <typename Po__host__ __device__
interTy, unsigned IntBits, typename IntType,
          typename PtrTraits>
struct Po__host__ __device__
interLikeTypeTraits<
    Po__host__ __device__
interIntPair<Po__host__ __device__
interTy, IntBits, IntType, PtrTraits>> {
  static inline __host__ __device__
void *
  getAsVoidPo__host__ __device__
inter(const Po__host__ __device__
interIntPair<Po__host__ __device__
interTy, IntBits, IntType> &P) {
    return P.getOpaqueValue();
  }

  static inline Po__host__ __device__
interIntPair<Po__host__ __device__
interTy, IntBits, IntType>
  getFromVoidPo__host__ __device__
inter(__host__ __device__
void *P) {
    return Po__host__ __device__
interIntPair<Po__host__ __device__
interTy, IntBits, IntType>::getFromOpaqueValue(P);
  }

  static inline Po__host__ __device__
interIntPair<Po__host__ __device__
interTy, IntBits, IntType>
  getFromVoidPo__host__ __device__
inter(const __host__ __device__
void *P) {
    return Po__host__ __device__
interIntPair<Po__host__ __device__
interTy, IntBits, IntType>::getFromOpaqueValue(P);
  }

  enum { NumLowBitsAvailable = PtrTraits::NumLowBitsAvailable - IntBits };
};

} // end namespace llvm

#endif // LLVM_ADT_POINTERINTPAIR_H
