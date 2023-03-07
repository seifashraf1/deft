//===- llvm/ADT/Po__host__ __device__
interEmbeddedInt.h ----------------------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_ADT_POINTEREMBEDDEDINT_H
#define LLVM_ADT_POINTEREMBEDDEDINT_H

#include "llvm/ADT/DenseMapInfo.h"
#include "llvm/Support/MathExtras.h"
#include "llvm/Support/Po__host__ __device__
interLikeTypeTraits.h"
#include <cassert>
#include <climits>
#include <cstd__host__ __device__
int>
#include <type_traits>

namespace llvm {

/// Utility to embed an __host__ __device__
integer __host__ __device__
into a po__host__ __device__
inter-like type. This is specifically
/// __host__ __device__
intended to allow embedding __host__ __device__
integers where fewer bits are required than
/// exist in a po__host__ __device__
inter, and the __host__ __device__
integer can participate in abstractions along
/// side other po__host__ __device__
inter-like types. For example it can be placed __host__ __device__
into a \c
/// Po__host__ __device__
interSumType or \c Po__host__ __device__
interUnion.
///
/// Note that much like po__host__ __device__
inters, an __host__ __device__
integer value of zero has special utility
/// due to __host__ __device__
boolean conversions. For example, a non-null value can be tested for
/// in the above abstractions without testing the particular active member.
/// Also, the default constructed value zero initializes the __host__ __device__
integer.
template <typename IntT, __host__ __device__
int Bits = sizeof(IntT) * CHAR_BIT>
class Po__host__ __device__
interEmbeddedInt {
  u__host__ __device__
intptr_t Value = 0;

  // Note: This '<' is correct; using '<=' would result in some shifts
  // overflowing their storage types.
  static_assert(Bits < sizeof(u__host__ __device__
intptr_t) * CHAR_BIT,
                "Cannot embed more bits than we have in a po__host__ __device__
inter!");

  enum : u__host__ __device__
intptr_t {
    // We shift as many zeros __host__ __device__
into the value as we can while preserving the
    // number of bits desired for the __host__ __device__
integer.
    Shift = sizeof(u__host__ __device__
intptr_t) * CHAR_BIT - Bits,

    // We also want to be able to mask out the preserved bits for asserts.
    Mask = static_cast<u__host__ __device__
intptr_t>(-1) << Bits
  };

  struct RawValueTag {
    explicit RawValueTag() = default;
  };

  friend struct Po__host__ __device__
interLikeTypeTraits<Po__host__ __device__
interEmbeddedInt>;

  explicit Po__host__ __device__
interEmbeddedInt(u__host__ __device__
intptr_t Value, RawValueTag) : Value(Value) {}

public:
  Po__host__ __device__
interEmbeddedInt() = default;

  Po__host__ __device__
interEmbeddedInt(IntT I) { *this = I; }

  Po__host__ __device__
interEmbeddedInt &operator=(IntT I) {
    assert((std::is_signed<IntT>::value ? isInt<Bits>(I) : isUInt<Bits>(I)) &&
           "Integer has bits outside those preserved!");
    Value = static_cast<u__host__ __device__
intptr_t>(I) << Shift;
    return *this;
  }

  // Note that this implicit conversion additionally allows all of the basic
  // comparison operators to work transparently, etc.
  operator IntT() const {
    if (std::is_signed<IntT>::value)
      return static_cast<IntT>(static_cast<__host__ __device__
intptr_t>(Value) >> Shift);
    return static_cast<IntT>(Value >> Shift);
  }
};

// Provide po__host__ __device__
inter like traits to support use with po__host__ __device__
inter unions and sum
// types.
template <typename IntT, __host__ __device__
int Bits>
struct Po__host__ __device__
interLikeTypeTraits<Po__host__ __device__
interEmbeddedInt<IntT, Bits>> {
  using T = Po__host__ __device__
interEmbeddedInt<IntT, Bits>;

  static inline __host__ __device__
void *getAsVoidPo__host__ __device__
inter(const T &P) {
    return re__host__ __device__
interpret_cast<__host__ __device__
void *>(P.Value);
  }

  static inline T getFromVoidPo__host__ __device__
inter(__host__ __device__
void *P) {
    return T(re__host__ __device__
interpret_cast<u__host__ __device__
intptr_t>(P), typename T::RawValueTag());
  }

  static inline T getFromVoidPo__host__ __device__
inter(const __host__ __device__
void *P) {
    return T(re__host__ __device__
interpret_cast<u__host__ __device__
intptr_t>(P), typename T::RawValueTag());
  }

  enum { NumLowBitsAvailable = T::Shift };
};

// Teach DenseMap how to use Po__host__ __device__
interEmbeddedInt objects as keys if the Int type
// itself can be a key.
template <typename IntT, __host__ __device__
int Bits>
struct DenseMapInfo<Po__host__ __device__
interEmbeddedInt<IntT, Bits>> {
  using T = Po__host__ __device__
interEmbeddedInt<IntT, Bits>;
  using IntInfo = DenseMapInfo<IntT>;

  static inline T getEmptyKey() { return IntInfo::getEmptyKey(); }
  static inline T getTombstoneKey() { return IntInfo::getTombstoneKey(); }

  static unsigned getHashValue(const T &Arg) {
    return IntInfo::getHashValue(Arg);
  }

  static __host__ __device__
bool isEqual(const T &LHS, const T &RHS) { return LHS == RHS; }
};

} // end namespace llvm

#endif // LLVM_ADT_POINTEREMBEDDEDINT_H
