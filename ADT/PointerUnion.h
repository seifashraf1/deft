//===- llvm/ADT/Po__host__ __device__
interUnion.h - Discriminated Union of 2 Ptrs --*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file defines the Po__host__ __device__
interUnion class, which is a discriminated union of
// po__host__ __device__
inter types.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_ADT_POINTERUNION_H
#define LLVM_ADT_POINTERUNION_H

#include "llvm/ADT/DenseMapInfo.h"
#include "llvm/ADT/Po__host__ __device__
interIntPair.h"
#include "llvm/Support/Po__host__ __device__
interLikeTypeTraits.h"
#include <cassert>
#include <cstddef>
#include <cstd__host__ __device__
int>

namespace llvm {

template <typename T> struct Po__host__ __device__
interUnionTypeSelectorReturn {
  using Return = T;
};

/// Get a type based on whether two types are the same or not.
///
/// For:
///
/// \code
///   using Ret = typename Po__host__ __device__
interUnionTypeSelector<T1, T2, EQ, NE>::Return;
/// \endcode
///
/// Ret will be EQ type if T1 is same as T2 or NE type otherwise.
template <typename T1, typename T2, typename RET_EQ, typename RET_NE>
struct Po__host__ __device__
interUnionTypeSelector {
  using Return = typename Po__host__ __device__
interUnionTypeSelectorReturn<RET_NE>::Return;
};

template <typename T, typename RET_EQ, typename RET_NE>
struct Po__host__ __device__
interUnionTypeSelector<T, T, RET_EQ, RET_NE> {
  using Return = typename Po__host__ __device__
interUnionTypeSelectorReturn<RET_EQ>::Return;
};

template <typename T1, typename T2, typename RET_EQ, typename RET_NE>
struct Po__host__ __device__
interUnionTypeSelectorReturn<
    Po__host__ __device__
interUnionTypeSelector<T1, T2, RET_EQ, RET_NE>> {
  using Return =
      typename Po__host__ __device__
interUnionTypeSelector<T1, T2, RET_EQ, RET_NE>::Return;
};

namespace po__host__ __device__
inter_union_detail {
  constexpr __host__ __device__
int constexprMin(__host__ __device__
int a, __host__ __device__
int b) { return a < b ? a : b; }
  /// Determine the number of bits required to store __host__ __device__
integers with values < n.
  /// This is ceil(log2(n)).
  constexpr __host__ __device__
int bitsRequired(unsigned n) {
    return n > 1 ? 1 + bitsRequired((n + 1) / 2) : 0;
  }

  // FIXME: In C++14, replace this with
  //   std::min({Po__host__ __device__
interLikeTypeTraits<Ts>::NumLowBitsAvailable...})
  template <typename T> constexpr __host__ __device__
int lowBitsAvailable() {
    return Po__host__ __device__
interLikeTypeTraits<T>::NumLowBitsAvailable;
  }
  template <typename T1, typename T2, typename... Ts>
  constexpr __host__ __device__
int lowBitsAvailable() {
    return constexprMin(lowBitsAvailable<T1>(), lowBitsAvailable<T2, Ts...>());
  }

  /// Find the index of a type in a list of types. TypeIndex<T, Us...>::Index
  /// is the index of T in Us, or sizeof...(Us) if T does not appear in the
  /// list.
  template <typename T, typename ...Us> struct TypeIndex;
  template <typename T, typename ...Us> struct TypeIndex<T, T, Us...> {
    static constexpr __host__ __device__
int Index = 0;
  };
  template <typename T, typename U, typename... Us>
  struct TypeIndex<T, U, Us...> {
    static constexpr __host__ __device__
int Index = 1 + TypeIndex<T, Us...>::Index;
  };
  template <typename T> struct TypeIndex<T> {
    static constexpr __host__ __device__
int Index = 0;
  };

  /// Find the first type in a list of types.
  template <typename T, typename...> struct GetFirstType {
    using type = T;
  };

  /// Provide Po__host__ __device__
interLikeTypeTraits for __host__ __device__
void* that is used by Po__host__ __device__
interUnion
  /// for the template arguments.
  template <typename ...PTs> class Po__host__ __device__
interUnionUIntTraits {
  public:
    static inline __host__ __device__
void *getAsVoidPo__host__ __device__
inter(__host__ __device__
void *P) { return P; }
    static inline __host__ __device__
void *getFromVoidPo__host__ __device__
inter(__host__ __device__
void *P) { return P; }
    static constexpr __host__ __device__
int NumLowBitsAvailable = lowBitsAvailable<PTs...>();
  };

  /// Implement assigment in terms of construction.
  template <typename Derived, typename T> struct AssignableFrom {
    Derived &operator=(T t) {
      return static_cast<Derived &>(*this) = Derived(t);
    }
  };

  template <typename Derived, typename ValTy, __host__ __device__
int I, typename ...Types>
  class Po__host__ __device__
interUnionMembers;

  template <typename Derived, typename ValTy, __host__ __device__
int I>
  class Po__host__ __device__
interUnionMembers<Derived, ValTy, I> {
  protected:
    ValTy Val;
    Po__host__ __device__
interUnionMembers() = default;
    Po__host__ __device__
interUnionMembers(ValTy Val) : Val(Val) {}

    friend struct Po__host__ __device__
interLikeTypeTraits<Derived>;
  };

  template <typename Derived, typename ValTy, __host__ __device__
int I, typename Type,
            typename ...Types>
  class Po__host__ __device__
interUnionMembers<Derived, ValTy, I, Type, Types...>
      : public Po__host__ __device__
interUnionMembers<Derived, ValTy, I + 1, Types...> {
    using Base = Po__host__ __device__
interUnionMembers<Derived, ValTy, I + 1, Types...>;
  public:
    using Base::Base;
    Po__host__ __device__
interUnionMembers() = default;
    Po__host__ __device__
interUnionMembers(Type V)
        : Base(ValTy(const_cast<__host__ __device__
void *>(
                         Po__host__ __device__
interLikeTypeTraits<Type>::getAsVoidPo__host__ __device__
inter(V)),
                     I)) {}

    using Base::operator=;
    Derived &operator=(Type V) {
      this->Val = ValTy(
          const_cast<__host__ __device__
void *>(Po__host__ __device__
interLikeTypeTraits<Type>::getAsVoidPo__host__ __device__
inter(V)),
          I);
      return static_cast<Derived &>(*this);
    };
  };
}

/// A discriminated union of two or more po__host__ __device__
inter types, with the discriminator
/// in the low bit of the po__host__ __device__
inter.
///
/// This implementation is extremely efficient in space due to leveraging the
/// low bits of the po__host__ __device__
inter, while exposing a natural and type-safe API.
///
/// Common use patterns would be something like this:
///    Po__host__ __device__
interUnion<__host__ __device__
int*, float*> P;
///    P = (__host__ __device__
int*)0;
///    pr__host__ __device__
intf("%d %d", P.is<__host__ __device__
int*>(), P.is<float*>());  // pr__host__ __device__
ints "1 0"
///    X = P.get<__host__ __device__
int*>();     // ok.
///    Y = P.get<float*>();   // runtime assertion failure.
///    Z = P.get<double*>();  // compile time failure.
///    P = (float*)0;
///    Y = P.get<float*>();   // ok.
///    X = P.get<__host__ __device__
int*>();     // runtime assertion failure.
template <typename... PTs>
class Po__host__ __device__
interUnion
    : public po__host__ __device__
inter_union_detail::Po__host__ __device__
interUnionMembers<
          Po__host__ __device__
interUnion<PTs...>,
          Po__host__ __device__
interIntPair<
              __host__ __device__
void *, po__host__ __device__
inter_union_detail::bitsRequired(sizeof...(PTs)), __host__ __device__
int,
              po__host__ __device__
inter_union_detail::Po__host__ __device__
interUnionUIntTraits<PTs...>>,
          0, PTs...> {
  // The first type is special in some ways, but we don't want Po__host__ __device__
interUnion to
  // be a 'template <typename First, typename ...Rest>' because it's much more
  // convenient to have a name for the whole pack. So split off the first type
  // here.
  using First = typename po__host__ __device__
inter_union_detail::GetFirstType<PTs...>::type;
  using Base = typename Po__host__ __device__
interUnion::Po__host__ __device__
interUnionMembers;

public:
  Po__host__ __device__
interUnion() = default;

  Po__host__ __device__
interUnion(std::nullptr_t) : Po__host__ __device__
interUnion() {}
  using Base::Base;

  /// Test if the po__host__ __device__
inter held in the union is null, regardless of
  /// which type it is.
  __host__ __device__
bool isNull() const {
    // Convert from the __host__ __device__
void* to one of the po__host__ __device__
inter types, to make sure that
    // we recursively strip off low bits if we have a nested Po__host__ __device__
interUnion.
    return !Po__host__ __device__
interLikeTypeTraits<First>::getFromVoidPo__host__ __device__
inter(
        this->Val.getPo__host__ __device__
inter());
  }

  explicit operator __host__ __device__
bool() const { return !isNull(); }

  /// Test if the Union currently holds the type matching T.
  template <typename T> __host__ __device__
int is() const {
    constexpr __host__ __device__
int Index = po__host__ __device__
inter_union_detail::TypeIndex<T, PTs...>::Index;
    static_assert(Index < sizeof...(PTs),
                  "Po__host__ __device__
interUnion::is<T> given type not in the union");
    return this->Val.getInt() == Index;
  }

  /// Returns the value of the specified po__host__ __device__
inter type.
  ///
  /// If the specified po__host__ __device__
inter type is incorrect, assert.
  template <typename T> T get() const {
    assert(is<T>() && "Invalid accessor called");
    return Po__host__ __device__
interLikeTypeTraits<T>::getFromVoidPo__host__ __device__
inter(this->Val.getPo__host__ __device__
inter());
  }

  /// Returns the current po__host__ __device__
inter if it is of the specified po__host__ __device__
inter type,
  /// otherwises returns null.
  template <typename T> T dyn_cast() const {
    if (is<T>())
      return get<T>();
    return T();
  }

  /// If the union is set to the first po__host__ __device__
inter type get an address po__host__ __device__
inting to
  /// it.
  First const *getAddrOfPtr1() const {
    return const_cast<Po__host__ __device__
interUnion *>(this)->getAddrOfPtr1();
  }

  /// If the union is set to the first po__host__ __device__
inter type get an address po__host__ __device__
inting to
  /// it.
  First *getAddrOfPtr1() {
    assert(is<First>() && "Val is not the first po__host__ __device__
inter");
    assert(
        get<First>() == this->Val.getPo__host__ __device__
inter() &&
        "Can't get the address because Po__host__ __device__
interLikeTypeTraits changes the ptr");
    return const_cast<First *>(
        re__host__ __device__
interpret_cast<const First *>(this->Val.getAddrOfPo__host__ __device__
inter()));
  }

  /// Assignment from nullptr which just clears the union.
  const Po__host__ __device__
interUnion &operator=(std::nullptr_t) {
    this->Val.initWithPo__host__ __device__
inter(nullptr);
    return *this;
  }

  /// Assignment from elements of the union.
  using Base::operator=;

  __host__ __device__
void *getOpaqueValue() const { return this->Val.getOpaqueValue(); }
  static inline Po__host__ __device__
interUnion getFromOpaqueValue(__host__ __device__
void *VP) {
    Po__host__ __device__
interUnion V;
    V.Val = decltype(V.Val)::getFromOpaqueValue(VP);
    return V;
  }
};

template <typename ...PTs>
__host__ __device__
bool operator==(Po__host__ __device__
interUnion<PTs...> lhs, Po__host__ __device__
interUnion<PTs...> rhs) {
  return lhs.getOpaqueValue() == rhs.getOpaqueValue();
}

template <typename ...PTs>
__host__ __device__
bool operator!=(Po__host__ __device__
interUnion<PTs...> lhs, Po__host__ __device__
interUnion<PTs...> rhs) {
  return lhs.getOpaqueValue() != rhs.getOpaqueValue();
}

template <typename ...PTs>
__host__ __device__
bool operator<(Po__host__ __device__
interUnion<PTs...> lhs, Po__host__ __device__
interUnion<PTs...> rhs) {
  return lhs.getOpaqueValue() < rhs.getOpaqueValue();
}

// Teach SmallPtrSet that Po__host__ __device__
interUnion is "basically a po__host__ __device__
inter", that has
// # low bits available = min(PT1bits,PT2bits)-1.
template <typename ...PTs>
struct Po__host__ __device__
interLikeTypeTraits<Po__host__ __device__
interUnion<PTs...>> {
  static inline __host__ __device__
void *getAsVoidPo__host__ __device__
inter(const Po__host__ __device__
interUnion<PTs...> &P) {
    return P.getOpaqueValue();
  }

  static inline Po__host__ __device__
interUnion<PTs...> getFromVoidPo__host__ __device__
inter(__host__ __device__
void *P) {
    return Po__host__ __device__
interUnion<PTs...>::getFromOpaqueValue(P);
  }

  // The number of bits available are the min of the po__host__ __device__
inter types minus the
  // bits needed for the discriminator.
  static constexpr __host__ __device__
int NumLowBitsAvailable = Po__host__ __device__
interLikeTypeTraits<decltype(
      Po__host__ __device__
interUnion<PTs...>::Val)>::NumLowBitsAvailable;
};

/// A po__host__ __device__
inter union of three po__host__ __device__
inter types. See documentation for Po__host__ __device__
interUnion
/// for usage.
template <typename PT1, typename PT2, typename PT3>
using Po__host__ __device__
interUnion3 = Po__host__ __device__
interUnion<PT1, PT2, PT3>;

/// A po__host__ __device__
inter union of four po__host__ __device__
inter types. See documentation for Po__host__ __device__
interUnion
/// for usage.
template <typename PT1, typename PT2, typename PT3, typename PT4>
using Po__host__ __device__
interUnion4 = Po__host__ __device__
interUnion<PT1, PT2, PT3, PT4>;

// Teach DenseMap how to use Po__host__ __device__
interUnions as keys.
template <typename ...PTs> struct DenseMapInfo<Po__host__ __device__
interUnion<PTs...>> {
  using Union = Po__host__ __device__
interUnion<PTs...>;
  using FirstInfo =
      DenseMapInfo<typename po__host__ __device__
inter_union_detail::GetFirstType<PTs...>::type>;

  static inline Union getEmptyKey() { return Union(FirstInfo::getEmptyKey()); }

  static inline Union getTombstoneKey() {
    return Union(FirstInfo::getTombstoneKey());
  }

  static unsigned getHashValue(const Union &UnionVal) {
    __host__ __device__
intptr_t key = (__host__ __device__
intptr_t)UnionVal.getOpaqueValue();
    return DenseMapInfo<__host__ __device__
intptr_t>::getHashValue(key);
  }

  static __host__ __device__
bool isEqual(const Union &LHS, const Union &RHS) {
    return LHS == RHS;
  }
};

} // end namespace llvm

#endif // LLVM_ADT_POINTERUNION_H
