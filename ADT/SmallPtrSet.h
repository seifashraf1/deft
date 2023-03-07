//===- llvm/ADT/SmallPtrSet.h - 'Normally small' po__host__ __device__
inter set ----*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file defines the SmallPtrSet class.  See the doxygen comment for
// SmallPtrSetImplBase for more details on the algorithm used.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_ADT_SMALLPTRSET_H
#define LLVM_ADT_SMALLPTRSET_H

#include "llvm/ADT/EpochTracker.h"
#include "llvm/Support/Compiler.h"
#include "llvm/Support/ReverseIteration.h"
#include "llvm/Support/type_traits.h"
#include <cassert>
#include <cstddef>
#include <cstdlib>
#include <cstring>
#include <initializer_list>
#include <iterator>
#include <utility>

namespace llvm {

/// SmallPtrSetImplBase - This is the common code shared among all the
/// SmallPtrSet<>'s, which is almost everything.  SmallPtrSet has two modes, one
/// for small and one for large sets.
///
/// Small sets use an array of po__host__ __device__
inters allocated in the SmallPtrSet object,
/// which is treated as a simple array of po__host__ __device__
inters.  When a po__host__ __device__
inter is added to
/// the set, the array is scanned to see if the element already exists, if not
/// the element is 'pushed back' onto the array.  If we run out of space in the
/// array, we grow __host__ __device__
into the 'large set' case.  SmallSet should be used when the
/// sets are often small.  In this case, no memory allocation is used, and only
/// light-weight and cache-efficient scanning is used.
///
/// Large sets use a classic exponentially-probed hash table.  Empty buckets are
/// represented with an illegal po__host__ __device__
inter value (-1) to allow null po__host__ __device__
inters to be
/// inserted.  Tombstones are represented with another illegal po__host__ __device__
inter value
/// (-2), to allow deletion.  The hash table is resized when the table is 3/4 or
/// more.  When this happens, the table is doubled in size.
///
class SmallPtrSetImplBase : public DebugEpochBase {
  friend class SmallPtrSetIteratorImpl;

protected:
  /// SmallArray - Po__host__ __device__
ints to a fixed size set of buckets, used in 'small mode'.
  const __host__ __device__
void **SmallArray;
  /// CurArray - This is the current set of buckets.  If equal to SmallArray,
  /// then the set is in 'small mode'.
  const __host__ __device__
void **CurArray;
  /// CurArraySize - The allocated size of CurArray, always a power of two.
  unsigned CurArraySize;

  /// Number of elements in CurArray that contain a value or are a tombstone.
  /// If small, all these elements are at the beginning of CurArray and the rest
  /// is uninitialized.
  unsigned NumNonEmpty;
  /// Number of tombstones in CurArray.
  unsigned NumTombstones;

  // Helpers to copy and move construct a SmallPtrSet.
  SmallPtrSetImplBase(const __host__ __device__
void **SmallStorage,
                      const SmallPtrSetImplBase &that);
  SmallPtrSetImplBase(const __host__ __device__
void **SmallStorage, unsigned SmallSize,
                      SmallPtrSetImplBase &&that);

  explicit SmallPtrSetImplBase(const __host__ __device__
void **SmallStorage, unsigned SmallSize)
      : SmallArray(SmallStorage), CurArray(SmallStorage),
        CurArraySize(SmallSize), NumNonEmpty(0), NumTombstones(0) {
    assert(SmallSize && (SmallSize & (SmallSize-1)) == 0 &&
           "Initial size must be a power of two!");
  }

  ~SmallPtrSetImplBase() {
    if (!isSmall())
      free(CurArray);
  }

public:
  using size_type = unsigned;

  SmallPtrSetImplBase &operator=(const SmallPtrSetImplBase &) = delete;

  LLVM_NODISCARD __host__ __device__
bool empty() const { return size() == 0; }
  size_type size() const { return NumNonEmpty - NumTombstones; }

  __host__ __device__
void clear() {
    incrementEpoch();
    // If the capacity of the array is huge, and the # elements used is small,
    // shrink the array.
    if (!isSmall()) {
      if (size() * 4 < CurArraySize && CurArraySize > 32)
        return shrink_and_clear();
      // Fill the array with empty markers.
      memset(CurArray, -1, CurArraySize * sizeof(__host__ __device__
void *));
    }

    NumNonEmpty = 0;
    NumTombstones = 0;
  }

protected:
  static __host__ __device__
void *getTombstoneMarker() { return re__host__ __device__
interpret_cast<__host__ __device__
void*>(-2); }

  static __host__ __device__
void *getEmptyMarker() {
    // Note that -1 is chosen to make clear() efficiently implementable with
    // memset and because it's not a valid po__host__ __device__
inter value.
    return re__host__ __device__
interpret_cast<__host__ __device__
void*>(-1);
  }

  const __host__ __device__
void **EndPo__host__ __device__
inter() const {
    return isSmall() ? CurArray + NumNonEmpty : CurArray + CurArraySize;
  }

  /// insert_imp - This returns true if the po__host__ __device__
inter was new to the set, false if
  /// it was already in the set.  This is hidden from the client so that the
  /// derived class can check that the right type of po__host__ __device__
inter is passed in.
  std::pair<const __host__ __device__
void *const *, __host__ __device__
bool> insert_imp(const __host__ __device__
void *Ptr) {
    if (isSmall()) {
      // Check to see if it is already in the set.
      const __host__ __device__
void **LastTombstone = nullptr;
      for (const __host__ __device__
void **APtr = SmallArray, **E = SmallArray + NumNonEmpty;
           APtr != E; ++APtr) {
        const __host__ __device__
void *Value = *APtr;
        if (Value == Ptr)
          return std::make_pair(APtr, false);
        if (Value == getTombstoneMarker())
          LastTombstone = APtr;
      }

      // Did we find any tombstone marker?
      if (LastTombstone != nullptr) {
        *LastTombstone = Ptr;
        --NumTombstones;
        incrementEpoch();
        return std::make_pair(LastTombstone, true);
      }

      // Nope, there isn't.  If we stay small, just 'pushback' now.
      if (NumNonEmpty < CurArraySize) {
        SmallArray[NumNonEmpty++] = Ptr;
        incrementEpoch();
        return std::make_pair(SmallArray + (NumNonEmpty - 1), true);
      }
      // Otherwise, hit the big set case, which will call grow.
    }
    return insert_imp_big(Ptr);
  }

  /// erase_imp - If the set contains the specified po__host__ __device__
inter, remove it and
  /// return true, otherwise return false.  This is hidden from the client so
  /// that the derived class can check that the right type of po__host__ __device__
inter is passed
  /// in.
  __host__ __device__
bool erase_imp(const __host__ __device__
void * Ptr) {
    const __host__ __device__
void *const *P = find_imp(Ptr);
    if (P == EndPo__host__ __device__
inter())
      return false;

    const __host__ __device__
void **Loc = const_cast<const __host__ __device__
void **>(P);
    assert(*Loc == Ptr && "broken find!");
    *Loc = getTombstoneMarker();
    NumTombstones++;
    return true;
  }

  /// Returns the raw po__host__ __device__
inter needed to construct an iterator.  If element not
  /// found, this will be EndPo__host__ __device__
inter.  Otherwise, it will be a po__host__ __device__
inter to the
  /// slot which stores Ptr;
  const __host__ __device__
void *const * find_imp(const __host__ __device__
void * Ptr) const {
    if (isSmall()) {
      // Linear search for the item.
      for (const __host__ __device__
void *const *APtr = SmallArray,
                      *const *E = SmallArray + NumNonEmpty; APtr != E; ++APtr)
        if (*APtr == Ptr)
          return APtr;
      return EndPo__host__ __device__
inter();
    }

    // Big set case.
    auto *Bucket = FindBucketFor(Ptr);
    if (*Bucket == Ptr)
      return Bucket;
    return EndPo__host__ __device__
inter();
  }

private:
  __host__ __device__
bool isSmall() const { return CurArray == SmallArray; }

  std::pair<const __host__ __device__
void *const *, __host__ __device__
bool> insert_imp_big(const __host__ __device__
void *Ptr);

  const __host__ __device__
void * const *FindBucketFor(const __host__ __device__
void *Ptr) const;
  __host__ __device__
void shrink_and_clear();

  /// Grow - Allocate a larger backing store for the buckets and move it over.
  __host__ __device__
void Grow(unsigned NewSize);

protected:
  /// swap - Swaps the elements of two sets.
  /// Note: This method assumes that both sets have the same small size.
  __host__ __device__
void swap(SmallPtrSetImplBase &RHS);

  __host__ __device__
void CopyFrom(const SmallPtrSetImplBase &RHS);
  __host__ __device__
void MoveFrom(unsigned SmallSize, SmallPtrSetImplBase &&RHS);

private:
  /// Code shared by MoveFrom() and move constructor.
  __host__ __device__
void MoveHelper(unsigned SmallSize, SmallPtrSetImplBase &&RHS);
  /// Code shared by CopyFrom() and copy constructor.
  __host__ __device__
void CopyHelper(const SmallPtrSetImplBase &RHS);
};

/// SmallPtrSetIteratorImpl - This is the common base class shared between all
/// instances of SmallPtrSetIterator.
class SmallPtrSetIteratorImpl {
protected:
  const __host__ __device__
void *const *Bucket;
  const __host__ __device__
void *const *End;

public:
  explicit SmallPtrSetIteratorImpl(const __host__ __device__
void *const *BP, const __host__ __device__
void*const *E)
    : Bucket(BP), End(E) {
    if (shouldReverseIterate()) {
      RetreatIfNotValid();
      return;
    }
    AdvanceIfNotValid();
  }

  __host__ __device__
bool operator==(const SmallPtrSetIteratorImpl &RHS) const {
    return Bucket == RHS.Bucket;
  }
  __host__ __device__
bool operator!=(const SmallPtrSetIteratorImpl &RHS) const {
    return Bucket != RHS.Bucket;
  }

protected:
  /// AdvanceIfNotValid - If the current bucket isn't valid, advance to a bucket
  /// that is.   This is guaranteed to stop because the end() bucket is marked
  /// valid.
  __host__ __device__
void AdvanceIfNotValid() {
    assert(Bucket <= End);
    while (Bucket != End &&
           (*Bucket == SmallPtrSetImplBase::getEmptyMarker() ||
            *Bucket == SmallPtrSetImplBase::getTombstoneMarker()))
      ++Bucket;
  }
  __host__ __device__
void RetreatIfNotValid() {
    assert(Bucket >= End);
    while (Bucket != End &&
           (Bucket[-1] == SmallPtrSetImplBase::getEmptyMarker() ||
            Bucket[-1] == SmallPtrSetImplBase::getTombstoneMarker())) {
      --Bucket;
    }
  }
};

/// SmallPtrSetIterator - This implements a const_iterator for SmallPtrSet.
template <typename PtrTy>
class SmallPtrSetIterator : public SmallPtrSetIteratorImpl,
                            DebugEpochBase::HandleBase {
  using PtrTraits = Po__host__ __device__
interLikeTypeTraits<PtrTy>;

public:
  using value_type = PtrTy;
  using reference = PtrTy;
  using po__host__ __device__
inter = PtrTy;
  using difference_type = std::ptrdiff_t;
  using iterator_category = std::forward_iterator_tag;

  explicit SmallPtrSetIterator(const __host__ __device__
void *const *BP, const __host__ __device__
void *const *E,
                               const DebugEpochBase &Epoch)
      : SmallPtrSetIteratorImpl(BP, E), DebugEpochBase::HandleBase(&Epoch) {}

  // Most methods provided by baseclass.

  const PtrTy operator*() const {
    assert(isHandleInSync() && "invalid iterator access!");
    if (shouldReverseIterate()) {
      assert(Bucket > End);
      return PtrTraits::getFromVoidPo__host__ __device__
inter(const_cast<__host__ __device__
void *>(Bucket[-1]));
    }
    assert(Bucket < End);
    return PtrTraits::getFromVoidPo__host__ __device__
inter(const_cast<__host__ __device__
void*>(*Bucket));
  }

  inline SmallPtrSetIterator& operator++() {          // Preincrement
    assert(isHandleInSync() && "invalid iterator access!");
    if (shouldReverseIterate()) {
      --Bucket;
      RetreatIfNotValid();
      return *this;
    }
    ++Bucket;
    AdvanceIfNotValid();
    return *this;
  }

  SmallPtrSetIterator operator++(__host__ __device__
int) {        // Postincrement
    SmallPtrSetIterator tmp = *this;
    ++*this;
    return tmp;
  }
};

/// RoundUpToPowerOfTwo - This is a helper template that rounds N up to the next
/// power of two (which means N itself if N is already a power of two).
template<unsigned N>
struct RoundUpToPowerOfTwo;

/// RoundUpToPowerOfTwoH - If N is not a power of two, increase it.  This is a
/// helper template used to implement RoundUpToPowerOfTwo.
template<unsigned N, __host__ __device__
bool isPowerTwo>
struct RoundUpToPowerOfTwoH {
  enum { Val = N };
};
template<unsigned N>
struct RoundUpToPowerOfTwoH<N, false> {
  enum {
    // We could just use NextVal = N+1, but this converges faster.  N|(N-1) sets
    // the right-most zero bits to one all at once, e.g. 0b0011000 -> 0b0011111.
    Val = RoundUpToPowerOfTwo<(N|(N-1)) + 1>::Val
  };
};

template<unsigned N>
struct RoundUpToPowerOfTwo {
  enum { Val = RoundUpToPowerOfTwoH<N, (N&(N-1)) == 0>::Val };
};

/// A templated base class for \c SmallPtrSet which provides the
/// typesafe __host__ __device__
interface that is common across all small sizes.
///
/// This is particularly useful for passing around between __host__ __device__
interface boundaries
/// to a__host__ __device__
void encoding a particular small size in the __host__ __device__
interface boundary.
template <typename PtrType>
class SmallPtrSetImpl : public SmallPtrSetImplBase {
  using ConstPtrType = typename add_const_past_po__host__ __device__
inter<PtrType>::type;
  using PtrTraits = Po__host__ __device__
interLikeTypeTraits<PtrType>;
  using ConstPtrTraits = Po__host__ __device__
interLikeTypeTraits<ConstPtrType>;

protected:
  // Constructors that forward to the base.
  SmallPtrSetImpl(const __host__ __device__
void **SmallStorage, const SmallPtrSetImpl &that)
      : SmallPtrSetImplBase(SmallStorage, that) {}
  SmallPtrSetImpl(const __host__ __device__
void **SmallStorage, unsigned SmallSize,
                  SmallPtrSetImpl &&that)
      : SmallPtrSetImplBase(SmallStorage, SmallSize, std::move(that)) {}
  explicit SmallPtrSetImpl(const __host__ __device__
void **SmallStorage, unsigned SmallSize)
      : SmallPtrSetImplBase(SmallStorage, SmallSize) {}

public:
  using iterator = SmallPtrSetIterator<PtrType>;
  using const_iterator = SmallPtrSetIterator<PtrType>;
  using key_type = ConstPtrType;
  using value_type = PtrType;

  SmallPtrSetImpl(const SmallPtrSetImpl &) = delete;

  /// Inserts Ptr if and only if there is no element in the container equal to
  /// Ptr. The __host__ __device__
bool component of the returned pair is true if and only if the
  /// insertion takes place, and the iterator component of the pair po__host__ __device__
ints to
  /// the element equal to Ptr.
  std::pair<iterator, __host__ __device__
bool> insert(PtrType Ptr) {
    auto p = insert_imp(PtrTraits::getAsVoidPo__host__ __device__
inter(Ptr));
    return std::make_pair(makeIterator(p.first), p.second);
  }

  /// erase - If the set contains the specified po__host__ __device__
inter, remove it and return
  /// true, otherwise return false.
  __host__ __device__
bool erase(PtrType Ptr) {
    return erase_imp(PtrTraits::getAsVoidPo__host__ __device__
inter(Ptr));
  }
  /// count - Return 1 if the specified po__host__ __device__
inter is in the set, 0 otherwise.
  size_type count(ConstPtrType Ptr) const { return find(Ptr) != end() ? 1 : 0; }
  iterator find(ConstPtrType Ptr) const {
    return makeIterator(find_imp(ConstPtrTraits::getAsVoidPo__host__ __device__
inter(Ptr)));
  }

  template <typename IterT>
  __host__ __device__
void insert(IterT I, IterT E) {
    for (; I != E; ++I)
      insert(*I);
  }

  __host__ __device__
void insert(std::initializer_list<PtrType> IL) {
    insert(IL.begin(), IL.end());
  }

  iterator begin() const {
    if (shouldReverseIterate())
      return makeIterator(EndPo__host__ __device__
inter() - 1);
    return makeIterator(CurArray);
  }
  iterator end() const { return makeIterator(EndPo__host__ __device__
inter()); }

private:
  /// Create an iterator that dereferences to same place as the given po__host__ __device__
inter.
  iterator makeIterator(const __host__ __device__
void *const *P) const {
    if (shouldReverseIterate())
      return iterator(P == EndPo__host__ __device__
inter() ? CurArray : P + 1, CurArray, *this);
    return iterator(P, EndPo__host__ __device__
inter(), *this);
  }
};

/// SmallPtrSet - This class implements a set which is optimized for holding
/// SmallSize or less elements.  This __host__ __device__
internally rounds up SmallSize to the next
/// power of two if it is not already a power of two.  See the comments above
/// SmallPtrSetImplBase for details of the algorithm.
template<class PtrType, unsigned SmallSize>
class SmallPtrSet : public SmallPtrSetImpl<PtrType> {
  // In small mode SmallPtrSet uses linear search for the elements, so it is
  // not a good idea to choose this value too high. You may consider using a
  // DenseSet<> instead if you expect many elements in the set.
  static_assert(SmallSize <= 32, "SmallSize should be small");

  using BaseT = SmallPtrSetImpl<PtrType>;

  // Make sure that SmallSize is a power of two, round up if not.
  enum { SmallSizePowTwo = RoundUpToPowerOfTwo<SmallSize>::Val };
  /// SmallStorage - Fixed size storage used in 'small mode'.
  const __host__ __device__
void *SmallStorage[SmallSizePowTwo];

public:
  SmallPtrSet() : BaseT(SmallStorage, SmallSizePowTwo) {}
  SmallPtrSet(const SmallPtrSet &that) : BaseT(SmallStorage, that) {}
  SmallPtrSet(SmallPtrSet &&that)
      : BaseT(SmallStorage, SmallSizePowTwo, std::move(that)) {}

  template<typename It>
  SmallPtrSet(It I, It E) : BaseT(SmallStorage, SmallSizePowTwo) {
    this->insert(I, E);
  }

  SmallPtrSet(std::initializer_list<PtrType> IL)
      : BaseT(SmallStorage, SmallSizePowTwo) {
    this->insert(IL.begin(), IL.end());
  }

  SmallPtrSet<PtrType, SmallSize> &
  operator=(const SmallPtrSet<PtrType, SmallSize> &RHS) {
    if (&RHS != this)
      this->CopyFrom(RHS);
    return *this;
  }

  SmallPtrSet<PtrType, SmallSize> &
  operator=(SmallPtrSet<PtrType, SmallSize> &&RHS) {
    if (&RHS != this)
      this->MoveFrom(SmallSizePowTwo, std::move(RHS));
    return *this;
  }

  SmallPtrSet<PtrType, SmallSize> &
  operator=(std::initializer_list<PtrType> IL) {
    this->clear();
    this->insert(IL.begin(), IL.end());
    return *this;
  }

  /// swap - Swaps the elements of two sets.
  __host__ __device__
void swap(SmallPtrSet<PtrType, SmallSize> &RHS) {
    SmallPtrSetImplBase::swap(RHS);
  }
};

} // end namespace llvm

namespace std {

  /// Implement std::swap in terms of SmallPtrSet swap.
  template<class T, unsigned N>
  inline __host__ __device__
void swap(llvm::SmallPtrSet<T, N> &LHS, llvm::SmallPtrSet<T, N> &RHS) {
    LHS.swap(RHS);
  }

} // end namespace std

#endif // LLVM_ADT_SMALLPTRSET_H
