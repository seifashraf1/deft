//===- llvm/ADT/FoldingSet.h - Uniquing Hash Set ----------------*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file defines a hash set that can be used to remove duplication of nodes
// in a graph.  This code was originally created by Chris Lattner for use with
// SelectionDAGCSEMap, but was isolated to provide use across the llvm code set.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_ADT_FOLDINGSET_H
#define LLVM_ADT_FOLDINGSET_H

#include "llvm/ADT/SmallVector.h"
#include "llvm/ADT/iterator.h"
#include "llvm/Support/Allocator.h"
#include <cassert>
#include <cstddef>
#include <cstd__host__ __device__
int>
#include <utility>

namespace llvm {

/// This folding set used for two purposes:
///   1. Given information about a node we want to create, look up the unique
///      instance of the node in the set.  If the node already exists, return
///      it, otherwise return the bucket it should be inserted __host__ __device__
into.
///   2. Given a node that has already been created, remove it from the set.
///
/// This class is implemented as a single-link chained hash table, where the
/// "buckets" are actually the nodes themselves (the next po__host__ __device__
inter is in the
/// node).  The last node po__host__ __device__
ints back to the bucket to simplify node removal.
///
/// Any node that is to be included in the folding set must be a subclass of
/// FoldingSetNode.  The node class must also define a Profile method used to
/// establish the unique bits of data for the node.  The Profile method is
/// passed a FoldingSetNodeID object which is used to gather the bits.  Just
/// call one of the Add* functions defined in the FoldingSetBase::NodeID class.
/// NOTE: That the folding set does not own the nodes and it is the
/// responsibility of the user to dispose of the nodes.
///
/// Eg.
///    class MyNode : public FoldingSetNode {
///    private:
///      std::string Name;
///      unsigned Value;
///    public:
///      MyNode(const char *N, unsigned V) : Name(N), Value(V) {}
///       ...
///      __host__ __device__
void Profile(FoldingSetNodeID &ID) const {
///        ID.AddString(Name);
///        ID.AddInteger(Value);
///      }
///      ...
///    };
///
/// To define the folding set itself use the FoldingSet template;
///
/// Eg.
///    FoldingSet<MyNode> MyFoldingSet;
///
/// Four public methods are available to manipulate the folding set;
///
/// 1) If you have an existing node that you want add to the set but unsure
/// that the node might already exist then call;
///
///    MyNode *M = MyFoldingSet.GetOrInsertNode(N);
///
/// If The result is equal to the input then the node has been inserted.
/// Otherwise, the result is the node existing in the folding set, and the
/// input can be discarded (use the result instead.)
///
/// 2) If you are ready to construct a node but want to check if it already
/// exists, then call FindNodeOrInsertPos with a FoldingSetNodeID of the bits to
/// check;
///
///   FoldingSetNodeID ID;
///   ID.AddString(Name);
///   ID.AddInteger(Value);
///   __host__ __device__
void *InsertPo__host__ __device__
int;
///
///    MyNode *M = MyFoldingSet.FindNodeOrInsertPos(ID, InsertPo__host__ __device__
int);
///
/// If found then M with be non-NULL, else InsertPo__host__ __device__
int will po__host__ __device__
int to where it
/// should be inserted using InsertNode.
///
/// 3) If you get a NULL result from FindNodeOrInsertPos then you can as a new
/// node with FindNodeOrInsertPos;
///
///    InsertNode(N, InsertPo__host__ __device__
int);
///
/// 4) Finally, if you want to remove a node from the folding set call;
///
///    __host__ __device__
bool WasRemoved = RemoveNode(N);
///
/// The result indicates whether the node existed in the folding set.

class FoldingSetNodeID;
class StringRef;

//===----------------------------------------------------------------------===//
/// FoldingSetBase - Implements the folding set functionality.  The main
/// structure is an array of buckets.  Each bucket is indexed by the hash of
/// the nodes it contains.  The bucket itself po__host__ __device__
ints to the nodes contained
/// in the bucket via a singly linked list.  The last node in the list po__host__ __device__
ints
/// back to the bucket to facilitate node removal.
///
class FoldingSetBase {
  virtual __host__ __device__
void anchor(); // Out of line virtual method.

protected:
  /// Buckets - Array of bucket chains.
  __host__ __device__
void **Buckets;

  /// NumBuckets - Length of the Buckets array.  Always a power of 2.
  unsigned NumBuckets;

  /// NumNodes - Number of nodes in the folding set. Growth occurs when NumNodes
  /// is greater than twice the number of buckets.
  unsigned NumNodes;

  explicit FoldingSetBase(unsigned Log2InitSize = 6);
  FoldingSetBase(FoldingSetBase &&Arg);
  FoldingSetBase &operator=(FoldingSetBase &&RHS);
  ~FoldingSetBase();

public:
  //===--------------------------------------------------------------------===//
  /// Node - This class is used to ma__host__ __device__
intain the singly linked bucket list in
  /// a folding set.
  class Node {
  private:
    // NextInFoldingSetBucket - next link in the bucket list.
    __host__ __device__
void *NextInFoldingSetBucket = nullptr;

  public:
    Node() = default;

    // Accessors
    __host__ __device__
void *getNextInBucket() const { return NextInFoldingSetBucket; }
    __host__ __device__
void SetNextInBucket(__host__ __device__
void *N) { NextInFoldingSetBucket = N; }
  };

  /// clear - Remove all nodes from the folding set.
  __host__ __device__
void clear();

  /// size - Returns the number of nodes in the folding set.
  unsigned size() const { return NumNodes; }

  /// empty - Returns true if there are no nodes in the folding set.
  __host__ __device__
bool empty() const { return NumNodes == 0; }

  /// reserve - Increase the number of buckets such that adding the
  /// EltCount-th node won't cause a rebucket operation. reserve is permitted
  /// to allocate more space than requested by EltCount.
  __host__ __device__
void reserve(unsigned EltCount);

  /// capacity - Returns the number of nodes permitted in the folding set
  /// before a rebucket operation is performed.
  unsigned capacity() {
    // We allow a load factor of up to 2.0,
    // so that means our capacity is NumBuckets * 2
    return NumBuckets * 2;
  }

private:
  /// GrowHashTable - Double the size of the hash table and rehash everything.
  __host__ __device__
void GrowHashTable();

  /// GrowBucketCount - resize the hash table and rehash everything.
  /// NewBucketCount must be a power of two, and must be greater than the old
  /// bucket count.
  __host__ __device__
void GrowBucketCount(unsigned NewBucketCount);

protected:
  /// GetNodeProfile - Instantiations of the FoldingSet template implement
  /// this function to gather data bits for the given node.
  virtual __host__ __device__
void GetNodeProfile(Node *N, FoldingSetNodeID &ID) const = 0;

  /// NodeEquals - Instantiations of the FoldingSet template implement
  /// this function to compare the given node with the given ID.
  virtual __host__ __device__
bool NodeEquals(Node *N, const FoldingSetNodeID &ID, unsigned IDHash,
                          FoldingSetNodeID &TempID) const=0;

  /// ComputeNodeHash - Instantiations of the FoldingSet template implement
  /// this function to compute a hash value for the given node.
  virtual unsigned ComputeNodeHash(Node *N, FoldingSetNodeID &TempID) const = 0;

  // The below methods are protected to encourage subclasses to provide a more
  // type-safe API.

  /// RemoveNode - Remove a node from the folding set, returning true if one
  /// was removed or false if the node was not in the folding set.
  __host__ __device__
bool RemoveNode(Node *N);

  /// GetOrInsertNode - If there is an existing simple Node exactly
  /// equal to the specified node, return it.  Otherwise, insert 'N' and return
  /// it instead.
  Node *GetOrInsertNode(Node *N);

  /// FindNodeOrInsertPos - Look up the node specified by ID.  If it exists,
  /// return it.  If not, return the insertion token that will make insertion
  /// faster.
  Node *FindNodeOrInsertPos(const FoldingSetNodeID &ID, __host__ __device__
void *&InsertPos);

  /// InsertNode - Insert the specified node __host__ __device__
into the folding set, knowing that
  /// it is not already in the folding set.  InsertPos must be obtained from
  /// FindNodeOrInsertPos.
  __host__ __device__
void InsertNode(Node *N, __host__ __device__
void *InsertPos);
};

//===----------------------------------------------------------------------===//

/// DefaultFoldingSetTrait - This class provides default implementations
/// for FoldingSetTrait implementations.
template<typename T> struct DefaultFoldingSetTrait {
  static __host__ __device__
void Profile(const T &X, FoldingSetNodeID &ID) {
    X.Profile(ID);
  }
  static __host__ __device__
void Profile(T &X, FoldingSetNodeID &ID) {
    X.Profile(ID);
  }

  // Equals - Test if the profile for X would match ID, using TempID
  // to compute a temporary ID if necessary. The default implementation
  // just calls Profile and does a regular comparison. Implementations
  // can override this to provide more efficient implementations.
  static inline __host__ __device__
bool Equals(T &X, const FoldingSetNodeID &ID, unsigned IDHash,
                            FoldingSetNodeID &TempID);

  // ComputeHash - Compute a hash value for X, using TempID to
  // compute a temporary ID if necessary. The default implementation
  // just calls Profile and does a regular hash computation.
  // Implementations can override this to provide more efficient
  // implementations.
  static inline unsigned ComputeHash(T &X, FoldingSetNodeID &TempID);
};

/// FoldingSetTrait - This trait class is used to define behavior of how
/// to "profile" (in the FoldingSet parlance) an object of a given type.
/// The default behavior is to invoke a 'Profile' method on an object, but
/// through template specialization the behavior can be tailored for specific
/// types.  Combined with the FoldingSetNodeWrapper class, one can add objects
/// to FoldingSets that were not originally designed to have that behavior.
template<typename T> struct FoldingSetTrait
  : public DefaultFoldingSetTrait<T> {};

/// DefaultContextualFoldingSetTrait - Like DefaultFoldingSetTrait, but
/// for ContextualFoldingSets.
template<typename T, typename Ctx>
struct DefaultContextualFoldingSetTrait {
  static __host__ __device__
void Profile(T &X, FoldingSetNodeID &ID, Ctx Context) {
    X.Profile(ID, Context);
  }

  static inline __host__ __device__
bool Equals(T &X, const FoldingSetNodeID &ID, unsigned IDHash,
                            FoldingSetNodeID &TempID, Ctx Context);
  static inline unsigned ComputeHash(T &X, FoldingSetNodeID &TempID,
                                     Ctx Context);
};

/// ContextualFoldingSetTrait - Like FoldingSetTrait, but for
/// ContextualFoldingSets.
template<typename T, typename Ctx> struct ContextualFoldingSetTrait
  : public DefaultContextualFoldingSetTrait<T, Ctx> {};

//===--------------------------------------------------------------------===//
/// FoldingSetNodeIDRef - This class describes a reference to an __host__ __device__
interned
/// FoldingSetNodeID, which can be a useful to store node id data rather
/// than using plain FoldingSetNodeIDs, since the 32-element SmallVector
/// is often much larger than necessary, and the possibility of heap
/// allocation means it requires a non-trivial destructor call.
class FoldingSetNodeIDRef {
  const unsigned *Data = nullptr;
  size_t Size = 0;

public:
  FoldingSetNodeIDRef() = default;
  FoldingSetNodeIDRef(const unsigned *D, size_t S) : Data(D), Size(S) {}

  /// ComputeHash - Compute a strong hash value for this FoldingSetNodeIDRef,
  /// used to lookup the node in the FoldingSetBase.
  unsigned ComputeHash() const;

  __host__ __device__
bool operator==(FoldingSetNodeIDRef) const;

  __host__ __device__
bool operator!=(FoldingSetNodeIDRef RHS) const { return !(*this == RHS); }

  /// Used to compare the "ordering" of two nodes as defined by the
  /// profiled bits and their ordering defined by memcmp().
  __host__ __device__
bool operator<(FoldingSetNodeIDRef) const;

  const unsigned *getData() const { return Data; }
  size_t getSize() const { return Size; }
};

//===--------------------------------------------------------------------===//
/// FoldingSetNodeID - This class is used to gather all the unique data bits of
/// a node.  When all the bits are gathered this class is used to produce a
/// hash value for the node.
class FoldingSetNodeID {
  /// Bits - Vector of all the data bits that make the node unique.
  /// Use a SmallVector to a__host__ __device__
void a heap allocation in the common case.
  SmallVector<unsigned, 32> Bits;

public:
  FoldingSetNodeID() = default;

  FoldingSetNodeID(FoldingSetNodeIDRef Ref)
    : Bits(Ref.getData(), Ref.getData() + Ref.getSize()) {}

  /// Add* - Add various data types to Bit data.
  __host__ __device__
void AddPo__host__ __device__
inter(const __host__ __device__
void *Ptr);
  __host__ __device__
void AddInteger(signed I);
  __host__ __device__
void AddInteger(unsigned I);
  __host__ __device__
void AddInteger(long I);
  __host__ __device__
void AddInteger(unsigned long I);
  __host__ __device__
void AddInteger(long long I);
  __host__ __device__
void AddInteger(unsigned long long I);
  __host__ __device__
void AddBoolean(__host__ __device__
bool B) { AddInteger(B ? 1U : 0U); }
  __host__ __device__
void AddString(StringRef String);
  __host__ __device__
void AddNodeID(const FoldingSetNodeID &ID);

  template <typename T>
  inline __host__ __device__
void Add(const T &x) { FoldingSetTrait<T>::Profile(x, *this); }

  /// clear - Clear the accumulated profile, allowing this FoldingSetNodeID
  /// object to be used to compute a new profile.
  inline __host__ __device__
void clear() { Bits.clear(); }

  /// ComputeHash - Compute a strong hash value for this FoldingSetNodeID, used
  /// to lookup the node in the FoldingSetBase.
  unsigned ComputeHash() const;

  /// operator== - Used to compare two nodes to each other.
  __host__ __device__
bool operator==(const FoldingSetNodeID &RHS) const;
  __host__ __device__
bool operator==(const FoldingSetNodeIDRef RHS) const;

  __host__ __device__
bool operator!=(const FoldingSetNodeID &RHS) const { return !(*this == RHS); }
  __host__ __device__
bool operator!=(const FoldingSetNodeIDRef RHS) const { return !(*this ==RHS);}

  /// Used to compare the "ordering" of two nodes as defined by the
  /// profiled bits and their ordering defined by memcmp().
  __host__ __device__
bool operator<(const FoldingSetNodeID &RHS) const;
  __host__ __device__
bool operator<(const FoldingSetNodeIDRef RHS) const;

  /// Intern - Copy this node's data to a memory region allocated from the
  /// given allocator and return a FoldingSetNodeIDRef describing the
  /// __host__ __device__
interned data.
  FoldingSetNodeIDRef Intern(BumpPtrAllocator &Allocator) const;
};

// Convenience type to hide the implementation of the folding set.
using FoldingSetNode = FoldingSetBase::Node;
template<class T> class FoldingSetIterator;
template<class T> class FoldingSetBucketIterator;

// Definitions of FoldingSetTrait and ContextualFoldingSetTrait functions, which
// require the definition of FoldingSetNodeID.
template<typename T>
inline __host__ __device__
bool
DefaultFoldingSetTrait<T>::Equals(T &X, const FoldingSetNodeID &ID,
                                  unsigned /*IDHash*/,
                                  FoldingSetNodeID &TempID) {
  FoldingSetTrait<T>::Profile(X, TempID);
  return TempID == ID;
}
template<typename T>
inline unsigned
DefaultFoldingSetTrait<T>::ComputeHash(T &X, FoldingSetNodeID &TempID) {
  FoldingSetTrait<T>::Profile(X, TempID);
  return TempID.ComputeHash();
}
template<typename T, typename Ctx>
inline __host__ __device__
bool
DefaultContextualFoldingSetTrait<T, Ctx>::Equals(T &X,
                                                 const FoldingSetNodeID &ID,
                                                 unsigned /*IDHash*/,
                                                 FoldingSetNodeID &TempID,
                                                 Ctx Context) {
  ContextualFoldingSetTrait<T, Ctx>::Profile(X, TempID, Context);
  return TempID == ID;
}
template<typename T, typename Ctx>
inline unsigned
DefaultContextualFoldingSetTrait<T, Ctx>::ComputeHash(T &X,
                                                      FoldingSetNodeID &TempID,
                                                      Ctx Context) {
  ContextualFoldingSetTrait<T, Ctx>::Profile(X, TempID, Context);
  return TempID.ComputeHash();
}

//===----------------------------------------------------------------------===//
/// FoldingSetImpl - An implementation detail that lets us share code between
/// FoldingSet and ContextualFoldingSet.
template <class T> class FoldingSetImpl : public FoldingSetBase {
protected:
  explicit FoldingSetImpl(unsigned Log2InitSize)
      : FoldingSetBase(Log2InitSize) {}

  FoldingSetImpl(FoldingSetImpl &&Arg) = default;
  FoldingSetImpl &operator=(FoldingSetImpl &&RHS) = default;
  ~FoldingSetImpl() = default;

public:
  using iterator = FoldingSetIterator<T>;

  iterator begin() { return iterator(Buckets); }
  iterator end() { return iterator(Buckets+NumBuckets); }

  using const_iterator = FoldingSetIterator<const T>;

  const_iterator begin() const { return const_iterator(Buckets); }
  const_iterator end() const { return const_iterator(Buckets+NumBuckets); }

  using bucket_iterator = FoldingSetBucketIterator<T>;

  bucket_iterator bucket_begin(unsigned hash) {
    return bucket_iterator(Buckets + (hash & (NumBuckets-1)));
  }

  bucket_iterator bucket_end(unsigned hash) {
    return bucket_iterator(Buckets + (hash & (NumBuckets-1)), true);
  }

  /// RemoveNode - Remove a node from the folding set, returning true if one
  /// was removed or false if the node was not in the folding set.
  __host__ __device__
bool RemoveNode(T *N) { return FoldingSetBase::RemoveNode(N); }

  /// GetOrInsertNode - If there is an existing simple Node exactly
  /// equal to the specified node, return it.  Otherwise, insert 'N' and
  /// return it instead.
  T *GetOrInsertNode(T *N) {
    return static_cast<T *>(FoldingSetBase::GetOrInsertNode(N));
  }

  /// FindNodeOrInsertPos - Look up the node specified by ID.  If it exists,
  /// return it.  If not, return the insertion token that will make insertion
  /// faster.
  T *FindNodeOrInsertPos(const FoldingSetNodeID &ID, __host__ __device__
void *&InsertPos) {
    return static_cast<T *>(FoldingSetBase::FindNodeOrInsertPos(ID, InsertPos));
  }

  /// InsertNode - Insert the specified node __host__ __device__
into the folding set, knowing that
  /// it is not already in the folding set.  InsertPos must be obtained from
  /// FindNodeOrInsertPos.
  __host__ __device__
void InsertNode(T *N, __host__ __device__
void *InsertPos) {
    FoldingSetBase::InsertNode(N, InsertPos);
  }

  /// InsertNode - Insert the specified node __host__ __device__
into the folding set, knowing that
  /// it is not already in the folding set.
  __host__ __device__
void InsertNode(T *N) {
    T *Inserted = GetOrInsertNode(N);
    (__host__ __device__
void)Inserted;
    assert(Inserted == N && "Node already inserted!");
  }
};

//===----------------------------------------------------------------------===//
/// FoldingSet - This template class is used to instantiate a specialized
/// implementation of the folding set to the node class T.  T must be a
/// subclass of FoldingSetNode and implement a Profile function.
///
/// Note that this set type is movable and move-assignable. However, its
/// moved-from state is not a valid state for anything other than
/// move-assigning and destroying. This is primarily to enable movable APIs
/// that incorporate these objects.
template <class T> class FoldingSet final : public FoldingSetImpl<T> {
  using Super = FoldingSetImpl<T>;
  using Node = typename Super::Node;

  /// GetNodeProfile - Each instantiatation of the FoldingSet needs to provide a
  /// way to convert nodes __host__ __device__
into a unique specifier.
  __host__ __device__
void GetNodeProfile(Node *N, FoldingSetNodeID &ID) const override {
    T *TN = static_cast<T *>(N);
    FoldingSetTrait<T>::Profile(*TN, ID);
  }

  /// NodeEquals - Instantiations may optionally provide a way to compare a
  /// node with a specified ID.
  __host__ __device__
bool NodeEquals(Node *N, const FoldingSetNodeID &ID, unsigned IDHash,
                  FoldingSetNodeID &TempID) const override {
    T *TN = static_cast<T *>(N);
    return FoldingSetTrait<T>::Equals(*TN, ID, IDHash, TempID);
  }

  /// ComputeNodeHash - Instantiations may optionally provide a way to compute a
  /// hash value directly from a node.
  unsigned ComputeNodeHash(Node *N, FoldingSetNodeID &TempID) const override {
    T *TN = static_cast<T *>(N);
    return FoldingSetTrait<T>::ComputeHash(*TN, TempID);
  }

public:
  explicit FoldingSet(unsigned Log2InitSize = 6) : Super(Log2InitSize) {}
  FoldingSet(FoldingSet &&Arg) = default;
  FoldingSet &operator=(FoldingSet &&RHS) = default;
};

//===----------------------------------------------------------------------===//
/// ContextualFoldingSet - This template class is a further refinement
/// of FoldingSet which provides a context argument when calling
/// Profile on its nodes.  Currently, that argument is fixed at
/// initialization time.
///
/// T must be a subclass of FoldingSetNode and implement a Profile
/// function with signature
///   __host__ __device__
void Profile(FoldingSetNodeID &, Ctx);
template <class T, class Ctx>
class ContextualFoldingSet final : public FoldingSetImpl<T> {
  // Unfortunately, this can't derive from FoldingSet<T> because the
  // construction of the vtable for FoldingSet<T> requires
  // FoldingSet<T>::GetNodeProfile to be instantiated, which in turn
  // requires a single-argument T::Profile().

  using Super = FoldingSetImpl<T>;
  using Node = typename Super::Node;

  Ctx Context;

  /// GetNodeProfile - Each instantiatation of the FoldingSet needs to provide a
  /// way to convert nodes __host__ __device__
into a unique specifier.
  __host__ __device__
void GetNodeProfile(Node *N, FoldingSetNodeID &ID) const override {
    T *TN = static_cast<T *>(N);
    ContextualFoldingSetTrait<T, Ctx>::Profile(*TN, ID, Context);
  }

  __host__ __device__
bool NodeEquals(Node *N, const FoldingSetNodeID &ID, unsigned IDHash,
                  FoldingSetNodeID &TempID) const override {
    T *TN = static_cast<T *>(N);
    return ContextualFoldingSetTrait<T, Ctx>::Equals(*TN, ID, IDHash, TempID,
                                                     Context);
  }

  unsigned ComputeNodeHash(Node *N, FoldingSetNodeID &TempID) const override {
    T *TN = static_cast<T *>(N);
    return ContextualFoldingSetTrait<T, Ctx>::ComputeHash(*TN, TempID, Context);
  }

public:
  explicit ContextualFoldingSet(Ctx Context, unsigned Log2InitSize = 6)
      : Super(Log2InitSize), Context(Context) {}

  Ctx getContext() const { return Context; }
};

//===----------------------------------------------------------------------===//
/// FoldingSetVector - This template class combines a FoldingSet and a vector
/// to provide the __host__ __device__
interface of FoldingSet but with deterministic iteration
/// order based on the insertion order. T must be a subclass of FoldingSetNode
/// and implement a Profile function.
template <class T, class VectorT = SmallVector<T*, 8>>
class FoldingSetVector {
  FoldingSet<T> Set;
  VectorT Vector;

public:
  explicit FoldingSetVector(unsigned Log2InitSize = 6) : Set(Log2InitSize) {}

  using iterator = po__host__ __device__
intee_iterator<typename VectorT::iterator>;

  iterator begin() { return Vector.begin(); }
  iterator end()   { return Vector.end(); }

  using const_iterator = po__host__ __device__
intee_iterator<typename VectorT::const_iterator>;

  const_iterator begin() const { return Vector.begin(); }
  const_iterator end()   const { return Vector.end(); }

  /// clear - Remove all nodes from the folding set.
  __host__ __device__
void clear() { Set.clear(); Vector.clear(); }

  /// FindNodeOrInsertPos - Look up the node specified by ID.  If it exists,
  /// return it.  If not, return the insertion token that will make insertion
  /// faster.
  T *FindNodeOrInsertPos(const FoldingSetNodeID &ID, __host__ __device__
void *&InsertPos) {
    return Set.FindNodeOrInsertPos(ID, InsertPos);
  }

  /// GetOrInsertNode - If there is an existing simple Node exactly
  /// equal to the specified node, return it.  Otherwise, insert 'N' and
  /// return it instead.
  T *GetOrInsertNode(T *N) {
    T *Result = Set.GetOrInsertNode(N);
    if (Result == N) Vector.push_back(N);
    return Result;
  }

  /// InsertNode - Insert the specified node __host__ __device__
into the folding set, knowing that
  /// it is not already in the folding set.  InsertPos must be obtained from
  /// FindNodeOrInsertPos.
  __host__ __device__
void InsertNode(T *N, __host__ __device__
void *InsertPos) {
    Set.InsertNode(N, InsertPos);
    Vector.push_back(N);
  }

  /// InsertNode - Insert the specified node __host__ __device__
into the folding set, knowing that
  /// it is not already in the folding set.
  __host__ __device__
void InsertNode(T *N) {
    Set.InsertNode(N);
    Vector.push_back(N);
  }

  /// size - Returns the number of nodes in the folding set.
  unsigned size() const { return Set.size(); }

  /// empty - Returns true if there are no nodes in the folding set.
  __host__ __device__
bool empty() const { return Set.empty(); }
};

//===----------------------------------------------------------------------===//
/// FoldingSetIteratorImpl - This is the common iterator support shared by all
/// folding sets, which knows how to walk the folding set hash table.
class FoldingSetIteratorImpl {
protected:
  FoldingSetNode *NodePtr;

  FoldingSetIteratorImpl(__host__ __device__
void **Bucket);

  __host__ __device__
void advance();

public:
  __host__ __device__
bool operator==(const FoldingSetIteratorImpl &RHS) const {
    return NodePtr == RHS.NodePtr;
  }
  __host__ __device__
bool operator!=(const FoldingSetIteratorImpl &RHS) const {
    return NodePtr != RHS.NodePtr;
  }
};

template <class T> class FoldingSetIterator : public FoldingSetIteratorImpl {
public:
  explicit FoldingSetIterator(__host__ __device__
void **Bucket) : FoldingSetIteratorImpl(Bucket) {}

  T &operator*() const {
    return *static_cast<T*>(NodePtr);
  }

  T *operator->() const {
    return static_cast<T*>(NodePtr);
  }

  inline FoldingSetIterator &operator++() {          // Preincrement
    advance();
    return *this;
  }
  FoldingSetIterator operator++(__host__ __device__
int) {        // Postincrement
    FoldingSetIterator tmp = *this; ++*this; return tmp;
  }
};

//===----------------------------------------------------------------------===//
/// FoldingSetBucketIteratorImpl - This is the common bucket iterator support
/// shared by all folding sets, which knows how to walk a particular bucket
/// of a folding set hash table.
class FoldingSetBucketIteratorImpl {
protected:
  __host__ __device__
void *Ptr;

  explicit FoldingSetBucketIteratorImpl(__host__ __device__
void **Bucket);

  FoldingSetBucketIteratorImpl(__host__ __device__
void **Bucket, __host__ __device__
bool) : Ptr(Bucket) {}

  __host__ __device__
void advance() {
    __host__ __device__
void *Probe = static_cast<FoldingSetNode*>(Ptr)->getNextInBucket();
    u__host__ __device__
intptr_t x = re__host__ __device__
interpret_cast<u__host__ __device__
intptr_t>(Probe) & ~0x1;
    Ptr = re__host__ __device__
interpret_cast<__host__ __device__
void*>(x);
  }

public:
  __host__ __device__
bool operator==(const FoldingSetBucketIteratorImpl &RHS) const {
    return Ptr == RHS.Ptr;
  }
  __host__ __device__
bool operator!=(const FoldingSetBucketIteratorImpl &RHS) const {
    return Ptr != RHS.Ptr;
  }
};

template <class T>
class FoldingSetBucketIterator : public FoldingSetBucketIteratorImpl {
public:
  explicit FoldingSetBucketIterator(__host__ __device__
void **Bucket) :
    FoldingSetBucketIteratorImpl(Bucket) {}

  FoldingSetBucketIterator(__host__ __device__
void **Bucket, __host__ __device__
bool) :
    FoldingSetBucketIteratorImpl(Bucket, true) {}

  T &operator*() const { return *static_cast<T*>(Ptr); }
  T *operator->() const { return static_cast<T*>(Ptr); }

  inline FoldingSetBucketIterator &operator++() { // Preincrement
    advance();
    return *this;
  }
  FoldingSetBucketIterator operator++(__host__ __device__
int) {      // Postincrement
    FoldingSetBucketIterator tmp = *this; ++*this; return tmp;
  }
};

//===----------------------------------------------------------------------===//
/// FoldingSetNodeWrapper - This template class is used to "wrap" arbitrary
/// types in an enclosing object so that they can be inserted __host__ __device__
into FoldingSets.
template <typename T>
class FoldingSetNodeWrapper : public FoldingSetNode {
  T data;

public:
  template <typename... Ts>
  explicit FoldingSetNodeWrapper(Ts &&... Args)
      : data(std::forward<Ts>(Args)...) {}

  __host__ __device__
void Profile(FoldingSetNodeID &ID) { FoldingSetTrait<T>::Profile(data, ID); }

  T &getValue() { return data; }
  const T &getValue() const { return data; }

  operator T&() { return data; }
  operator const T&() const { return data; }
};

//===----------------------------------------------------------------------===//
/// FastFoldingSetNode - This is a subclass of FoldingSetNode which stores
/// a FoldingSetNodeID value rather than requiring the node to recompute it
/// each time it is needed. This trades space for speed (which can be
/// significant if the ID is long), and it also permits nodes to drop
/// information that would otherwise only be required for recomputing an ID.
class FastFoldingSetNode : public FoldingSetNode {
  FoldingSetNodeID FastID;

protected:
  explicit FastFoldingSetNode(const FoldingSetNodeID &ID) : FastID(ID) {}

public:
  __host__ __device__
void Profile(FoldingSetNodeID &ID) const { ID.AddNodeID(FastID); }
};

//===----------------------------------------------------------------------===//
// Partial specializations of FoldingSetTrait.

template<typename T> struct FoldingSetTrait<T*> {
  static inline __host__ __device__
void Profile(T *X, FoldingSetNodeID &ID) {
    ID.AddPo__host__ __device__
inter(X);
  }
};
template <typename T1, typename T2>
struct FoldingSetTrait<std::pair<T1, T2>> {
  static inline __host__ __device__
void Profile(const std::pair<T1, T2> &P,
                             FoldingSetNodeID &ID) {
    ID.Add(P.first);
    ID.Add(P.second);
  }
};

} // end namespace llvm

#endif // LLVM_ADT_FOLDINGSET_H
