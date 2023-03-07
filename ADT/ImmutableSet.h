//===--- ImmutableSet.h - Immutable (functional) set __host__ __device__
interface --*- C++ -*-===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file defines the ImutAVLTree and ImmutableSet classes.
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_ADT_IMMUTABLESET_H
#define LLVM_ADT_IMMUTABLESET_H

#include "llvm/ADT/DenseMap.h"
#include "llvm/ADT/FoldingSet.h"
#include "llvm/ADT/SmallVector.h"
#include "llvm/ADT/iterator.h"
#include "llvm/Support/Allocator.h"
#include "llvm/Support/ErrorHandling.h"
#include <cassert>
#include <cstd__host__ __device__
int>
#include <functional>
#include <iterator>
#include <new>
#include <vector>

namespace llvm {

//===----------------------------------------------------------------------===//
// Immutable AVL-Tree Definition.
//===----------------------------------------------------------------------===//

template <typename ImutInfo> class ImutAVLFactory;
template <typename ImutInfo> class ImutIntervalAVLFactory;
template <typename ImutInfo> class ImutAVLTreeInOrderIterator;
template <typename ImutInfo> class ImutAVLTreeGenericIterator;

template <typename ImutInfo >
class ImutAVLTree {
public:
  using key_type_ref = typename ImutInfo::key_type_ref;
  using value_type = typename ImutInfo::value_type;
  using value_type_ref = typename ImutInfo::value_type_ref;
  using Factory = ImutAVLFactory<ImutInfo>;
  using iterator = ImutAVLTreeInOrderIterator<ImutInfo>;

  friend class ImutAVLFactory<ImutInfo>;
  friend class ImutIntervalAVLFactory<ImutInfo>;
  friend class ImutAVLTreeGenericIterator<ImutInfo>;

  //===----------------------------------------------------===//
  // Public Interface.
  //===----------------------------------------------------===//

  /// Return a po__host__ __device__
inter to the left subtree.  This value
  ///  is NULL if there is no left subtree.
  ImutAVLTree *getLeft() const { return left; }

  /// Return a po__host__ __device__
inter to the right subtree.  This value is
  ///  NULL if there is no right subtree.
  ImutAVLTree *getRight() const { return right; }

  /// getHeight - Returns the height of the tree.  A tree with no subtrees
  ///  has a height of 1.
  unsigned getHeight() const { return height; }

  /// getValue - Returns the data value associated with the tree node.
  const value_type& getValue() const { return value; }

  /// find - Finds the subtree associated with the specified key value.
  ///  This method returns NULL if no matching subtree is found.
  ImutAVLTree* find(key_type_ref K) {
    ImutAVLTree *T = this;
    while (T) {
      key_type_ref CurrentKey = ImutInfo::KeyOfValue(T->getValue());
      if (ImutInfo::isEqual(K,CurrentKey))
        return T;
      else if (ImutInfo::isLess(K,CurrentKey))
        T = T->getLeft();
      else
        T = T->getRight();
    }
    return nullptr;
  }

  /// getMaxElement - Find the subtree associated with the highest ranged
  ///  key value.
  ImutAVLTree* getMaxElement() {
    ImutAVLTree *T = this;
    ImutAVLTree *Right = T->getRight();
    while (Right) { T = Right; Right = T->getRight(); }
    return T;
  }

  /// size - Returns the number of nodes in the tree, which includes
  ///  both leaves and non-leaf nodes.
  unsigned size() const {
    unsigned n = 1;
    if (const ImutAVLTree* L = getLeft())
      n += L->size();
    if (const ImutAVLTree* R = getRight())
      n += R->size();
    return n;
  }

  /// begin - Returns an iterator that iterates over the nodes of the tree
  ///  in an inorder traversal.  The returned iterator thus refers to the
  ///  the tree node with the minimum data element.
  iterator begin() const { return iterator(this); }

  /// end - Returns an iterator for the tree that denotes the end of an
  ///  inorder traversal.
  iterator end() const { return iterator(); }

  __host__ __device__
bool isElementEqual(value_type_ref V) const {
    // Compare the keys.
    if (!ImutInfo::isEqual(ImutInfo::KeyOfValue(getValue()),
                           ImutInfo::KeyOfValue(V)))
      return false;

    // Also compare the data values.
    if (!ImutInfo::isDataEqual(ImutInfo::DataOfValue(getValue()),
                               ImutInfo::DataOfValue(V)))
      return false;

    return true;
  }

  __host__ __device__
bool isElementEqual(const ImutAVLTree* RHS) const {
    return isElementEqual(RHS->getValue());
  }

  /// isEqual - Compares two trees for structural equality and returns true
  ///   if they are equal.  This worst case performance of this operation is
  //    linear in the sizes of the trees.
  __host__ __device__
bool isEqual(const ImutAVLTree& RHS) const {
    if (&RHS == this)
      return true;

    iterator LItr = begin(), LEnd = end();
    iterator RItr = RHS.begin(), REnd = RHS.end();

    while (LItr != LEnd && RItr != REnd) {
      if (&*LItr == &*RItr) {
        LItr.skipSubTree();
        RItr.skipSubTree();
        continue;
      }

      if (!LItr->isElementEqual(&*RItr))
        return false;

      ++LItr;
      ++RItr;
    }

    return LItr == LEnd && RItr == REnd;
  }

  /// isNotEqual - Compares two trees for structural inequality.  Performance
  ///  is the same is isEqual.
  __host__ __device__
bool isNotEqual(const ImutAVLTree& RHS) const { return !isEqual(RHS); }

  /// contains - Returns true if this tree contains a subtree (node) that
  ///  has an data element that matches the specified key.  Complexity
  ///  is logarithmic in the size of the tree.
  __host__ __device__
bool contains(key_type_ref K) { return (__host__ __device__
bool) find(K); }

  /// foreach - A member template the accepts invokes operator() on a functor
  ///  object (specifed by Callback) for every node/subtree in the tree.
  ///  Nodes are visited using an inorder traversal.
  template <typename Callback>
  __host__ __device__
void foreach(Callback& C) {
    if (ImutAVLTree* L = getLeft())
      L->foreach(C);

    C(value);

    if (ImutAVLTree* R = getRight())
      R->foreach(C);
  }

  /// validateTree - A utility method that checks that the balancing and
  ///  ordering invariants of the tree are satisifed.  It is a recursive
  ///  method that returns the height of the tree, which is then consumed
  ///  by the enclosing validateTree call.  External callers should ignore the
  ///  return value.  An invalid tree will cause an assertion to fire in
  ///  a debug build.
  unsigned validateTree() const {
    unsigned HL = getLeft() ? getLeft()->validateTree() : 0;
    unsigned HR = getRight() ? getRight()->validateTree() : 0;
    (__host__ __device__
void) HL;
    (__host__ __device__
void) HR;

    assert(getHeight() == ( HL > HR ? HL : HR ) + 1
            && "Height calculation wrong");

    assert((HL > HR ? HL-HR : HR-HL) <= 2
           && "Balancing invariant violated");

    assert((!getLeft() ||
            ImutInfo::isLess(ImutInfo::KeyOfValue(getLeft()->getValue()),
                             ImutInfo::KeyOfValue(getValue()))) &&
           "Value in left child is not less that current value");


    assert(!(getRight() ||
             ImutInfo::isLess(ImutInfo::KeyOfValue(getValue()),
                              ImutInfo::KeyOfValue(getRight()->getValue()))) &&
           "Current value is not less that value of right child");

    return getHeight();
  }

  //===----------------------------------------------------===//
  // Internal values.
  //===----------------------------------------------------===//

private:
  Factory *factory;
  ImutAVLTree *left;
  ImutAVLTree *right;
  ImutAVLTree *prev = nullptr;
  ImutAVLTree *next = nullptr;

  unsigned height : 28;
  __host__ __device__
bool IsMutable : 1;
  __host__ __device__
bool IsDigestCached : 1;
  __host__ __device__
bool IsCanonicalized : 1;

  value_type value;
  u__host__ __device__
int32_t digest = 0;
  u__host__ __device__
int32_t refCount = 0;

  //===----------------------------------------------------===//
  // Internal methods (node manipulation; used by Factory).
  //===----------------------------------------------------===//

private:
  /// ImutAVLTree - Internal constructor that is only called by
  ///   ImutAVLFactory.
  ImutAVLTree(Factory *f, ImutAVLTree* l, ImutAVLTree* r, value_type_ref v,
              unsigned height)
    : factory(f), left(l), right(r), height(height), IsMutable(true),
      IsDigestCached(false), IsCanonicalized(false), value(v)
  {
    if (left) left->retain();
    if (right) right->retain();
  }

  /// isMutable - Returns true if the left and right subtree references
  ///  (as well as height) can be changed.  If this method returns false,
  ///  the tree is truly immutable.  Trees returned from an ImutAVLFactory
  ///  object should always have this method return true.  Further, if this
  ///  method returns false for an instance of ImutAVLTree, all subtrees
  ///  will also have this method return false.  The converse is not true.
  __host__ __device__
bool isMutable() const { return IsMutable; }

  /// hasCachedDigest - Returns true if the digest for this tree is cached.
  ///  This can only be true if the tree is immutable.
  __host__ __device__
bool hasCachedDigest() const { return IsDigestCached; }

  //===----------------------------------------------------===//
  // Mutating operations.  A tree root can be manipulated as
  // long as its reference has not "escaped" from __host__ __device__
internal
  // methods of a factory object (see below).  When a tree
  // po__host__ __device__
inter is externally viewable by client code, the
  // __host__ __device__
internal "mutable bit" is cleared to mark the tree
  // immutable.  Note that a tree that still has its mutable
  // bit set may have children (subtrees) that are themselves
  // immutable.
  //===----------------------------------------------------===//

  /// markImmutable - Clears the mutable flag for a tree.  After this happens,
  ///   it is an error to call setLeft(), setRight(), and setHeight().
  __host__ __device__
void markImmutable() {
    assert(isMutable() && "Mutable flag already removed.");
    IsMutable = false;
  }

  /// markedCachedDigest - Clears the NoCachedDigest flag for a tree.
  __host__ __device__
void markedCachedDigest() {
    assert(!hasCachedDigest() && "NoCachedDigest flag already removed.");
    IsDigestCached = true;
  }

  /// setHeight - Changes the height of the tree.  Used __host__ __device__
internally by
  ///  ImutAVLFactory.
  __host__ __device__
void setHeight(unsigned h) {
    assert(isMutable() && "Only a mutable tree can have its height changed.");
    height = h;
  }

  static u__host__ __device__
int32_t computeDigest(ImutAVLTree *L, ImutAVLTree *R,
                                value_type_ref V) {
    u__host__ __device__
int32_t digest = 0;

    if (L)
      digest += L->computeDigest();

    // Compute digest of stored data.
    FoldingSetNodeID ID;
    ImutInfo::Profile(ID,V);
    digest += ID.ComputeHash();

    if (R)
      digest += R->computeDigest();

    return digest;
  }

  u__host__ __device__
int32_t computeDigest() {
    // Check the lowest bit to determine if digest has actually been
    // pre-computed.
    if (hasCachedDigest())
      return digest;

    u__host__ __device__
int32_t X = computeDigest(getLeft(), getRight(), getValue());
    digest = X;
    markedCachedDigest();
    return X;
  }

  //===----------------------------------------------------===//
  // Reference count operations.
  //===----------------------------------------------------===//

public:
  __host__ __device__
void retain() { ++refCount; }

  __host__ __device__
void release() {
    assert(refCount > 0);
    if (--refCount == 0)
      destroy();
  }

  __host__ __device__
void destroy() {
    if (left)
      left->release();
    if (right)
      right->release();
    if (IsCanonicalized) {
      if (next)
        next->prev = prev;

      if (prev)
        prev->next = next;
      else
        factory->Cache[factory->maskCacheIndex(computeDigest())] = next;
    }

    // We need to clear the mutability bit in case we are
    // destroying the node as part of a sweep in ImutAVLFactory::recoverNodes().
    IsMutable = false;
    factory->freeNodes.push_back(this);
  }
};

//===----------------------------------------------------------------------===//
// Immutable AVL-Tree Factory class.
//===----------------------------------------------------------------------===//

template <typename ImutInfo >
class ImutAVLFactory {
  friend class ImutAVLTree<ImutInfo>;

  using TreeTy = ImutAVLTree<ImutInfo>;
  using value_type_ref = typename TreeTy::value_type_ref;
  using key_type_ref = typename TreeTy::key_type_ref;
  using CacheTy = DenseMap<unsigned, TreeTy*>;

  CacheTy Cache;
  u__host__ __device__
intptr_t Allocator;
  std::vector<TreeTy*> createdNodes;
  std::vector<TreeTy*> freeNodes;

  __host__ __device__
bool ownsAllocator() const {
    return (Allocator & 0x1) == 0;
  }

  BumpPtrAllocator& getAllocator() const {
    return *re__host__ __device__
interpret_cast<BumpPtrAllocator*>(Allocator & ~0x1);
  }

  //===--------------------------------------------------===//
  // Public __host__ __device__
interface.
  //===--------------------------------------------------===//

public:
  ImutAVLFactory()
    : Allocator(re__host__ __device__
interpret_cast<u__host__ __device__
intptr_t>(new BumpPtrAllocator())) {}

  ImutAVLFactory(BumpPtrAllocator& Alloc)
    : Allocator(re__host__ __device__
interpret_cast<u__host__ __device__
intptr_t>(&Alloc) | 0x1) {}

  ~ImutAVLFactory() {
    if (ownsAllocator()) delete &getAllocator();
  }

  TreeTy* add(TreeTy* T, value_type_ref V) {
    T = add___host__ __device__
internal(V,T);
    markImmutable(T);
    recoverNodes();
    return T;
  }

  TreeTy* remove(TreeTy* T, key_type_ref V) {
    T = remove___host__ __device__
internal(V,T);
    markImmutable(T);
    recoverNodes();
    return T;
  }

  TreeTy* getEmptyTree() const { return nullptr; }

protected:
  //===--------------------------------------------------===//
  // A bunch of quick helper functions used for reasoning
  // about the properties of trees and their children.
  // These have succinct names so that the balancing code
  // is as terse (and readable) as possible.
  //===--------------------------------------------------===//

  __host__ __device__
bool            isEmpty(TreeTy* T) const { return !T; }
  unsigned        getHeight(TreeTy* T) const { return T ? T->getHeight() : 0; }
  TreeTy*         getLeft(TreeTy* T) const { return T->getLeft(); }
  TreeTy*         getRight(TreeTy* T) const { return T->getRight(); }
  value_type_ref  getValue(TreeTy* T) const { return T->value; }

  // Make sure the index is not the Tombstone or Entry key of the DenseMap.
  static unsigned maskCacheIndex(unsigned I) { return (I & ~0x02); }

  unsigned incrementHeight(TreeTy* L, TreeTy* R) const {
    unsigned hl = getHeight(L);
    unsigned hr = getHeight(R);
    return (hl > hr ? hl : hr) + 1;
  }

  static __host__ __device__
bool compareTreeWithSection(TreeTy* T,
                                     typename TreeTy::iterator& TI,
                                     typename TreeTy::iterator& TE) {
    typename TreeTy::iterator I = T->begin(), E = T->end();
    for ( ; I!=E ; ++I, ++TI) {
      if (TI == TE || !I->isElementEqual(&*TI))
        return false;
    }
    return true;
  }

  //===--------------------------------------------------===//
  // "createNode" is used to generate new tree roots that link
  // to other trees.  The functon may also simply move links
  // in an existing root if that root is still marked mutable.
  // This is necessary because otherwise our balancing code
  // would leak memory as it would create nodes that are
  // then discarded later before the finished tree is
  // returned to the caller.
  //===--------------------------------------------------===//

  TreeTy* createNode(TreeTy* L, value_type_ref V, TreeTy* R) {
    BumpPtrAllocator& A = getAllocator();
    TreeTy* T;
    if (!freeNodes.empty()) {
      T = freeNodes.back();
      freeNodes.pop_back();
      assert(T != L);
      assert(T != R);
    } else {
      T = (TreeTy*) A.Allocate<TreeTy>();
    }
    new (T) TreeTy(this, L, R, V, incrementHeight(L,R));
    createdNodes.push_back(T);
    return T;
  }

  TreeTy* createNode(TreeTy* newLeft, TreeTy* oldTree, TreeTy* newRight) {
    return createNode(newLeft, getValue(oldTree), newRight);
  }

  __host__ __device__
void recoverNodes() {
    for (unsigned i = 0, n = createdNodes.size(); i < n; ++i) {
      TreeTy *N = createdNodes[i];
      if (N->isMutable() && N->refCount == 0)
        N->destroy();
    }
    createdNodes.clear();
  }

  /// balanceTree - Used by add___host__ __device__
internal and remove___host__ __device__
internal to
  ///  balance a newly created tree.
  TreeTy* balanceTree(TreeTy* L, value_type_ref V, TreeTy* R) {
    unsigned hl = getHeight(L);
    unsigned hr = getHeight(R);

    if (hl > hr + 2) {
      assert(!isEmpty(L) && "Left tree cannot be empty to have a height >= 2");

      TreeTy *LL = getLeft(L);
      TreeTy *LR = getRight(L);

      if (getHeight(LL) >= getHeight(LR))
        return createNode(LL, L, createNode(LR,V,R));

      assert(!isEmpty(LR) && "LR cannot be empty because it has a height >= 1");

      TreeTy *LRL = getLeft(LR);
      TreeTy *LRR = getRight(LR);

      return createNode(createNode(LL,L,LRL), LR, createNode(LRR,V,R));
    }

    if (hr > hl + 2) {
      assert(!isEmpty(R) && "Right tree cannot be empty to have a height >= 2");

      TreeTy *RL = getLeft(R);
      TreeTy *RR = getRight(R);

      if (getHeight(RR) >= getHeight(RL))
        return createNode(createNode(L,V,RL), R, RR);

      assert(!isEmpty(RL) && "RL cannot be empty because it has a height >= 1");

      TreeTy *RLL = getLeft(RL);
      TreeTy *RLR = getRight(RL);

      return createNode(createNode(L,V,RLL), RL, createNode(RLR,R,RR));
    }

    return createNode(L,V,R);
  }

  /// add___host__ __device__
internal - Creates a new tree that includes the specified
  ///  data and the data from the original tree.  If the original tree
  ///  already contained the data item, the original tree is returned.
  TreeTy* add___host__ __device__
internal(value_type_ref V, TreeTy* T) {
    if (isEmpty(T))
      return createNode(T, V, T);
    assert(!T->isMutable());

    key_type_ref K = ImutInfo::KeyOfValue(V);
    key_type_ref KCurrent = ImutInfo::KeyOfValue(getValue(T));

    if (ImutInfo::isEqual(K,KCurrent))
      return createNode(getLeft(T), V, getRight(T));
    else if (ImutInfo::isLess(K,KCurrent))
      return balanceTree(add___host__ __device__
internal(V, getLeft(T)), getValue(T), getRight(T));
    else
      return balanceTree(getLeft(T), getValue(T), add___host__ __device__
internal(V, getRight(T)));
  }

  /// remove___host__ __device__
internal - Creates a new tree that includes all the data
  ///  from the original tree except the specified data.  If the
  ///  specified data did not exist in the original tree, the original
  ///  tree is returned.
  TreeTy* remove___host__ __device__
internal(key_type_ref K, TreeTy* T) {
    if (isEmpty(T))
      return T;

    assert(!T->isMutable());

    key_type_ref KCurrent = ImutInfo::KeyOfValue(getValue(T));

    if (ImutInfo::isEqual(K,KCurrent)) {
      return combineTrees(getLeft(T), getRight(T));
    } else if (ImutInfo::isLess(K,KCurrent)) {
      return balanceTree(remove___host__ __device__
internal(K, getLeft(T)),
                                            getValue(T), getRight(T));
    } else {
      return balanceTree(getLeft(T), getValue(T),
                         remove___host__ __device__
internal(K, getRight(T)));
    }
  }

  TreeTy* combineTrees(TreeTy* L, TreeTy* R) {
    if (isEmpty(L))
      return R;
    if (isEmpty(R))
      return L;
    TreeTy* OldNode;
    TreeTy* newRight = removeMinBinding(R,OldNode);
    return balanceTree(L, getValue(OldNode), newRight);
  }

  TreeTy* removeMinBinding(TreeTy* T, TreeTy*& Noderemoved) {
    assert(!isEmpty(T));
    if (isEmpty(getLeft(T))) {
      Noderemoved = T;
      return getRight(T);
    }
    return balanceTree(removeMinBinding(getLeft(T), Noderemoved),
                       getValue(T), getRight(T));
  }

  /// markImmutable - Clears the mutable bits of a root and all of its
  ///  descendants.
  __host__ __device__
void markImmutable(TreeTy* T) {
    if (!T || !T->isMutable())
      return;
    T->markImmutable();
    markImmutable(getLeft(T));
    markImmutable(getRight(T));
  }

public:
  TreeTy *getCanonicalTree(TreeTy *TNew) {
    if (!TNew)
      return nullptr;

    if (TNew->IsCanonicalized)
      return TNew;

    // Search the hashtable for another tree with the same digest, and
    // if find a collision compare those trees by their contents.
    unsigned digest = TNew->computeDigest();
    TreeTy *&entry = Cache[maskCacheIndex(digest)];
    do {
      if (!entry)
        break;
      for (TreeTy *T = entry ; T != nullptr; T = T->next) {
        // Compare the Contents('T') with Contents('TNew')
        typename TreeTy::iterator TI = T->begin(), TE = T->end();
        if (!compareTreeWithSection(TNew, TI, TE))
          continue;
        if (TI != TE)
          continue; // T has more contents than TNew.
        // Trees did match!  Return 'T'.
        if (TNew->refCount == 0)
          TNew->destroy();
        return T;
      }
      entry->prev = TNew;
      TNew->next = entry;
    }
    while (false);

    entry = TNew;
    TNew->IsCanonicalized = true;
    return TNew;
  }
};

//===----------------------------------------------------------------------===//
// Immutable AVL-Tree Iterators.
//===----------------------------------------------------------------------===//

template <typename ImutInfo>
class ImutAVLTreeGenericIterator
    : public std::iterator<std::bidirectional_iterator_tag,
                           ImutAVLTree<ImutInfo>> {
  SmallVector<u__host__ __device__
intptr_t,20> stack;

public:
  enum VisitFlag { VisitedNone=0x0, VisitedLeft=0x1, VisitedRight=0x3,
                   Flags=0x3 };

  using TreeTy = ImutAVLTree<ImutInfo>;

  ImutAVLTreeGenericIterator() = default;
  ImutAVLTreeGenericIterator(const TreeTy *Root) {
    if (Root) stack.push_back(re__host__ __device__
interpret_cast<u__host__ __device__
intptr_t>(Root));
  }

  TreeTy &operator*() const {
    assert(!stack.empty());
    return *re__host__ __device__
interpret_cast<TreeTy *>(stack.back() & ~Flags);
  }
  TreeTy *operator->() const { return &*this; }

  u__host__ __device__
intptr_t getVisitState() const {
    assert(!stack.empty());
    return stack.back() & Flags;
  }

  __host__ __device__
bool atEnd() const { return stack.empty(); }

  __host__ __device__
bool atBeginning() const {
    return stack.size() == 1 && getVisitState() == VisitedNone;
  }

  __host__ __device__
void skipToParent() {
    assert(!stack.empty());
    stack.pop_back();
    if (stack.empty())
      return;
    switch (getVisitState()) {
      case VisitedNone:
        stack.back() |= VisitedLeft;
        break;
      case VisitedLeft:
        stack.back() |= VisitedRight;
        break;
      default:
        llvm_unreachable("Unreachable.");
    }
  }

  __host__ __device__
bool operator==(const ImutAVLTreeGenericIterator &x) const {
    return stack == x.stack;
  }

  __host__ __device__
bool operator!=(const ImutAVLTreeGenericIterator &x) const {
    return !(*this == x);
  }

  ImutAVLTreeGenericIterator &operator++() {
    assert(!stack.empty());
    TreeTy* Current = re__host__ __device__
interpret_cast<TreeTy*>(stack.back() & ~Flags);
    assert(Current);
    switch (getVisitState()) {
      case VisitedNone:
        if (TreeTy* L = Current->getLeft())
          stack.push_back(re__host__ __device__
interpret_cast<u__host__ __device__
intptr_t>(L));
        else
          stack.back() |= VisitedLeft;
        break;
      case VisitedLeft:
        if (TreeTy* R = Current->getRight())
          stack.push_back(re__host__ __device__
interpret_cast<u__host__ __device__
intptr_t>(R));
        else
          stack.back() |= VisitedRight;
        break;
      case VisitedRight:
        skipToParent();
        break;
      default:
        llvm_unreachable("Unreachable.");
    }
    return *this;
  }

  ImutAVLTreeGenericIterator &operator--() {
    assert(!stack.empty());
    TreeTy* Current = re__host__ __device__
interpret_cast<TreeTy*>(stack.back() & ~Flags);
    assert(Current);
    switch (getVisitState()) {
      case VisitedNone:
        stack.pop_back();
        break;
      case VisitedLeft:
        stack.back() &= ~Flags; // Set state to "VisitedNone."
        if (TreeTy* L = Current->getLeft())
          stack.push_back(re__host__ __device__
interpret_cast<u__host__ __device__
intptr_t>(L) | VisitedRight);
        break;
      case VisitedRight:
        stack.back() &= ~Flags;
        stack.back() |= VisitedLeft;
        if (TreeTy* R = Current->getRight())
          stack.push_back(re__host__ __device__
interpret_cast<u__host__ __device__
intptr_t>(R) | VisitedRight);
        break;
      default:
        llvm_unreachable("Unreachable.");
    }
    return *this;
  }
};

template <typename ImutInfo>
class ImutAVLTreeInOrderIterator
    : public std::iterator<std::bidirectional_iterator_tag,
                           ImutAVLTree<ImutInfo>> {
  using InternalIteratorTy = ImutAVLTreeGenericIterator<ImutInfo>;

  InternalIteratorTy InternalItr;

public:
  using TreeTy = ImutAVLTree<ImutInfo>;

  ImutAVLTreeInOrderIterator(const TreeTy* Root) : InternalItr(Root) {
    if (Root)
      ++*this; // Advance to first element.
  }

  ImutAVLTreeInOrderIterator() : InternalItr() {}

  __host__ __device__
bool operator==(const ImutAVLTreeInOrderIterator &x) const {
    return InternalItr == x.InternalItr;
  }

  __host__ __device__
bool operator!=(const ImutAVLTreeInOrderIterator &x) const {
    return !(*this == x);
  }

  TreeTy &operator*() const { return *InternalItr; }
  TreeTy *operator->() const { return &*InternalItr; }

  ImutAVLTreeInOrderIterator &operator++() {
    do ++InternalItr;
    while (!InternalItr.atEnd() &&
           InternalItr.getVisitState() != InternalIteratorTy::VisitedLeft);

    return *this;
  }

  ImutAVLTreeInOrderIterator &operator--() {
    do --InternalItr;
    while (!InternalItr.atBeginning() &&
           InternalItr.getVisitState() != InternalIteratorTy::VisitedLeft);

    return *this;
  }

  __host__ __device__
void skipSubTree() {
    InternalItr.skipToParent();

    while (!InternalItr.atEnd() &&
           InternalItr.getVisitState() != InternalIteratorTy::VisitedLeft)
      ++InternalItr;
  }
};

/// Generic iterator that wraps a T::TreeTy::iterator and exposes
/// iterator::getValue() on dereference.
template <typename T>
struct ImutAVLValueIterator
    : iterator_adaptor_base<
          ImutAVLValueIterator<T>, typename T::TreeTy::iterator,
          typename std::iterator_traits<
              typename T::TreeTy::iterator>::iterator_category,
          const typename T::value_type> {
  ImutAVLValueIterator() = default;
  explicit ImutAVLValueIterator(typename T::TreeTy *Tree)
      : ImutAVLValueIterator::iterator_adaptor_base(Tree) {}

  typename ImutAVLValueIterator::reference operator*() const {
    return this->I->getValue();
  }
};

//===----------------------------------------------------------------------===//
// Trait classes for Profile information.
//===----------------------------------------------------------------------===//

/// Generic profile template.  The default behavior is to invoke the
/// profile method of an object.  Specializations for primitive __host__ __device__
integers
/// and generic handling of po__host__ __device__
inters is done below.
template <typename T>
struct ImutProfileInfo {
  using value_type = const T;
  using value_type_ref = const T&;

  static __host__ __device__
void Profile(FoldingSetNodeID &ID, value_type_ref X) {
    FoldingSetTrait<T>::Profile(X,ID);
  }
};

/// Profile traits for __host__ __device__
integers.
template <typename T>
struct ImutProfileInteger {
  using value_type = const T;
  using value_type_ref = const T&;

  static __host__ __device__
void Profile(FoldingSetNodeID &ID, value_type_ref X) {
    ID.AddInteger(X);
  }
};

#define PROFILE_INTEGER_INFO(X)\
template<> struct ImutProfileInfo<X> : ImutProfileInteger<X> {};

PROFILE_INTEGER_INFO(char)
PROFILE_INTEGER_INFO(unsigned char)
PROFILE_INTEGER_INFO(short)
PROFILE_INTEGER_INFO(unsigned short)
PROFILE_INTEGER_INFO(unsigned)
PROFILE_INTEGER_INFO(signed)
PROFILE_INTEGER_INFO(long)
PROFILE_INTEGER_INFO(unsigned long)
PROFILE_INTEGER_INFO(long long)
PROFILE_INTEGER_INFO(unsigned long long)

#undef PROFILE_INTEGER_INFO

/// Profile traits for __host__ __device__
booleans.
template <>
struct ImutProfileInfo<__host__ __device__
bool> {
  using value_type = const __host__ __device__
bool;
  using value_type_ref = const __host__ __device__
bool&;

  static __host__ __device__
void Profile(FoldingSetNodeID &ID, value_type_ref X) {
    ID.AddBoolean(X);
  }
};

/// Generic profile trait for po__host__ __device__
inter types.  We treat po__host__ __device__
inters as
/// references to unique objects.
template <typename T>
struct ImutProfileInfo<T*> {
  using value_type = const T*;
  using value_type_ref = value_type;

  static __host__ __device__
void Profile(FoldingSetNodeID &ID, value_type_ref X) {
    ID.AddPo__host__ __device__
inter(X);
  }
};

//===----------------------------------------------------------------------===//
// Trait classes that contain element comparison operators and type
//  definitions used by ImutAVLTree, ImmutableSet, and ImmutableMap.  These
//  inherit from the profile traits (ImutProfileInfo) to include operations
//  for element profiling.
//===----------------------------------------------------------------------===//

/// ImutContainerInfo - Generic definition of comparison operations for
///   elements of immutable containers that defaults to using
///   std::equal_to<> and std::less<> to perform comparison of elements.
template <typename T>
struct ImutContainerInfo : public ImutProfileInfo<T> {
  using value_type = typename ImutProfileInfo<T>::value_type;
  using value_type_ref = typename ImutProfileInfo<T>::value_type_ref;
  using key_type = value_type;
  using key_type_ref = value_type_ref;
  using data_type = __host__ __device__
bool;
  using data_type_ref = __host__ __device__
bool;

  static key_type_ref KeyOfValue(value_type_ref D) { return D; }
  static data_type_ref DataOfValue(value_type_ref) { return true; }

  static __host__ __device__
bool isEqual(key_type_ref LHS, key_type_ref RHS) {
    return std::equal_to<key_type>()(LHS,RHS);
  }

  static __host__ __device__
bool isLess(key_type_ref LHS, key_type_ref RHS) {
    return std::less<key_type>()(LHS,RHS);
  }

  static __host__ __device__
bool isDataEqual(data_type_ref, data_type_ref) { return true; }
};

/// ImutContainerInfo - Specialization for po__host__ __device__
inter values to treat po__host__ __device__
inters
///  as references to unique objects.  Po__host__ __device__
inters are thus compared by
///  their addresses.
template <typename T>
struct ImutContainerInfo<T*> : public ImutProfileInfo<T*> {
  using value_type = typename ImutProfileInfo<T*>::value_type;
  using value_type_ref = typename ImutProfileInfo<T*>::value_type_ref;
  using key_type = value_type;
  using key_type_ref = value_type_ref;
  using data_type = __host__ __device__
bool;
  using data_type_ref = __host__ __device__
bool;

  static key_type_ref KeyOfValue(value_type_ref D) { return D; }
  static data_type_ref DataOfValue(value_type_ref) { return true; }

  static __host__ __device__
bool isEqual(key_type_ref LHS, key_type_ref RHS) { return LHS == RHS; }

  static __host__ __device__
bool isLess(key_type_ref LHS, key_type_ref RHS) { return LHS < RHS; }

  static __host__ __device__
bool isDataEqual(data_type_ref, data_type_ref) { return true; }
};

//===----------------------------------------------------------------------===//
// Immutable Set
//===----------------------------------------------------------------------===//

template <typename ValT, typename ValInfo = ImutContainerInfo<ValT>>
class ImmutableSet {
public:
  using value_type = typename ValInfo::value_type;
  using value_type_ref = typename ValInfo::value_type_ref;
  using TreeTy = ImutAVLTree<ValInfo>;

private:
  TreeTy *Root;

public:
  /// Constructs a set from a po__host__ __device__
inter to a tree root.  In general one
  /// should use a Factory object to create sets instead of directly
  /// invoking the constructor, but there are cases where make this
  /// constructor public is useful.
  explicit ImmutableSet(TreeTy* R) : Root(R) {
    if (Root) { Root->retain(); }
  }

  ImmutableSet(const ImmutableSet &X) : Root(X.Root) {
    if (Root) { Root->retain(); }
  }

  ~ImmutableSet() {
    if (Root) { Root->release(); }
  }

  ImmutableSet &operator=(const ImmutableSet &X) {
    if (Root != X.Root) {
      if (X.Root) { X.Root->retain(); }
      if (Root) { Root->release(); }
      Root = X.Root;
    }
    return *this;
  }

  class Factory {
    typename TreeTy::Factory F;
    const __host__ __device__
bool Canonicalize;

  public:
    Factory(__host__ __device__
bool canonicalize = true)
      : Canonicalize(canonicalize) {}

    Factory(BumpPtrAllocator& Alloc, __host__ __device__
bool canonicalize = true)
      : F(Alloc), Canonicalize(canonicalize) {}

    Factory(const Factory& RHS) = delete;
    __host__ __device__
void operator=(const Factory& RHS) = delete;

    /// getEmptySet - Returns an immutable set that contains no elements.
    ImmutableSet getEmptySet() {
      return ImmutableSet(F.getEmptyTree());
    }

    /// add - Creates a new immutable set that contains all of the values
    ///  of the original set with the addition of the specified value.  If
    ///  the original set already included the value, then the original set is
    ///  returned and no memory is allocated.  The time and space complexity
    ///  of this operation is logarithmic in the size of the original set.
    ///  The memory allocated to represent the set is released when the
    ///  factory object that created the set is destroyed.
    LLVM_NODISCARD ImmutableSet add(ImmutableSet Old, value_type_ref V) {
      TreeTy *NewT = F.add(Old.Root, V);
      return ImmutableSet(Canonicalize ? F.getCanonicalTree(NewT) : NewT);
    }

    /// remove - Creates a new immutable set that contains all of the values
    ///  of the original set with the exception of the specified value.  If
    ///  the original set did not contain the value, the original set is
    ///  returned and no memory is allocated.  The time and space complexity
    ///  of this operation is logarithmic in the size of the original set.
    ///  The memory allocated to represent the set is released when the
    ///  factory object that created the set is destroyed.
    LLVM_NODISCARD ImmutableSet remove(ImmutableSet Old, value_type_ref V) {
      TreeTy *NewT = F.remove(Old.Root, V);
      return ImmutableSet(Canonicalize ? F.getCanonicalTree(NewT) : NewT);
    }

    BumpPtrAllocator& getAllocator() { return F.getAllocator(); }

    typename TreeTy::Factory *getTreeFactory() const {
      return const_cast<typename TreeTy::Factory *>(&F);
    }
  };

  friend class Factory;

  /// Returns true if the set contains the specified value.
  __host__ __device__
bool contains(value_type_ref V) const {
    return Root ? Root->contains(V) : false;
  }

  __host__ __device__
bool operator==(const ImmutableSet &RHS) const {
    return Root && RHS.Root ? Root->isEqual(*RHS.Root) : Root == RHS.Root;
  }

  __host__ __device__
bool operator!=(const ImmutableSet &RHS) const {
    return Root && RHS.Root ? Root->isNotEqual(*RHS.Root) : Root != RHS.Root;
  }

  TreeTy *getRoot() {
    if (Root) { Root->retain(); }
    return Root;
  }

  TreeTy *getRootWithoutRetain() const {
    return Root;
  }

  /// isEmpty - Return true if the set contains no elements.
  __host__ __device__
bool isEmpty() const { return !Root; }

  /// isSingleton - Return true if the set contains exactly one element.
  ///   This method runs in constant time.
  __host__ __device__
bool isSingleton() const { return getHeight() == 1; }

  template <typename Callback>
  __host__ __device__
void foreach(Callback& C) { if (Root) Root->foreach(C); }

  template <typename Callback>
  __host__ __device__
void foreach() { if (Root) { Callback C; Root->foreach(C); } }

  //===--------------------------------------------------===//
  // Iterators.
  //===--------------------------------------------------===//

  using iterator = ImutAVLValueIterator<ImmutableSet>;

  iterator begin() const { return iterator(Root); }
  iterator end() const { return iterator(); }

  //===--------------------------------------------------===//
  // Utility methods.
  //===--------------------------------------------------===//

  unsigned getHeight() const { return Root ? Root->getHeight() : 0; }

  static __host__ __device__
void Profile(FoldingSetNodeID &ID, const ImmutableSet &S) {
    ID.AddPo__host__ __device__
inter(S.Root);
  }

  __host__ __device__
void Profile(FoldingSetNodeID &ID) const { return Profile(ID, *this); }

  //===--------------------------------------------------===//
  // For testing.
  //===--------------------------------------------------===//

  __host__ __device__
void validateTree() const { if (Root) Root->validateTree(); }
};

// NOTE: This may some day replace the current ImmutableSet.
template <typename ValT, typename ValInfo = ImutContainerInfo<ValT>>
class ImmutableSetRef {
public:
  using value_type = typename ValInfo::value_type;
  using value_type_ref = typename ValInfo::value_type_ref;
  using TreeTy = ImutAVLTree<ValInfo>;
  using FactoryTy = typename TreeTy::Factory;

private:
  TreeTy *Root;
  FactoryTy *Factory;

public:
  /// Constructs a set from a po__host__ __device__
inter to a tree root.  In general one
  /// should use a Factory object to create sets instead of directly
  /// invoking the constructor, but there are cases where make this
  /// constructor public is useful.
  explicit ImmutableSetRef(TreeTy* R, FactoryTy *F)
    : Root(R),
      Factory(F) {
    if (Root) { Root->retain(); }
  }

  ImmutableSetRef(const ImmutableSetRef &X)
    : Root(X.Root),
      Factory(X.Factory) {
    if (Root) { Root->retain(); }
  }

  ~ImmutableSetRef() {
    if (Root) { Root->release(); }
  }

  ImmutableSetRef &operator=(const ImmutableSetRef &X) {
    if (Root != X.Root) {
      if (X.Root) { X.Root->retain(); }
      if (Root) { Root->release(); }
      Root = X.Root;
      Factory = X.Factory;
    }
    return *this;
  }

  static ImmutableSetRef getEmptySet(FactoryTy *F) {
    return ImmutableSetRef(0, F);
  }

  ImmutableSetRef add(value_type_ref V) {
    return ImmutableSetRef(Factory->add(Root, V), Factory);
  }

  ImmutableSetRef remove(value_type_ref V) {
    return ImmutableSetRef(Factory->remove(Root, V), Factory);
  }

  /// Returns true if the set contains the specified value.
  __host__ __device__
bool contains(value_type_ref V) const {
    return Root ? Root->contains(V) : false;
  }

  ImmutableSet<ValT> asImmutableSet(__host__ __device__
bool canonicalize = true) const {
    return ImmutableSet<ValT>(canonicalize ?
                              Factory->getCanonicalTree(Root) : Root);
  }

  TreeTy *getRootWithoutRetain() const {
    return Root;
  }

  __host__ __device__
bool operator==(const ImmutableSetRef &RHS) const {
    return Root && RHS.Root ? Root->isEqual(*RHS.Root) : Root == RHS.Root;
  }

  __host__ __device__
bool operator!=(const ImmutableSetRef &RHS) const {
    return Root && RHS.Root ? Root->isNotEqual(*RHS.Root) : Root != RHS.Root;
  }

  /// isEmpty - Return true if the set contains no elements.
  __host__ __device__
bool isEmpty() const { return !Root; }

  /// isSingleton - Return true if the set contains exactly one element.
  ///   This method runs in constant time.
  __host__ __device__
bool isSingleton() const { return getHeight() == 1; }

  //===--------------------------------------------------===//
  // Iterators.
  //===--------------------------------------------------===//

  using iterator = ImutAVLValueIterator<ImmutableSetRef>;

  iterator begin() const { return iterator(Root); }
  iterator end() const { return iterator(); }

  //===--------------------------------------------------===//
  // Utility methods.
  //===--------------------------------------------------===//

  unsigned getHeight() const { return Root ? Root->getHeight() : 0; }

  static __host__ __device__
void Profile(FoldingSetNodeID &ID, const ImmutableSetRef &S) {
    ID.AddPo__host__ __device__
inter(S.Root);
  }

  __host__ __device__
void Profile(FoldingSetNodeID &ID) const { return Profile(ID, *this); }

  //===--------------------------------------------------===//
  // For testing.
  //===--------------------------------------------------===//

  __host__ __device__
void validateTree() const { if (Root) Root->validateTree(); }
};

} // end namespace llvm

#endif // LLVM_ADT_IMMUTABLESET_H
