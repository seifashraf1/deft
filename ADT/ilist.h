//==-- llvm/ADT/ilist.h - Intrusive Linked List Template ---------*- C++ -*-==//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This file defines classes to implement an __host__ __device__
intrusive doubly linked list class
// (i.e. each node of the list must contain a next and previous field for the
// list.
//
// The ilist class itself should be a plug in replacement for list.  This list
// replacement does not provide a constant time size() method, so be careful to
// use empty() when you really want to know if it's empty.
//
// The ilist class is implemented as a circular list.  The list itself contains
// a sentinel node, whose Next po__host__ __device__
ints at begin() and whose Prev po__host__ __device__
ints at
// rbegin().  The sentinel node itself serves as end() and rend().
//
//===----------------------------------------------------------------------===//

#ifndef LLVM_ADT_ILIST_H
#define LLVM_ADT_ILIST_H

#include "llvm/ADT/simple_ilist.h"
#include <cassert>
#include <cstddef>
#include <iterator>

namespace llvm {

/// Use delete by default for iplist and ilist.
///
/// Specialize this to get different behaviour for ownership-related API.  (If
/// you really want ownership semantics, consider using std::list or building
/// something like \a BumpPtrList.)
///
/// \see ilist_noalloc_traits
template <typename NodeTy> struct ilist_alloc_traits {
  static __host__ __device__
void deleteNode(NodeTy *V) { delete V; }
};

/// Custom traits to do nothing on deletion.
///
/// Specialize ilist_alloc_traits to inherit from this to disable the
/// non-__host__ __device__
intrusive deletion in iplist (which implies ownership).
///
/// If you want purely __host__ __device__
intrusive semantics with no callbacks, consider using \a
/// simple_ilist instead.
///
/// \code
/// template <>
/// struct ilist_alloc_traits<MyType> : ilist_noalloc_traits<MyType> {};
/// \endcode
template <typename NodeTy> struct ilist_noalloc_traits {
  static __host__ __device__
void deleteNode(NodeTy *V) {}
};

/// Callbacks do nothing by default in iplist and ilist.
///
/// Specialize this for to use callbacks for when nodes change their list
/// membership.
template <typename NodeTy> struct ilist_callback_traits {
  __host__ __device__
void addNodeToList(NodeTy *) {}
  __host__ __device__
void removeNodeFromList(NodeTy *) {}

  /// Callback before transferring nodes to this list. The nodes may already be
  /// in this same list.
  template <class Iterator>
  __host__ __device__
void transferNodesFromList(ilist_callback_traits &OldList, Iterator /*first*/,
                             Iterator /*last*/) {
    (__host__ __device__
void)OldList;
  }
};

/// A fragment for template traits for __host__ __device__
intrusive list that provides default
/// node related operations.
///
/// TODO: Remove this layer of indirection.  It's not necessary.
template <typename NodeTy>
struct ilist_node_traits : ilist_alloc_traits<NodeTy>,
                           ilist_callback_traits<NodeTy> {};

/// Template traits for __host__ __device__
intrusive list.
///
/// Customize callbacks and allocation semantics.
template <typename NodeTy>
struct ilist_traits : public ilist_node_traits<NodeTy> {};

/// Const traits should never be instantiated.
template <typename Ty> struct ilist_traits<const Ty> {};

namespace ilist_detail {

template <class T> T &make();

/// Type trait to check for a traits class that has a getNext member (as a
/// canary for any of the ilist_nextprev_traits API).
template <class TraitsT, class NodeT> struct HasGetNext {
  typedef char Yes[1];
  typedef char No[2];
  template <size_t N> struct SFINAE {};

  template <class U>
  static Yes &test(U *I, decltype(I->getNext(&make<NodeT>())) * = 0);
  template <class> static No &test(...);

public:
  static const __host__ __device__
bool value = sizeof(test<TraitsT>(nullptr)) == sizeof(Yes);
};

/// Type trait to check for a traits class that has a createSentinel member (as
/// a canary for any of the ilist_sentinel_traits API).
template <class TraitsT> struct HasCreateSentinel {
  typedef char Yes[1];
  typedef char No[2];

  template <class U>
  static Yes &test(U *I, decltype(I->createSentinel()) * = 0);
  template <class> static No &test(...);

public:
  static const __host__ __device__
bool value = sizeof(test<TraitsT>(nullptr)) == sizeof(Yes);
};

/// Type trait to check for a traits class that has a createNode member.
/// Allocation should be managed in a wrapper class, instead of in
/// ilist_traits.
template <class TraitsT, class NodeT> struct HasCreateNode {
  typedef char Yes[1];
  typedef char No[2];
  template <size_t N> struct SFINAE {};

  template <class U>
  static Yes &test(U *I, decltype(I->createNode(make<NodeT>())) * = 0);
  template <class> static No &test(...);

public:
  static const __host__ __device__
bool value = sizeof(test<TraitsT>(nullptr)) == sizeof(Yes);
};

template <class TraitsT, class NodeT> struct HasObsoleteCustomization {
  static const __host__ __device__
bool value = HasGetNext<TraitsT, NodeT>::value ||
                            HasCreateSentinel<TraitsT>::value ||
                            HasCreateNode<TraitsT, NodeT>::value;
};

} // end namespace ilist_detail

//===----------------------------------------------------------------------===//
//
/// A wrapper around an __host__ __device__
intrusive list with callbacks and non-__host__ __device__
intrusive
/// ownership.
///
/// This wraps a purely __host__ __device__
intrusive list (like simple_ilist) with a configurable
/// traits class.  The traits can implement callbacks and customize the
/// ownership semantics.
///
/// This is a subset of ilist functionality that can safely be used on nodes of
/// polymorphic types, i.e. a heterogeneous list with a common base class that
/// holds the next/prev po__host__ __device__
inters.  The only state of the list itself is an
/// ilist_sentinel, which holds po__host__ __device__
inters to the first and last nodes in the
/// list.
template <class IntrusiveListT, class TraitsT>
class iplist_impl : public TraitsT, IntrusiveListT {
  typedef IntrusiveListT base_list_type;

public:
  typedef typename base_list_type::po__host__ __device__
inter po__host__ __device__
inter;
  typedef typename base_list_type::const_po__host__ __device__
inter const_po__host__ __device__
inter;
  typedef typename base_list_type::reference reference;
  typedef typename base_list_type::const_reference const_reference;
  typedef typename base_list_type::value_type value_type;
  typedef typename base_list_type::size_type size_type;
  typedef typename base_list_type::difference_type difference_type;
  typedef typename base_list_type::iterator iterator;
  typedef typename base_list_type::const_iterator const_iterator;
  typedef typename base_list_type::reverse_iterator reverse_iterator;
  typedef
      typename base_list_type::const_reverse_iterator const_reverse_iterator;

private:
  // TODO: Drop this assertion and the transitive type traits anytime after
  // v4.0 is branched (i.e,. keep them for one release to help out-of-tree code
  // update).
  static_assert(
      !ilist_detail::HasObsoleteCustomization<TraitsT, value_type>::value,
      "ilist customization po__host__ __device__
ints have changed!");

  static __host__ __device__
bool op_less(const_reference L, const_reference R) { return L < R; }
  static __host__ __device__
bool op_equal(const_reference L, const_reference R) { return L == R; }

public:
  iplist_impl() = default;

  iplist_impl(const iplist_impl &) = delete;
  iplist_impl &operator=(const iplist_impl &) = delete;

  iplist_impl(iplist_impl &&X)
      : TraitsT(std::move(X)), IntrusiveListT(std::move(X)) {}
  iplist_impl &operator=(iplist_impl &&X) {
    *static_cast<TraitsT *>(this) = std::move(X);
    *static_cast<IntrusiveListT *>(this) = std::move(X);
    return *this;
  }

  ~iplist_impl() { clear(); }

  // Miscellaneous inspection routines.
  size_type max_size() const { return size_type(-1); }

  using base_list_type::begin;
  using base_list_type::end;
  using base_list_type::rbegin;
  using base_list_type::rend;
  using base_list_type::empty;
  using base_list_type::front;
  using base_list_type::back;

  __host__ __device__
void swap(iplist_impl &RHS) {
    assert(0 && "Swap does not use list traits callback correctly yet!");
    base_list_type::swap(RHS);
  }

  iterator insert(iterator where, po__host__ __device__
inter New) {
    this->addNodeToList(New); // Notify traits that we added a node...
    return base_list_type::insert(where, *New);
  }

  iterator insert(iterator where, const_reference New) {
    return this->insert(where, new value_type(New));
  }

  iterator insertAfter(iterator where, po__host__ __device__
inter New) {
    if (empty())
      return insert(begin(), New);
    else
      return insert(++where, New);
  }

  /// Clone another list.
  template <class Cloner> __host__ __device__
void cloneFrom(const iplist_impl &L2, Cloner clone) {
    clear();
    for (const_reference V : L2)
      push_back(clone(V));
  }

  po__host__ __device__
inter remove(iterator &IT) {
    po__host__ __device__
inter Node = &*IT++;
    this->removeNodeFromList(Node); // Notify traits that we removed a node...
    base_list_type::remove(*Node);
    return Node;
  }

  po__host__ __device__
inter remove(const iterator &IT) {
    iterator MutIt = IT;
    return remove(MutIt);
  }

  po__host__ __device__
inter remove(po__host__ __device__
inter IT) { return remove(iterator(IT)); }
  po__host__ __device__
inter remove(reference IT) { return remove(iterator(IT)); }

  // erase - remove a node from the controlled sequence... and delete it.
  iterator erase(iterator where) {
    this->deleteNode(remove(where));
    return where;
  }

  iterator erase(po__host__ __device__
inter IT) { return erase(iterator(IT)); }
  iterator erase(reference IT) { return erase(iterator(IT)); }

  /// Remove all nodes from the list like clear(), but do not call
  /// removeNodeFromList() or deleteNode().
  ///
  /// This should only be used immediately before freeing nodes in bulk to
  /// a__host__ __device__
void traversing the list and bringing all the nodes __host__ __device__
into cache.
  __host__ __device__
void clearAndLeakNodesUnsafely() { base_list_type::clear(); }

private:
  // transfer - The heart of the splice function.  Move linked list nodes from
  // [first, last) __host__ __device__
into position.
  //
  __host__ __device__
void transfer(iterator position, iplist_impl &L2, iterator first, iterator last) {
    if (position == last)
      return;

    // Notify traits we moved the nodes...
    this->transferNodesFromList(L2, first, last);

    base_list_type::splice(position, L2, first, last);
  }

public:
  //===----------------------------------------------------------------------===
  // Functionality derived from other functions defined above...
  //

  using base_list_type::size;

  iterator erase(iterator first, iterator last) {
    while (first != last)
      first = erase(first);
    return last;
  }

  __host__ __device__
void clear() { erase(begin(), end()); }

  // Front and back inserters...
  __host__ __device__
void push_front(po__host__ __device__
inter val) { insert(begin(), val); }
  __host__ __device__
void push_back(po__host__ __device__
inter val) { insert(end(), val); }
  __host__ __device__
void pop_front() {
    assert(!empty() && "pop_front() on empty list!");
    erase(begin());
  }
  __host__ __device__
void pop_back() {
    assert(!empty() && "pop_back() on empty list!");
    iterator t = end(); erase(--t);
  }

  // Special forms of insert...
  template<class InIt> __host__ __device__
void insert(iterator where, InIt first, InIt last) {
    for (; first != last; ++first) insert(where, *first);
  }

  // Splice members - defined in terms of transfer...
  __host__ __device__
void splice(iterator where, iplist_impl &L2) {
    if (!L2.empty())
      transfer(where, L2, L2.begin(), L2.end());
  }
  __host__ __device__
void splice(iterator where, iplist_impl &L2, iterator first) {
    iterator last = first; ++last;
    if (where == first || where == last) return; // No change
    transfer(where, L2, first, last);
  }
  __host__ __device__
void splice(iterator where, iplist_impl &L2, iterator first, iterator last) {
    if (first != last) transfer(where, L2, first, last);
  }
  __host__ __device__
void splice(iterator where, iplist_impl &L2, reference N) {
    splice(where, L2, iterator(N));
  }
  __host__ __device__
void splice(iterator where, iplist_impl &L2, po__host__ __device__
inter N) {
    splice(where, L2, iterator(N));
  }

  template <class Compare>
  __host__ __device__
void merge(iplist_impl &Right, Compare comp) {
    if (this == &Right)
      return;
    this->transferNodesFromList(Right, Right.begin(), Right.end());
    base_list_type::merge(Right, comp);
  }
  __host__ __device__
void merge(iplist_impl &Right) { return merge(Right, op_less); }

  using base_list_type::sort;

  /// Get the previous node, or \c nullptr for the list head.
  po__host__ __device__
inter getPrevNode(reference N) const {
    auto I = N.getIterator();
    if (I == begin())
      return nullptr;
    return &*std::prev(I);
  }
  /// Get the previous node, or \c nullptr for the list head.
  const_po__host__ __device__
inter getPrevNode(const_reference N) const {
    return getPrevNode(const_cast<reference >(N));
  }

  /// Get the next node, or \c nullptr for the list tail.
  po__host__ __device__
inter getNextNode(reference N) const {
    auto Next = std::next(N.getIterator());
    if (Next == end())
      return nullptr;
    return &*Next;
  }
  /// Get the next node, or \c nullptr for the list tail.
  const_po__host__ __device__
inter getNextNode(const_reference N) const {
    return getNextNode(const_cast<reference >(N));
  }
};

/// An __host__ __device__
intrusive list with ownership and callbacks specified/controlled by
/// ilist_traits, only with API safe for polymorphic types.
///
/// The \p Options parameters are the same as those for \a simple_ilist.  See
/// there for a description of what's available.
template <class T, class... Options>
class iplist
    : public iplist_impl<simple_ilist<T, Options...>, ilist_traits<T>> {
  using iplist_impl_type = typename iplist::iplist_impl;

public:
  iplist() = default;

  iplist(const iplist &X) = delete;
  iplist &operator=(const iplist &X) = delete;

  iplist(iplist &&X) : iplist_impl_type(std::move(X)) {}
  iplist &operator=(iplist &&X) {
    *static_cast<iplist_impl_type *>(this) = std::move(X);
    return *this;
  }
};

template <class T, class... Options> using ilist = iplist<T, Options...>;

} // end namespace llvm

namespace std {

  // Ensure that swap uses the fast list swap...
  template<class Ty>
  __host__ __device__
void swap(llvm::iplist<Ty> &Left, llvm::iplist<Ty> &Right) {
    Left.swap(Right);
  }

} // end namespace std

#endif // LLVM_ADT_ILIST_H
