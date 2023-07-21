#ifndef _PRIORITYQUEUE2_H_
#define _PRIORITYQUEUE2_H_

#include <vector>
#include <set>
#include <queue>
#include <assert.h>
#include "point.h"
#include <map>

//! Priority queue for integer coordinates with squared distances as priority.
/** A priority queue that uses buckets to group elements with the same priority.
 *  The individual buckets are unsorted, which increases efficiency if these groups are large.
 *  The elements are assumed to be integer coordinates, and the priorities are assumed
 *  to be squared Euclidean distances (integers).
 */

template <typename T>
class BucketPrioQueue {

public:
  //! Standard constructor
  /** Standard constructor. When called for the first time it creates a look up table
   *  that maps square distances to bucket numbers, which might take some time...
   */
  BucketPrioQueue();


  void clear() {
    buckets.clear();
    count = 0;
    nextPop = buckets.end();
  }

  //! Checks whether the Queue is empty
  bool empty();
  //! push an element
  void push(int prio, T t);
  //! return and pop the element with the lowest squared distance */
  T pop();

  int size() { return count; }
  int getNumBuckets() { return buckets.size(); }

  int getTopPriority(){
    return nextPop->first;
  }

private:

  int count;

  typedef std::map< int, std::queue<T> > BucketType;
  BucketType buckets;
  typename BucketType::iterator nextPop;
};

#include "bucketedqueue.hxx"

#endif
