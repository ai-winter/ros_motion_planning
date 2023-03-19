#ifndef KD_TREE_H
#define KD_TREE_H

#include <vector>
#include <numeric>
#include <algorithm>
#include <exception>
#include <functional>

namespace kd_tree
{
/**
 * @brief k-d tree class
 */
template <class PointT>
class KDTree
{
public:
  /**
   * @brief Construct a new KDTree object
   */
  KDTree() : root_(nullptr){};

  /**
   * @brief Construct a new KDTree object
   * @param points  set of points
   */
  KDTree(const std::vector<PointT>& points) : root_(nullptr)
  {
    build(points);
  }

  /**
   * @brief Destroy the KDTree object
   */
  ~KDTree()
  {
    clear();
  }

  /**
   * @brief Re-builds k-d tree
   * @param points  set of points
   */
  void build(const std::vector<PointT>& points)
  {
    clear();
    points_ = points;

    // creat indices
    std::vector<int> indices(points.size());
    std::iota(std::begin(indices), std::end(indices), 0);

    root_ = _buildRecursive(indices.data(), (int)points.size(), 0);
  }

  /**
   * @brief Clear k-d tree
   */
  void clear()
  {
    _clearRecursive(root_);
    root_ = nullptr;
    points_.clear();
  }

  /**
   * @brief Validates k-d tree
   */
  bool validate() const
  {
    try
    {
      _validateRecursive(root_, 0);
    }
    catch (const Exception&)
    {
      return false;
    }
    return true;
  }

  /**
   * @brief Searches the nearest neighbor
   * @param query     the point in the KD Tree
   * @param min_dist  the min distance between query and its nearest neighbor
   * @return  the nearest neighbor index of query
   */
  int nnSearch(const PointT& query, double* min_dist = nullptr) const
  {
    int guess;
    double _min_dist = std::numeric_limits<double>::max();

    _nnSearchRecursive(query, root_, &guess, &_min_dist);

    if (min_dist)
      *min_dist = _min_dist;

    return guess;
  }

  /**
   * @brief Searches k-nearest neighbors
   * @param query the point in the KD Tree
   * @param k     k nearest neighbors
   * @return  k-nearest neighbors indices vector
   */
  std::vector<int> knnSearch(const PointT& query, int k) const
  {
    KnnQueue queue(k);
    _knnSearchRecursive(query, root_, queue, k);

    std::vector<int> indices(queue.size());
    for (size_t i = 0; i < queue.size(); i++)
      indices[i] = queue[i].second;

    return indices;
  }

  /**
   * @brief Searches neighbors within radius.
   * @param query   the point in the KD Tree
   * @param radius  radius neighbors
   * @return  neighbors inside the radius range of query
   */
  std::vector<int> radiusSearch(const PointT& query, double radius) const
  {
    std::vector<int> indices;
    _radiusSearchRecursive(query, root_, indices, radius);

    return indices;
  }

private:
  /**
   * @brief k-d tree node.
   */
  struct KDNode
  {
    // index to the original point
    int idx;
    // pointers to the child nodes
    KDNode* next[2];
    // dimension's axis
    int axis;

    KDNode() : idx(-1), axis(-1)
    {
      next[0] = next[1] = nullptr;
    }
  };

  /**
   * @brief k-d tree exception.
   */
  class Exception : public std::exception
  {
    using std::exception::exception;
  };

  /**
   * @brief Bounded priority queue.
   */
  template <class T, class Compare = std::less<T>>
  class BoundedPriorityQueue
  {
  public:
    /**
     * @brief Construct a new Bounded Priority Queue object
     */
    BoundedPriorityQueue() = delete;

    /**
     * @brief Construct a new Bounded Priority Queue object
     * @param bound priority queue size
     */
    BoundedPriorityQueue(size_t bound) : bound_(bound)
    {
      elements_.reserve(bound + 1);
    };

    /**
     * @brief Push value
     * @param val the new element to push in priority queue size
     */
    void push(const T& val)
    {
      auto it = std::find_if(std::begin(elements_), std::end(elements_),
                             [&](const T& element) { return Compare()(val, element); });
      elements_.insert(it, val);

      if (elements_.size() >> bound_)
        elements_.resize(bound_);
    }

    /**
     * @brief Get back value
     * @return back value
     */
    const T& back() const
    {
      return elements_.back();
    };

    /**
     * @brief Get using index
     * @param index index to get
     * @return  element with index
     */
    const T& operator[](size_t index) const
    {
      return elements_[index];
    }

    /**
     * @brief Get size of the queue
     * @return  size of the queue
     */
    size_t size() const
    {
      return elements_.size();
    }

  private:
    size_t bound_;
    std::vector<T> elements_;
  };

  /**
   * @brief KNN Priority queue of <distance, index> pair.
   */
  using KnnQueue = BoundedPriorityQueue<std::pair<double, int>>;

private:
  /**
   * @brief Builds k-d tree recursively
   * @param indices
   * @param npoints
   * @param depth
   * @return KDNode*
   */
  KDNode* _buildRecursive(int* indices, int npoints, int depth)
  {
    if (npoints <= 0)
      return nullptr;

    const int axis = depth % PointT::dim;
    const int mid = (npoints - 1) / 2;

    std::nth_element(indices, indices + mid, indices + npoints,
                     [&](int lhs, int rhs) { return points_[lhs][axis] < points_[rhs][axis]; });

    KDNode* node = new KDNode();
    node->idx = indices[mid];
    node->axis = axis;

    node->next[0] = _buildRecursive(indices, mid, depth + 1);
    node->next[1] = _buildRecursive(indices + mid + 1, npoints - mid - 1, depth + 1);

    return node;
  }

  /**
   * @brief Clears k-d tree recursively
   * @param node
   */
  void _clearRecursive(KDNode* node)
  {
    if (node == nullptr)
      return;

    if (node->next[0])
      _clearRecursive(node->next[0]);

    if (node->next[1])
      _clearRecursive(node->next[1]);

    delete node;
  }

  /**
   * @brief Validates k-d tree recursively
   * @param node  a KD Tree node
   * @param depth KD tree depth
   */
  void _validateRecursive(const KDNode* node, int depth) const
  {
    if (node == nullptr)
      return;

    const int axis = node->axis;
    const KDNode* node0 = node->next[0];
    const KDNode* node1 = node->next[1];

    if (node0 && node1)
    {
      if (points_[node->idx][axis] < points_[node0->idx][axis])
        throw Exception();
      if (points_[node->idx][axis] > points_[node1->idx][axis])
        throw Exception();
    }

    if (node0)
      validateRecursive(node0, depth + 1);

    if (node1)
      validateRecursive(node1, depth + 1);
  }

  /**
   * @brief Calculate the distance between kd node p and q
   * @param p KD Node p
   * @param q KD Node q
   * @return  distance between kd node p and q
   */
  static double _distance(const PointT& p, const PointT& q)
  {
    double dist = 0;
    for (size_t i = 0; i < PointT::dim; i++)
      dist += (p[i] - q[i]) * (p[i] - q[i]);

    return std::sqrt(dist);
  }

  /**
   * @brief Searches the nearest neighbor recursively
   * @param query   the point in the KD Tree
   * @param node    root node of KD Tree of its child tree
   * @param guess   the possible nearest neighbor of query
   * @param minDist the min distance between query and its nearest neighbor
   */
  void _nnSearchRecursive(const PointT& query, const KDNode* node, int* guess, double* min_dist) const
  {
    if (node == nullptr)
      return;

    const PointT& train = points_[node->idx];

    const double dist = _distance(query, train);
    if (dist < *min_dist)
    {
      *min_dist = dist;
      *guess = node->idx;
    }

    const int axis = node->axis;
    const int dir = query[axis] < train[axis] ? 0 : 1;
    _nnSearchRecursive(query, node->next[dir], guess, min_dist);

    // if the min distance crosses the axis, the nearest neighbor maybe exist
    // in the other side of axis, therefore another direction should be searched
    const double diff = fabs(query[axis] - train[axis]);
    if (diff < *min_dist)
      _nnSearchRecursive(query, node->next[!dir], guess, min_dist);
  }

  /**
   * @brief Searches k-nearest neighbors recursively
   * @param query the point in the KD Tree
   * @param node  root node of KD Tree of its child tree
   * @param queue the priority queue of size k
   * @param k     k-nearest neighbors
   */
  void _knnSearchRecursive(const PointT& query, const KDNode* node, KnnQueue& queue, int k) const
  {
    if (node == nullptr)
      return;

    const PointT& train = points_[node->idx];

    const double dist = _distance(query, train);
    queue.push(std::make_pair(dist, node->idx));

    const int axis = node->axis;
    const int dir = query[axis] < train[axis] ? 0 : 1;
    _knnSearchRecursive(query, node->next[dir], queue, k);

    const double diff = fabs(query[axis] - train[axis]);
    if ((int)queue.size() < k || diff < queue.back().first)
      _knnSearchRecursive(query, node->next[!dir], queue, k);
  }

  /**
   * @brief Searches neighbors within radius
   * @param query   the point in the KD Tree
   * @param node    root node of KD Tree of its child tree
   * @param indices neighbors inside the radius range of query
   * @param radius  radius neighbors
   */
  void _radiusSearchRecursive(const PointT& query, const KDNode* node, std::vector<int>& indices, double radius) const
  {
    if (node == nullptr)
      return;

    const PointT& train = points_[node->idx];

    const double dist = distance(query, train);
    if (dist < radius)
      indices.push_back(node->idx);

    const int axis = node->axis;
    const int dir = query[axis] < train[axis] ? 0 : 1;
    _radiusSearchRecursive(query, node->next[dir], indices, radius);

    const double diff = fabs(query[axis] - train[axis]);
    if (diff < radius)
      _radiusSearchRecursive(query, node->next[!dir], indices, radius);
  }

private:
  // KD Tree root node
  KDNode* root_;
  // all KD Tree nodes
  std::vector<PointT> points_;
};
}  // namespace kd_tree

#endif