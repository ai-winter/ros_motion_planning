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
   * @brief Constructors
   * @param points    set of points
   */
  KDTree() : root_(nullptr){};
  KDTree(const std::vector<PointT>& points) : root_(nullptr)
  {
    this->build(points);
  }
  /**
   * @brief The destructor.
   */
  ~KDTree()
  {
    this->clear();
  }
  /**
   * @brief Re-builds k-d tree.
   */
  void build(const std::vector<PointT>& points)
  {
    this->clear();
    this->points_ = points;

    // creat indices
    std::vector<int> indices(points.size());
    std::iota(std::begin(indices), std::end(indices), 0);

    this->root_ = this->_buildRecursive(indices.data(), (int)points.size(), 0);
  }
  /**
   * @brief Clears k-d tree.
   */
  void clear()
  {
    this->_clearRecursive(root_);
    this->root_ = nullptr;
    this->points_.clear();
  }
  /**
   * @brief Validates k-d tree.
   */
  bool validate() const
  {
    try
    {
      this->_validateRecursive(this->root_, 0);
    }
    catch (const Exception&)
    {
      return false;
    }
    return true;
  }

  /**
   * @brief Searches the nearest neighbor
   * @param   query   the point in the KD Tree
   * @param   minDist the min distance between query and its nearest neighbor
   * @return  guess   the nearest neighbor index of query
   */
  int nnSearch(const PointT& query, double* min_dist = nullptr) const
  {
    int guess;
    double _min_dist = std::numeric_limits<double>::max();

    this->_nnSearchRecursive(query, this->root_, &guess, &_min_dist);

    if (min_dist)
      *min_dist = _min_dist;

    return guess;
  }

  /**
   * @brief Searches k-nearest neighbors
   * @param   query   the point in the KD Tree
   * @param   k       k nearest neighbors
   * @return  k-nearest neighbors indices vector
   */
  std::vector<int> knnSearch(const PointT& query, int k) const
  {
    KnnQueue queue(k);
    this->_knnSearchRecursive(query, root_, queue, k);

    std::vector<int> indices(queue.size());
    for (size_t i = 0; i < queue.size(); i++)
      indices[i] = queue[i].second;

    return indices;
  }

  /**
   * @brief Searches neighbors within radius.
   * @param   query   the point in the KD Tree
   * @param   radius  radius neighbors
   * @return  neighbors inside the radius range of query
   */
  std::vector<int> radiusSearch(const PointT& query, double radius) const
  {
    std::vector<int> indices;
    this->_radiusSearchRecursive(query, this->root_, indices, radius);
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
     * @brief  Constructor
     * @param   bound       priority queue size
     */
    BoundedPriorityQueue() = delete;
    BoundedPriorityQueue(size_t bound) : bound_(bound)
    {
      this->elements_.reserve(bound + 1);
    };
    /**
     * @brief  Constructor
     * @param   val    the new element to push in priority queue size
     */
    void push(const T& val)
    {
      auto it = std::find_if(std::begin(this->elements_), std::end(this->elements_),
                             [&](const T& element) { return Compare()(val, element); });
      this->elements_.insert(it, val);

      if (this->elements_.size() > this->bound_)
        this->elements_.resize(this->bound_);
    }

    const T& back() const
    {
      return this->elements_.back();
    };
    const T& operator[](size_t index) const
    {
      return this->elements_[index];
    }
    size_t size() const
    {
      return this->elements_.size();
    }

  private:
    size_t bound_;
    std::vector<T> elements_;
  };

  /**
   * @brief KNN Priority queue of <distance, index> pair.
   */
  using KnnQueue = BoundedPriorityQueue<std::pair<double, int>>;

  /**
   * @brief Builds k-d tree recursively.
   */
  KDNode* _buildRecursive(int* indices, int npoints, int depth)
  {
    if (npoints <= 0)
      return nullptr;

    const int axis = depth % PointT::dim;
    const int mid = (npoints - 1) / 2;

    std::nth_element(indices, indices + mid, indices + npoints,
                     [&](int lhs, int rhs) { return this->points_[lhs][axis] < this->points_[rhs][axis]; });

    KDNode* node = new KDNode();
    node->idx = indices[mid];
    node->axis = axis;

    node->next[0] = this->_buildRecursive(indices, mid, depth + 1);
    node->next[1] = this->_buildRecursive(indices + mid + 1, npoints - mid - 1, depth + 1);

    return node;
  }

  /** @brief Clears k-d tree recursively.
   */
  void _clearRecursive(KDNode* node)
  {
    if (node == nullptr)
      return;

    if (node->next[0])
      this->_clearRecursive(node->next[0]);

    if (node->next[1])
      this->_clearRecursive(node->next[1]);

    delete node;
  }

  /**
   * @brief Validates k-d tree recursively.
   * @param   node    a KD Tree node
   * @param   depth   KD tree depth
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
      if (this->points_[node->idx][axis] < this->points_[node0->idx][axis])
        throw Exception();
      if (this->points_[node->idx][axis] > this->points_[node1->idx][axis])
        throw Exception();
    }

    if (node0)
      validateRecursive(node0, depth + 1);

    if (node1)
      validateRecursive(node1, depth + 1);
  }

  /**
   * @brief calculate the distance between kd node p and q.
   * @param   p   KD Node p
   * @param   q   KD Node q
   * @return  distance between kd node p and q
   */
  static double _distance(const PointT& p, const PointT& q)
  {
    double dist = 0;
    for (size_t i = 0; i < PointT::dim; i++)
      dist += (p[i] - q[i]) * (p[i] - q[i]);
    return sqrt(dist);
  }

  /**
   * @brief Searches the nearest neighbor recursively.
   * @param   query   the point in the KD Tree
   * @param   node    root node of KD Tree of its child tree
   * @param   guess   the possible nearest neighbor of query
   * @param   minDist the min distance between query and its nearest neighbor
   */
  void _nnSearchRecursive(const PointT& query, const KDNode* node, int* guess, double* min_dist) const
  {
    if (node == nullptr)
      return;

    const PointT& train = points_[node->idx];

    const double dist = this->_distance(query, train);
    if (dist < *min_dist)
    {
      *min_dist = dist;
      *guess = node->idx;
    }

    const int axis = node->axis;
    const int dir = query[axis] < train[axis] ? 0 : 1;
    this->_nnSearchRecursive(query, node->next[dir], guess, min_dist);

    // if the min distance crosses the axis, the nearest neighbor maybe exist
    // in the other side of axis, therefore another direction should be searched
    const double diff = fabs(query[axis] - train[axis]);
    if (diff < *min_dist)
      this->_nnSearchRecursive(query, node->next[!dir], guess, min_dist);
  }

  /**
   * @brief Searches k-nearest neighbors recursively.
   * @param   query   the point in the KD Tree
   * @param   node    root node of KD Tree of its child tree
   * @param   queue   the priority queue of size k
   * @param   k       k-nearest neighbors
   */
  void _knnSearchRecursive(const PointT& query, const KDNode* node, KnnQueue& queue, int k) const
  {
    if (node == nullptr)
      return;

    const PointT& train = this->points_[node->idx];

    const double dist = this->_distance(query, train);
    queue.push(std::make_pair(dist, node->idx));

    const int axis = node->axis;
    const int dir = query[axis] < train[axis] ? 0 : 1;
    this->_knnSearchRecursive(query, node->next[dir], queue, k);

    const double diff = fabs(query[axis] - train[axis]);
    if ((int)queue.size() < k || diff < queue.back().first)
      this->_knnSearchRecursive(query, node->next[!dir], queue, k);
  }

  /**
   * @brief Searches neighbors within radius.
   * @param   query   the point in the KD Tree
   * @param   node    root node of KD Tree of its child tree
   * @param   indices neighbors inside the radius range of query
   * @param   radius  radius neighbors
   */
  void _radiusSearchRecursive(const PointT& query, const KDNode* node, std::vector<int>& indices, double radius) const
  {
    if (node == nullptr)
      return;

    const PointT& train = this->points_[node->idx];

    const double dist = distance(query, train);
    if (dist < radius)
      indices.push_back(node->idx);

    const int axis = node->axis;
    const int dir = query[axis] < train[axis] ? 0 : 1;
    this->_radiusSearchRecursive(query, node->next[dir], indices, radius);

    const double diff = fabs(query[axis] - train[axis]);
    if (diff < radius)
      this->_radiusSearchRecursive(query, node->next[!dir], indices, radius);
  }

  // KD Tree root node
  KDNode* root_;
  // all KD Tree nodes
  std::vector<PointT> points_;
};
}  // namespace kd_tree

#endif