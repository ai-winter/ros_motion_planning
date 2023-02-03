#ifndef D_STAR_H
#define D_STAR_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

#include <map>
#include <algorithm>

#include "global_planner.h"

#define inf 10000

enum Tag
{
    NEW = 0,
    OPEN = 1,
    CLOSED = 2
};

namespace d_star_planner
{
    class DNode;
    typedef DNode *DNodePtr;

    class DStar : public global_planner::GlobalPlanner
    {
    public:
        // global costmap
        unsigned char *global_costmap;
        // grid map
        DNodePtr **DNodeMap;
        // open list
        std::multimap<double, DNodePtr> open_list;
        // path
        std::vector<Node> path;
        // goal
        Node goal_;
        // forward distance, range of the sensor (in grid)
        int N_;

    public:
        DStar(int nx, int ny, double resolution);

        void reset();

        void initMap();

        void insert(DNodePtr nodePtr, double h_new);

        bool isCollision(DNodePtr n1, DNodePtr n2);

        void getNeighbours(DNodePtr nodePtr, std::vector<DNodePtr> &neighbours);

        double cost(DNodePtr n1, DNodePtr n2);

        double processState();

        void extractExpand(std::vector<Node> &expand);

        void extractPath(const Node &start, const Node &goal);

        Node getState(const Node &current);

        void modify(DNodePtr x, DNodePtr y);

        std::tuple<bool, std::vector<Node>> plan(const unsigned char *costs, const Node &start,
                                                 const Node &goal, std::vector<Node> &expand);
    };

    class DNode : public Node
    {
    public:
        // Node's tag among enum Tag
        int tag;
        // Node's k_min in history
        double k;
        // Node's iterator from multimap, default from small to large
        std::multimap<double, DNodePtr>::iterator nodeMapIt;

    public:
        DNode(const int x = 0, const int y = 0, const double cost = inf, const double h_cost = inf,
              const int id = 0, const int pid = -1, const int tag = NEW, const double k = inf)
            : Node(x, y, cost, h_cost, id, pid), tag(tag), k(k) {}
    };
}

#endif
