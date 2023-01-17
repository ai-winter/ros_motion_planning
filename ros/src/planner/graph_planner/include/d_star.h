
#ifndef D_STAR_H
#define D_STAR_H

#include <map>
#include <algorithm>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

#include "global_planner.h"

#define inf 1 << 20

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
        const unsigned char *global_costmap;
        // local costmap pointer pointer
        nav_msgs::OccupancyGrid **pp_local_costmap;
        // init plan flag
        bool init_plan;
        // grid map
        DNodePtr **DNodeMap;
        // open list
        std::multimap<double, DNodePtr> open_list;
        // path
        std::vector<Node> path;

    public:
        DStar(int nx, int ny, double resolution, nav_msgs::OccupancyGrid *p_local_costmap);

        void reset();

        void initMap();

        void insert(DNodePtr nodePtr, double h_new);

        bool isCollision(DNodePtr n1, DNodePtr n2);

        void getNeighbours(DNodePtr nodePtr, std::vector<DNodePtr> &neighbours);

        double cost(DNodePtr n1, DNodePtr n2);

        double processState();

        void extractExpand(std::vector<Node> &expand);

        void extractPath(const Node &start, const Node &goal);

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
        // Node's iterator from multimap
        std::multimap<double, DNodePtr>::iterator nodeMapIt;

    public:
        DNode(const int x = 0, const int y = 0, const double cost = 0, const double h_cost = 0,
              const int id = 0, const int pid = 0, const int tag = NEW, const double k = 0)
            : Node(x, y, cost, h_cost, id, pid), tag(tag), k(k) {}
    };

}

#endif
