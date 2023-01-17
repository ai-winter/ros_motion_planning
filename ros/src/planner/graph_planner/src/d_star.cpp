#include "d_star.h"

namespace d_star_planner
{
    DStar::DStar(int nx, int ny, double resolution, nav_msgs::OccupancyGrid *p_local_costmap)
        : global_planner::GlobalPlanner(nx, ny, resolution)
    {
        this->init_plan = false;

        this->pp_local_costmap = new nav_msgs::OccupancyGrid *;
        *(this->pp_local_costmap) = p_local_costmap;

        initMap();
    }

    void DStar::initMap()
    {
        this->DNodeMap = new DNodePtr *[this->nx_];
        for (int i = 0; i < this->nx_; i++)
        {
            this->DNodeMap[i] = new DNodePtr[this->ny_];
            for (int j = 0; j < this->ny_; j++)
                this->DNodeMap[i][j] = new DNode(i, j, inf, inf, this->grid2Index(i, j));
        }
    }

    void DStar::reset()
    {
        this->open_list.clear();

        for (int i = 0; i < this->nx_; i++)
            for (int j = 0; j < this->ny_; j++)
                delete this->DNodeMap[i][j];

        for (int i = 0; i < this->nx_; i++)
            delete[] this->DNodeMap[i];

        delete[] this->DNodeMap;

        initMap();
    }

    void DStar::insert(DNodePtr nodePtr, double h_new)
    {
        if (nodePtr->tag == NEW)
            nodePtr->k = h_new;
        else if (nodePtr->tag == OPEN)
            nodePtr->k = std::min(nodePtr->k, h_new);
        else if (nodePtr->tag == CLOSED)
            nodePtr->k = std::min(nodePtr->cost, h_new);

        nodePtr->cost = h_new;
        nodePtr->tag = OPEN;
        nodePtr->nodeMapIt = this->open_list.insert(std::make_pair(nodePtr->k, nodePtr));
    }

    bool DStar::isCollision(DNodePtr n1, DNodePtr n2)
    {
        int n1_id = n1->id, n2_id = n2->id;
        return this->global_costmap[n1_id] > this->lethal_cost_ * this->factor_ ||
               this->global_costmap[n2_id] > this->lethal_cost_ * this->factor_;
    }

    void DStar::getNeighbours(DNodePtr nodePtr, std::vector<DNodePtr> &neighbours)
    {
        int x = nodePtr->x, y = nodePtr->y;
        for (int i = -1; i <= 1; i++)
        {
            for (int j = -1; j <= 1; j++)
            {
                if (i == 0 && j == 0)
                    continue;

                DNodePtr neigbourPtr = this->DNodeMap[x + i][y + j];
                if (isCollision(nodePtr, neigbourPtr))
                    continue;
                if (neigbourPtr->id < 0 || neigbourPtr->id >= this->ns_)
                    continue;

                neighbours.push_back(neigbourPtr);
            }
        }
    }

    double DStar::cost(DNodePtr n1, DNodePtr n2)
    {
        if (isCollision(n1, n2))
            return inf;
        return std::sqrt(std::pow(n1->x - n2->x, 2) + std::pow(n1->y - n2->y, 2));
    }

    double DStar::processState()
    {
        if (this->open_list.empty())
            return -1;

        double k_old = this->open_list.begin()->first;
        DNodePtr x = this->open_list.begin()->second;
        this->open_list.erase(this->open_list.begin());
        x->tag = CLOSED;

        std::vector<DNodePtr> neigbours;
        this->getNeighbours(x, neigbours);

        if (k_old < x->cost)
        {
            for (int i = 0; i < (int)neigbours.size(); i++)
            {
                DNodePtr y = neigbours[i];
                if (y->cost <= k_old && x->cost > y->cost + this->cost(y, x))
                {
                    x->pid = y->id;
                    x->cost = y->cost + this->cost(y, x);
                }
            }
        }

        if (k_old == x->cost)
        {
            for (int i = 0; i < (int)neigbours.size(); i++)
            {
                DNodePtr y = neigbours[i];
                if (y->tag == NEW ||
                    (y->pid == x->id && y->cost != x->cost + this->cost(x, y)) ||
                    (y->pid != x->id && y->cost > x->cost + this->cost(x, y)))
                {
                    y->pid = x->id;
                    this->insert(y, x->cost + this->cost(x, y));
                }
            }
        }
        else
        {
            for (int i = 0; i < (int)neigbours.size(); i++)
            {
                DNodePtr y = neigbours[i];

                if (y->tag == NEW || (y->pid == x->id && y->cost != x->cost + this->cost(x, y)))
                {
                    y->pid = x->id;
                    this->insert(y, x->cost + this->cost(x, y));
                }
                else
                {
                    if (y->pid != x->id && y->cost > x->cost + this->cost(x, y))
                    {
                        this->insert(x, x->cost);
                    }
                    else
                    {
                        if (y->pid != x->id && x->cost > y->cost + this->cost(x, y) &&
                            y->tag == CLOSED && y->cost > k_old)
                        {
                            this->insert(y, y->cost);
                        }
                    }
                }
            }
        }

        return open_list.begin()->first;
    }

    void DStar::extractExpand(std::vector<Node> &expand)
    {
        for (int i = 0; i < this->nx_; i++)
        {
            for (int j = 0; j < this->ny_; j++)
            {
                DNodePtr tmp = this->DNodeMap[i][j];
                if (tmp->tag == CLOSED)
                    expand.push_back(*tmp);
            }
        }
    }

    void DStar::extractPath(const Node &start, const Node &goal)
    {
        this->path.clear();
        DNodePtr nPtr = this->DNodeMap[start.x][start.y];
        while (nPtr->x != goal.x && nPtr->y != goal.y)
        {
            this->path.push_back(*nPtr);

            int x, y;
            this->index2Grid(nPtr->pid, x, y);
            nPtr = this->DNodeMap[x][y];
        }
        std::reverse(this->path.begin(), this->path.end());
    }

    std::tuple<bool, std::vector<Node>> DStar::plan(const unsigned char *costs, const Node &start,
                                                    const Node &goal, std::vector<Node> &expand)
    {
        // ===== NOTE: TEMP, need to rewrite when incroperating local costmap =====
        expand.clear();
        this->reset();
        // ========================================================================
        this->global_costmap = costs;
        DNodePtr sPtr = this->DNodeMap[start.x][start.y];
        DNodePtr gPtr = this->DNodeMap[goal.x][goal.y];

        // if (!this->init_plan)
        if (1)
        {
            this->init_plan = true;
            gPtr->cost = 0;
            this->insert(gPtr, gPtr->cost);

            while (1)
            {
                processState();
                if (sPtr->tag == CLOSED)
                    break;
            }

            this->extractExpand(expand);
            this->extractPath(start, goal);
            return {true, this->path};
        }
        // else
        // {
        // }
    }
}
