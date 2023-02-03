#include "d_star.h"

namespace d_star_planner
{
    DStar::DStar(int nx, int ny, double resolution)
        : global_planner::GlobalPlanner(nx, ny, resolution) // (, nav_msgs::OccupancyGrid *p_local_costmap)
    {
        // this->init_plan = false;

        // this->pp_local_costmap = new nav_msgs::OccupancyGrid *;
        // *this->pp_local_costmap = p_local_costmap;
        this->global_costmap = new unsigned char[this->ns_];

        initMap();
    }

    void DStar::initMap()
    {
        this->DNodeMap = new DNodePtr *[this->nx_];
        for (int i = 0; i < this->nx_; i++)
        {
            this->DNodeMap[i] = new DNodePtr[this->ny_];
            for (int j = 0; j < this->ny_; j++)
                this->DNodeMap[i][j] = new DNode(i, j, inf, inf, this->grid2Index(i, j), -1, NEW, inf);
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
        // return std::sqrt(std::pow(n1->x - n2->x, 2) + std::pow(n1->y - n2->y, 2));
        return hypot(n1->x - n2->x, n1->y - n2->y);
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

    // void DStar::updateMap()
    // {
    //     nav_msgs::OccupancyGrid *p_local_costmap = *this->pp_local_costmap;
    //     int x_l = (int)p_local_costmap->info.origin.position.x;
    //     int y_l = (int)p_local_costmap->info.origin.position.y;
    //     int height_l = (int)p_local_costmap->info.height;
    //     int width_l = (int)p_local_costmap->info.width;
    //     int thresh = 90;

    //     int min_x = std::min(std::max(0, x_l - height_l / 2), this->nx_);
    //     int min_y = std::min(std::max(0, y_l - height_l / 2), this->ny_);
    //     int max_x = std::min(std::max(0, x_l + height_l / 2), this->nx_);
    //     int max_y = std::min(std::max(0, y_l + height_l / 2), this->ny_);

    //     int min_id = this->grid2Index(min_x, min_y);
    //     int max_id = this->grid2Index(max_x, max_y);

    //     auto local_costmap = p_local_costmap->data; // [0, 100]
    //     for (int i = min_id, j = 0; i < max_id && j < height_l * width_l; i++, j++)
    //         this->global_costmap[i] = local_costmap[j] < thresh ? 0 : this->lethal_cost_;
    // }

    Node DStar::getState(const Node &current)
    {
        Node state(this->path[0].x, this->path[0].y);
        int dis_min = std::abs(state.x - current.x) + std::abs(state.y - current.y);
        int idx_min = 0;
        for (int i = 1; i < this->path.size(); i++)
        {
            int dis = std::abs(this->path[i].x - current.x) + std::abs(this->path[i].y - current.y);
            if (dis < dis_min)
            {
                dis_min = dis;
                idx_min = i;
                state.x = path[i].x;
                state.y = path[i].y;
            }
        }
        // // delete travelled nodes
        // this->path.erase(this->path.begin() + idx_min, this->path.end());
        return state;
    }

    void DStar::modify(DNodePtr x, DNodePtr y)
    {
        if (x->tag == CLOSED)
            this->insert(x, y->cost + this->cost(x, y));

        while (1)
        {
            double k_min = this->processState();
            if (k_min >= x->cost)
                break;
        }
    }

    std::tuple<bool, std::vector<Node>> DStar::plan(const unsigned char *costs, const Node &start, const Node &goal, std::vector<Node> &expand)
    {
        // ROS_INFO("Get new plan.");
        // update costmap
        memcpy(this->global_costmap, costs, this->ns_);

        // DNodePtr sPtr = this->DNodeMap[start.x][start.y];
        // DNodePtr gPtr = this->DNodeMap[goal.x][goal.y];

        if (this->goal_.x != goal.x || this->goal_.y != goal.y) // (!this->init_plan)
        {
            this->reset();

            DNodePtr sPtr = this->DNodeMap[start.x][start.y];
            DNodePtr gPtr = this->DNodeMap[goal.x][goal.y];

            // this->init_plan = true;
            this->goal_ = goal;

            this->insert(gPtr, 0);

            while (1)
            {
                processState();
                if (sPtr->tag == CLOSED)
                    break;
            }

            this->path.clear();
            this->extractPath(start, goal);

            expand.clear();
            this->extractExpand(expand);
            return {true, this->path};
        }
        else
        {
            // update map: from local to global
            // .... global costmap will update the obstacles...
            // maybe no use to update from local costmap...
            // this->updateMap();

            // get current state from path, argmin distance
            Node state = this->getState(start);
            DNodePtr x = this->DNodeMap[state.x][state.y];
            DNodePtr y;

            // walk forward N points, once collision, modify
            int i = 0;
            int N = 10;
            while (i < N && x->pid != -1)
            {
                i++;
                int x_val, y_val;
                this->index2Grid(x->pid, x_val, y_val);
                y = this->DNodeMap[x_val][y_val];
                if (isCollision(x, y))
                {
                    this->modify(x, y);
                    continue;
                }
                x = y;
            }

            this->path.clear();
            this->extractPath(state, goal);

            expand.clear();
            this->extractExpand(expand);
            return {true, this->path};
        }
    }
}
