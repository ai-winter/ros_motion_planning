/***********************************************************
 *
 * @file: trajectoryGeneration.cpp
 * @breif: Contains trajectory generation class
 * @author: Jing Zongxin
 * @update: 2023-12-4
 * @version: 1.0
 *
 * Copyright (c) 2023，Jing Zongxin
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/

#include <iostream>
#include <cmath>
#include "trajectoryGeneration.h"

trajectoryGeneration::trajectoryGeneration()
{
}

trajectoryGeneration::~trajectoryGeneration()
{
}

int trajectoryGeneration::splineOrder = 3;  //Set the order of the B-spline curve

//Generate B-spline curve control points
void trajectoryGeneration::GenerateControlPoints(const std::pair<double, double>& start_d,
                    const std::pair<double, double>& goal_d,
                    const std::vector<std::pair<int, int>>& initial_position,
                    std::vector<std::pair<double, double>>& initial_point) 
{
    initial_point.clear();  // Clear the vector to start fresh
    initial_point.push_back(start_d);
    std::pair<double, double> last_point = start_d;

    for (const auto& position : initial_position) 
    {
        double x = static_cast<double>(position.first);
        double y = static_cast<double>(position.second);
        // Check if the current point is different from the last point
        if (x != last_point.first || y != last_point.second) 
        {
            initial_point.emplace_back(x, y);
            last_point = {x, y};
        }
    }
    // Check if the goal point is different from the last point
    if (goal_d != last_point) 
    {
        initial_point.push_back(goal_d);
    }
}

// Use De Boor-Cox recursion to calculate Bik(u)
double trajectoryGeneration::BaseFun(int i, int k, double u, std::vector<double> NodeVector)
{
    // 1st order B-spline
    if (k == 0)
    {
        if ((u >= NodeVector[i]) && (u < NodeVector[i + 1]))
        {
            return 1.0;
        }
        else
        {
            return 0;
        }
    }
    // 2nd order and higher B-spline
    else
    {
        double Length1 = double(NodeVector[i + k]) - NodeVector[i];
        double Length2 = double(NodeVector[i + k + 1]) - NodeVector[i + 1];

        // Handle the case where the denominator is 0 by replacing it with 1, defining 0/0 as 0
        if (Length1 == 0)
        {
            Length1 = 1.0;
        }
        if (Length2 == 0)
        {
            Length2 = 1.0;
        }

        return ((double((u - NodeVector[i])) / Length1) * BaseFun(i, k - 1, u, NodeVector) +
                (double((NodeVector[i + k + 1] - u)) / Length2) * BaseFun(i + 1, k - 1, u, NodeVector));
    }
}

// B-spline curve for trajectory smoothing, where K is the order
void trajectoryGeneration::B_spline_curve(std::vector<std::pair<double, double>> &plan, int k)
{
    // Rough estimate of the path length
    double plan_length = 0;

    for (int i = 1; i < plan.size(); i++)
    {
        plan_length = plan_length + std::sqrt((plan[i].first - plan[i - 1].first) * (plan[i].first - plan[i - 1].first) +
                                              (plan[i].second - plan[i - 1].second) * (plan[i].second - plan[i - 1].second));
    }

    double d;
    std::pair<double, double> new_control_point;
    // Extrapolate control points at the starting point
    d = std::sqrt((plan[1].first - plan[0].first) * (plan[1].first - plan[0].first) +
                  (plan[1].second - plan[0].second) * (plan[1].second - plan[0].second));
    new_control_point.first = plan[0].first - (0.1 / d) * (plan[1].first - plan[0].first);
    new_control_point.second = plan[0].second - (0.1 / d) * (plan[1].second - plan[0].second);
    plan.insert(plan.begin(), new_control_point);
    // Get the current number of control points
    int n = plan.size();
    // Extrapolate control points at the end point
    d = std::sqrt((plan[n - 2].first - plan[n - 1].first) * (plan[n - 2].first - plan[n - 1].first) +
                  (plan[n - 2].second - plan[n - 1].second) * (plan[n - 2].second - plan[n - 1].second));
    new_control_point.first = plan[n - 1].first - (0.1 / d) * (plan[n - 2].first - plan[n - 1].first);
    new_control_point.second = plan[n - 1].second - (0.1 / d) * (plan[n - 2].second - plan[n - 1].second);
    plan.push_back(new_control_point);
    // Update the number of control points
    n = plan.size();

    // Set up the node vector
    std::vector<double> NodeVector;

    for (int i = 0; i < k + 1; i++)
    {
        NodeVector.push_back(0.0);
    }

    for (int i = 1; i <= (n - k - 1); i++)
    {
        NodeVector.push_back(double(i) / (n - k));
    }

    for (int i = 0; i < k + 1; i++)
    {
        NodeVector.push_back(1.0);
    }

    // Set up u
    std::vector<double> u;

    double temp_u = 0.0;
    double end_u = 1.0;

    // Calculate the interval of temp_u based on the rough estimate of the path length and 0.05m precision
    double temp_dt = 1 / plan_length;

    while (temp_u < end_u)
    {
        u.push_back(temp_u);
        temp_u = temp_u + temp_dt;
    }

    // Initialize Bik
    std::vector<double> Bik;
    for (int i = 0; i < n; i++)
    {
        Bik.push_back(0);
    }

    // Initialize B-spline curve
    std::vector<std::pair<double, double>> B_plan;
    // Clever use of vector swap to release memory; the principle is to define a new empty container and then swap the contents of the two containers
    std::vector<std::pair<double, double>>().swap(B_plan);

    for (int i = 0; i < u.size(); i++)
    {
        double B_plan_x = 0;
        double B_plan_y = 0;
        std::pair<double, double> temp_B_plan;

        for (int j = 0; j < n; j++)
        {
            Bik[j] = BaseFun(j, k - 1, u[i], NodeVector);
        }

        for (int m = 0; m < n; m++)
        {
            B_plan_x = B_plan_x + plan[m].first * Bik[m];
            B_plan_y = B_plan_y + plan[m].second * Bik[m];
        }
        temp_B_plan.first = B_plan_x;
        temp_B_plan.second = B_plan_y;
        B_plan.push_back(temp_B_plan);
    }

    // std::cout << B_plan.size()<<std::endl;

    // Return the B-spline curve
    plan.swap(B_plan);
}

// Calculate the Euclidean distance between two points
double trajectoryGeneration::calculateDistance(const std::pair<double, double>& point1, const std::pair<double, double>& point2) 
{
    double dx = point1.first - point2.first;
    double dy = point1.second - point2.second;
    return std::sqrt(dx * dx + dy * dy);
}

//Calculate path length
double trajectoryGeneration::calculatePathLength(const std::vector<std::pair<double, double>>& path) 
{
    double length = 0.0;
    for (size_t i = 1; i < path.size(); ++i) 
    {
        length += calculateDistance(path[i - 1], path[i]);
    }
    return length;
}

