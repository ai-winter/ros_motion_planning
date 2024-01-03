/***********************************************************
 *
 * @file: optimizer.cpp
 * @breif: Trajectory optimization
 * @author: Yang Haodong
 * @update: 2023-12-29
 * @version: 1.0
 *
 * Copyright (c) 2023, Yang Haodong
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#include "optimizer.h"

namespace trajectory_optimization
{
Optimizer::Optimizer(int max_iter) : max_iter_(max_iter)
{
}

void Optimizer::setVoronoiDiagram(DynamicVoronoi& voronoi)
{
  voronoi_ = voronoi;
}
}  // namespace trajectory_optimization