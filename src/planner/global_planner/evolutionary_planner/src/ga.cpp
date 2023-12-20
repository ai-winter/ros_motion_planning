/***********************************************************
 *
 * @file: ga.cpp
 * @breif: Contains the Genetic Algorithm(GA) planner class
 * @author: Jing Zongxin
 * @update: 2023-12-15
 * @version: 1.0
 *
 * Copyright (c) 2023， Jing Zongxin
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <iostream>
#include <cmath>
#include "ga.h"

namespace global_planner
{
  /**
   * @brief Construct a new GA object
   * @param nx            pixel number in costmap x direction
   * @param ny            pixel number in costmap y direction
   * @param resolution    costmap resolution
   * @param origin_x      origin coordinate in the x direction.
   * @param origin_y      origin coordinate in the y direction.
   * @param n_genets	    number of genets
   * @param n_inherited   number of inherited genets
   * @param chromLength   number of position points contained in each genets
   * @param p_select	    selection probability
   * @param p_crs		      crossover probability
   * @param p_mut	        mutation probability
   * @param max_speed		  The maximum movement speed of genets
   * @param initposmode	  Set the generation mode for the initial position points of the genets swarm
   * @param pub_genets    Boolean flag to publish genets.
   * @param max_iter		  maximum iterations
   */
  GA::GA(int nx, int ny, double resolution, double origin_x, double origin_y, int n_genets,int n_inherited, int chromLength , double p_select, double p_crs, double p_mut, int max_speed,int initposmode ,bool pub_genets,int max_iter)
    : GlobalPlanner(nx, ny, resolution)
    , origin_x_(origin_x)
    , origin_y_(origin_y)
    , n_genets_(n_genets)
    , n_inherited_(n_inherited)
    , chromLength_(chromLength)
    , p_select_(p_select)
    , p_crs_(p_crs)
    , p_mut_(p_mut)
    , max_speed_(max_speed)
    , initposmode_(initposmode)
    , pub_genets_(pub_genets)
    , max_iter_(max_iter)
  {
     inherited_genets_.emplace_back(std::vector<std::pair<int, int>>(chromLength, std::make_pair(1, 1)), 0.0);
    // Initialize ROS publisher
    ros::NodeHandle nh;
    genets_pub_ = nh.advertise<visualization_msgs::Marker>("genets_swarm_markers", 10);
  }

  GA::~GA()
  {
  }

  /**
   * @brief GA implementation
   * @param global_costmap global costmap
   * @param start         start node
   * @param goal          goal node
   * @param path          optimal path consists of Node
   * @param expand        containing the node been search during the process(unused)
   * @return  true if path found, else false
   */
  bool GA::plan(const unsigned char* global_costmap, const Node& start, const Node& goal, std::vector<Node>& path,
                std::vector<Node>& expand)
  {
   
    if ((n_genets_ <= 0) || (n_genets_ % 2 != 0))
    {
      std::cout<<" GA : The parameter n_genets is set improperly. Please ensure that it is a positive even number."<<std::endl;
      return false;
    }

    std::cout<<" GA planning started..."<<std::endl;

    // variable initialization
    double pathLength;
    double initial_fitness;
    double obstacle_cost;
    Genets Best_genets;
    PositionSequence initialPositions;
    std::vector<Genets> genets_swarm;
    std::vector<Genets> genets_parent;
    std::vector<Genets> genets_children;
    std::vector<std::pair<double, double>> initial_point;
    std::pair<double, double> start_d(static_cast<double>(start.x_), static_cast<double>(start.y_));
    std::pair<double, double>  goal_d(static_cast<double>(goal.x_), static_cast<double>(goal.y_));

    if(static_cast<int>(100*p_select_) % 2 == 0){select_mode_=0;}
    else{select_mode_=1;}
   
    //Generate initial position of genets swarm
    if(initposmode_==1){generateRandomInitialPositions(initialPositions,start_d,goal_d);}
    else{generateCircularInitialPositions(initialPositions,start_d,goal_d);}
    
    std::cout<<"GA:  Successfully generated initial position of genets swarm"<<std::endl;

    // genets initialization
    for (int i = 0; i < n_genets_; ++i)
    {
      std::vector<std::pair<int, int>> initial_position;
      
      if ((i<n_inherited_)&&(inherited_genets_.size()==n_inherited_))
      {
        initial_position=inherited_genets_[i].personal_best_pos;
      }
      else
      {
        initial_position = initialPositions[i];
      }
      
      //Generate B-spline curve control points
      path_generation.GenerateControlPoints(start_d,goal_d,initial_position,initial_point);
      //Generate B-spline curves    
      path_generation.B_spline_curve(initial_point, path_generation.splineOrder);
      //Calculate path length
      pathLength = path_generation.calculatePathLength(initial_point);
      //collision detection
      obstacle_cost=ObstacleCost(global_costmap,initial_point);
      //Calculate genets fitness
      initial_fitness = 100000.0 / (pathLength + 1000*obstacle_cost);

      if ((i==0)||(initial_fitness>Best_genets.fitness))
      {
        GlobalBest_genets_=i;
        Best_genets.fitness=initial_fitness;
      } 
      //Create and add genets objects to containers
      genets_swarm.emplace_back(initial_position, initial_fitness);
       
    }

    Best_genets.position=genets_swarm[GlobalBest_genets_].position;

    std::cout<<"GA:  Successfully generated initial genets swarm"<<std::endl;

    //random data
    std::random_device rd;
    std::mt19937 gen(rd());

    std::cout<<"GA: genets swarm iteration progress : "<<std::endl;

    // Iterative optimization
    for (size_t iter = 0; iter < max_iter_; iter++)
    {
      std::cout<<"GA"<<iter<<" ";

      if(select_mode_){ga_select(genets_swarm, 0.5 ,genets_parent);}
      else {ga_select_optimal(genets_swarm, 0.5 ,genets_parent);}

      genets_children=genets_parent;
      std::rotate(genets_children.begin(), genets_children.begin() + 1, genets_children.end()); //Use std:: rotate to move the first element to the end

      std::vector<std::thread> genets_list = std::vector<std::thread>(n_genets_/2);
      for (size_t i = 0; i < n_genets_/2; ++i)
        genets_list[i] = std::thread(&GA::optimizeGenets, this, std::cref(genets_parent[i]), std::ref(genets_children[i]), std::ref(Best_genets), std::cref(global_costmap), std::cref(start_d), std::cref(goal_d), i, std::ref(gen));
      for (size_t i = 0; i < n_genets_/2; ++i)
        genets_list[i].join();
      
       // Copy the elements from genets_parent and genets_children to genets_swarm
      std::copy(genets_children.begin(), genets_children.end(), genets_swarm.begin());
      std::copy(genets_parent.begin(), genets_parent.end(), genets_swarm.begin() + genets_children.size());

    }

    //Generating Paths from Optimal genets
    path_generation.GenerateControlPoints(start_d,goal_d,Best_genets.position,initial_point);
    path_generation.B_spline_curve(initial_point, path_generation.splineOrder);
   
    std::cout<<"GA: Iteration completed, optimal fitness is: "<<Best_genets.fitness<<std::endl;

    // Path data structure conversion
    path.clear();

    if (!initial_point.empty()) 
    {
      // Add the last point
      path.emplace_back(static_cast<int>(initial_point.back().first), static_cast<int>(initial_point.back().second), 0.0, 0.0, static_cast<int>(initial_point.size()) - 1, 0);

      // Iterate in reverse order, starting from the second-to-last point
      for (int p = initial_point.size() - 2; p >= 0; --p) 
      {
          int x = static_cast<int>(initial_point[p].first);
          int y = static_cast<int>(initial_point[p].second);
          // Check if the current point is different from the last point
          if (x != path.back().x_ || y != path.back().y_) 
          {
            path.emplace_back(x, y, 0.0, 0.0, p, 0);
          }
      }
    }

    //Update inheritance genets based on optimal fitness
    std::sort(genets_swarm.begin(), genets_swarm.end(), [](const Genets& a, const Genets& b) { return a.personal_best_fitness > b.personal_best_fitness;});
    inherited_genets_.clear();

    for (size_t inherit = 0; inherit < n_inherited_; ++inherit)
    {
      inherited_genets_.emplace_back(genets_swarm[inherit]);
    }

    if(!path.empty()){std::cout<<"GA:  Planning Successful ! "<<std::endl;}

    return !path.empty();
  }

  // Generate n genets with chromLength_ positions each within the map range
  void GA::generateRandomInitialPositions(PositionSequence &initialPositions,const std::pair<double, double> start_d,const std::pair<double, double> goal_d)
  {
      // Use a random device and engine to generate random numbers
      std::random_device rd;
      std::mt19937 gen(rd());
      int x[chromLength_], y[chromLength_];
      int point_id;

      //Calculate sequence direction
      bool xorder = (goal_d.first > start_d.first);
      bool yorder = (goal_d.second > start_d.second);

      for (int i = 0; i < n_genets_; ++i)
      {
          std::unordered_set<int> visited;
          std::vector<std::pair<int, int>> genetsPositions;
          point_id=0;
          // Generate pointNum_ unique coordinates
          while (point_id < chromLength_)
          {
              x[point_id] = std::uniform_int_distribution<int>(0, nx_-1)(gen);
              y[point_id] = std::uniform_int_distribution<int>(0, ny_-1)(gen);
              int uniqueId = x[point_id] * (ny_ + 1) + y[point_id];  // Represent coordinates by a unique ID

              // Check if the coordinates have already been used
              if (visited.find(uniqueId) == visited.end())
              {   
                  point_id=point_id+1;
                  visited.insert(uniqueId);
              }
          }

          //sort
          if(xorder){std::sort(x, x + chromLength_, &GA::ascendingOrder);}
          else{std::sort(x, x + chromLength_, &GA::descendingOrder);}

          if(yorder){std::sort(y, y + chromLength_, &GA::ascendingOrder);}
          else{std::sort(y, y + chromLength_, &GA::descendingOrder);}

          // Store elements from x and y in genetsPositions
          for (int ii = 0; ii < chromLength_; ++ii)
          {
              genetsPositions.emplace_back(x[ii], y[ii]);
          }

          initialPositions.push_back(genetsPositions);
      }
  }

  // Generate n genets with pointNum_ positions each within the map range
  void GA::generateCircularInitialPositions(PositionSequence &initialPositions,const std::pair<double, double> start_d,const std::pair<double, double> goal_d)
  {
      // Use a random device and engine to generate random numbers
      std::random_device rd;
      std::mt19937 gen(rd());
      int x[chromLength_], y[chromLength_];
      int point_id;
      //Calculate sequence direction
      bool xorder = (goal_d.first > start_d.first);
      bool yorder = (goal_d.second > start_d.second);
      // Calculate the center of the circle (midpoint between start and goal)
      int centerX = (start_d.first + goal_d.first) / 2;
      int centerY = (start_d.second + goal_d.second) / 2;
      // Calculate the radius of the circle (half of the distance between start and goal)
      double radius = path_generation.calculateDistance(start_d,goal_d) / 2.0;

      if (radius<5){radius=5;}

      for (int i = 0; i < n_genets_; ++i)
      {
          std::unordered_set<int> visited;
          std::vector<std::pair<int, int>> genetsPositions;
          point_id=0;
          // Generate pointNum_ unique coordinates
          while (point_id < chromLength_)
          {  
            // Generate random angle in radians
            double angle = std::uniform_real_distribution<double>(0, 2 * M_PI)(gen);
            // Generate random distance from the center within the circle
            double r = std::sqrt(std::uniform_real_distribution<double>(0, 1)(gen)) * radius;
            // Convert polar coordinates to Cartesian coordinates
            x[point_id] = static_cast<int>(std::round(centerX + r * std::cos(angle)));
            y[point_id] = static_cast<int>(std::round(centerY + r * std::sin(angle)));

            // Check if the coordinates are within the map range
            if (x[point_id] >= 0 && x[point_id] < nx_ && y[point_id] >= 0 && y[point_id] < ny_) 
            {
                int uniqueId = x[point_id] * (ny_ + 1) + y[point_id];
                // Check if the coordinates have already been used
                if (visited.find(uniqueId) == visited.end()) 
                {
                    point_id = point_id + 1;
                    visited.insert(uniqueId);
                }
            }

          }

          //sort
          if(xorder){std::sort(x, x + chromLength_, &GA::ascendingOrder);}
          else{std::sort(x, x + chromLength_, &GA::descendingOrder);}

          if(yorder){std::sort(y, y + chromLength_, &GA::ascendingOrder);}
          else{std::sort(y, y + chromLength_, &GA::descendingOrder);}

          // 将 x 和 y 中的元素存放到 genetsPositions 中
          for (int ii = 0; ii < chromLength_; ++ii)
          {
              genetsPositions.emplace_back(x[ii], y[ii]);
          }

          initialPositions.push_back(genetsPositions);
      }
  }

  //Calculate Obstacle avoidance cost
  double GA::ObstacleCost(const unsigned char* global_costmap,const std::vector<std::pair<double, double>>& ga_path)
  {
    int point_index;
    double Obscost=1;

    for (size_t i = 1; i < ga_path.size(); ++i) 
    {
      point_index=grid2Index(static_cast<int>(ga_path[i].first),  static_cast<int>(ga_path[i].second));
      // next node hit the boundary or obstacle
      if ((point_index < 0) || (point_index >= ns_) || (global_costmap[point_index] >= lethal_cost_ * factor_))
      {
        Obscost=Obscost+1;
      }
    }

    return Obscost;

  }


  // Perform selection using roulette wheel method.
  void GA::ga_select(const std::vector<Genets>& population, double p_select, std::vector<Genets>& selectedPopulation)
  {
        // Calculate the inverse fitness values
        std::vector<double> fit_reverse;
        fit_reverse.reserve(population.size());
        for (const auto& genet : population)
        {
            fit_reverse.push_back(genet.fitness);
        }

        // Calculate the total fitness
        double totalFit = std::accumulate(fit_reverse.begin(), fit_reverse.end(), 0.0);

        // Calculate cumulative probability
        std::vector<double> accP;
        accP.reserve(population.size());
        double sum = 0.0;
        for (const auto& reverseFit : fit_reverse)
        {
            sum += reverseFit / totalFit;
            accP.push_back(sum);
        }

        // Calculate the number of individuals to be selected
        size_t selectNum = static_cast<size_t>(population.size() * p_select);

        // Initialize the selected individuals with the first individual from the population
        selectedPopulation.clear();
        selectedPopulation.push_back(population.front());

        std::vector<size_t> selectedIndices;
        selectedIndices.reserve(selectNum);

          // Set up random number generation
          std::random_device rd;
          std::mt19937 gen(rd());
          std::uniform_real_distribution<double> dis(0.0, 1.0);

        // Execute the selection process
        for (size_t i = 1; i < selectNum; ++i)
        {
              // Generate a random number
              double randNum = dis(gen);
            // Find the cumulative probability greater than the random number
            auto it = std::find_if(accP.begin(), accP.end(), [randNum](double val) { return val > randNum; });

            if (it != accP.end())
            {
                // Get the index of the first cumulative probability greater than the random number
                size_t idx = std::distance(accP.begin(), it);

                // Allow duplicate selection
                  selectedPopulation.push_back(population[idx]);
                  selectedIndices.push_back(idx);
            } 
            else 
            {
              --i; // Retry if not found, i.e., random number is too close to 1.0
            }
        }
  }

  // Select individuals to be retained based on their fitness level
  void GA::ga_select_optimal(const std::vector<Genets>& population, double p_select, std::vector<Genets>& selectedPopulation)
  {
    auto population_c = population;

    std::sort(population_c.begin(), population_c.end(), [](const Genets& a, const Genets& b) { return a.fitness > b.fitness;});
    selectedPopulation.clear();    
    
    // Calculate the number of individuals to be selected
    size_t selectNum = static_cast<size_t>(population_c.size() * p_select);

    for (size_t inherit = 0; inherit < selectNum; ++inherit)
    {
      selectedPopulation.emplace_back(population_c[inherit]);
    }
  }


  // genets update optimization iteration
  void GA::optimizeGenets(const Genets& Genets_p, Genets& Genets_c, Genets& Best_genets, const unsigned char* global_costmap, const std::pair<double, double>& start_d, const std::pair<double, double>& goal_d,const int& index_i,std::mt19937& gen)
  {

    std::vector<std::pair<double, double>> process_path;

    std::uniform_real_distribution<double> distribution(0.0, 1.0);
    double random1 = distribution(gen);
    double random2 = distribution(gen);
 
    if (random1<p_crs_)
    {
      std::uniform_int_distribution<int> distribution3(0, chromLength_ - 1);
      int randomIndex = distribution3(gen);
      // Perform crossover by using std::copy to copy the elements from the i-th position and beyond in Genets_p.position to Genets_c.position.
      std::copy(Genets_p.position.begin() + randomIndex, Genets_p.position.end(), Genets_c.position.begin() + randomIndex);
    }

    if (random2<p_mut_)
    {
      std::uniform_int_distribution<int> distribution4(0, chromLength_ - 1);
      int randomIndex = distribution4(gen);

      for(int ob=0;ob<10;ob++)
      {
        std::uniform_real_distribution<double> distribution2(-1.0, 1.0);
        double random3 = distribution2(gen);
        double random4 = distribution2(gen);
        Genets_c.position[randomIndex].first= Genets_c.position[randomIndex].first+static_cast<int>(random3*max_speed_);
        Genets_c.position[randomIndex].second= Genets_c.position[randomIndex].second+static_cast<int>(random4*max_speed_);

        int point_index=grid2Index(Genets_c.position[randomIndex].first,  Genets_c.position[randomIndex].second);

        if ((point_index >= 0) && (point_index < ns_) && (global_costmap[point_index] < lethal_cost_ * factor_))
        {
          break;
        }
      }
    }
 
    //Generate B-spline curve control points
    path_generation.GenerateControlPoints(start_d,goal_d,Genets_c.position,process_path);
    //Generate B-spline curves    
    path_generation.B_spline_curve(process_path, path_generation.splineOrder);
    //Calculate path length
    double pathLength = path_generation.calculatePathLength(process_path);
    //collision detection
    double obstacle_cost=ObstacleCost(global_costmap,process_path);
    //Calculate genets fitness
    Genets_c.fitness = 100000.0 / (pathLength + 1000*obstacle_cost);

    // Update individual optima
    if (Genets_c.fitness>Genets_c.personal_best_fitness)
    {
      Genets_c.personal_best_fitness=Genets_c.fitness;
      Genets_c.personal_best_pos=Genets_c.position;
    }

    // Publish genets markers
    if(pub_genets_){publishGenetsMarkers(Genets_c.position, index_i);}

    //Update global optimal genets
    genets_lock_.lock();

    if (Genets_c.personal_best_fitness>Best_genets.fitness)
    {
        Best_genets.fitness=Genets_c.personal_best_fitness;
        GlobalBest_genets_=index_i;
        Best_genets.position=Genets_c.personal_best_pos; 
    }

    genets_lock_.unlock();

  }


  void GA::publishGenetsMarkers(const std::vector<std::pair<int, int>>& positions, const int& index) 
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "genets_swarm";
    marker.id = index;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;

    // Convert genets positions to geometry_msgs::Point
    for (const auto& position : positions) {
      geometry_msgs::Point p;
      p.x = origin_x_ + position.first * resolution_;
      p.y = origin_y_ + position.second * resolution_;
      p.z = 0.0;
      marker.points.push_back(p);
    }

    // Set the lifetime of the marker (e.g., 1 second)
    marker.lifetime = ros::Duration(1.0);

    genets_pub_.publish(marker);
  }



}  // namespace global_planner