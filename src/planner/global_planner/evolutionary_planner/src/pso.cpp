/***********************************************************
 *
 * @file: pso.cpp
 * @breif: Contains the Particle Swarm Optimization(PSO) planner class
 * @author: Jing Zongxin
 * @update: 2023-12-11
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
#include "pso.h"


namespace global_planner
{
  /**
   * @brief Construct a new PSO object
   * @param nx            pixel number in costmap x direction
   * @param ny            pixel number in costmap y direction
   * @param resolution    costmap resolution
   * @param origin_x      origin coordinate in the x direction.
   * @param origin_y      origin coordinate in the y direction.
   * @param n_particles	  number of particles
   * @param n_inherited   number of inherited particles
   * @param pointNum      number of position points contained in each particle
   * @param w_inertial	  inertia weight
   * @param w_social		  social weight
   * @param w_cognitive	  cognitive weight
   * @param max_speed		  The maximum movement speed of particles
   * @param initposmode	  Set the generation mode for the initial position points of the particle swarm
   * @param pub_particles Boolean flag to publish particles.
   * @param max_iter		  maximum iterations
   */
  PSO::PSO(int nx, int ny, double resolution, double origin_x, double origin_y,int n_particles,int n_inherited,int pointNum , double w_inertial, double w_social, double w_cognitive, int max_speed, int initposmode, bool pub_particles,int max_iter)
    : GlobalPlanner(nx, ny, resolution)
    , origin_x_(origin_x)
    , origin_y_(origin_y)
    , n_particles_(n_particles)
    , n_inherited_(n_inherited)
    , pointNum_(pointNum)
    , w_inertial_(w_inertial)
    , w_social_(w_social)
    , w_cognitive_(w_cognitive)
    , max_speed_(max_speed)
    , initposmode_(initposmode)
    , pub_particles_(pub_particles)
    , max_iter_(max_iter)
  {
     inherited_particles_.emplace_back(std::vector<std::pair<int, int>>(pointNum, std::make_pair(1, 1)),
                                 std::vector<std::pair<int, int>>(pointNum, std::make_pair(0, 0)),
                                 0.0);
    // Initialize ROS publisher
    ros::NodeHandle nh;
    particle_pub = nh.advertise<visualization_msgs::Marker>("particle_swarm_markers", 10);
  }

  PSO::~PSO()
  {
  }

  /**
   * @brief PSO implementation
   * @param global_costmap global costmap
   * @param start         start node
   * @param goal          goal node
   * @param path          optimal path consists of Node
   * @param expand        containing the node been search during the process(unused)
   * @return  true if path found, else false
   */
  bool PSO::plan(const unsigned char* global_costmap, const Node& start, const Node& goal, std::vector<Node>& path,
                std::vector<Node>& expand)
  {
    std::cout<<" PSO planning started..."<<std::endl;

    // variable initialization
    double pathLength;
    double initial_fitness;
    double obstacle_cost;
    Particle Best_particle;
    PositionSequence initialPositions;
    std::vector<Particle> particles;
    std::vector<std::pair<double, double>> initial_point;
    std::pair<double, double> start_d(static_cast<double>(start.x_), static_cast<double>(start.y_));
    std::pair<double, double>  goal_d(static_cast<double>(goal.x_), static_cast<double>(goal.y_));
   
    //Generate initial position of particle swarm
    if(initposmode_==1){generateRandomInitialPositions(initialPositions,start_d,goal_d);}
    else{generateCircularInitialPositions(initialPositions,start_d,goal_d);}
    
    std::cout<<"PSO:  Successfully generated initial position of particle swarm"<<std::endl;

    // particle initialization
    for (int i = 0; i < n_particles_; ++i)
    {
      std::vector<std::pair<int, int>> initial_position;
      
      if ((i<n_inherited_)&&(inherited_particles_.size()==n_inherited_))
      {
        initial_position=inherited_particles_[i].personal_best_pos;
      }
      else
      {
        initial_position = initialPositions[i];
      }
      
      std::vector<std::pair<int, int>> initial_velocity(pointNum_, std::make_pair(0, 0));
      //Generate B-spline curve control points
      path_generation.GenerateControlPoints(start_d,goal_d,initial_position,initial_point);
      //Generate B-spline curves    
      path_generation.B_spline_curve(initial_point, path_generation.splineOrder);
      //Calculate path length
      pathLength = path_generation.calculatePathLength(initial_point);
      //collision detection
      obstacle_cost=ObstacleCost(global_costmap,initial_point);
      //Calculate particle fitness
      initial_fitness = 100000.0 / (pathLength + 1000*obstacle_cost);

      if ((i==0)||(initial_fitness>Best_particle.fitness))
      {
        GlobalBest_particle_=i;
        Best_particle.fitness=initial_fitness;
      } 
      //Create and add particle objects to containers
      particles.emplace_back(initial_position, initial_velocity, initial_fitness);
       
    }

    Best_particle.position=particles[GlobalBest_particle_].position;

    std::cout<<"PSO:  Successfully generated initial particle swarm"<<std::endl;

    //random data
    std::random_device rd;
    std::mt19937 gen(rd());

    std::cout<<"PSO:Particle swarm iteration progress : "<<std::endl;

    // Iterative optimization
    for (size_t iter = 0; iter < max_iter_; iter++)
    {
      std::cout<<"PSO"<<iter<<" ";

      std::vector<std::thread> particle_list = std::vector<std::thread>(n_particles_);
      for (size_t i = 0; i < n_particles_; ++i)
        particle_list[i] = std::thread(&PSO::optimizeParticle, this, std::ref(particles[i]), std::ref(Best_particle), std::cref(global_costmap), std::cref(start_d), std::cref(goal_d), i, std::ref(gen));
      for (size_t i = 0; i < n_particles_; ++i)
        particle_list[i].join();

      Best_particle.position=particles[GlobalBest_particle_].personal_best_pos; 
    }

    //Generating Paths from Optimal Particles
    path_generation.GenerateControlPoints(start_d,goal_d,Best_particle.position,initial_point);
    path_generation.B_spline_curve(initial_point, path_generation.splineOrder);
   
    std::cout<<"PSO: Iteration completed, optimal fitness is: "<<Best_particle.fitness<<std::endl;

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

    //Update inheritance particles based on optimal fitness
    std::sort(particles.begin(), particles.end(), [](const Particle& a, const Particle& b) { return a.personal_best_fitness > b.personal_best_fitness;});
    inherited_particles_.clear();

    for (size_t inherit = 0; inherit < n_inherited_; ++inherit)
    {
      inherited_particles_.emplace_back(particles[inherit]);
    }

    if(!path.empty()){std::cout<<"PSO:  Planning Successful ! "<<std::endl;}

    return !path.empty();
  }

  // Generate n particles with pointNum_ positions each within the map range
  void PSO::generateRandomInitialPositions(PositionSequence &initialPositions,const std::pair<double, double> start_d,const std::pair<double, double> goal_d)
  {
      // Use a random device and engine to generate random numbers
      std::random_device rd;
      std::mt19937 gen(rd());
      int x[pointNum_], y[pointNum_];
      int point_id;

      //Calculate sequence direction
      bool xorder = (goal_d.first > start_d.first);
      bool yorder = (goal_d.second > start_d.second);

      for (int i = 0; i < n_particles_; ++i)
      {
          std::unordered_set<int> visited;
          std::vector<std::pair<int, int>> particlePositions;
          point_id=0;
          // Generate pointNum_ unique coordinates
          while (point_id < pointNum_)
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
          if(xorder){std::sort(x, x + pointNum_, &PSO::ascendingOrder);}
          else{std::sort(x, x + pointNum_, &PSO::descendingOrder);}

          if(yorder){std::sort(y, y + pointNum_, &PSO::ascendingOrder);}
          else{std::sort(y, y + pointNum_, &PSO::descendingOrder);}

          // Store elements from x and y in particlePositions
          for (int ii = 0; ii < pointNum_; ++ii)
          {
              particlePositions.emplace_back(x[ii], y[ii]);
          }

          initialPositions.push_back(particlePositions);
      }
  }

  // Generate n particles with pointNum_ positions each within the map range
  void PSO::generateCircularInitialPositions(PositionSequence &initialPositions,const std::pair<double, double> start_d,const std::pair<double, double> goal_d)
  {
      // Use a random device and engine to generate random numbers
      std::random_device rd;
      std::mt19937 gen(rd());
      int x[pointNum_], y[pointNum_];
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

      for (int i = 0; i < n_particles_; ++i)
      {
          std::unordered_set<int> visited;
          std::vector<std::pair<int, int>> particlePositions;
          point_id=0;
          // Generate pointNum_ unique coordinates
          while (point_id < pointNum_)
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
          if(xorder){std::sort(x, x + pointNum_, &PSO::ascendingOrder);}
          else{std::sort(x, x + pointNum_, &PSO::descendingOrder);}

          if(yorder){std::sort(y, y + pointNum_, &PSO::ascendingOrder);}
          else{std::sort(y, y + pointNum_, &PSO::descendingOrder);}

          // 将 x 和 y 中的元素存放到 particlePositions 中
          for (int ii = 0; ii < pointNum_; ++ii)
          {
              particlePositions.emplace_back(x[ii], y[ii]);
          }

          initialPositions.push_back(particlePositions);
      }
  }

  //Calculate Obstacle avoidance cost
  double PSO::ObstacleCost(const unsigned char* global_costmap,const std::vector<std::pair<double, double>>& pso_path)
  {
    int point_index;
    double Obscost=1;

    for (size_t i = 1; i < pso_path.size(); ++i) 
    {
      point_index=grid2Index(static_cast<int>(pso_path[i].first),  static_cast<int>(pso_path[i].second));
      // next node hit the boundary or obstacle
      if ((point_index < 0) || (point_index >= ns_) || (global_costmap[point_index] >= lethal_cost_ * factor_))
      {
        Obscost=Obscost+1;
      }
    }

    return Obscost;

  }

  // A function to update the particle velocity
  void PSO::updateParticleVelocity(Particle& particle,const Particle& global_best,std::mt19937& gen) 
  {
      //The random numbers are distributed between [0, 1).
      std::uniform_real_distribution<double> dist(0.0, 1.0);

      // update Velocity
      for (size_t i = 0; i < pointNum_; ++i) 
      {
          double rand1 = dist(gen);
          double rand2 = dist(gen);

          particle.velocity[i].first = static_cast<int>( w_inertial_ * particle.velocity[i].first +
              w_social_ * rand1 * (particle.personal_best_pos[i].first - particle.position[i].first) +
              w_cognitive_ * rand2 * (global_best.position[i].first - particle.position[i].first));

          particle.velocity[i].second = static_cast<int>(w_inertial_ * particle.velocity[i].second +
              w_social_ * rand1 * (particle.personal_best_pos[i].second - particle.position[i].second) +
              w_cognitive_ * rand2 * (global_best.position[i].second - particle.position[i].second));

          // Velocity limit  
          particle.velocity[i].first =clamp(particle.velocity[i].first , -1*max_speed_, max_speed_);
          particle.velocity[i].second=clamp(particle.velocity[i].second, -1*max_speed_, max_speed_);
      }    
  }

  // A function to update the particle position
  void PSO::updateParticlePosition(Particle& particle) 
  {
      // update Position
      for (size_t i = 0; i < pointNum_; ++i) 
      {
          particle.position[i].first  = particle.position[i].first + particle.velocity[i].first;
          particle.position[i].second = particle.position[i].second+ particle.velocity[i].second;

          // Position limit  
          particle.position[i].first =clamp(particle.position[i].first , 1, nx_-1);
          particle.position[i].second=clamp(particle.position[i].second, 1, ny_-1);
      }    
  }

  // Particle update optimization iteration
  void PSO::optimizeParticle(Particle& particle, Particle& best_particle, const unsigned char* global_costmap, const std::pair<double, double>& start_d, const std::pair<double, double>& goal_d,const int& index_i,std::mt19937& gen)
  {

    std::vector<std::pair<double, double>> process_path;

    //update speed
    updateParticleVelocity(particle,best_particle,gen);
    //update position
    updateParticlePosition(particle);
    
    //Generate B-spline curve control points
    path_generation.GenerateControlPoints(start_d,goal_d,particle.position,process_path);
    //Generate B-spline curves    
    path_generation.B_spline_curve(process_path, path_generation.splineOrder);
    //Calculate path length
    double pathLength = path_generation.calculatePathLength(process_path);
    //collision detection
    double obstacle_cost=ObstacleCost(global_costmap,process_path);
    //Calculate particle fitness
    particle.fitness = 100000.0 / (pathLength + 1000*obstacle_cost);

    // Update individual optima
    if (particle.fitness>particle.personal_best_fitness)
    {
      particle.personal_best_fitness=particle.fitness;
      particle.personal_best_pos=particle.position;
    }

    // Publish particle markers
    if(pub_particles_){publishParticleMarkers(particle.position, index_i);}

    //Update global optimal particles
    particles_lock_.lock();

    if (particle.personal_best_fitness>best_particle.fitness)
    {
        best_particle.fitness=particle.personal_best_fitness;
        GlobalBest_particle_=index_i;
    }

    particles_lock_.unlock();

  }


  void PSO::publishParticleMarkers(const std::vector<std::pair<int, int>>& positions, const int& index) 
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "particle_swarm";
    marker.id = index;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.color.r = 1.0;
    marker.color.a = 1.0;

    // Convert particle positions to geometry_msgs::Point
    for (const auto& position : positions) {
      geometry_msgs::Point p;
      p.x = origin_x_ + position.first * resolution_;
      p.y = origin_y_ + position.second * resolution_;
      p.z = 0.0;
      marker.points.push_back(p);
    }

    // Set the lifetime of the marker (e.g., 1 second)
    marker.lifetime = ros::Duration(1.0);

    particle_pub.publish(marker);
  }



}  // namespace global_planner