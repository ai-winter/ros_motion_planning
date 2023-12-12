/***********************************************************
 *
 * @file: pso.h
 * @breif: Contains the Particle Swarm Optimization(PSO) planner class
 * @author: Jing Zongxin
 * @update: 2023-12-11
 * @version: 1.0
 *
 * Copyright (c) 2023，Jing Zongxin
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#ifndef PSO_H
#define PSO_H

#include <random>
#include <thread>
#include <mutex>
#include <vector>
#include "global_planner.h"
#include "trajectoryGeneration.h"
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

using PositionSequence = std::vector<std::vector<std::pair<int, int>>>;

namespace global_planner
{
  struct Particle
  {
    std::vector<std::pair<int, int>> position;               // Particle position
    std::vector<std::pair<int, int>> velocity;                           // Particle velocity
    double fitness;                                          // Particle fitness
    std::vector<std::pair<int, int>> personal_best_pos;      // Personal best position in iteration
    double personal_best_fitness;                            // Personal best fitness in iteration

    Particle() = default;  

    Particle(const std::vector<std::pair<int, int>>& initial_position,
            const  std::vector<std::pair<int, int>>& initial_velocity,
            double initial_fitness)
        : position(initial_position),
          velocity(initial_velocity),
          fitness(initial_fitness),
          personal_best_pos(initial_position),
          personal_best_fitness(initial_fitness)
    {
    }
  };

  /**
   * @brief Class for objects that plan using the PSO algorithm
   */

  class PSO : public GlobalPlanner
  {
  public:
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
    PSO(int nx, int ny, double resolution, double origin_x, double origin_y, int n_particles,int n_inherited, int pointNum , double w_inertial, double w_social, double w_cognitive, int max_speed,int initposmode ,bool pub_particles,int max_iter);
    ~PSO();

    /**
     * @brief PSO implementation
     * @param global_costmap global costmap
     * @param start         start node
     * @param goal          goal node
     * @param path          optimal path consists of Node
     * @param expand        containing the node been search during the process(unused)
     * @return  true if path found, else false
     */
    bool plan(const unsigned char* global_costmap, const Node& start, const Node& goal, std::vector<Node>& path,
              std::vector<Node>& expand);

    /**
     * @brief Generate n particles with pointNum_ positions each within the map range
     * @param initialPositions The initial position sequence of particle swarm
     * @param start_d        starting point
     * @param goal_d         Target point
     */
    void generateRandomInitialPositions(PositionSequence &initialPositions,const std::pair<double, double> start_d,const std::pair<double, double> goal_d);

    /**
     * @brief Generate an initial position point sequence within a circular area, located within the map range
     * @param initialPositions The initial position sequence of particle swarm
     * @param start_d        starting point
     * @param goal_d         Target point
     */
    void generateCircularInitialPositions(PositionSequence &initialPositions,const std::pair<double, double> start_d,const std::pair<double, double> goal_d);

    /**
     * @brief Calculate Obstacle avoidance cost
     * @param global_costmap   global costmap
     * @param pso_path         Path to perform collision detection
     * @return  The collision cost of the path
     */
    double ObstacleCost(const unsigned char* global_costmap,const std::vector<std::pair<double, double>>& pso_path);

    /**
     * @brief A function to update the particle velocity
     * @param particle     Particles to be updated for velocity
     * @param global_best  Global optimal particle
     * @param gen          randomizer
     */
    void updateParticleVelocity(Particle& particle,const Particle& global_best,std::mt19937& gen); 

    /**
     * @brief A function to update the particle position
     * @param particle     Particles to be updated for velocity
     */
    void updateParticlePosition(Particle& particle);

    
    /**
     * @brief Particle update optimization iteration
     * @param particle       Particles to be updated for velocity
     * @param best_particle  Global optimal particle
     * @param start_d        starting point
     * @param goal_d         Target point
     * @param index_i        Particle ID
     * @param global_costmap global costmap
     * @param gen            randomizer
     */
    void optimizeParticle(Particle& particle, Particle& best_particle, const unsigned char* global_costmap, 
                               const std::pair<double, double>& start_d, const std::pair<double, double>& goal_d,
                               const int& index_i,std::mt19937& gen) ;
    
    /**
     * @brief Clamps a value within a specified range.
     * @tparam T             The type of the values to be clamped.
     * @param value          The value to be clamped.
     * @param low            The lower bound of the range.
     * @param high           The upper bound of the range.
     * @return const T&      The clamped value within the specified range.
     */
    template <typename T>
    const T& clamp(const T& value, const T& low, const T& high) {return std::max(low, std::min(value, high));}

    /**
     * @brief Custom comparison function for ascending order.
     * @param a   The first element to be compared.
     * @param b   The second element to be compared.
     * @return bool  True if 'a' is less than 'b', indicating ascending order; otherwise, false.
     */
    static bool ascendingOrder(int a, int b) { return a < b;}

    /**
     * @brief Custom comparison function for descending order.
     * @param a   The first element to be compared.
     * @param b   The second element to be compared.
     * @return bool  True if 'a' is greater than 'b', indicating descending order; otherwise, false.
     */
    static bool descendingOrder(int a, int b){ return a > b;}


    /**
     * @brief Publishes particle markers based on given positions and index.
     * @param positions   Vector of pairs representing positions (x, y) of particles.
     * @param index       Index identifier for the particles.
     */
    void publishParticleMarkers(const std::vector<std::pair<int, int>>& positions, const int& index);


  protected:
    double origin_x_,origin_y_;   // origin coordinate in the x、y direction.
    bool pub_particles_;          // boolean flag to publish particles.
    int max_iter_;                // maximum iterations
    int n_particles_;             // number of particles
    int n_inherited_;             // number of inherited particles
    int pointNum_;                // number of position points contained in each particle
    double w_inertial_, w_social_, w_cognitive_;   // Weight coefficients for fitness calculation: inertia weight, social weight, cognitive weight
    int max_speed_;               // The maximum velocity of particle motion
    int initposmode_;             // Set the generation mode for the initial position points of the particle swarm
    
  private:
    ros::Publisher particle_pub;                    //The publisher of real-time particle visualization 
    int GlobalBest_particle_;                       //The ID of the globally optimal particle
    std::mutex particles_lock_;                     //thread lock
    std::vector<Particle> inherited_particles_;     //inherited particles
    trajectoryGeneration path_generation;           //Path generation
  };

}  // namespace global_planner
#endif
