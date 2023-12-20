/***********************************************************
 *
 * @file: ga.h
 * @breif: Contains the Genetic Algorithm(GA) planner class
 * @author: Jing Zongxin
 * @update: 2023-12-15
 * @version: 1.0
 *
 * Copyright (c) 2023，Jing Zongxin
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#ifndef GA_H_
#define GA_H_

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
  struct Genets
  {
    std::vector<std::pair<int, int>> position;               // genets position
    double fitness;                                          // genets fitness
    std::vector<std::pair<int, int>> personal_best_pos;      // Personal best position in iteration
    double personal_best_fitness;                            // Personal best fitness in iteration

    Genets() = default;  

    Genets(const std::vector<std::pair<int, int>>& initial_position,
            double initial_fitness)
        : position(initial_position),
          fitness(initial_fitness),
          personal_best_pos(initial_position),
          personal_best_fitness(initial_fitness)
    {
    }
  };

  /**
   * @brief Class for objects that plan using the GA algorithm
   */

  class GA : public GlobalPlanner
  {
  public:
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
    GA(int nx, int ny, double resolution, double origin_x, double origin_y, int n_genets,int n_inherited, int chromLength , double p_select, double p_crs, double p_mut, int max_speed,int initposmode ,bool pub_genets,int max_iter);
    ~GA();

    /**
     * @brief GA implementation
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
     * @brief Generate n genets with pointNum_ positions each within the map range
     * @param initialPositions The initial position sequence of genets swarm
     * @param start_d        starting point
     * @param goal_d         Target point
     */
    void generateRandomInitialPositions(PositionSequence &initialPositions,const std::pair<double, double> start_d,const std::pair<double, double> goal_d);

    /**
     * @brief Generate an initial position point sequence within a circular area, located within the map range
     * @param initialPositions The initial position sequence of genets swarm
     * @param start_d        starting point
     * @param goal_d         Target point
     */
    void generateCircularInitialPositions(PositionSequence &initialPositions,const std::pair<double, double> start_d,const std::pair<double, double> goal_d);

    /**
     * @brief Calculate Obstacle avoidance cost
     * @param global_costmap   global costmap
     * @param ga_path         Path to perform collision detection
     * @return  The collision cost of the path
     */
    double ObstacleCost(const unsigned char* global_costmap,const std::vector<std::pair<double, double>>& ga_path);

    
    /**
     * @brief Perform selection using roulette wheel method.
     * @param population        The population of Genets.
     * @param p_select          The selection probability.
     * @param selectedPopulation The selected individuals will be stored in this vector.
     */
    void ga_select(const std::vector<Genets>& population, double p_select, std::vector<Genets>& selectedPopulation);
    
    /**
     * @brief Select individuals to be retained based on their fitness level
     * @param population        The population of Genets.
     * @param p_select          The selection probability.
     * @param selectedPopulation The selected individuals will be stored in this vector.
     */
    void ga_select_optimal(const std::vector<Genets>& population, double p_select, std::vector<Genets>& selectedPopulation);
    
    /**
     * @brief Genets update optimization iteration
     * @param Genets_p       individuals selected for retention
     * @param Genets_c       descendants of selected individuals to be retained
     * @param Best_genets    Global optimal genets
     * @param start_d        starting point
     * @param goal_d         Target point
     * @param index_i        genets ID
     * @param global_costmap global costmap
     * @param gen            randomizer
     */
     void optimizeGenets(const Genets& Genets_p, Genets& Genets_c, Genets& Best_genets, 
                         const unsigned char* global_costmap, const std::pair<double, double>& start_d, 
                         const std::pair<double, double>& goal_d,const int& index_i,std::mt19937& gen);
    
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
     * @brief Publishes genets markers based on given positions and index.
     * @param positions   Vector of pairs representing positions (x, y) of genets.
     * @param index       Index identifier for the genets.
     */
    void publishGenetsMarkers(const std::vector<std::pair<int, int>>& positions, const int& index);


  protected:
    double origin_x_,origin_y_;   // origin coordinate in the x、y direction.
    bool pub_genets_;             // boolean flag to publish genets.
    int max_iter_;                // maximum iterations
    int n_genets_;                // number of genets
    int n_inherited_;             // number of inherited genets
    int chromLength_;             // number of position points contained in each genets
    bool select_mode_;            // Selection process settings
    double p_select_, p_crs_, p_mut_;   // selection probability  crossover probability  mutation probability
    int max_speed_;               // The maximum velocity of genets motion
    int initposmode_;             // Set the generation mode for the initial position points of the genets swarm
    
  private:
    ros::Publisher genets_pub_;                    //The publisher of real-time genets visualization 
    int GlobalBest_genets_;                        //The ID of the globally optimal genets
    std::mutex genets_lock_;                       //thread lock
    std::vector<Genets> inherited_genets_;         //inherited genets
    trajectoryGeneration path_generation;          //Path generation
  };

}  // namespace global_planner
#endif
