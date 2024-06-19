/***********************************************************************/
/**                                                                    */
/** map.hpp                                                            */
/**                                                                    */
/** Copyright (c) 2016, Service Robotics Lab.                          */
/**                     http://robotics.upo.es                         */
/**                                                                    */
/** All rights reserved.                                               */
/**                                                                    */
/** Authors:                                                           */
/** Ignacio Perez-Hurtado (maintainer)                                 */
/** Jesus Capitan                                                      */
/** Fernando Caballero                                                 */
/** Luis Merino                                                        */
/**                                                                    */
/** This software may be modified and distributed under the terms      */
/** of the BSD license. See the LICENSE file for details.              */
/**                                                                    */
/** http://www.opensource.org/licenses/BSD-3-Clause                    */
/**                                                                    */
/***********************************************************************/

#ifndef _MAP_HPP_
#define _MAP_HPP_

#include "vector2d.hpp"

namespace sfm
{
class Map
{
public:
  struct Obstacle
  {
    Obstacle() : distance(-1)
    {
    }
    double distance;
    utils::Vector2d position;
  };

  Map()
  {
  }
  virtual ~Map()
  {
  }
  virtual const Obstacle& getNearestObstacle(const utils::Vector2d& x) = 0;
  virtual bool isObstacle(const utils::Vector2d& x) const = 0;
};
}  // namespace sfm

#endif
