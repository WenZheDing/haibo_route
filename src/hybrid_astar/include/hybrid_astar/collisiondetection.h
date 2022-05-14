#ifndef COLLISIONDETECTION_H
#define COLLISIONDETECTION_H

#include <nav_msgs/OccupancyGrid.h>

#include "constants.h"
#include "lookup.h"
#include "node2d.h"
#include "node3d.h"

namespace HybridAStar
{
namespace
{
void getConfiguration(const Node2D *node, float &x, float &y, float &t)
{
  x = node->getX();
  y = node->getY();
  // avoid 2D collision checking
  t = 99;
}

void getConfiguration(const Node3D *node, float &x, float &y, float &t)
{
  x = node->getX();
  y = node->getY();
  t = node->getT();
}
} // namespace
/*!
   \brief The CollisionDetection class determines whether a given configuration q of the robot will result in a collision with the environment.

   It is supposed to return a boolean value that returns true for collisions and false in the case of a safe node.
*/
class CollisionDetection
{
public:
  /// Constructor
  CollisionDetection();

  /*!
     \brief evaluates whether the configuration is safe
     \return true if it is traversable, else false
  */
  template <typename T>
  bool isTraversable(const T *node)
  {
    /* Depending on the used collision checking mechanism this needs to be adjusted
       standard: collision checking using the spatial occupancy enumeration
       other: collision checking using the 2d costmap and the navigation stack
    */
    float cost = 0;
    float x;
    float y;
    float t;
    // assign values to the configuration
    getConfiguration(node, x, y, t);
    // 2D collision test
    if (t == 99)
    {
      return !grid.data[node->getIdx()];
    }

    if (true)
    {
      cost = configurationTest(x, y, t) ? 0 : 1;
    }
    else
    {
      cost = configurationCost(x, y, t);
    }

    return cost <= 0;
  }

    template <typename T>
  bool isTraversable(const T *node, const T &start, const T &goal, bool AlpOut)
  {
    float cost = 0;
    float x;
    float y;
    float t;
    // assign values to the configuration
    getConfiguration(node, x, y, t);
    // 2D collision test
  float k = (start.getY() - goal.getY())/(start.getX() - goal.getX());
  float b = start.getY() - k*start.getX();
  float distTostart = sqrt((node->getX()-start.getX())*(node->getX()-start.getX())+(node->getY()-start.getY())*(node->getY()-start.getY()));
  float distTogoal = sqrt((node->getX()-goal.getX())*(node->getX()-goal.getX())+(node->getY()-goal.getY())*(node->getY()-goal.getY()));
  float distThreshshold = 2;

  // if ( AlpOut == false && k*node->getX() + b < node->getY()  && distTogoal > distThreshshold && distTostart > distThreshshold)//ALP IN planning in upper half plane
  // {
  //   std::cout << "WARNNING! ALP IN planning in lower plane" << std::endl;
  //   return false;
  // }

  //   if ( AlpOut == false &&  node->getY() > 181  && distTogoal > distThreshshold && distTostart > distThreshshold)//ALP IN planning in upper half plane
  // {
  //   std::cout << "WARNNING! exceed line" << std::endl;
  //   return false;
  // }

  //   if ( AlpOut == true && k*node->getX() + b > node->getY() &&  distTogoal > distThreshshold && distTostart > distThreshshold)//ALP OUT planning in lower half plane
  // {
  //   std::cout << "WARNNING! ALP OUT planning in lower plane" << std::endl;
  //   return false;
  // }
    if (t == 99)
    {
      return !grid.data[node->getIdx()];
    }

    if (true)
    {
      cost = configurationTest(x, y, t) ? 0 : 1;
    }
    else
    {
      cost = configurationCost(x, y, t);
    }

    return cost <= 0;

  }

  /*!
     \brief Calculates the cost of the robot taking a specific configuration q int the World W
     \param x the x position
     \param y the y position
     \param t the theta angle
     \return the cost of the configuration q of W(q)
     \todo needs to be implemented correctly
  */
  float configurationCost(float x, float y, float t) { return 0; }

  /*!
     \brief Tests whether the configuration q of the robot is in C_free
     \param x the x position
     \param y the y position
     \param t the theta angle
     \return true if it is in C_free, else false
  */
  bool configurationTest(float x, float y, float t);

  /*!
     \brief updates the grid with the world map
  */
  void updateGrid(nav_msgs::OccupancyGrid map)
  {
    grid = map; //--Mapping debug--
    //std::cout << "Current collision lookup is updated" << std::endl;
    Lookup::collisionLookup(collisionLookup);
  }

  void viewGrid()
  {
    for (int i = 0; i < grid.info.height; ++i)
    {
        std::cout << "\n";

        for (int j = 0; j < grid.info.width; ++j)
        {
            if (grid.data[i * grid.info.width + j])
            {
                std::cout << "#";
            }
            else
            {
                std::cout << ".";
            }
        }
    }
  }
  

private:
  /// The occupancy grid
  nav_msgs::OccupancyGrid grid;
  /// The collision lookup table
  Constants::config collisionLookup[Constants::headings * Constants::positions];
};
} // namespace HybridAStar
#endif // COLLISIONDETECTION_H
