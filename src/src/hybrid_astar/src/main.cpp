#include <cstring>
#include <iostream>
#include <ros/ros.h>

#include "constants.h"
#include "planner.h"

template<typename T, typename T1>
void message(const T& msg, T1 val = T1()) {
  if (!val) {
    std::cout << "### " << msg << std::endl;
  } else {
    std::cout << "### " << msg << val << std::endl;
  }
}

//###################################################
//                                               MAIN
//###################################################
/**
   \fn main(int argc, char** argv)
   \brief Starting the program
   \param argc The standard main argument count
   \param argv The standard main argument value
   \return 0
*/
int main(int argc, char** argv) {

  message<string, int>("Hybrid A* Search\nA pathfinding algorithm on grids, by Karl Kurzer");

  message("cell size: ", HybridAStar::Constants::cellSize);

  if (HybridAStar::Constants::manual) {
    message("mode: ", "manual");
  } else {
    message("mode: ", "auto");
  }

  ros::init(argc, argv, "a_star");
  HybridAStar::Planner hy;
  std::cout << "plan started" << std::endl;
//   std::cout << "dy: " << HybridAStar::Node3D::dy[0] <<" "<<HybridAStar::Node3D::dy[1] <<" "<< HybridAStar::Node3D::dy[2] << std::endl;
  hy.plan(); 
  
  ros::spin();
  return 0;
}