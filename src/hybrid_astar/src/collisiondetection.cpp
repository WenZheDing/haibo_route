#include "collisiondetection.h"

using namespace HybridAStar;

CollisionDetection::CollisionDetection()
{
//   this->grid = nullptr;
  Lookup::collisionLookup(collisionLookup);
}

bool CollisionDetection::configurationTest(float x, float y, float t)
{
  //--Mapping debug--
  //int X = (int)x;
  //int Y = (int)y;

  int X = (int)((x - map_origin_x) / map_resolution);
  int Y = (int)((y - map_origin_y) / map_resolution);
  //--Mapping debug--
  //int iX = (int)((x - (long)x) * Constants::positionResolution);
  //iX iY 0-9的整数 iT 0-72
  int iX = 0;
  if (x >= 0)
  {
    iX = (int)((x - (long)x) * Constants::positionResolution);
  }
  else
  {
    iX = (int)((1 + (x - (long)x)) * Constants::positionResolution);
  }

  iX = iX > 0 ? iX : 0;

  int iY = 0;
  if (y >= 0)
  {
    iY = (int)((y - (long)y) * Constants::positionResolution);
  }
  else
  {
    iY = (int)((1 + (y - (long)y)) * Constants::positionResolution);
  }

  iY = iY > 0 ? iY : 0;
  int iT = (int)(t / Constants::deltaHeadingRad);
  //std::cout<<"iX:"<<iX<<'\t'<<"iY:"<<iY<<'\t'<<"iT:"<<iT<<'\t'<<std::endl;
  int idx = iY * Constants::positionResolution * Constants::headings + iX * Constants::headings + iT;
  // std::cout<<"idx: \t"<<idx<<'\t'<<sizeof(collisionLookup)/sizeof(collisionLookup[0])<<'\t';
  // std::cout<<"collisionLookup[idx].length \t"<<collisionLookup[idx].length<<std::endl;
  int cX;
  int cY;
  // std::cout << "debug" << std::endl;
  // std::cout << "some of collision lookup " << std::endl;
  // int count = 0;
  // for (int id = 0; id < 7200; id++)
  // {
  //   if (collisionLookup[id].length == 0)
  //   {
  //     count++;
  //     std::cout<<"collisionLookup["<< id<<"].length \t"<<collisionLookup[idx].length<<std::endl;
  //   }
  // }
  // std::cout << "count:" << count << std::endl;
  for (int i = 0; i < collisionLookup[idx].length; ++i)
  {
    cX = (X + collisionLookup[idx].pos[i].x);
    cY = (Y + collisionLookup[idx].pos[i].y);
    // std::cout<<"cX: "<<cX<<std::endl;
    // std::cout<<"cY: "<<cY<<std::endl;
    // std::cout<<cY * grid->info.width + cX<<" "<< grid->info.width *  grid->info.height<<std::endl;
    // make sure the configuration coordinates are actually on the grid
    if (cX >= 0 && (unsigned int)cX < grid.info.width && cY >= 0 && (unsigned int)cY < grid.info.height)
    {
      if (grid.data[cY * grid.info.width + cX])
      {
        //std::cout << "debug!" << std::endl;
        return false;
      }
    }
  }
  //std::cout << "debug!" << std::endl;
  return true;
}
