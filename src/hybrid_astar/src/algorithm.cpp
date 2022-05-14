#include "algorithm.h"
#include "reedsshepp.h"
#include "dubins.h"
#include <boost/heap/binomial_heap.hpp>
#include <typeinfo>

using namespace HybridAStar;

float aStar(Node2D &start, Node2D &goal, Node2D *nodes2D, int width, int height, CollisionDetection &configurationSpace, Visualize &visualization,  bool AlpOut);
void updateH(Node3D &start, const Node3D &goal, Node2D *nodes2D, float *dubinsLookup, int width, int height, CollisionDetection &configurationSpace, Visualize &visualization, bool AlpOut);
Node3D *reedssheppShot(Node3D &start, const Node3D &goal, CollisionDetection &configurationSpace,int &goalposleft);
Node3D *dubinsShot(Node3D &start, const Node3D &goal, CollisionDetection &configurationSpace);
bool straightLineToDevideMap(const Node3D &start, const Node3D &goal, float &k, float &b);
bool dubinflag = Constants::dubinsShot;

//###################################################
//                                    NODE COMPARISON
//###################################################
/*!
   \brief A structure to sort nodes in a heap structure
*/
struct CompareNodes
{
  /// Sorting 3D nodes by increasing C value - the total estimated cost
  bool operator()(const Node3D *lhs, const Node3D *rhs) const
  {
    return lhs->getC() > rhs->getC();
  }
  /// Sorting 2D nodes by increasing C value - the total estimated cost
  bool operator()(const Node2D *lhs, const Node2D *rhs) const
  {
    return lhs->getC() > rhs->getC();
  }
};

//###################################################
//                                        3D A*
//###################################################
Node3D *Algorithm::hybridAStar(Node3D &start,
                               const Node3D &goal,
                               Node3D *nodes3D,
                               Node2D *nodes2D,
                               int width,
                               int height,
                               CollisionDetection &configurationSpace,
                               float *dubinsLookup,
                               Visualize &visualization,
                               DynamicVoronoi  &voronoiDiagram,
                               bool AlpOut,
                               int goalposleft)
{
  // PREDECESSOR AND SUCCESSOR INDEX
  int iPred, iSucc;
  float newG;
  bool onlyForward = false;
  if(AlpOut)
  {
     onlyForward = true; 
    //  dubinflag = false;
  }
  // Number of possible directions, 3 for forward driving and an additional 3 for reversing
  int dir = onlyForward ? 3 : 6;
   std::cout << "dir  " << dir << std::endl;
  // std::cout << "Dubin flag:  " << dubinflag << std::endl;

  //--DEBUG--
  //dir = 3;
  //std::cout << "dir:" << dir <<std::endl;
  // Number of iterations the algorithm has run for stopping based on Constants::iterations
  int iterations = 0;
 //--TEST--
  //float distToObstacle = INFINITY;
  static int whileTImes = 0;


  // VISUALIZATION DELAY
  ros::Duration d(0.003);
  // OPEN LIST AS BOOST IMPLEMENTATION
  typedef boost::heap::binomial_heap<Node3D *,
                                     boost::heap::compare<CompareNodes>>
      priorityQueue;
  priorityQueue O;
  //check if start or goal will collide
  // std::cout << "check start or goal will collide" << std::endl;
  // std::cout << configurationSpace.isTraversable(&start) << std::endl;
  // std::cout << start.getX()<<" "<<start.getY()<<" "<<start.getT()<< std::endl;
  // std::cout << configurationSpace.configurationTest(goal.getX() - 15 * cos(goal.getT()),goal.getY() - 15 * sin(goal.getT()),goal.getT())<< std::endl;
  // configurationSpace.viewGrid();
  // if (!configurationSpace.isTraversable(&start, start, goal, AlpOut ) || !configurationSpace.configurationTest(goal.getX() - 15 * cos(goal.getT()),goal.getY() - 15 * sin(goal.getT()),goal.getT()))
  // {
  //   // cout << "start or goal will collide" << endl;
  //   //std::cout<<"FALSE WILL COLLIDE:"<<configurationSpace.configurationTest(goal.getX() - 15 * cos(goal.getT()),goal.getY() - 15 * sin(goal.getT()),goal.getT());
  //   return nullptr;
  // }
  // update h value
  updateH(start, goal, nodes2D, dubinsLookup, width, height, configurationSpace, visualization, AlpOut);
  // mark start as open
  start.open();
  // push on priority queue aka open list
  O.push(&start);
  iPred = start.setIdx(width, height);
  nodes3D[iPred] = start;
  // NODE POINTER
  Node3D *nPred;
  Node3D *nSucc;

  // float max = 0.f;
  // continue until O empty
  //ros::Time tWhile = ros::Time::now();
  //std::cout << "while" << std::endl;
  while (!O.empty())
  {

    //    // DEBUG
    //    Node3D* pre = nullptr;
    //    Node3D* succ = nullptr;

    //    std::cout << "\t--->>>" << std::endl;

    //    for (priorityQueue::ordered_iterator it = O.ordered_begin(); it != O.ordered_end(); ++it) {
    //      succ = (*it);
    //      std::cout << "VAL"
    //                << " | C:" << succ->getC()
    //                << " | x:" << succ->getX()
    //                << " | y:" << succ->getY()
    //                << " | t:" << helper::toDeg(succ->getT())
    //                << " | i:" << succ->getIdx()
    //                << " | O:" << succ->isOpen()
    //                << " | pred:" << succ->getPred()
    //                << std::endl;

    //      if (pre != nullptr) {

    //        if (pre->getC() > succ->getC()) {
    //          std::cout << "PRE"
    //                    << " | C:" << pre->getC()
    //                    << " | x:" << pre->getX()
    //                    << " | y:" << pre->getY()
    //                    << " | t:" << helper::toDeg(pre->getT())
    //                    << " | i:" << pre->getIdx()
    //                    << " | O:" << pre->isOpen()
    //                    << " | pred:" << pre->getPred()
    //                    << std::endl;
    //          std::cout << "SCC"
    //                    << " | C:" << succ->getC()
    //                    << " | x:" << succ->getX()
    //                    << " | y:" << succ->getY()
    //                    << " | t:" << helper::toDeg(succ->getT())
    //                    << " | i:" << succ->getIdx()
    //                    << " | O:" << succ->isOpen()
    //                    << " | pred:" << succ->getPred()
    //                    << std::endl;

    //          if (pre->getC() - succ->getC() > max) {
    //            max = pre->getC() - succ->getC();
    //          }
    //        }
    //      }

    //      pre = succ;
    //    }

    // pop node with lowest cost from priority queue
    whileTImes += 1;
    nPred = O.top();

    // set index
    iPred = nPred->setIdx(width, height);
    iterations++;
    //ros::Time t_exp_for = ros::Time::now();
    //RViz visualization
      //std::cout << "visualization in while" << std::endl;
    if (Constants::visualization)
    {
      visualization.publishNode3DPoses(*nPred);
      visualization.publishNode3DPose(*nPred);
      d.sleep();
    }
    //ros::Time t_v = ros::Time::now();
    //std::cout << "visulizztion in while :(ms) " << (t_v - t_exp_for).toSec()*1000 << std::endl;

    // _____________________________
    // LAZY DELETION of rewired node
    // if there exists a pointer this node has already been expanded
    if (nodes3D[iPred].isClosed())
    {
      // pop node from the open list and start with a fresh node
      O.pop();
      continue;
    }
    // _________________
    // EXPANSION OF NODE
    else if (nodes3D[iPred].isOpen())
    {
      // add node to closed list
      nodes3D[iPred].close();
      // remove node from open list
      O.pop();
      // _________
      // GOAL TEST
      if (*nPred == goal || iterations > Constants::iterations)
      {
        // DEBUG
        return nPred;
      }
      // ____________________
      // CONTINUE WITH SEARCH
      else
      {
        // _______________________
        // SEARCH WITH DUBINS SHOT
        //if (Constants::reedssheppShot && nPred->isInRange(goal) && nPred->getPrim() < 3)

        // SEARCH WITH RS SHOT
        // if (Constants::reedssheppShot && nPred->isInRange(goal))
        if (!onlyForward && nPred->isInRange(goal))
        {
          // ros::Time t0 = ros::Time::now();
          //cout<<"nPred   : "<<nPred->getX()<<'\t'<<nPred->getY()<<'\t'<<nPred->getT()<<endl;
          nSucc = reedssheppShot(*nPred, goal, configurationSpace, goalposleft);
          //  ros::Time t1 = ros::Time::now();
          //  ros::Duration d(t1 - t0);
          //  std::cout<<"RS_time(ms):"<<d*1000<<std::endl;
          //cout << "Try RS shot! "<< "goal-nSucc: (" << goal.getX()-nSucc->getX()<<", "<< goal.getY()-nSucc->getY()<<")" << endl;
          if (nSucc != nullptr && *nSucc == goal)
          {
            //DEBUG
            // std::cout << "max diff " << max << std::endl;
           // std::cout << "while times" << whileTImes << std::endl;
            //ros::Time tWhile_end = ros::Time::now();
            //std::cout << "while time:" << (tWhile_end-tWhile).toSec() <<  " s" << std::endl;
            cout << "use RS shot successfully!!" << endl;
            return nSucc;
          }
          // else
          // {
          // }
        }
        else if  (onlyForward && nPred->isInRange(goal)){
        //std::cout << "judege dubins" << std::endl;
        // ros::Time t0 = ros::Time::now();
          //cout<<"nPred   : "<<nPred->getX()<<'\t'<<nPred->getY()<<'\t'<<nPred->getT()<<endl;
          nSucc = dubinsShot(*nPred, goal, configurationSpace);
          //  ros::Time t1 = ros::Time::now();
          //  ros::Duration d(t1 - t0);
          //  std::cout<<"RS_time(ms):"<<d*1000<<std::endl;
  
          //cout << "Try RS shot! "<< "goal-nSucc: (" << goal.getX()-nSucc->getX()<<", "<< goal.getY()-nSucc->getY()<<")" << endl;
          if (nSucc != nullptr && *nSucc == goal)
          {
            //DEBUG
            // std::cout << "max diff " << max << std::endl;
           // std::cout << "while times" << whileTImes << std::endl;
            //ros::Time tWhile_end = ros::Time::now();
            //std::cout << "while time:" << (tWhile_end-tWhile).toSec() <<  " s" << std::endl;
            cout << "use dubins shot successfully!!" << endl;
            return nSucc;
          }
        }


        // ______________________________
        // CALCULATE CURRENT DISTANCE TO THE GOAL
        // float fDistToGoal=fabs(nPred->getX()-goal.getX())+fabs(nPred->getY()-goal.getY());
        // std::cout<<"Current distance to goal is "<< fDistToGoal << std::endl;

        // ______________________________
        // SEARCH WITH FORWARD SIMULATION
        // std::cout << "dir:" << dir << std::endl; 
        //ros::Time  t_for = ros::Time::now();
        //std::cout << "except for time:" << (t_for-t_exp_for).toSec()*1000 << std::endl;
       // std::cout << "start for" << std::endl;
        for (int i = 0; i < dir; i++)
        {
          // create possible successor
          nSucc = nPred->createSuccessor(i);
          // set index of the successor
          iSucc = nSucc->setIdx(width, height);

          // ensure successor is on grid and traversable
          if (nSucc->isOnGrid(width, height) && configurationSpace.isTraversable(nSucc))
          {
              // std::cout << "nSucc->isOnGrid(width, height) && configurationSpace.isTraversable(nSucc)" << std::endl; 
            // ensure successor is not on closed list or it has the same index as the predecessor
            if (!nodes3D[iSucc].isClosed() || iPred == iSucc)
            {
              //cout<<"my prim number is:  "<<nSucc->getPrim()<<endl;

              // calculate new G value
              nSucc->updateG();
              newG = nSucc->getG();

              // if successor not on open list or found a shorter way to the cell
              if (!nodes3D[iSucc].isOpen() || newG < nodes3D[iSucc].getG() || iPred == iSucc)
              {

                // calculate H value
                //distToObstacle = fabs(voronoiDiagram.getDistance((nSucc->getX()-map_origin_x/map_resolution),(nSucc->getY()-map_origin_y) / map_resolution));
                //std::cout << "distance between successor position and the closed obstacle:" << distToObstacle  << " x:" <<  nSucc->getX() <<  "y:" << nSucc->getY() << std::endl;
                //ros::Time  t_updateH = ros::Time::now();
                //std::cout << "updateH in for" << std::endl;
                updateH(*nSucc, goal, nodes2D, dubinsLookup, width, height, configurationSpace, visualization, AlpOut);
                 //std::cout << "updateH" << std::endl; 
                //std::cout << "updateH in for successfully" << std::endl;
                //ros::Time  t_updateH_end = ros::Time::now();
                //std::cout<<"updateH time(ms):"<<(t_updateH_end-t_updateH).toSec()*1000<<std::endl;
                // if the successor is in the same cell but the C value is larger
                if (iPred == iSucc && nSucc->getC() > nPred->getC() + Constants::tieBreaker)
                {
                  delete nSucc;
                  continue;
                }
                // if successor is in the same cell and the C value is lower, set predecessor to predecessor of predecessor
                else if (iPred == iSucc && nSucc->getC() <= nPred->getC() + Constants::tieBreaker)
                {
                  nSucc->setPred(nPred->getPred());
                }

                if (nSucc->getPred() == nSucc)
                {
                  std::cout << "looping";
                }

                // put successor on open list
                nSucc->open();
                nodes3D[iSucc] = *nSucc;
                O.push(&nodes3D[iSucc]);
                delete nSucc;
              }
              else
              {
                delete nSucc;
              }
            }
            else
            {
              delete nSucc;
            }
          }
          else
          {
            delete nSucc;
          }
        }
        //ros::Time  t_for_end = ros::Time::now();
        //ros::Duration d_for(t_for_end-t_for);
        //static ros::Duration d_total (0);
        //d_total += d_for;
        //std::cout<<"for time(ms):"<<d_total.toSec()*1000<<std::endl;
      }
    }
  }


  if (O.empty())
  {
    std::cout << "O is empty:" << std::endl;
    return nullptr;
  }

  return nullptr;
}

//###################################################
//                                        2D A*
//###################################################
float aStar(Node2D &start,
            Node2D &goal,
            Node2D *nodes2D,
            int width,
            int height,
            CollisionDetection &configurationSpace,
            Visualize &visualization,
            bool AlpOut)
{

  // PREDECESSOR AND SUCCESSOR INDEX
  int iPred, iSucc;
  float newG;
  static int invoke_conter = 0;
  //ROS_INFO("the invoked times of A* is(%d) ",++invoke_conter);

  // reset the open and closed list
  for (int i = 0; i < width * height; ++i)
  {
    nodes2D[i].reset();
  }

  // VISUALIZATION DELAY
  ros::Duration d(0.001);

  boost::heap::binomial_heap<Node2D *,
                             boost::heap::compare<CompareNodes>>
      O;
  // update h value
  start.updateH(goal);
  // mark start as open
  start.open();
  // push on priority queue
  O.push(&start);
  iPred = start.setIdx(width);
  nodes2D[iPred] = start;

  // NODE POINTER
  Node2D *nPred;
  Node2D *nSucc;

  // continue until O empty
  while (!O.empty())
  {
    // pop node with lowest cost from priority queue
    nPred = O.top();
    //std::cout << "O.top() successfully" << std::endl;
    // set index
    iPred = nPred->setIdx(width);

    // _____________________________
    // LAZY DELETION of rewired node
    // if there exists a pointer this node has already been expanded
    if (nodes2D[iPred].isClosed())
    {
      // pop node from the open list and start with a fresh node
      O.pop();
      continue;
    }
    // _________________
    // EXPANSION OF NODE
    else if (nodes2D[iPred].isOpen())
    {
       // std::cout << "nodes2D[iPred].isOpen()" <<std::endl;
      // add node to closed list
      nodes2D[iPred].close();
      nodes2D[iPred].discover();

      // // RViz visualization
      if (Constants::visualization2D)
      {
        visualization.publishNode2DPoses(*nPred);
        visualization.publishNode2DPose(*nPred);
        //        d.sleep();
      }

      // remove node from open list
      O.pop();

      // _________
      // GOAL TEST
      if (*nPred == goal)
      {
        return nPred->getG();
      }
      // ____________________
      // CONTINUE WITH SEARCH
      else
      {
        // _______________________________
        // CREATE POSSIBLE SUCCESSOR NODES
        for (int i = 0; i < Node2D::dir; i++)
        {
          // create possible successor
          nSucc = nPred->createSuccessor(i);
          // set index of the successor
          iSucc = nSucc->setIdx(width);
          //std::cout << "setIdX successfully" <<std::endl;
          // ensure successor is on grid ROW MAJOR
          // ensure successor is not blocked by obstacle
          // ensure successor is not on closed list
          if (nSucc->isOnGrid(width, height) && configurationSpace.isTraversable(nSucc) && !nodes2D[iSucc].isClosed())
          {
            //std::cout << "nSucc->isOnGrid(width, height) && configurationSpace.isTraversable(nSucc) && !nodes2D[iSucc].isClosed()" <<std::endl;
            // calculate new G value
            nSucc->updateG();
            newG = nSucc->getG();

            // if successor not on open list or g value lower than before put it on open list
            if (!nodes2D[iSucc].isOpen() || newG < nodes2D[iSucc].getG())
            {
             // std::cout << "!nodes2D[iSucc].isOpen() || newG < nodes2D[iSucc].getG()" <<std::endl;
              // calculate the H value
              nSucc->updateH(goal);
              // put successor on open list
              nSucc->open();
              nodes2D[iSucc] = *nSucc;
              O.push(&nodes2D[iSucc]);
              delete nSucc;
            }
            else
            {
              delete nSucc;
            }
          }
          else
          {
            delete nSucc;
          }
        }
      }
    }
  }

  // return large number to guide search away
  return 1000;
}

//###################################################
//                                         COST TO GO
//###################################################
void updateH(Node3D &start, const Node3D &goal, Node2D *nodes2D, float *dubinsLookup, int width, int height, CollisionDetection &configurationSpace, Visualize &visualization, bool AlpOut)
{
  float dubinsCost = 0;
  float reedsSheppCost = 0;
  float twoDCost = 0;
  float twoDoffset = 0;
  float HcostWeight=1.0f;
  float distThreshold = 10;
  // if (distToObstacle > distThreshold)
  // {
  //     HcostWeight=1.5;
  // }

     // std::cout << "current Hweight = "<< HcostWeight << std::endl;
  

  // if dubins heuristic is activated calculate the shortest path
  // constrained without obstacles
  if (Constants::dubins)
  {

    // ONLY FOR dubinsLookup
    //    int uX = std::abs((int)goal.getX() - (int)start.getX());
    //    int uY = std::abs((int)goal.getY() - (int)start.getY());
    //    // if the lookup table flag is set and the vehicle is in the lookup area
    //    if (Constants::dubinsLookup && uX < Constants::dubinsWidth - 1 && uY < Constants::dubinsWidth - 1) {
    //      int X = (int)goal.getX() - (int)start.getX();
    //      int Y = (int)goal.getY() - (int)start.getY();
    //      int h0;
    //      int h1;

    //      // mirror on x axis
    //      if (X >= 0 && Y <= 0) {
    //        h0 = (int)(helper::normalizeHeadingRad(M_PI_2 - t) / Constants::deltaHeadingRad);
    //        h1 = (int)(helper::normalizeHeadingRad(M_PI_2 - goal.getT()) / Constants::deltaHeadingRad);
    //      }
    //      // mirror on y axis
    //      else if (X <= 0 && Y >= 0) {
    //        h0 = (int)(helper::normalizeHeadingRad(M_PI_2 - t) / Constants::deltaHeadingRad);
    //        h1 = (int)(helper::normalizeHeadingRad(M_PI_2 - goal.getT()) / Constants::deltaHeadingRad);

    //      }
    //      // mirror on xy axis
    //      else if (X <= 0 && Y <= 0) {
    //        h0 = (int)(helper::normalizeHeadingRad(M_PI - t) / Constants::deltaHeadingRad);
    //        h1 = (int)(helper::normalizeHeadingRad(M_PI - goal.getT()) / Constants::deltaHeadingRad);

    //      } else {
    //        h0 = (int)(t / Constants::deltaHeadingRad);
    //        h1 = (int)(goal.getT() / Constants::deltaHeadingRad);
    //      }

    //      dubinsCost = dubinsLookup[uX * Constants::dubinsWidth * ConstantsupdateH::headings * Constants::headings
    //                                + uY *  Constants::headings * Constants::headings
    //                                + h0 * Constants::headings
    //                                + h1];
    //    } else {

    /*if (Constants::dubinsShot && std::abs(start.getX() - goal.getX()) >= 10 && std::abs(start.getY() - goal.getY()) >= 10)*/
    //      // start
    //      double q0[] = { start.getX(), start.getY(), start.getT()};
    //      // goal
    //      double q1[] = { goal.getX(), goal.getY(), goal.getT()};
    //      DubinsPath dubinsPath;
    //      dubins_init(q0, q1, Constants::r, &dubinsPath);
    //      dubinsCost = dubins_path_length(&dubinsPath);

    ompl::base::DubinsStateSpace dubinsPath(Constants::dubins_r);
    State *dbStart = (State *)dubinsPath.allocState();
    State *dbEnd = (State *)dubinsPath.allocState();
    dbStart->setXY(start.getX(), start.getY());
    dbStart->setYaw(start.getT());
    dbEnd->setXY(goal.getX(), goal.getY());
    dbEnd->setYaw(goal.getT());
    dubinsCost = dubinsPath.distance(dbStart, dbEnd);
  }

  // if reversing is active use a
  //if ( Constants::reverse && !Constants::dubins)
    if ( Constants::reverse && !Constants::dubins)
  {
    //    ros::Time t0 = ros::Time::now();
    ompl::base::ReedsSheppStateSpace reedsSheppPath(Constants::r);
    State *rsStart = (State *)reedsSheppPath.allocState();
    State *rsEnd = (State *)reedsSheppPath.allocState();
    rsStart->setXY(start.getX(), start.getY());
    rsStart->setYaw(start.getT());
    rsEnd->setXY(goal.getX(), goal.getY());
    rsEnd->setYaw(goal.getT());
    reedsSheppCost = reedsSheppPath.distance(rsStart, rsEnd);
    //    ros::Time t1 = ros::Time::now();
    //    ros::Duration d(t1 - t0);
    //    std::cout << "calculated Reed-Sheep Heuristic in ms: " << d * 1000 << std::endl;
  }

  // if twoD heuristic is activated determine shortest path
  // unconstrained with obstacles
  //--Mapping debug--
  if (Constants::twoD && !nodes2D[((int)((start.getY() - map_origin_y) / map_resolution)) * width + ((int)((start.getX() - map_origin_x) / map_resolution))].isDiscovered())
  //if(true)
  {
      //std::cout << "enter if !isDiscovered" << std::endl;

    // ros::Time t0 = ros::Time::now();
    // create a 2d start node
    Node2D start2d(start.getX(), start.getY(), 0, 0, nullptr);
    // create a 2d goal node
    Node2D goal2d(goal.getX(), goal.getY(), 0, 0, nullptr);
    // run 2d astar and return the cost of the cheapest path for that node
    //--Mapping debug--
    //nodes2D[(int)start.getY() * width + (int)start.getX()].setG(aStar(goal2d, start2d, nodes2D, width, height, configurationSpace, visualization));
     // ros::Time ta_start = ros::Time::now();
    nodes2D[((int)((start.getY() - map_origin_y) / map_resolution)) * width + ((int)((start.getX() - map_origin_x) / map_resolution))].setG(aStar( goal2d, start2d, nodes2D, width, height, configurationSpace, visualization, AlpOut));
    //std::cout << "astar" << std::endl; 
     // ros::Time ta_end = ros::Time::now();
      //std::cout << "A* total time = " << (ta_end - ta_start)*1000 << "ms" << std::endl;  
    // ros::Time t1 = ros::Time::now();
    // ros::Duration d(t1 - t0);
    // std::cout << "calculated 2D Heuristic in ms: " << d * 1000 << std::endl;
  }

  if (Constants::twoD)
  {
     //std::cout << "enter if 2D" << std::endl;
    // offset for same node in cell
    //--Mapping debug--
    //twoDoffset = sqrt(((start.getX() - (long)start.getX()) - (goal.getX() - (long)goal.getX())) * ((start.getX() - (long)start.getX()) - (goal.getX() - (long)goal.getX())) +
    //                  ((start.getY() - (long)start.getY()) - (goal.getY() - (long)goal.getY())) * ((start.getY() - (long)start.getY()) - (goal.getY() - (long)goal.getY())));
    // static int twoDcaltime=0;
    // std::cout<<"sqrt cal time:"<<++twoDcaltime<<std::endl;
    /*
    // add a cost that concerns  the theta error
    float  deltaT = start.getT() - goal.getT();
    float EuclidSq = (start.getX()-goal.getX())*(start.getX()-goal.getX())+(start.getY()-goal.getY())*(start.getY()-goal.getY());
    float lamda = sqrt(EuclidSq)/fabs(Helper::normalizeHeadingRad(deltaT));
    float lamdaupper = 50.0f, lamdalower = 20.0f;
    float  Tweight;
    if(lamda < lamdalower)
    {
        Tweight = 1.5;
    } 
    else if (lamda < lamdaupper && lamda > lamdalower )
    {
      Tweight = 0.75;
    }
    else
    {
      Tweight = 0;
    }
    float Tcost = Tweight * fabs(deltaT);
    std::cout << "lamda:" << lamda << "Tweight:" << Tweight << ""<<"Tcost:" << Tcost << std::endl;
*/
    twoDoffset = sqrt((std::fabs(start.getX() - (long)start.getX()) - std::fabs(goal.getX() - (long)goal.getX())) * (std::fabs(start.getX() - (long)start.getX()) - std::fabs(goal.getX() - (long)goal.getX())) +
                      (std::fabs(start.getY() - (long)start.getY()) - std::fabs(goal.getY() - (long)goal.getY())) * (std::fabs(start.getY() - (long)start.getY()) - std::fabs(goal.getY() - (long)goal.getY())));
     //std::cout << "twoDoffset:" << twoDoffset<<std::endl;                 
    //--Mapping debug--
    //twoDCost = nodes2D[(int)start.getY() * width + (int)start.getX()].getG() - twoDoffset;
    //common
    twoDCost = nodes2D[((int)((start.getY() - map_origin_y) / map_resolution)) * width + ((int)((start.getX() - map_origin_x) / map_resolution))].getG() - twoDoffset;


    //include theta cost
    // twoDCost = nodes2D[((int)((start.getY() - map_origin_y) / map_resolution)) * width + ((int)((start.getX() - map_origin_x) / map_resolution))].getG() - twoDoffset+Tcost;
    
  }

  // return the maximum of the heuristics, making the heuristic admissable
 // start.setH(std::max(reedsSheppCost, std::max(dubinsCost, twoDCost)));

  //weighted A* test
  start.setH(HcostWeight*std::max(reedsSheppCost, std::max(dubinsCost, twoDCost)));
  //std::cout << "setH successfully" << std::endl;
  //std::cout <<"x:"<<start.getX()<<"y:"<<start.getY()<<"H_cost:"<<std::max(reedsSheppCost, std::max(dubinsCost, twoDCost))<<std::endl;
}


//###################################################
//                                        DUBINS SHOT
//###################################################
Node3D* dubinsShot(Node3D& start, const Node3D& goal, CollisionDetection& configurationSpace) {
  // start
  double q0[] = { start.getX(), start.getY(), start.getT() };
  // goal
  double q1[] = { goal.getX(), goal.getY(), goal.getT() };
  // initialize the path
  DubinsPath path;
  // calculate the path
  // dubins_init(q0, q1, Constants::r, &path);
  if (dubins_init(q0, q1, Constants::dubins_r, &path) == EDUBNOPATH)
  {return nullptr;}

  int i = 0;
  float x = 0.f;
  float length = dubins_path_length(&path);

  Node3D* dubinsNodes = new Node3D [(int)(length / Constants::dubinsStepSize) + 1];

  while (x <  length) {
    double q[3];
    dubins_path_sample(&path, x, q);
    dubinsNodes[i].setX(q[0]);
    dubinsNodes[i].setY(q[1]);
    dubinsNodes[i].setT(Helper::normalizeHeadingRad(q[2]));

    // collision check
    if (configurationSpace.isTraversable(&dubinsNodes[i])) {

      // set the predecessor to the previous step
      if (i > 0) {
        dubinsNodes[i].setPred(&dubinsNodes[i - 1]);
      } else {
        dubinsNodes[i].setPred(&start);
      }

      if (&dubinsNodes[i] == dubinsNodes[i].getPred()) {
        std::cout << "looping shot";
      }

      x += Constants::dubinsStepSize;
      i++;
    } else {
      //      std::cout << "Dubins shot collided, discarding the path" << "\n";
      // delete all nodes
      delete [] dubinsNodes;
      return nullptr;
    }
  }

  //  std::cout << "Dubins shot connected, returning the path" << "\n";
  std::cout << "path type:" << path.type <<std::endl;
  std::cout << "path param:" <<path.param[0] << "  " <<path.param[1] << "  " <<path.param[2] <<std::endl;
  return &dubinsNodes[i - 1];
}


//###################################################
//                                    REEDSSHEPP SHOT
//###################################################

Node3D *reedssheppShot(Node3D &start, const Node3D &goal, CollisionDetection &configurationSpace, int &goalposleft)
{
  // start
  double q0[] = {start.getX(), start.getY(), start.getT()};
  //cout<<"RS start : "<<q0[0]<<'\t'<<q0[1]<<'\t'<<q0[2]<<endl;
  // goal
  double q1[] = {goal.getX(), goal.getY(), goal.getT()};
  //cout<<"RS end : "<<q1[0]<<'\t'<<q1[1]<<'\t'<<q1[2]<<endl;
  // initialize the path
  ReedsSheppPath path;
  // calculate the path
  if(reedsshepp_init(q0, q1, Constants::r, &path, goalposleft) == EDUBNOPATH)
  {return nullptr;}

  /*
  float reedsSheppLength = 0.f;
  ompl::base::ReedsSheppStateSpace::ReedsSheppPath path_ompl;
  ompl::base::ReedsSheppStateSpace reedsSheppPath(Constants::r);
  State* rsStart = (State*)reedsSheppPath.allocState();
  State* rsEnd = (State*)reedsSheppPath.allocState();
  rsStart->setXY(start.getX(), start.getY());
  rsStart->setYaw(start.getT());
  rsEnd->setXY(goal.getX(), goal.getY());
  rsEnd->setYaw(goal.getT());
  reedsSheppLength = reedsSheppPath.distance(rsStart, rsEnd);
  path_ompl = reedsSheppPath.reedsShepp(rsStart, rsEnd);
  cout<<"a test for ompl effect"<<endl;
  cout<<"segment print"<<endl;
  cout<<path_ompl.type_<<endl;
  for(int i = 0; i<18; i++)
  {
    if (path_ompl.type_ == ompl::base::ReedsSheppStateSpace::reedsSheppPathType[i])
    {
      //cout<<"I found it, and it is"<<i<<"and"<<ompl::base::ReedsSheppStateSpace::reedsSheppPathType[i]<<endl;
      //cout<<"check type:"<<typeid(path_ompl.type_).name()<<endl;
    }
  }
  cout<<path_ompl.length_[0]<<endl;
  cout<<path_ompl.length_[1]<<endl;
  cout<<path_ompl.length_[2]<<endl;
  cout<<path_ompl.length_[3]<<endl;
  cout<<path_ompl.length_[4]<<endl;
  cout<<path_ompl.totalLength_<<endl;
  cout<<"find the rho:"<<Constants::r<<endl;
  cout<<reedsSheppLength<<endl;
  */

  // cout<<"the return value is:"<<reedsshepp_init(q0, q1, Constants::r, &path)<<endl;
  int i = 0;
  float x = 0.f;
  //float length = reedsshepp_path_length(&path);
  float length = reedsshepp_path_length(&path);
  //cout<<"RS length: "<<length<<endl;

  Node3D *reedssheppNodes = new Node3D[(int)(length / Constants::reedssheppStepSize) + 1];
  while (x < length)
  {
    double q[3];
    int prim_val[1];
    reedsshepp_path_sample(&path, x, q, prim_val);
    reedssheppNodes[i].setX(q[0]);
    reedssheppNodes[i].setY(q[1]);
    reedssheppNodes[i].setT(Helper::normalizeHeadingRad(q[2]));

    reedssheppNodes[i].setPrim(prim_val[0]);
    if (i == 1)
    {
      reedssheppNodes[i - 1].setPrim(reedssheppNodes[i].getPrim());
    }
    // collision check
    if (configurationSpace.isTraversable(&reedssheppNodes[i]))
    {
      // set the predecessor to the previous step
      if (i > 0)
      {
        reedssheppNodes[i].setPred(&reedssheppNodes[i - 1]);
      }
      else
      {
        reedssheppNodes[i].setPred(&start);
      }

      if (&reedssheppNodes[i] == reedssheppNodes[i].getPred())
      {
        std::cout << "looping shot";
      }

      x += Constants::reedssheppStepSize;
      i++;
    }
    else
    {
      //      std::cout << "Dubins shot collided, discarding the path" << "\n";
      // delete all nodes
      delete[] reedssheppNodes;
      return nullptr;
    }
  }
  //std::cout << "path_length:" << length <<std::endl;
  //std::cout << "Dubins shot connected, returning the path" << "\n";
  //cout<< "my RS-Prim is:"<<reedssheppNodes[i-1].getPrim()<<endl;
  //cout<< "my RS-Prim is:"<<reedssheppNodes[i].getPrim()<<endl;
  cout << "path->type:  "<< path.type<<endl;
  std::cout << "path param:" <<path.param[0] << "  " <<path.param[1] << "  " <<path.param[2] << "  " <<path.param[3] << "  " <<path.param[4] << "  " <<std::endl;
  return &reedssheppNodes[i - 1];
}
