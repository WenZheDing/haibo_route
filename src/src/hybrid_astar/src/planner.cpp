#include "planner.h"

using namespace HybridAStar;
double time_all = 0.0;
//###################################################
//                                        CONSTRUCTOR
//###################################################
Planner::Planner()
{
    ROS_INFO("INITIALIZING PLANNER...");
    // _____
    // TODOS
    //    initializeLookups();
    // Lookup::collisionLookup(collisionLookup);
    // ___________________
    // COLLISION DETECTION
    //    CollisionDetection configurationSpace;
    // _________________
    // TOPICS TO PUBLISH
    pubStart = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/start", 1);
    pubStartTime = n.advertise<std_msgs::Float32>("/hybrid_start_time", 1, true);
    tf_clockwise_rot_.setRotation(tf::Vector3(0, 0, 1), 1.57);

    // ___________________
    // TOPICS TO SUBSCRIBE
    if (Constants::manual)
    {
       // subMap = n.subscribe("/simu/global_map", 1, &Planner::setMap, this);
        subMap = n.subscribe("/map", 1, &Planner::setMap, this);
        // subMap = n.subscribe("/global_map/global_map/global_map", 1, &Planner::setMap, this);
    }
    else
    {
        subMap = n.subscribe("/occ_map", 1, &Planner::setMap, this);
    }

    subGoal = n.subscribe("/hybrid_astar_goal", 1, &Planner::setGoal, this);
    subStart = n.subscribe("/hybrid_astar_start", 1, &Planner::setStart, this);
    subALPOut = n.subscribe("/ALPOut", 1, &Planner::setALPout, this);

    visualization.publishMainRoad();

};

//###################################################
//                                       LOOKUPTABLES
//###################################################
void Planner::initializeLookups()
{
    ROS_INFO("INITIALIZING LOOKUP...");
    if (Constants::dubinsLookup)
    {
        Lookup::dubinsLookup(dubinsLookup);
    }

    Lookup::collisionLookup(collisionLookup);
}

//###################################################
//                                                MAP
//###################################################
void Planner::setMap(nav_msgs::OccupancyGrid map)
{
    if (Constants::coutDEBUG)
    {
        std::cout << "I am seeing the map..." << std::endl;
    }
    //std::cout << "I am seeing the map..." << std::endl;
    // grid.reset();
    // nav_msgs::OccupancyGrid::Ptr newGrid(map);
    // grid = map;
    grid.header.seq = map.header.seq;
    grid.header.stamp = map.header.stamp;
    
    grid.header.frame_id = map.header.frame_id;
    
    
    grid.info = map.info;
    grid.data = map.data;
    //update the configuration space with the current map
    // std::cout << "line 67 voronoidiagram.open.empty() = " << voronoiDiagram.open.empty() << std::endl;
    configurationSpace.updateGrid(map); // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    // std::cout << "line 69 voronoidiagram.open.empty() = " << voronoiDiagram.open.empty() << std::endl;
    //create array for Voronoi diagram
   //ros::Time t0 = ros::Time::now();
    int height = map.info.height;
    int width = map.info.width;
    bool **binMap;
    binMap = new bool *[width];
    //--Mapping debug--
    map_origin_x = map.info.origin.position.x;
    map_origin_y = map.info.origin.position.y;
    map_resolution = map.info.resolution;

    int count = 0;
    for (int i = 0; i < width; i++)
        for (int j = 0; j < height; j++)
            if (map.data[width * j + i] > 0)
                count++;
    //cout<<"count:"<<count<<"\t"<<"Height: "<<height<<"\t"<<"Width: "<<width<<endl;

    for (int x = 0; x < width; x++)
    {
        binMap[x] = new bool[height];
    }

    for (int x = 0; x < width; ++x)
    {
        for (int y = 0; y < height; ++y)
        {
            binMap[x][y] = map.data[y * width + x] ? true : false;
        }
    }
    //ROS_INFO("voronoiDiagram.initializeMap");
    //std::cout<<"debug"<<endl;
     //std::cout << "smooth_flag:" <<smooth_flag<< std::endl;
    if (smooth_flag)
    {
        voronoiDiagram.initializeMap(width, height, binMap);
        // ROS_INFO("voronoiDiagram.update");
        // cout << "count before update:" << voronoiDiagram.open.resetCount() << endl;
        // //std::cout<<"debug"<<endl;
        voronoiDiagram.update();
        // ROS_INFO("voronoiDiagram.visualize");
        // cout << "count after update:" << voronoiDiagram.open.resetCount() << endl;
        // //std::cout<<"debug"<<endl;
        std::string decision_package_path = ros::package::getPath("decision");
        voronoiDiagram.visualize(decision_package_path +"/json_files/result.pgm");
    }
    // std::cout<<"debug"<<endl;
    //  ros::Time t1 = ros::Time::now();
    //  ros::Duration d(t1 - t0);
    //  std::cout << "created Voronoi Diagram in ms: " << d * 1000 << std::endl;

    // plan if the switch is not set to manual and a transform is available 这段不用管
    if (!Constants::manual && listener.canTransform("/map", ros::Time(0), "/base_link", ros::Time(0), "/map", nullptr))
    {

        listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);

        // assign the values to start from base_link
        start.pose.position.x = transform.getOrigin().x();
        start.pose.position.y = transform.getOrigin().y();
        tf::quaternionTFToMsg(transform.getRotation(), start.pose.orientation);

        if (grid.info.height >= start.pose.position.y && start.pose.position.y >= 0 &&
            grid.info.width >= start.pose.position.x && start.pose.position.x >= 0)
        {
            // set the start as valid and plan
            validStart = true;
        }
        else
        {
            validStart = false;
        }
         //visualization.publishMainRoad();
        plan();
    }
}

//###################################################
//                                   INITIALIZE START
//###################################################
void Planner::setStart(const geometry_msgs::PoseStamped::ConstPtr &initial)
{
    ROS_INFO("SETTING A START...");
    
    float x = initial->pose.position.x;
    float y = initial->pose.position.y;
    float t = tf::getYaw(initial->pose.orientation);


    //--Mapping debug--
    if ((y - map_origin_y) / map_resolution >= 0 && (y - map_origin_y) / map_resolution <= grid.info.height && (x - map_origin_x) / map_resolution >= 0 && (x - map_origin_x) / map_resolution <= grid.info.width)
    {

        
        validStart = true;
        start = *initial;
        // if (ALPOut == true)
        // {
        //     start.header = initial->header;
        //     start.pose = lastgoal.pose;
        //     ROS_INFO("ALPOUT IS TRUE , LAST GOAL WILL BE TREATED AS THE NEW START");
        //     x = lastgoal.pose.position.x;
        //     y = lastgoal.pose.position.y;
        //     t  = tf::getYaw(lastgoal.pose.orientation);
        // }
        std::cout << "I am seeing a new start x:" << x << " y:" << y << " t:" << t << std::endl;

        if (Constants::manual)
        {
            plan();
        }

    }
    else
    {
        std::cout << "invalid start x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;
    }
}

//###################################################
//                                    INITIALIZE GOAL
//###################################################
void Planner::setGoal(const geometry_msgs::PoseStamped::ConstPtr &end)
{
    ROS_INFO("SETTING A GOAL...");
    // retrieving goal position
    float x = end->pose.position.x;
    float y = end->pose.position.y;
    float t = tf::getYaw(end->pose.orientation);

    //--Mapping debug--
    if ((y - map_origin_y) / map_resolution >= 0 && (y - map_origin_y) / map_resolution <= grid.info.height && (x - map_origin_x) / map_resolution >= 0 && (x - map_origin_x) / map_resolution <= grid.info.width)
    {
        std::cout << "I am seeing a new goal x:" << x << " y:" << y << " t:" << t << std::endl;
        validGoal = true;
        goal.header.frame_id = end->header.frame_id;
        goal.header.stamp = end->header.stamp;
        goal.pose.orientation = end->pose.orientation;
        goal.pose.position.x = end->pose.position.x;
        goal.pose.position.y = end->pose.position.y;
        goal.pose.position.z = 0;

        if(!ALPOut)
        {
            lastgoal = goal;
            lastgoal.header.stamp = ros::Time::now();
        }

        // lastgoal.header.frame_id = goal.header.frame_id;
        // lastgoal.header.stamp = goal.header.stamp;
        // lastgoal.pose.orientation = goal.pose.orientation;
        // lastgoal.pose.position.x = goal.pose.position.x - 15 * cos(t);
        // lastgoal.pose.position.y = goal.pose.position.y - 15 *  sin(t);
        // lastgoal.pose.position.z = 0;

        // goal = *end;

        if (Constants::manual)
        {
            plan(); //检测到新终点，重新plan
        }
    }
    else
    {
        std::cout << "invalid goal x:" << x << " y:" << y << " t:" << Helper::toDeg(t) << std::endl;
    }
}

void Planner::setALPout(const std_msgs::Bool  ALP_command)
 {
    ALPOut = ALP_command.data;
    std::cout <<  "ALPout flag in HA* has been changed into" << ALPOut << std::endl;
    receiveFlag = true;
    plan();
}

//###################################################
//                                      PLAN THE PATH
//###################################################
void Planner::plan()
{
    // if a start as well as goal are defined go ahead and plan
    if (validStart && validGoal && receiveFlag)
    {
        receiveFlag = false;
        ROS_INFO("Start hybrid_astar planning...");
        // ___________________________
        // LISTS ALLOWCATED ROW MAJOR ORDER
        int width = grid.info.width;
        int height = grid.info.height;
        int depth = Constants::headings;
        int length = width * height * depth;
        // define list pointers and initialize lists
        Node3D *nodes3D = new Node3D[length]();
        Node2D *nodes2D = new Node2D[width * height]();
        //visualization.publishMainRoad();
        // ________________________
        // retrieving goal position
        float x = goal.pose.position.x;
        float y = goal.pose.position.y;
        float t = tf::getYaw(goal.pose.orientation);
        // set theta to a value (0,2PI]
        t = Helper::normalizeHeadingRad(t);
        std::cout << "the last goal point x:" << x << " y:" << y << " t:" << t << std::endl;
        const Node3D nGoal(x, y, t, 0, 0, nullptr);
        // __________
        // DEBUG GOAL
        //const Node3D nGoal(94.24, 103.38, 0.7, 0, 0, nullptr);

        // _________________________
        // retrieving start position
        x = start.pose.position.x;
        y = start.pose.position.y;
        t = tf::getYaw(start.pose.orientation);
        // set theta to a value (0,2PI]
        t = Helper::normalizeHeadingRad(t);
        std::cout << "the last start point x:" << x << " y:" << y << " t:" << t << std::endl;
        Node3D nStart(x, y, t, 0, 0, nullptr);
        // ___________
        // DEBUG START
        //Node3D nStart(13.82, 22.64, 0.02, 0, 0, nullptr);
        
        // 判断终点在起点的左/右边
        float tstart2goal = atan2(nGoal.getY() - nStart.getY(),(nGoal.getX() - nStart.getX()));
        cout << "tstart2goal = " << tstart2goal*180/3.1415926 << endl;
        float twithy = Helper::normalizeHeadingRad(nStart.getT()-tstart2goal-M_PI_2);
        cout << "twithy = " << twithy*180/3.1415926 << endl;
        if(twithy<M_PI_2 || twithy>M_PI*1.5)
        {
            cout << "right"  << endl;
            goalposleft = 0;
        }
        else
        {
            cout << "left"  << endl;
            goalposleft = 1;
        }

        // ___________________________
        // START AND TIME THE PLANNING
        ros::Time t0 = ros::Time::now();
        startTime.data = t0.toSec();
        // startTime.data = t0.sec + t0.nsec * 1e-9;
        pubStartTime.publish(startTime);

        // CLEAR THE VISUALIZATION
        visualization.clear();
        //visualization.publishMainRoad();
        // CLEAR THE PATH
        path.clear();
        smoothedPath.clear();
         if(ALPOut)
        {path.publishLastPath();}
        // FIND THE PATH
        Node3D *nSolution = Algorithm::hybridAStar(nStart, nGoal, nodes3D, nodes2D, width, height, configurationSpace, dubinsLookup, visualization, voronoiDiagram, ALPOut,goalposleft);

         ros::Time t1 = ros::Time::now();
        ros::Duration d(t1 - t0);
        time_all = time_all + d.toSec();
        // std::cout << "TIME in ms: " << d * 1000 << std::endl;
        std::cout << "HA PLANNING TIME in s: " << time_all << std::endl;

        // TRACE THE PATH
        smoother.tracePath(nSolution);
        // CREATE THE UPDATED PATH
        path.updatePath(smoother.getPath());
        //PUBLISH CHOSEN POINTS
        // path.generatePoints(smoother.getPath());
        path.publishPathPoints(smoother.getPath());
        if(Constants::savecsvFlag)
            {path.savePathPointsTocsv(smoother.getPath());}
         t0 = ros::Time::now();
        if (smooth_flag)
        {
            // SMOOTH THE PATH
            smoother.smoothPath(voronoiDiagram);
            // CREATE THE UPDATED PATH
            smoothedPath.updatePath(smoother.getPath());
        }
         t1 = ros::Time::now();
        d= t1 - t0;
        // std::cout << "TIME in ms: " << d * 1000 << std::endl;
        std::cout << "SMOOTHING TIME in s: " << d << std::endl;
        // _________________________________
        // PUBLISH THE RESULTS OF THE SEARCH
        if (smooth_flag)
        {
            smoothedPath.publishPath();
            smoothedPath.publishPathNodes();
            smoothedPath.publishPathVehicles();
        }
        else
        {
            path.publishPath();
            path.publishPathNodes();
            path.publishPathVehicles();
        }
        visualization.publishNode3DCosts(nodes3D, width, height, depth);
        visualization.publishNode2DCosts(nodes2D, width, height);

        validStart = false;
        validGoal = false;

        delete[] nodes3D;
        delete[] nodes2D;
    }
    else
    {
        std::cout << "missing goal or start or alpout" << std::endl;
    }
}
