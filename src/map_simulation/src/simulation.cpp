#include "simulation.h"

LocalMap::LocalMap(ros::NodeHandle &node_handle, string &frame_id)
{
	frame_id_ = frame_id;
	local_map.header.frame_id = frame_id_;
	laser_map.header.frame_id = frame_id_;

	offsetx = -25.0;
	offsety = -30.0;
	resolution = 1;
	width = 50;
	height = 70;

	offsetx = -50.0;
	offsety = -50.0;
	resolution = 1;
	width = 100;
	height = 100;

	deg = 5;
	groups = 360 / deg;

	local_map.info.resolution = resolution;
	local_map.info.width = width;
	local_map.info.height = height;
	local_map.info.origin.position.x = offsetx;
	local_map.info.origin.position.y = offsety;
	local_map.data.resize(width * height);

	laser_map.info.resolution = resolution;
	laser_map.info.width = width;
	laser_map.info.height = height;
	laser_map.info.origin.position.x = offsetx;
	laser_map.info.origin.position.y = offsety;
	laser_map.data.resize(width * height);

	// ROS_INFO("sim data loaded.");
	local_points.resize(width * height);
	// 为每个栅格点赋上xy坐标值，z为0；
	for (int i = 0; i < height * width; i++)
	{
		local_points[i].header.frame_id = frame_id_;
		local_points[i].point.x = (i % width + 1) * resolution + offsetx;
		local_points[i].point.y = (i / width + 1) * resolution + offsety;
		local_points[i].point.z = 0.0;
	}

	localmap_pub = n.advertise<nav_msgs::OccupancyGrid>("/simu/local_map", 1);
	lasermap_pub = n.advertise<nav_msgs::OccupancyGrid>("/simu/laser_map", 1);
	global_detectedmap_pub = n.advertise<nav_msgs::OccupancyGrid>("/simu/global_map", 1);
	globalmap_sub = n.subscribe("/map", 1, &LocalMap::saveMap, this);
	sim_pose_sub = n.subscribe("/dwa_planner/sim_position", 1, &LocalMap::get_SimPose, this);
	start_sub = n.subscribe("/initialpose", 1, &LocalMap::init_map, this);

	global_detectedmap.header.frame_id = "map";
	global_detectedmap.info.resolution = resolution;
	global_detectedmap.info.width = width;
	global_detectedmap.info.height = height;
	global_detectedmap.info.origin.position.x = offsetx;
	global_detectedmap.info.origin.position.y = offsety;
	global_detectedmap.data.resize(width * height);
	detectedmap_initialize_flag = false;
}

void LocalMap::get_SimPose(const geometry_msgs::PoseStamped sim_pose)
{
    update_localmap();

    // extract_border();

    // update_multimap();

    update_lasermap();

	update_globalmap();

    return;
}

void LocalMap::init_map(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &initial)
{
    update_localmap();

    // extract_border();

    // update_multimap();

    update_lasermap();

	update_globalmap();

    return;
}

void LocalMap::update_localmap()
{
	int grid_x, grid_y; // 临时存放全局地图中的栅格坐标；
	int last_index, key;
	float theta;
	global_points.clear();
	scan_points.clear();
	// 对multimap初始化，建立72个key；
	for (int i = 0; i < groups; i++){scan_points.insert(pair<int, int>(i, -1));};

	// 对感知范围（矩形）内所有点进行遍历，找出其在全局地图中对应的珊格点；
	// for (int i = 0; i < width * height; i++)
	for (int i = 0; i < local_map.info.height; i++)	
	{
		last_index = i * local_map.info.width;

		for (int j = 0; j < local_map.info.width; j++)
		{
			geometry_msgs::PointStamped global_point;
			local_points[i * local_map.info.width + j].header.stamp = ros::Time(0);

			try
			{
				listener_.transformPoint("map", ros::Time(0), local_points[i * local_map.info.width + j], "base", global_point);
			}
			catch(tf::TransformException &ex)
			{
				// ROS_ERROR("%s",ex.what());
	        	ros::Duration(1.0).sleep();
			}
			global_points.push_back(global_point);

			grid_x = (global_points[i * local_map.info.width + j].point.x - map_->info.origin.position.x) / map_->info.resolution - 1;
			grid_y = (global_points[i * local_map.info.width + j].point.y - map_->info.origin.position.y) / map_->info.resolution - 1;

			if (grid_x < 0 || grid_x >= map_->info.width || grid_y < 0 || grid_y >= map_->info.height) 
			{
				// ROS_INFO("point out of border!!!");
				local_map.data[i * local_map.info.width + j] = 100; // 将感知范围超出边界的点设为占据状态；
			}
			else{
				// std::cout << map->data.size() << " " << grid_y * width + grid_x << std::endl;
				local_map.data[i * local_map.info.width + j] = map_->data[grid_y * map_->info.width + grid_x]; // 未出界时，将全局地图中对应栅格的值赋给局部地图中的栅格；
			}

			// 判断是否是交界点；
			if (local_map.data[last_index] != local_map.data[i * local_map.info.width + j]) 
			{
				if (local_map.data[last_index] > local_map.data[i * local_map.info.width + j]){
					border_points.push_back(last_index);
					theta = atan2(local_points[last_index].point.y, local_points[last_index].point.x) / M_PI * 180;
					theta = theta > 0 ? theta : 360 + theta;
					key = static_cast<int>(theta) / deg;
    				scan_points.insert(pair<int, int>(key, last_index));
				}
				else{
					border_points.push_back(i * local_map.info.width + j);
					theta = atan2(local_points[i * local_map.info.width + j].point.y, local_points[i * local_map.info.width + j].point.x) / M_PI * 180;
					theta = theta > 0 ? theta : 360 + theta;
					key = static_cast<int>(theta) / deg;
    				scan_points.insert(pair<int, int>(key, i * local_map.info.width + j));
				}
			}
			last_index = i * local_map.info.width + j;

		}

	}

	return ;
}

void LocalMap::update_lasermap()
{
	for (int i = 0; i < laser_map.data.size(); i++)
	{
		laser_map.data[i] = 0;
	}

    laser_map.header.stamp = ros::Time::now();
	// 遍历multimap中的所有key，找出每个key下离车辆最近的点，并在laser_map中将其设置为占据状态；
    multimap<int, int>::iterator ele; // first为key值，代表位于哪一块扇形区域；second为该点在local_map中的索引值；
    float dist, min_dist;
    int min_dist_ind;
    for (int i = 0; i < groups; i++)
    {
    	ele = scan_points.find(i); // find()返回指向第i组点的第一个元素的迭代器；
    	// if (ele->second != -1)
    	// 	cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1 " << ele->second << endl;
    	if (scan_points.count(i) > 1){
	    	// 使用第一个元素的距离和索引初始化；
	    	if (ele->second == -1)
	    	{
	    		ele++;
	    		min_dist_ind = ele->second;
	    		min_dist = pow(local_points[ele->second].point.x, 2) + pow(local_points[ele->second].point.y, 2);
	    	}

	    	for (int j = 1; j < scan_points.count(i); j++, ele++)
	    	{
	    		dist = pow(local_points[ele->second].point.x, 2) + pow(local_points[ele->second].point.y, 2);
	    		if (dist < min_dist)
	    		{
	    			min_dist = dist ;
	    			min_dist_ind = ele->second;
	    		}
	    	}

	    	laser_map.data[min_dist_ind] = 100;
    	}
    }

    return ;
}

void LocalMap::update_globalmap()
{
	// ROS_INFO("update_globalmap started!");
	global_detectedmap.header.stamp = ros::Time(0);
	int grid_x, grid_y; 
	// for (int i = 0; i < global_detectedmap.data.size(); i++)
	// {
	// 	global_detectedmap.data[i] = 0;
	// }
	for (int i = 0; i < laser_map.info.height; i++)	
	{
		for (int j = 0; j < laser_map.info.width; j++)
		{
			if(laser_map.data[i*laser_map.info.width + j] == 100)
			{
				grid_x = (global_points[i * local_map.info.width + j].point.x - global_detectedmap.info.origin.position.x) / global_detectedmap.info.resolution - 1;
				grid_y = (global_points[i * local_map.info.width + j].point.y - global_detectedmap.info.origin.position.y) / global_detectedmap.info.resolution - 1;
				if(grid_x <0)  {grid_x = 0;}
				if(grid_y <0)  {grid_y = 0;}				
				// ROS_INFO("grid_x: %d grid_y: %d",grid_x,grid_y);
				if(grid_x>=0 && grid_x <global_detectedmap.info.width && grid_y>=0 && grid_y<global_detectedmap.info.height)
				{
					global_detectedmap.data[grid_y * global_detectedmap.info.width + grid_x] = 100; 
				}
				else
				{
					// ROS_INFO("grid_x: %d grid_y: %d",grid_x,grid_y);
				}
			}
		}
	}
	// ROS_INFO("update_globalmap finished!");
    return ;
}