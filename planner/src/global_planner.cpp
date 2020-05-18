#include <ros/ros.h>
#include <iostream>

// #include <geometry_msgs/PointStamped.h>
// #include <nav_msgs/Odometry.h>
// #include <tf/transform_listener.h>
// #include <tf/transform_datatypes.h>
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// #include <geometry_msgs/PoseWithCovarianceStamped.h>
// #include <ros/package.h>
// #include <nav_msgs/Path.h>
// #include <nav_msgs/OccupancyGrid.h>
// #include <vector>
// #include <stdio.h>
#include "global_planner.h"

#define CELL_GRID_SIZE 0.15

GlobalPlanner::GlobalPlanner(ros::NodeHandle *nodeHandle, std::string path) : nh(*nodeHandle), path(path)
{
	/**
    * @brief ROS Subscribers
    */
	goalSubscriber		 = nh.subscribe("/move_base/goal", 1, &GlobalPlanner::goalCallback, this);
	globalCostmapSubscriber = nh.subscribe("/move_base/global_costmap/costmap", 1, &GlobalPlanner::costmapCallback, this);
	robotPoseSubscriber = nh.subscribe("/robot_pose", 1, &GlobalPlanner::robot_cb, this);

	/**
	* @brief ROS Publishers
    */
	globalPlanPublisher = nh.advertise<nav_msgs::Path>("/global_planner", 10);


	initializeEnviromentVariables();

	full_path.header.frame_id = "map";
}

GlobalPlanner::~GlobalPlanner()
{
}
void GlobalPlanner::costmapCallback(const nav_msgs::OccupancyGridConstPtr data)
{
	ROS_WARN("Costmap Callback");
	m_gridWidth = data->info.width;
	m_gridHeight = data->info.height;
	m_resolution = data->info.resolution;
	m_offsetX = data->info.origin.position.x;
	m_offsetY = data->info.origin.position.y;
	for (auto i : data->data)
	{
		this->m_mapData.push_back(i);
	}
}

void GlobalPlanner::robot_cb(const geometry_msgs::PoseConstPtr &msg)
{
	m_robotX = msg->position.x - m_offsetX;
	m_robotY = msg->position.y - m_offsetY;

	tf::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);

	m_robotTheta = yaw;
}



void GlobalPlanner::goalCallback(const move_base_msgs::MoveBaseActionGoal &goal_msg)
{
    // ROS_WARN("RosClass:: Received New goal");
    // m_goalPose.position      = goal_msg.goal.target_pose.pose.position;
    // m_goalPose.orientation   = goal_msg.goal.target_pose.pose.orientation;


	std::vector<double> start, end;

	m_goalX     = goal_msg.goal.target_pose.pose.position.x - m_offsetX;
	m_goalY     = goal_msg.goal.target_pose.pose.position.y - m_offsetY;


	tf::Quaternion q(0.0, 0.0, goal_msg.goal.target_pose.pose.orientation.z, goal_msg.goal.target_pose.pose.orientation.w);
	tf::Matrix3x3 m(q);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);

	m_goalTheta = yaw;

	std::cout << "target position " << m_goalX << " " << m_goalY << " " << m_goalTheta << '\n';
	std::cout << "current position " << m_robotX << " " << m_robotY << " " << m_robotTheta << '\n';

	start.push_back(m_robotX);
	start.push_back(m_robotY);
	start.push_back(m_robotTheta);

	end.push_back(m_goalX);
	end.push_back(m_goalY);
	end.push_back(m_goalTheta);
	plan(start, end);

	start.clear();
	end.clear();

}


bool GlobalPlanner::plan(std::vector<double> from, std::vector<double> to)
{
	std::string searchDir = "forward";
	std::string plannerType = "arastar";
	PlannerType planner = sbpl_planner.StrToPlannerType(plannerType.c_str());
	std::string mot_prim_file_name = path + "/config/robot1.mprim";
	bool forwardSearch = !strcmp(searchDir.c_str(), "forward");
	setEnvironmentVariables();



	std::vector<std::vector<int>> full_solution;

	full_solution = sbpl_planner.planxythetamlevlat(planner,from,to,mot_prim_file_name.c_str(),this->m_mapData,sbpl_planner.map_info);


	//full_solution = sbpl_planner.planxythetamlevlat(planner, m_gridWidth, m_gridHeight, from, to, m_resolution, 80, mot_prim_file_name.c_str(), this->m_mapData);

	if (full_solution.size() > 0)
	{

		ROS_INFO("Planning done");
		std::vector<std::vector<double>> current_full_path;
		for (auto path : full_solution)
		{
			std::vector<double> v;
			v.push_back(DISCXY2CONT(path[0], m_resolution));
			v.push_back(DISCXY2CONT(path[1], m_resolution));
			v.push_back(DiscTheta2Cont(path[2], 4));
			current_full_path.push_back(v);
		}
		full_path.poses.clear();
		for (auto path : current_full_path)
		{
			geometry_msgs::PoseStamped pose;
			pose.header.frame_id = "map";
			pose.pose.position.x = path[0] + m_offsetX;
			pose.pose.position.y = path[1] + m_offsetY;
			tf2::Quaternion quater;
			quater.setRPY(0, 0, path[2]);
			tf2::convert(quater, pose.pose.orientation);
			full_path.poses.push_back(pose);
		}
		globalPlanPublisher.publish(full_path);
		return true;
	}
	else
	{
		ROS_INFO("something wrong . unable to plan ");
	}
	return false;
}

void GlobalPlanner::setEnvironmentVariables()
{
	sbpl_planner.map_info.width = m_gridWidth;
	sbpl_planner.map_info.height = m_gridHeight;
	sbpl_planner.map_info.obsthresh = m_obstacleThreshold;
	sbpl_planner.map_info.cost_inscribed_thresh = m_costInscribedThreshold;
	sbpl_planner.map_info.cost_possibly_circumscribed_thresh = m_costPossiblyCircumscribedThreshold;
	sbpl_planner.map_info.cell_size = m_resolution;
	sbpl_planner.map_info.nominalvel = m_nominalVelocity;
	sbpl_planner.map_info.timetoturn45degsinplace = m_timeToTurn45DegreeInplace;
	sbpl_planner.map_info.robotWidth = m_robotWidth;
	sbpl_planner.map_info.robotLength = m_robotLength;
    std::cout << "setEnvironmentVariables()" << m_nominalVelocity << m_timeToTurn45DegreeInplace << m_obstacleThreshold << std::endl;  

}


void GlobalPlanner::initializeEnviromentVariables()
{
	nh.getParam("/global_planner/obstacleThreshold", m_obstacleThreshold); 
	nh.getParam("/global_planner/costInscribedThreshold", m_costInscribedThreshold); 
	nh.getParam("/global_planner/costPossiblyCircumscribedThreshold", m_costPossiblyCircumscribedThreshold); 
	nh.getParam("/global_planner/nominalVelocity", m_nominalVelocity); 
	nh.getParam("/global_planner/timeToTurn45DegreeInplace", m_timeToTurn45DegreeInplace); 

	nh.getParam("/global_planner/robotWidth", m_robotWidth); 	
	nh.getParam("/global_planner/robotLength", m_robotLength); 

	ROS_ERROR("initialized Environment variables : %d   %f   %f",m_obstacleThreshold,m_nominalVelocity,m_timeToTurn45DegreeInplace);
   
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "global_planner");
	ros::NodeHandle nh_;
	std::string path = ros::package::getPath("planner");

	GlobalPlanner global_planner(&nh_, path);
	while (ros::ok())
	{
		//	test.print() ;
		ros::spinOnce();
		usleep(25000);
	}
}
