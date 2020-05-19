#ifndef PLANNER_H
#define PLANNER_H

#include <iostream>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <vector>
#include <ros/package.h>
#include <stdio.h>

#include <planner/plannerConfig.h> 
#include <dynamic_reconfigure/server.h>

#include "sbpl_includes.h"

class GlobalPlanner
{
public:
	/**
        * @brief  Constructor for the TrajectoryController
        */

	GlobalPlanner(ros::NodeHandle *nodeHandle, std::string path);

	/**
        * @brief  Destructor for the TrajectoryController
        */
	~GlobalPlanner();




	SBPLIncludes sbpl_planner;

private:

	ros::NodeHandle nh;


	/**
     * @brief ROS Subscribers
     */
	ros::Subscriber globalCostmapSubscriber;
	ros::Subscriber robotPoseSubscriber;
	ros::Subscriber goalSubscriber;
	
	/**
     * @brief ROS Publishers
     */
	ros::Publisher globalPlanPublisher;



	/**
     * @brief Subscriber Callbacks
     */

	void costmapCallback(const nav_msgs::OccupancyGridConstPtr data);
	void robot_cb(const geometry_msgs::PoseConstPtr &robot_msg);
	void goalCallback(const move_base_msgs::MoveBaseActionGoal &goal_msg);

    dynamic_reconfigure::Server<planner::plannerConfig> server;
    dynamic_reconfigure::Server<planner::plannerConfig>::CallbackType f;
	void reconfigureCallback(planner::plannerConfig &config, uint32_t level);

	bool plan(std::vector<double> from, std::vector<double> to);
	void initializeEnviromentVariables();
	void setEnvironmentVariables();

	double m_allocatedTimeSecs,m_initialEpsilon;

	double m_robotX, m_robotY, m_robotTheta;
	double m_goalX, m_goalY, m_goalTheta;
	double m_resolution,m_nominalVelocity,m_timeToTurn45DegreeInplace;
	double m_robotWidth,m_robotLength;


	int m_gridWidth, m_gridHeight,m_obstacleThreshold,m_costInscribedThreshold,m_costPossiblyCircumscribedThreshold;
	int m_offsetX, m_offsetY;

	nav_msgs::Path full_path;

	std::vector<int> m_mapData;
	std::string path;

	bool m_setup = false;


    /* Reconfigure parameters */

    planner::plannerConfig lastConfig;
    planner::plannerConfig defaultConfig;
	
};

#endif