#ifndef PLANNER_H
#define PLANNER_H

#include <iostream>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose.h>
#include <ros/package.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <vector>
#include <stdio.h>

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
	ros::Subscriber clickedPointSubsriber;
	ros::Subscriber odomSubscriber;
	ros::Subscriber globalCostmapSubscriber;
	ros::Subscriber robotPoseSubscriber;
	
	/**
     * @brief ROS Publishers
     */
	ros::Publisher globalPlanPublisher;



	/**
     * @brief Subscriber Callbacks
     */

	void clickedPointCallback(const geometry_msgs::PointStampedConstPtr &msg);
	void odomCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg);
	void costmapCallback(const nav_msgs::OccupancyGridConstPtr data);
	void robot_cb(const geometry_msgs::PoseConstPtr &robot_msg);

	bool plan(std::vector<double> from, std::vector<double> to);
	void initializeEnviromentVariables();
	void setEnvironmentVariables();

	double m_robotX, m_robotY, m_robotTheta;
	double m_goalX, m_goalY, m_goalTheta;
	double m_resolution,m_nominalVelocity,m_timeToTurn45DegreeInplace;
	double m_robotWidth,m_robotLength;


	int m_gridWidth, m_gridHeight,m_obstacleThreshold,m_costInscribedThreshold,m_costPossiblyCircumscribedThreshold;
	int m_offsetX, m_offsetY;

	nav_msgs::Path full_path;

	std::vector<int> m_mapData;
	std::string path;

	
};

#endif