#ifndef PLANNER_H
#define PLANNER_H

#include <iostream>
#include <ros/package.h>

#include "ros_class.h"
#include "environment.h"

class GlobalPlanner
{
public:
	/**
        * @brief  Constructor for the TrajectoryController
        */

	GlobalPlanner(RosClass *enviromentVars);

	/**
        * @brief  Destructor for the TrajectoryController
        */
	~GlobalPlanner();

	void plannerFlow();

	std::string path = ros::package::getPath("planner");
	Environment m_environment;
	RosClass *m_getVariables;

private:

	bool comparePose(geometry_msgs::Pose poseA, geometry_msgs::Pose poseB);

	void setMapEnvironment();

	bool m_setup;
	geometry_msgs::Pose m_goalPose, m_robotPose;
	std::vector<double> goalPoint;
	std::vector<double> startPoint;
	std::vector<int> mapData;
};

#endif