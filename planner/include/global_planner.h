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
    * @brief  Constructor for the GlobalPlanner
    */

	GlobalPlanner(RosClass *enviromentVars);

	/**
    * @brief  Destructor for the GlobalPlanner
    */
	~GlobalPlanner();

	/**
	* @brief  Sequence of planner execution
	*/	
	void plannerFlow();
	
	
	std::string path = ros::package::getPath("planner");


private:

	/**
	* @brief  Compares between two pose values given
	* @return true if the given poses are different ,otherwise false 
	*/
	bool comparePose(geometry_msgs::Pose p1, geometry_msgs::Pose p2);
	
	/**
	* @brief  Setting up the map environment parameters
	*/
	void setMapEnvironment();


	/*Member varibales*/
	bool m_setup;
	geometry_msgs::Pose m_goalPose, m_robotPose;
	std::vector<double> goalPoint;
	std::vector<double> startPoint;
	std::vector<int> mapData;

	/*Instances of the classes Environment and RosClass*/
	Environment m_environment;
	RosClass *m_getVariables;
};

#endif