#include <ros/ros.h>
#include <iostream>
#include "global_planner.h"



/**
* @brief  Constructor for the GlobalPlanner
*/
GlobalPlanner::GlobalPlanner(RosClass *envVars) : m_getVariables(envVars), m_setup(false)
{
	m_goalPose  = m_getVariables->getGoalPose();
	m_robotPose = m_getVariables->getRobotPose();

}


/**
* @brief  Destructor for the GlobalPlanner
*/
GlobalPlanner::~GlobalPlanner()
{
}


/**
* @brief  Setting up the map environment parameters
*/
void GlobalPlanner::setMapEnvironment()
{
	m_environment.costMapInfo.width 	= m_getVariables->getGridWidth();
	m_environment.costMapInfo.height 	= m_getVariables->getGridHeight();
	m_environment.costMapInfo.offsetX 	= m_getVariables->getOffsetX();
	m_environment.costMapInfo.offsetY 	= m_getVariables->getOffsetY();
	m_environment.costMapInfo.resolution= m_getVariables->getResolution();
	startPoint 							= m_getVariables->getStart();
	goalPoint 							= m_getVariables->getGoal();
	mapData 							= m_getVariables->getMapdata();
}

/**
* @brief  Compares between two pose values given
* @return true if the given poses are different ,otherwise false 
*/

bool GlobalPlanner::comparePose(geometry_msgs::Pose p1, geometry_msgs::Pose p2)
{
	return ((p1.position.x != p2.position.x) || (p1.position.y != p2.position.y) || (p1.orientation.z != p2.orientation.z) || (p1.orientation.w != p2.orientation.w));
}

/**
* @brief  Sequence of planner execution
*/
void GlobalPlanner::plannerFlow()
{

	setMapEnvironment();
	bool goalPoseChange 	= comparePose(m_goalPose, m_getVariables->getGoalPose());
	//bool robotPoseChange 	= comparePose(m_robotPose, m_getVariables->getRobotPose());
	if (goalPoseChange) //|| robotPoseChange)
	{
		m_environment.plan(startPoint, goalPoint, m_environment.costMapInfo, mapData, path);
		nav_msgs::Path pathGlobal = m_environment.getGlobalPath();
		m_getVariables->publishPath(pathGlobal);
	}
	m_goalPose  = m_getVariables->getGoalPose();
	//m_robotPose = m_getVariables->getRobotPose();
}





/**
 * @brief Main function
*/

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "global_planner");
	GlobalPlanner global_planner(new RosClass());
	ros::Rate rate(20);

	while (ros::ok())
	{
		global_planner.plannerFlow();
		ros::spinOnce();
		rate.sleep();
	}
}
