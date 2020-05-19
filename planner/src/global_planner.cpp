#include <ros/ros.h>
#include <iostream>
#include "global_planner.h"

GlobalPlanner::GlobalPlanner(RosClass *envVars) : m_getVariables(envVars), m_setup(false)
{
	m_goalPose  = m_getVariables->getGoalPose();
	m_robotPose = m_getVariables->getRobotPose();
}

GlobalPlanner::~GlobalPlanner()
{
}

void GlobalPlanner::setMapEnvironment()
{

	m_environment.m_mapInfo.width 		= m_getVariables->getGridWidth();
	m_environment.m_mapInfo.height 		= m_getVariables->getGridHeight();
	m_environment.m_mapInfo.offsetX 	= m_getVariables->getOffsetX();
	m_environment.m_mapInfo.offsetY 	= m_getVariables->getOffsetY();
	m_environment.m_mapInfo.resolution  = m_getVariables->getResolution();
	startPoint 							= m_getVariables->getStart();
	goalPoint 							= m_getVariables->getGoal();
	mapData 							= m_getVariables->getMapdata();
}

void GlobalPlanner::plannerFlow()
{


	setMapEnvironment();
	bool goalPoseChange 	= comparePose(m_goalPose, m_getVariables->getGoalPose());
	bool robotPoseChange 	= comparePose(m_robotPose, m_getVariables->getRobotPose());
	if (goalPoseChange) //|| robotPoseChange)
	{
		//ROS_INFO("got inside with %d",robotPoseChange);
		m_environment.plan(startPoint, goalPoint, m_environment.m_mapInfo, mapData, path);
		nav_msgs::Path pathGlobal = m_environment.getGlobalPath();
		m_getVariables->publishPath(pathGlobal);
	}
	m_goalPose  = m_getVariables->getGoalPose();
	m_robotPose = m_getVariables->getRobotPose();
}

bool GlobalPlanner::comparePose(geometry_msgs::Pose poseA, geometry_msgs::Pose poseB)
{
	return ((poseA.position.x != poseB.position.x) || (poseA.position.y != poseB.position.y) || (poseA.orientation.z != poseB.orientation.z) || (poseA.orientation.w != poseB.orientation.w));
}

int main(int argc, char *argv[])
{
	ros::init(argc, argv, "global_planner");
	GlobalPlanner global_planner(new RosClass());
	ros::Rate rate(1);

	while (ros::ok())
	{
		global_planner.plannerFlow();
		ros::spinOnce();
		rate.sleep();
	}
}
