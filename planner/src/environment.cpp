#include <ros/console.h>
#include "environment.h"
/**
* @brief  Constructor for the Environment
*/
Environment::Environment()
{
	initializeEnviromentVariables();

	/*  Reconfigure callback  */
	f = boost::bind(&Environment::reconfigureCallback, this, _1, _2);
	server.setCallback(f);
}

/**
* @brief  Destructor for the Environment class
*/
Environment::~Environment()
{
}

/**
* @brief  Initializes the environment variables from Parameter server
*/
void Environment::initializeEnviromentVariables()
{
	nh_.getParam("/globalPlanner/obstacleThreshold", m_obstacleThreshold);
	nh_.getParam("/globalPlanner/costInscribedThreshold", m_costInscribedThreshold);
	nh_.getParam("/globalPlanner/costPossiblyCircumscribedThreshold", m_costPossiblyCircumscribedThreshold);
	nh_.getParam("/globalPlanner/nominalVelocity", m_nominalVelocity);
	nh_.getParam("/globalPlanner/timeToTurn45DegreeInplace", m_timeToTurn45DegreeInplace);
	nh_.getParam("/globalPlanner/robotWidth", m_robotWidth);
	nh_.getParam("/globalPlanner/robotLength", m_robotLength);
	nh_.getParam("/motionPrimitiveDirectory",m_pathDirectory);
	//std::cout << "Read value is  " <<m_pathDirectory<< std::endl;
	//ROS_ERROR("initialized Environment variables : %d   %f   %f",m_obstacleThreshold,m_nominalVelocity,m_timeToTurn45DegreeInplace);
}



/**
* @brief  Reconfiguration Call back
*/
void Environment::reconfigureCallback(planner::plannerConfig &config, uint32_t level)
{
	ROS_INFO_ONCE("In reconfiguration call back");

	m_allocatedTimeSecs 				 = config.allocatedTimeSecs;
	m_initialEpsilon 					 = config.initialEpsilon;
	m_factor 							 = config.factor;	

	m_obstacleThreshold  				 = config.obstacleThreshold;
	m_costInscribedThreshold 			 = config.costInscribedThreshold;
	m_costPossiblyCircumscribedThreshold = config.costPossiblyCircumscribedThreshold;
	m_nominalVelocity 					 = config.nominalVelocity;
	m_timeToTurn45DegreeInplace 		 = config.timeToTurn45DegreeInplace;
	m_robotWidth 						 = config.robotWidth;
	m_robotLength 						 = config.robotLength;

	// /*The first time we're called, we just want to make sure we have the original configuration
	//  * if restore defaults is set on the parameter server, prevent looping    */
	if (config.restore_defaults)
	{
		ROS_WARN_ONCE("Environment:: Re-setting default values");
		config = defaultConfig;
		config.restore_defaults = false;
	}

	if (!m_setup)
	{
		lastConfig = config;
		defaultConfig = config;
		m_setup = true;
		return;
	}
	lastConfig = config;
}

/**
* @brief  Member function used to call the planner
* @param from Starting point for the planner
* @param to Goal point for the planner 
* @param mapInfo contains the environment variables from the costmap
* @param mapData contains the map data from the costmap
* @param path contains the directory where the motion primitive file is placed
*/

bool Environment::plan(std::vector<double> &from, std::vector<double> &to, CostMapInfo &costMapInfo, const std::vector<int> &mapData, std::string &path)
{
	if(!m_initialize)
	{
		searchDir 		    = "forward";
		plannerType 		= "arastar";
		planner 			= sbpl_planner.StrToPlannerType(plannerType.c_str());
		mot_prim_file_name  = path + m_pathDirectory.c_str();
		forwardSearch 		= !strcmp(searchDir.c_str(), "forward");
		m_initialize 		= true;
	}


	CostMapInfo map_info = costMapInfo;
	setEnvironmentVariables(map_info);

	std::vector<std::vector<double>> full_solution;
	std::vector<int> destMapData ;
	
	if(m_factor != 1.0)
	{
		//(std::vector<int> sourceMatrix,std::vector<int> &destMatrix,const int &m_factor,const int &width,const int &height)
		resizeMatrix(mapData,destMapData,m_factor,costMapInfo.width,costMapInfo.height);
		std::cout<< "Increased Resolution by factor of  "<< m_factor << std::endl;
		full_solution = sbpl_planner.planxythetamlevlat(planner,from,to,mot_prim_file_name.c_str(),destMapData,sbpl_planner.planner_info);
	}
	else
	{
		std::cout<< "Original resolution  "<< std::endl;
		full_solution = sbpl_planner.planxythetamlevlat(planner,from,to,mot_prim_file_name.c_str(),mapData,sbpl_planner.planner_info);
	}
	if (full_solution.size() > 0)
	{

		m_fullPath.poses.clear();
		m_fullPath.header.frame_id = m_globalFrame;
		for (auto path : full_solution)
		{
			geometry_msgs::PoseStamped pathPose;
			pathPose.header.frame_id = m_globalFrame;

			pathPose.pose.position.x = path[0] + map_info.offsetX;
			pathPose.pose.position.y = path[1] + map_info.offsetY;
			tf2::Quaternion quater;
			quater.setRPY(0, 0, path[2]);
			tf2::convert(quater, pathPose.pose.orientation);
			m_fullPath.poses.push_back(pathPose);
		}
		//globalPlanPublisher.publish(m_fullPath);
		return true;
	}
	else
	{
		ROS_INFO("something wrong . unable to plan ");
	}
	return false;
}

/**
* @brief  Sets the Environment variables for passing to the planner.Updated from parameter server and costmap data
* @param mapInfo contains the environment variables from the costmap
*/
void Environment::setEnvironmentVariables(CostMapInfo &map_info)
{

	if(m_factor !=  1.0)
	{
		sbpl_planner.planner_info.width           = map_info.width / m_factor;
        sbpl_planner.planner_info.height          = map_info.height / m_factor;
        sbpl_planner.planner_info.cell_size       = map_info.resolution * m_factor;
	}
	else
	{
		sbpl_planner.planner_info.width 		 = map_info.width;
		sbpl_planner.planner_info.height 		 = map_info.height;
		sbpl_planner.planner_info.cell_size 	 = map_info.resolution;
	}
	

	sbpl_planner.planner_info.obsthresh 						 = m_obstacleThreshold;
	sbpl_planner.planner_info.cost_inscribed_thresh 			 = m_costInscribedThreshold;
	sbpl_planner.planner_info.cost_possibly_circumscribed_thresh = m_costPossiblyCircumscribedThreshold;
	sbpl_planner.planner_info.nominalvel 					     = m_nominalVelocity;
	sbpl_planner.planner_info.timetoturn45degsinplace 			 = m_timeToTurn45DegreeInplace;
	sbpl_planner.planner_info.robotWidth 						 = m_robotWidth;
	sbpl_planner.planner_info.robotLength 						 = m_robotLength;
	sbpl_planner.planner_info.allocatedTimeSecs 			     = m_allocatedTimeSecs;
	sbpl_planner.planner_info.initialEpsilon 					 = m_initialEpsilon;
	//std::cout << "setEnvironmentVariables()" << m_nominalVelocity << m_timeToTurn45DegreeInplace << m_obstacleThreshold << std::endl;
}


/**
* @brief  Updates the costmap by the factor of m_factor
*/

void Environment::resizeMatrix(const std::vector<int> &sourceMatrix,std::vector<int> &destMatrix,const int &factor,const int &width,const int &height)
{
	Mat source, dest;
	source.create(width, height, CV_8UC1);
	for (int i = 0; i < height; i++)
	{
		for (int j = 0; j < width; j++)
		{
			source.at<uchar>(i, j) = sourceMatrix[(height * i) + j];
		}
	}

	resize(source, dest, Size(width/factor, height/factor), INTER_LINEAR);
	for (int i = 0; i < (height/factor); i++)
	{
		for (int j = 0; j < (width/factor); j++)
		{
			destMatrix.push_back((int)dest.at<uchar>(i, j));  /*Need to verify*/
		}
	}
}



/**
* @brief  Getter for the global path computed
* @return Returns the global path
*/
nav_msgs::Path Environment::getGlobalPath()
{
	return m_fullPath;
}

