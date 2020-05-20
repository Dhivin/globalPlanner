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
* @brief  Reconfiguration Call back
*/
void Environment::reconfigureCallback(planner::plannerConfig &config, uint32_t level)
{
    ROS_INFO_ONCE("In reconfiguration call back");

	m_allocatedTimeSecs		= config.allocatedTimeSecs;
	m_initialEpsilon		= config.initialEpsilon;



    // /*The first time we're called, we just want to make sure we have the original configuration
    //  * if restore defaults is set on the parameter server, prevent looping    */
    if (config.restore_defaults)
    {
        ROS_WARN_ONCE("Environment:: Re-setting default values");
        config 					= defaultConfig;
        config.restore_defaults = false;
    }

    if (!m_setup)
    {
        lastConfig 		= config;
        defaultConfig   = config;
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

bool Environment::plan(std::vector<double>& from, std::vector<double>& to,CostMapInfo& costMapInfo, const std::vector<int>& mapData,std::string& path)
{
	std::string searchDir = "forward";
	std::string plannerType = "arastar";
	PlannerType planner = sbpl_planner.StrToPlannerType(plannerType.c_str());
	std::string mot_prim_file_name = path + "/config/robot1.mprim";
	bool forwardSearch = !strcmp(searchDir.c_str(), "forward");

    CostMapInfo map_info    = costMapInfo;
	setEnvironmentVariables(map_info);
	std::vector<std::vector<double>> full_solution;

	full_solution = sbpl_planner.planxythetamlevlat(planner,from,to,mot_prim_file_name.c_str(),mapData,sbpl_planner.planner_info);

	if (full_solution.size() > 0)
	{

		m_fullPath.poses.clear();
        m_fullPath.header.frame_id = "map";
		for (auto path : full_solution)
		{
			geometry_msgs::PoseStamped pathPose;
			pathPose.header.frame_id = "map";

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
void Environment::setEnvironmentVariables(CostMapInfo& map_info)
{
	sbpl_planner.planner_info.width                               = map_info.width;
	sbpl_planner.planner_info.height                              = map_info.height;
	sbpl_planner.planner_info.cell_size                           = map_info.resolution;
	sbpl_planner.planner_info.obsthresh                           = m_obstacleThreshold;
	sbpl_planner.planner_info.cost_inscribed_thresh               = m_costInscribedThreshold;
	sbpl_planner.planner_info.cost_possibly_circumscribed_thresh  = m_costPossiblyCircumscribedThreshold;
	sbpl_planner.planner_info.nominalvel                          = m_nominalVelocity;
	sbpl_planner.planner_info.timetoturn45degsinplace             = m_timeToTurn45DegreeInplace;
	sbpl_planner.planner_info.robotWidth                          = m_robotWidth;
	sbpl_planner.planner_info.robotLength                         = m_robotLength;
	sbpl_planner.planner_info.allocatedTimeSecs                   = m_allocatedTimeSecs;
	sbpl_planner.planner_info.initialEpsilon                      = m_initialEpsilon;
    //std::cout << "setEnvironmentVariables()" << m_nominalVelocity << m_timeToTurn45DegreeInplace << m_obstacleThreshold << std::endl;  
}

/**
* @brief  Initializes the environment variables from Parameter server
*/
void Environment::initializeEnviromentVariables()
{
	nh_.getParam("/global_planner/obstacleThreshold", m_obstacleThreshold); 
	nh_.getParam("/global_planner/costInscribedThreshold", m_costInscribedThreshold); 
	nh_.getParam("/global_planner/costPossiblyCircumscribedThreshold", m_costPossiblyCircumscribedThreshold); 
	nh_.getParam("/global_planner/nominalVelocity", m_nominalVelocity); 
	nh_.getParam("/global_planner/timeToTurn45DegreeInplace", m_timeToTurn45DegreeInplace); 
	nh_.getParam("/global_planner/robotWidth", m_robotWidth); 	
	nh_.getParam("/global_planner/robotLength", m_robotLength); 
	//ROS_ERROR("initialized Environment variables : %d   %f   %f",m_obstacleThreshold,m_nominalVelocity,m_timeToTurn45DegreeInplace);
}


/**
* @brief  Getter for the global path computed
* @return Returns the global path
*/
nav_msgs::Path Environment::getGlobalPath()
{
    return m_fullPath;
}