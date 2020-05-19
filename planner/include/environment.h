#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H
#include <ros/ros.h>
#include <ros/time.h>
#include <iostream>
#include <vector>
#include <planner/plannerConfig.h> 
#include <dynamic_reconfigure/server.h>

//#include "utils.h"
#include "sbpl_includes.h"


#include <nav_msgs/Path.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


class Environment
{
public:

     /**
     * @brief  Constructor for the Environment class
     */

    Environment();
    /**
    * @brief  Destructor for the Environment class
    */
    ~Environment();

    struct MapInfo
    {
        int width;
        int height;
        int offsetX;
        int offsetY;
        double resolution;
    };
     MapInfo m_mapInfo;

	bool plan(std::vector<double> from, std::vector<double> to,MapInfo& mapInfo,std::vector<int>& mapData,std::string path);
	void initializeEnviromentVariables();
	void setEnvironmentVariables(MapInfo& map_info);
    nav_msgs::Path getGlobalPath();


    
private:
	
    ros::NodeHandle nh_;
	/**
     * @brief Reconfiguration callback functions
    */

    dynamic_reconfigure::Server<planner::plannerConfig> server;
    dynamic_reconfigure::Server<planner::plannerConfig>::CallbackType f;
	void reconfigureCallback(planner::plannerConfig &config, uint32_t level);




    int m_obstacleThreshold,m_costInscribedThreshold,m_costPossiblyCircumscribedThreshold;
    
    double m_nominalVelocity,m_timeToTurn45DegreeInplace;
    double m_robotWidth,m_robotLength;
    double m_allocatedTimeSecs,m_initialEpsilon;

    bool m_setup = false;



    nav_msgs::Path m_fullPath;

    planner::plannerConfig lastConfig;
    planner::plannerConfig defaultConfig;
    
	SBPLIncludes sbpl_planner;
    
};

#endif