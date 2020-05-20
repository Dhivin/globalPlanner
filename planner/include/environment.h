#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H
#include <ros/ros.h>
#include <ros/time.h>
#include <iostream>
#include <vector>
#include <planner/plannerConfig.h> 
#include <dynamic_reconfigure/server.h>

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

    /**
    * @brief  Structure for updating the environment variables from the costmap data
    */
    struct CostMapInfo
    {
        int width;
        int height;
        int offsetX;
        int offsetY;
        double resolution;
    };
    CostMapInfo costMapInfo;


    /**
    * @brief  Member function used to call the planner
    * @param from Starting point for the planner
    * @param to Goal point for the planner 
    * @param mapInfo contains the environment variables from the costmap
    * @param mapData contains the map data from the costmap
    * @param path contains the directory where the motion primitive file is placed
    */
	bool plan(std::vector<double>& from, std::vector<double>& to,CostMapInfo& costMapInfo,const std::vector<int>& mapData,std::string& path);


    /**
    * @brief  Getter for the global path computed
    * @return Returns the global path
    */
    nav_msgs::Path getGlobalPath();


    
private:
    /**
    * @brief NodeHandle ROS
    */		
    ros::NodeHandle nh_;

	/**
     * @brief Reconfiguration callback functions
    */
    dynamic_reconfigure::Server<planner::plannerConfig> server;
    dynamic_reconfigure::Server<planner::plannerConfig>::CallbackType f;
	void reconfigureCallback(planner::plannerConfig &config, uint32_t level);




    /**
    * @brief  Initializes the environment variables from Parameter server
    */
	void initializeEnviromentVariables();

    /**
    * @brief  Sets the Environment variables for passing to the planner.Updated from parameter server and costmap data
    * @param mapInfo contains the environment variables from the costmap
    */
	void setEnvironmentVariables(CostMapInfo& costMapInfo);

    /*Member variables*/
    int m_obstacleThreshold,m_costInscribedThreshold,m_costPossiblyCircumscribedThreshold;
    double m_nominalVelocity,m_timeToTurn45DegreeInplace;
    double m_robotWidth,m_robotLength;
    double m_allocatedTimeSecs,m_initialEpsilon;
    bool m_setup = false;
    nav_msgs::Path m_fullPath;
    planner::plannerConfig lastConfig;
    planner::plannerConfig defaultConfig;
    
    /*Instances of the classe  SBPLIncludes*/
	SBPLIncludes sbpl_planner;
    
};

#endif