#ifndef ROS_CLASS_H
#define ROS_CLASS_H


#include <ros/ros.h>
#include <ros/time.h>
#include <iostream>
#include <vector>


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




class RosClass
{
public:

     /**
     * @brief  Constructor for the Ros class
     */

    RosClass();
    /**
    * @brief  Destructor for the Ros class
    */
    ~RosClass();

    /**
    * @brief Getter functions
    */

    int getGridWidth();
    int getGridHeight();
    int getOffsetX();
    int getOffsetY();
    double getResolution();
    std::vector<int> getMapdata();
    std::vector<double> getStart();
    std::vector<double> getGoal();

    geometry_msgs::Pose getRobotPose();    
    geometry_msgs::Pose getGoalPose(); 

    void publishPath(nav_msgs::Path m_Path);
    
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
	void robot_cb(const geometry_msgs::Pose& robot_msg);
	void goalCallback(const move_base_msgs::MoveBaseActionGoal &goal_msg);





    int m_gridWidth, m_gridHeight,m_offsetX, m_offsetY;
    
    double m_resolution;
    double m_robotX, m_robotY, m_robotTheta;
    double m_goalX, m_goalY, m_goalTheta;

    std::vector<int> m_mapData;
    std::vector<double> start, end;

    geometry_msgs::Pose m_goalPose,m_robotPose;
    
};

#endif