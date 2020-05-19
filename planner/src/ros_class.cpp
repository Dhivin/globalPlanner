#include <ros/console.h>
#include "ros_class.h"
/**
    * @brief  Constructor for the Computations
    */
RosClass::RosClass()
{
    /**
    * @brief ROS Subscribers
    */
    goalSubscriber = nh.subscribe("/move_base/goal", 1, &RosClass::goalCallback, this);
    globalCostmapSubscriber = nh.subscribe("/move_base/global_costmap/costmap", 1, &RosClass::costmapCallback, this);
    robotPoseSubscriber = nh.subscribe("/robot_pose", 1, &RosClass::robot_cb, this);

    /**
	* @brief ROS Publishers
    */
    globalPlanPublisher = nh.advertise<nav_msgs::Path>("/global_planner", 10);

    ROS_WARN("ROS class constructor called");
    /*Subscribers*/
    // clickedPoints               = nh.subscribe("/clicked_point", 1, &RosClass::clickedPointCallback, this);
    // odomSubscriber              = nh.subscribe("/amcl_pose", 1, &RosClass::odomCallback, this);
    // globalCostmapSubscriber     = nh.subscribe("/move_base/global_costmap/costmap", 1, &RosClass::costmapCallback, this);
    /*Publishers*/
}

RosClass::~RosClass()
{
}

void RosClass::costmapCallback(const nav_msgs::OccupancyGridConstPtr data)
{
    //ROS_WARN("Costmap Callback");
    m_mapData.clear();
    m_gridWidth = data->info.width;
    m_gridHeight = data->info.height;
    m_resolution = data->info.resolution;
    m_offsetX = data->info.origin.position.x;
    m_offsetY = data->info.origin.position.y;
    for (auto i : data->data)
    {
        this->m_mapData.push_back(i);
    }
}

void RosClass::robot_cb(const geometry_msgs::Pose& msg)
{
    m_robotPose = msg;
    m_robotX    = msg.position.x - m_offsetX;
    m_robotY    = msg.position.y - m_offsetY;

    tf::Quaternion q(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    m_robotTheta = yaw;
}

void RosClass::goalCallback(const move_base_msgs::MoveBaseActionGoal &goal_msg)
{

    start.clear();
    end.clear();
    m_goalPose.position = goal_msg.goal.target_pose.pose.position;
    m_goalPose.orientation = goal_msg.goal.target_pose.pose.orientation;

    m_goalX = m_goalPose.position.x - m_offsetX;
    m_goalY = m_goalPose.position.y - m_offsetY;

    tf::Quaternion q(0.0, 0.0, goal_msg.goal.target_pose.pose.orientation.z, goal_msg.goal.target_pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    m_goalTheta = yaw;

    //std::cout << "target position " << m_goalX << " " << m_goalY << " " << m_goalTheta << '\n';
    //std::cout << "current position " << m_robotX << " " << m_robotY << " " << m_robotTheta << '\n';

    start.push_back(m_robotX);
    start.push_back(m_robotY);
    start.push_back(m_robotTheta);

    end.push_back(m_goalX);
    end.push_back(m_goalY);
    end.push_back(m_goalTheta);
    //plan(start, end);
}

/**
* @brief Getter functions
*/

int RosClass::getGridWidth()
{
    return m_gridWidth;
}

int RosClass::getGridHeight()
{
    return m_gridHeight;
}

double RosClass::getResolution()
{
    return m_resolution;
}

int RosClass::getOffsetX()
{
    return m_offsetX;
}

int RosClass::getOffsetY()
{
    return m_offsetY;
}

std::vector<int> RosClass::getMapdata()
{
    return m_mapData;
}

std::vector<double> RosClass::getStart()
{
    return start;
}

std::vector<double> RosClass::getGoal()
{
    return end;
}

geometry_msgs::Pose RosClass::getRobotPose()
{
    return m_robotPose;
}

geometry_msgs::Pose RosClass::getGoalPose()
{
    return m_goalPose;
}



/**
* @brief Setter functions
*/

void RosClass::publishPath(nav_msgs::Path m_Path)
{
    globalPlanPublisher.publish(m_Path);
}