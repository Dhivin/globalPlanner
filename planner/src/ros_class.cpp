#include <ros/console.h>
#include "ros_class.h"
/**
* @brief  Constructor for the RosClass
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

}

/**
* @brief  Destructor for the RosClass
*/
RosClass::~RosClass()
{
}

/**
* @brief  Callback for updating the global costmap data
*/
void RosClass::costmapCallback(const nav_msgs::OccupancyGridConstPtr data)
{
    //ROS_WARN("Costmap Callback");
    m_mapData.clear();
    m_gridWidth  = data->info.width;
    m_gridHeight = data->info.height;
    m_resolution = data->info.resolution;
    m_offsetX    = data->info.origin.position.x;
    m_offsetY    = data->info.origin.position.y;
    for (auto i : data->data)
    {
        this->m_mapData.push_back(i);
    }

//  	Mat sourceImage,destinationImage;
//  	sourceImage.create(m_gridWidth,m_gridHeight, CV_8UC1);
//     for (int y = 0; y < m_gridHeight; y++)
//     {
//         for (int x = 0; x < m_gridWidth; x++)
//         {
//             sourceImage.at<uchar>(x, y) = data->data[x + y* m_gridHeight];//15*i + j;
//             //EnvNAVXYTHETALATCfg.Grid2D[x][y] = map_data[x + y * width];
//         }
//     }

//    // std::cout << "sourceImage = " << std::endl << " "  << " "  << sourceImage.size << std::endl  << std::endl ;
    
//     resize(sourceImage, destinationImage, Size(m_gridWidth/2, m_gridHeight/2),INTER_LINEAR);

   // std::cout << "M = " << std::endl  << " "  << " "  << destinationImage.size << std::endl  << std::endl ;

}

/**
* @brief  Callback for updating the robot position data
*/
void RosClass::robot_cb(const geometry_msgs::Pose& msg)
{
    start.clear();
    m_robotPose = msg;
    m_robotX    = msg.position.x - m_offsetX;
    m_robotY    = msg.position.y - m_offsetY;

    tf::Quaternion q(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    m_robotTheta = yaw;
    if(yaw < 0.0)
    {
        m_robotTheta = yaw + 6.28;
    }

    start.push_back(m_robotX);
    start.push_back(m_robotY);
    start.push_back(m_robotTheta);
    //std::cout << "current position " << m_robotX << " " << m_robotY << " " << m_robotTheta << '\n';

}

/**
* @brief  Callback for updating the final goal pose
*/
void RosClass::goalCallback(const move_base_msgs::MoveBaseActionGoal &goal_msg)
{


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

    if(yaw < 0.0)
    {
        m_goalTheta = yaw + 6.28;
    }

    end.push_back(m_goalX);
    end.push_back(m_goalY);
    end.push_back(m_goalTheta);
    //std::cout << "target position " << m_goalX << " " << m_goalY << " " << m_goalTheta << '\n';

}

/**
* @brief Getter functions
*/

/**
* @brief  Function for getting the gridwidth
* @return the grid width from the costmap  
*/
int RosClass::getGridWidth()
{
    return m_gridWidth;
}



/**
* @brief  Function for getting the grid height
* @return the grid height from the costmap  
*/
int RosClass::getGridHeight()
{
    return m_gridHeight;
}

/**
* @brief  Function for getting the grid resolution
* @return the resolution from the costmap  
*/
double RosClass::getResolution()
{
    return m_resolution;
}

/**
* @brief  Function for getting the offsetX
* @return the offset in X axis for the costmap   
*/
int RosClass::getOffsetX()
{
    return m_offsetX;
}

/**
* @brief  Function for getting the offset Y
* @return the offset in Y axis for the costmap  
*/
int RosClass::getOffsetY()
{
    return m_offsetY;
}

/**
* @brief  Function for getting the map data
* @return pccupancy matrix from the costmap 
*/
std::vector<int> RosClass::getMapdata()
{
    return m_mapData;
}

/**
* @brief  Function for getting the start point
* @return the coordinates of the start position 
*/
std::vector<double> RosClass::getStart()
{
    return start;
}

/**
* @brief  Function for getting the goal point
* @return the coordinates of target goal position 
*/
std::vector<double> RosClass::getGoal()
{
    return end;
}

/**
* @brief  Function for getting getting the current robot position
*/
geometry_msgs::Pose RosClass::getRobotPose()
{
    return m_robotPose;
}

/**
* @brief  Function for getting getting the goal position
*/
geometry_msgs::Pose RosClass::getGoalPose()
{
    return m_goalPose;
}



/**
* @brief Setter functions
*/

/**
* @brief  Publishes the path calculated by the planner
*/
void RosClass::publishPath(nav_msgs::Path m_Path)
{
    globalPlanPublisher.publish(m_Path);
}