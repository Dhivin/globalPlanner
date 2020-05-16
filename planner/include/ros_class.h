#ifndef ROS_CLASS_H
#define ROS_CLASS_H


#include <ros/ros.h>
#include <ros/time.h>
#include <iostream>
#include <vector>


#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>

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

    /* Accessors */
   
    double xytest = 10.0;
    
private:
     /**
    * @brief NodeHandle ROS
    */
    ros::NodeHandle nh;

    void costmapCallback(const nav_msgs::OccupancyGridConstPtr data);

    void odomCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg);

	void clickedPointCallback(const geometry_msgs::PointStampedConstPtr &msg);

    // /**
    //  * @brief ROS Subscribers
    //  */
    ros::Subscriber clickedPoints;
    ros::Subscriber odomSubscriber;
    ros::Subscriber globalCostmapSubscriber;


 	int grid_height;
	int grid_width;
	int offset_x;
	int offset_y;
  	double resolution ;   
    
    std::vector<int> data;



    
};

#endif