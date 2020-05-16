#include <ros/ros.h>
#include <iostream>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/package.h>
#include <nav_msgs/Path.h>
#include "sbpl_includes.h"



#define CELL_GRID_SIZE 0.15

class TestPath
{
	ros::NodeHandle nh;
	ros::Subscriber clickedPoints;
	ros::Subscriber odom_subscriber;	
	ros::Publisher markpoints ;
	double current_x;
	double current_y;
	double current_z;
	double current_w;
	double current_theta;

	double target_x;
	double target_y;
	double target_z;
	double target_theta ;
	
	nav_msgs::Path full_path;

	std::string path ;

	public:
	TestPath(ros::NodeHandle *nodeHandle,std::string path):nh(*nodeHandle),path(path)
	{
		clickedPoints = nh.subscribe("/clicked_point",1,&TestPath::clicked_point_callback,this);
		odom_subscriber = nh.subscribe("/amcl_pose", 1,&TestPath::odom_callback,this);

		markpoints = nh.advertise<nav_msgs::Path>("/mark_map",10);
		current_x = 0.00;
		current_y = 0.00;
		current_z = 0.00;
		current_w = 0.00;
		target_x = 0.00;
		target_z = 0.00;
		target_theta = 0.00;
		current_theta = 0.00 ;
		full_path.header.frame_id = "map";
	}	

	void odom_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg)
	{
		current_x = msg->pose.pose.position.x+10;
		current_y = msg->pose.pose.position.y+10;
		current_z = msg->pose.pose.orientation.z;
		current_w = msg->pose.pose.orientation.w;

		tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
		tf::Matrix3x3 m(q);
		double current_roll,current_pitch,current_yaw;	
		m.getRPY(current_roll, current_pitch, current_yaw);
		current_theta = current_yaw ;

	}
	void clicked_point_callback(const geometry_msgs::PointStampedConstPtr &msg)
	{
		target_x = msg->point.x+10 ;
		target_y = msg->point.y+10 ;
		target_theta = msg->point.z ;

		std::cout << "target position "  << target_x << " "  << target_y << " " << target_theta << '\n';
		std::cout << "current position "  << current_x << " "  << current_y << " " << current_theta << '\n';
		std::vector<double> start,end;
		start.push_back(current_x);
		start.push_back(current_y);
		start.push_back(0.00);
		end.push_back(target_x);
		end.push_back(target_y);
		end.push_back(0.00);
		plan(start,end);
		
	}
	bool plan(std::vector<double> from, std::vector<double> to)
	{
		std::string searchDir = "forward";
		std::string plannerType = "arastar";
		PlannerType planner = StrToPlannerType(plannerType.c_str());
		std::string env_file_name = path + "/config/env_sorter.cfg";
		std::string mot_prim_file_name = path + "/config/test.mprim";
		bool forwardSearch = !strcmp(searchDir.c_str(), "forward");
		bool usingMotionPrimitives = true;
		unsigned char **grid;
		int grid_height;
		int grid_width;
		bool first_run = false;
		int dx = -100;
		int dy = -100;
		int robot_id = 0;
		bool request_replan = true;
		//start = from;
		//end = to;
		// if (request_replan)
		// {
		//     dx = collision_dx;
		//     dy = collision_dy;
		//     if (dx == 0)
		//         dx = -100;
		//     if (dy == 0)
		//         dy = -100;
		// }

		int x =10;
		std::vector<std::vector<int> > full_solution;
		full_solution = planxythetamlevlat(planner, env_file_name.c_str(), mot_prim_file_name.c_str(), forwardSearch,from,to);
//		full_solution = planxythetalat(planner, env_file_name.c_str(), mot_prim_file_name.c_str(), forwardSearch,from,to);

		// // for(int i = 0; i < grid_height; i++)
		// //      delete[] grid[i];
		// //                        delete[] grid;

		if (full_solution.size() > 0)
		{
			//assign_path(full_solution);
			// request_replan = false;
			ROS_INFO("Planning done");
			std::vector<std::vector<double>> current_full_path;
			for(auto path : full_solution)
			{
				std::vector<double> v;
				v.push_back(DISCXY2CONT(path[0], CELL_GRID_SIZE));
                               	v.push_back(DISCXY2CONT(path[1], CELL_GRID_SIZE));
                                v.push_back(DiscTheta2Cont(path[2], 4));
				current_full_path.push_back(v);

			}
			full_path.poses.clear();
			for(auto path: current_full_path)
			{
				geometry_msgs::PoseStamped pose;
				pose.header.frame_id = "map";
				pose.pose.position.x = path[0] -10;
				pose.pose.position.y = path[1] -10;
				tf2::Quaternion quater ;
				quater.setRPY(0,0,path[2]);
				tf2::convert(quater, pose.pose.orientation);
				full_path.poses.push_back(pose);
			}		
			markpoints.publish(full_path);
			return true;
		}
		else
		{
			ROS_INFO("something wrong . unable to plan "  );
		}
		return false;
	}

};
int main (int argc,char *argv[])
{
	ros::init(argc,argv,"test_path");
	ros::NodeHandle nh_;
	std::string path =  ros::package::getPath("planner");

	TestPath test(&nh_,path);
	while(ros::ok())
	{	
		ros::spinOnce();
		usleep(25000);
	}
}


