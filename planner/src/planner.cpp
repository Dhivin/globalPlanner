#include <iostream>
#include <ros/ros.h>
#include <ros/time.h>
#include <vector>
#include <sbpl_includes.h>
#include <ros/package.h>


bool plan(std::vector<double> from, std::vector<double> to , std::string path)
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

    // // for(int i = 0; i < grid_height; i++)
    // //      delete[] grid[i];
    // //                        delete[] grid;

    if (full_solution.size())
    {
        //assign_path(full_solution);
        // request_replan = false;
        ROS_INFO("Planning done");

        return true;
    }
    else
    {
        ROS_INFO("something wrong . unable to plan "  );
    }
    return false;
}

/**
 * @brief Main function
*/
int main(int argc, char **argv)
{
    ros::init(argc, argv, "planner");
    ros::NodeHandle n;
    ros::Rate rate(20);
    std::string path =  ros::package::getPath("planner");
    //char* motPrimFilename = (char *)"/home/ra-1/catkin_dev/src/planner/config/test.mprim";

    ROS_INFO("Planner initialized");
    vector<double > start;
    start.push_back(1.275);
    start.push_back(1.575);
    start.push_back(0.00);
    vector<double> end;
    end.push_back(2.325);
    end.push_back(1.575);
    end.push_back(1.57);
    //vector<double > end(1.0, 15.0,2.31);
   // plan(start,end,path);
    larr();
    while (ros::ok())
    {
       rate.sleep();
       ros::spinOnce();
    }
    return 0;
}
