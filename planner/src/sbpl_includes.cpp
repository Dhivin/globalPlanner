#include "sbpl_includes.h"

SBPLIncludes::SBPLIncludes()
{
}

SBPLIncludes::~SBPLIncludes()
{
}
std::string SBPLIncludes::PlannerTypeToStr(PlannerType plannerType)
{
    switch (plannerType)
    {
    case PLANNER_TYPE_ADSTAR:
        return std::string("adstar");
    case PLANNER_TYPE_ARASTAR:
        return std::string("arastar");
    case PLANNER_TYPE_PPCP:
        return std::string("ppcp");
    case PLANNER_TYPE_RSTAR:
        return std::string("rstar");
    case PLANNER_TYPE_VI:
        return std::string("vi");
    case PLANNER_TYPE_ANASTAR:
        return std::string("anastar");
    default:
        return std::string("invalid");
    }
}

PlannerType SBPLIncludes::StrToPlannerType(const char *str)
{
    if (!strcmp(str, "adstar"))
    {
        return PLANNER_TYPE_ADSTAR;
    }
    else if (!strcmp(str, "arastar"))
    {
        return PLANNER_TYPE_ARASTAR;
    }
    else if (!strcmp(str, "ppcp"))
    {
        return PLANNER_TYPE_PPCP;
    }
    else if (!strcmp(str, "rstar"))
    {
        return PLANNER_TYPE_RSTAR;
    }
    else if (!strcmp(str, "vi"))
    {
        return PLANNER_TYPE_VI;
    }
    else if (!strcmp(str, "anastar"))
    {
        return PLANNER_TYPE_ANASTAR;
    }
    else
    {
        return INVALID_PLANNER_TYPE;
    }
}

std::string SBPLIncludes::EnvironmentTypeToStr(EnvironmentType environmentType)
{
    switch (environmentType)
    {
    case ENV_TYPE_2D:
        return std::string("2d");
    case ENV_TYPE_2DUU:
        return std::string("2duu");
    case ENV_TYPE_XYTHETA:
        return std::string("xytheta");
    case ENV_TYPE_XYTHETAMLEV:
        return std::string("xythetamlev");
    case ENV_TYPE_ROBARM:
        return std::string("robarm");
    default:
        return std::string("invalid");
    }
}

EnvironmentType SBPLIncludes::StrToEnvironmentType(const char *str)
{
    if (!strcmp(str, "2d"))
    {
        return ENV_TYPE_2D;
    }
    else if (!strcmp(str, "2duu"))
    {
        return ENV_TYPE_2DUU;
    }
    else if (!strcmp(str, "xytheta"))
    {
        return ENV_TYPE_XYTHETA;
    }
    else if (!strcmp(str, "xythetamlev"))
    {
        return ENV_TYPE_XYTHETAMLEV;
    }
    else if (!strcmp(str, "robarm"))
    {
        return ENV_TYPE_ROBARM;
    }
    else
    {
        return INVALID_ENV_TYPE;
    }
}

std::vector<std::vector<double>> SBPLIncludes::planxythetamlevlat(PlannerType plannerType,std::vector<double> start, std::vector<double> end,const char *smotPrimFile, std::vector<int> &map_data,MapInfo map_info)
{
    int bRet = 0;
    double allocated_time_secs = map_info.allocatedTimeSecs;//10.0; //in seconds
    double initialEpsilon = map_info.initialEpsilon;//3.0;
    MDPConfig MDPCfg;
    bool bsearchuntilfirstsolution = false;
    bool bforwardsearch = true;

    std::cout << "Initial assumptions  alloted time:"<< allocated_time_secs << " seconds  Epsilon:"<<initialEpsilon <<std::endl;
    //set the perimeter of the robot (it is given with 0,0,0 robot ref. point for which planning is done)
    //this is for the default level - base level
    vector<sbpl_2Dpt_t> perimeterptsV;
    sbpl_2Dpt_t pt_m;
    double halfwidth  = map_info.robotWidth/2;     //0.02;  //0.3;
    double halflength = map_info.robotLength/2;    //0.02; //0.45;
    pt_m.x = -halflength;
    pt_m.y = -halfwidth;
    perimeterptsV.push_back(pt_m);
    pt_m.x = halflength;
    pt_m.y = -halfwidth;
    perimeterptsV.push_back(pt_m);
    pt_m.x = halflength;
    pt_m.y = halfwidth;
    perimeterptsV.push_back(pt_m);
    pt_m.x = -halflength;
    pt_m.y = halfwidth;
    perimeterptsV.push_back(pt_m);

    //Initialize Environment (should be called before initializing anything else)
    EnvironmentNAVXYTHETAMLEVLAT environment_navxythetalat;
    unsigned char *mapdata = NULL;



    // std::cout << "ENV SETUP DONE sbplincludes.cpp" << map_info.nominalvel << "  "<< map_info.timetoturn45degsinplace << "  " << map_info.obsthresh << std::endl;  
    
    if (!environment_navxythetalat.InitializeEnv(map_info.width, map_info.height, mapdata,start[0], start[1], start[2], end[0], end[1], end[2], perimeterptsV, map_info.cell_size, map_info.nominalvel,map_info.timetoturn45degsinplace,map_info.obsthresh, smotPrimFile))
    {
        throw SBPL_Exception("ERROR: InitializeEnv failed");
    }
    
    //setting grid with global cost values
    environment_navxythetalat.InitializeMapdata(map_data);

    environment_navxythetalat.SetStart(start[0], start[1], start[2]);
    environment_navxythetalat.SetGoal(end[0], end[1], end[2]);

    //this is for the second level - upper body level
    vector<sbpl_2Dpt_t> perimeterptsVV[2];
    perimeterptsVV[0].clear();
    halfwidth = 0.02;
    halflength = 0.02;
    pt_m.x = -halflength;
    pt_m.y = -halfwidth;
    perimeterptsVV[0].push_back(pt_m);
    pt_m.x = halflength;
    pt_m.y = -halfwidth;
    perimeterptsVV[0].push_back(pt_m);
    pt_m.x = halflength;
    pt_m.y = halfwidth;
    perimeterptsVV[0].push_back(pt_m);
    pt_m.x = -halflength;
    pt_m.y = halfwidth;
    perimeterptsVV[0].push_back(pt_m);

    //	perimeterptsV.clear();
    //	perimeterptsVV[0].clear();

    //enable the second level
    int numofaddlevels = 1;
    //printf("Number of additional levels = %d\n", numofaddlevels);
    unsigned char cost_inscribed_thresh_addlevels[2];              //size should be at least numofaddlevels
    unsigned char cost_possibly_circumscribed_thresh_addlevels[2]; //size should be at least numofaddlevels
    //no costs are indicative of whether a cell is within inner circle
    cost_inscribed_thresh_addlevels[0] = map_info.cost_inscribed_thresh;//255;       cost_inscribed_thresh
    //no costs are indicative of whether a cell is within outer circle
    cost_possibly_circumscribed_thresh_addlevels[0] = map_info.cost_possibly_circumscribed_thresh;//0;  
    //no costs are indicative of whether a cell is within inner circle
    cost_inscribed_thresh_addlevels[1] = map_info.cost_inscribed_thresh;//255;
    //no costs are indicative of whether a cell is within outer circle
    cost_possibly_circumscribed_thresh_addlevels[1] = map_info.cost_possibly_circumscribed_thresh;//0;
    if (!environment_navxythetalat.InitializeAdditionalLevels(numofaddlevels, perimeterptsVV,
                                                              cost_inscribed_thresh_addlevels,
                                                              cost_possibly_circumscribed_thresh_addlevels))
    {
        std::stringstream ss("ERROR: InitializeAdditionalLevels failed with numofaddlevels=");
        ss << numofaddlevels;
        throw SBPL_Exception(ss.str());
    }

    //set the map for the second level (index parameter for the additional levels and is zero based)
    //for this example, we pass in the same map as the map for the base. In general, it can be a totally different map
    //as it corresponds to a different level
    //NOTE: this map has to have costs set correctly with respect to inner and outer radii of the robot
    //if the second level of the robot has these radii different than at the base level, then costs
    //should reflect this
    //(see explanation for cost_possibly_circumscribed_thresh and
    //cost_inscribed_thresh parameters in environment_navxythetalat.h file)
    int addlevind = 0;
    if (!environment_navxythetalat.Set2DMapforAddLev(
            (const unsigned char **)(environment_navxythetalat.GetEnvNavConfig()->Grid2D), addlevind))
    {
        std::stringstream ss("ERROR: Setting Map for the Additional Level failed with level ");
        ss << addlevind;
        throw SBPL_Exception(ss.str());
    }

    // Initialize MDP Info
    if (!environment_navxythetalat.InitializeMDPCfg(&MDPCfg))
    {
        throw SBPL_Exception("ERROR: InitializeMDPCfg failed");
    }

    // plan a path
    vector<int> solution_stateIDs_V;

    SBPLPlanner *planner = NULL;
    switch (plannerType)
    {
    case PLANNER_TYPE_ARASTAR:
        printf("Initializing ARAPlanner...\n");
        planner = new ARAPlanner(&environment_navxythetalat, bforwardsearch);
        break;
    case PLANNER_TYPE_ADSTAR:
        printf("Initializing ADPlanner...\n");
        planner = new ADPlanner(&environment_navxythetalat, bforwardsearch);
        break;
    case PLANNER_TYPE_RSTAR:
        printf("Invalid configuration: xytheta environment does not support rstar planner...\n");
        break;
    case PLANNER_TYPE_ANASTAR:
        printf("Initializing anaPlanner...\n");
        planner = new anaPlanner(&environment_navxythetalat, bforwardsearch);
        break;
    default:
        printf("Invalid planner type\n");
        break;
    }

    if (planner->set_start(MDPCfg.startstateid) == 0)
    {
        printf("ERROR: failed to set start state\n");
        throw SBPL_Exception("ERROR: failed to set start state");
    }

    if (planner->set_goal(MDPCfg.goalstateid) == 0)
    {
        printf("ERROR: failed to set goal state\n");
        throw SBPL_Exception("ERROR: failed to set goal state");
    }

    planner->set_initialsolution_eps(initialEpsilon);

    //set search mode
    planner->set_search_mode(bsearchuntilfirstsolution);

    //std::cout << "Start Planning " << std::endl;
    auto t1 = std::chrono::high_resolution_clock::now();

    bRet = planner->replan(allocated_time_secs, &solution_stateIDs_V);

    auto t2 = std::chrono::high_resolution_clock::now(); // stop time measurement
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
    std::cout << "Execution time: " << duration << " milliseconds" << std::endl;   

    environment_navxythetalat.PrintTimeStat(stdout);
    vector<sbpl_xy_theta_pt_t> xythetaPath;
    environment_navxythetalat.ConvertStateIDPathintoXYThetaPath(&solution_stateIDs_V, &xythetaPath);

    std::cout << "Done Planning.Size of solution = " << solution_stateIDs_V.size()<<" and including intermediate points: "<<(unsigned int)xythetaPath.size() << std::endl;

//    std::cout << "Full Solution size including intermediate points  = " << (unsigned int)xythetaPath.size() << std::endl;

    
    std::vector<std::vector<double>> path;
    std::vector<double> coordinates;
    int x, y, theta;

    for (size_t i = 0; i < xythetaPath.size(); i++) //solution_stateIDs_V.size()
    {
        coordinates.clear();
        //environment_navxythetalat.GetCoordFromState(solution_stateIDs_V[i], x, y, theta);
        //printf("%d %d %d\t\t%.3f %.3f %.3f\n", x, y, theta, DISCXY2CONT(x, map_info.cell_size), DISCXY2CONT(y, map_info.cell_size), DiscTheta2Cont(theta, 4));


        coordinates.push_back(xythetaPath[i].x);
        coordinates.push_back(xythetaPath[i].y);
        coordinates.push_back(xythetaPath[i].theta);

        path.push_back(coordinates);
    }

    environment_navxythetalat.PrintTimeStat(stdout);

    //print a path
    if (bRet)
    {
        std::cout << "Solution is found " << std::endl;
        std::cout << "\n" << std::endl;
    }
    else
    {
        std::cout << "Solution doesn't exist" << std::endl;
    }

    fflush(NULL);

    delete planner;

    return path;
}
