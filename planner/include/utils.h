#ifndef UTILITIES_H
#define UTILITIES_H

/**
    * @brief  Constructor for the Computations
    */
class Utils
{
public:

     /**
     * @brief  Constructor for the Environment class
     */

    Utils()
    {
        
    }
    /**
    * @brief  Destructor for the Environment class
    */
    ~Utils()
    {
        
    }

    struct EnvironmentInfo
    {
        int width;
        int height;
        int obsthresh;
        int cost_inscribed_thresh;
        int cost_possibly_circumscribed_thresh;
        double cell_size;
        double nominalvel;
        double timetoturn45degsinplace;
        double robotLength;
        double robotWidth;
        double allocatedTimeSecs;
        double initialEpsilon;
    };

    EnvironmentInfo m_envInfo;

};

#endif