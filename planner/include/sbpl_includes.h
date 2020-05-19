/*
 * Copyright (c) 2008, Maxim Likhachev
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Carnegie Mellon University nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#include <cmath>
#include <cstring>
#include <iostream>
#include <sstream>
#include <string>
#include <map>
#include <chrono>

using namespace std;

#include <sbpl/headers.h>

enum PlannerType
{
    INVALID_PLANNER_TYPE = -1,
    PLANNER_TYPE_ADSTAR,
    PLANNER_TYPE_ARASTAR,
    PLANNER_TYPE_PPCP,
    PLANNER_TYPE_RSTAR,
    PLANNER_TYPE_VI,
    PLANNER_TYPE_ANASTAR,

    NUM_PLANNER_TYPES
};

enum EnvironmentType
{
    INVALID_ENV_TYPE = -1,
    ENV_TYPE_2D,
    ENV_TYPE_2DUU,
    ENV_TYPE_XYTHETA,
    ENV_TYPE_XYTHETAMLEV,
    ENV_TYPE_ROBARM,

    NUM_ENV_TYPES
};

class SBPLIncludes
{
public:
    SBPLIncludes();
    ~SBPLIncludes();

    struct MapInfo
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

    MapInfo map_info;

    std::string PlannerTypeToStr(PlannerType plannerType);
    PlannerType StrToPlannerType(const char *str);
    std::string EnvironmentTypeToStr(EnvironmentType environmentType);
    EnvironmentType StrToEnvironmentType(const char *str);
    std::vector<std::vector<double>> planxythetamlevlat(PlannerType plannerType,std::vector<double> start, std::vector<double> end,const char *smotPrimFile, std::vector<int> &map_data,MapInfo map_info);


};