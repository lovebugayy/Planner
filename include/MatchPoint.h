//
// Created by youfa on 2021/11/3.
//

#ifndef UNTITLED_MATCHPOINT_H
#define UNTITLED_MATCHPOINT_H

#include <vector>
#include "DataType.h"

using namespace std;

namespace ReferencePathMatcher{
    std::vector<WaypointData> getStoredRefPoints();
    WaypointData matchToRefernecePathByXY(const std::vector< WaypointData >& ref_line_points,
                             double x,
                             double y);
    WaypointData findProjectionPoint(const WaypointData& p0,
                                                           const WaypointData& p1,
                                                           const double x,
                                                           const double y);
    WaypointData interpolateUsingLinearApproximation(const WaypointData& p0,
                                                                           const WaypointData& p1,
                                                                           const double s);
    double slerp(const double a0, const double t0, const double a1, const double t1, const double t);
    double calcNormalizedAngle(const double angle);
}//end namespace ReferencePathMather

namespace CartesianFrenetConverter{
    VehicleState convertCartesianToFrenetCoord(const WaypointData& matched_point,
                                  const WaypointData& cartesian_state);
}//end namespace CartesianFrenetConverter

namespace TrajectoryStitcher
{
    std::vector< TrajectoryAllInfo > addSLInfoForTajectoryPoint(TrajectoryPoint& current_vehicle_pose);
    void show( std::vector< TrajectoryAllInfo > trajectory_all_info);
}//end namespace TrajectoryStitcher

#endif //UNTITLED_MATCHPOINT_H
