#include <iostream>
#include <cmath>
#include <iomanip>
#include "../include/MatchPoint.h"

using namespace std;
using namespace TrajectoryStitcher;

int main() {
    TrajectoryPoint current_vehicle_pose;
    current_vehicle_pose.x=1.1;
    current_vehicle_pose.y=1.5;
    current_vehicle_pose.yaw=M_PI_4/2;
    current_vehicle_pose.v=0.1;
    current_vehicle_pose.a=0;
    current_vehicle_pose.kappa=0;

    std::vector< TrajectoryAllInfo > trajectory_all_info=TrajectoryStitcher::addSLInfoForTajectoryPoint(current_vehicle_pose);
    TrajectoryStitcher::show(trajectory_all_info);

    return 0;
}
