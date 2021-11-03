//
// Created by youfa on 2021/11/3.
//
#include <iostream>
#include <iomanip>
#include <cmath>
#include "../include/DataType.h"
#include "../include/MatchPoint.h"

using namespace std;
using namespace ReferencePathMatcher;
using namespace CartesianFrenetConverter;
using namespace TrajectoryStitcher;


    std::vector<WaypointData> ReferencePathMatcher::getStoredRefPoints() {
        std::vector<WaypointData> stored_ref_points;
        for (auto i = 0; i < 10; i++) {
            WaypointData temp_waypoint;
            temp_waypoint.x = (i - 1) * 1;
            temp_waypoint.y = (i - 1) * 1;
            temp_waypoint.yaw = M_PI_4;
            temp_waypoint.d_x = 1;
            temp_waypoint.d_y = 1;
            temp_waypoint.kappa = 0.1;
            temp_waypoint.kappa_ds = 0.01;
            stored_ref_points.emplace_back(temp_waypoint);
        }
        return stored_ref_points;
    }

    WaypointData ReferencePathMatcher::matchToRefernecePathByXY(const std::vector< WaypointData >& ref_line_points,double x,double y)
    {
        //首先找出ref_line中距離點(x,y)最近的點的下摽,然後找出匹配的起點和重點,最後找出匹配的節點
        auto square_distance=[](double x1,double y1,const WaypointData& p)

        {
            double dx=x1-p.x;
            double dy=y1-p.y;
            return dx*dx+dy*dy;
        };
        int min_index=0;
        double min_dis=square_distance(x,y,ref_line_points[0]);
        for(unsigned int i=1;i<ref_line_points.size();i++)
        {
            if(min_dis>square_distance(x,y,ref_line_points[i]))
            {
                min_dis=square_distance(x,y,ref_line_points[i]);
                min_index=i;
            }
        }

        //此處已經找到了min index,需要找出min_start和min_end下表,然後進行插值
        int min_start=(min_index==0)?min_index:min_index-1;
        int min_end=(min_index+1==ref_line_points.size())?min_index: min_index+1;
        if(min_start==min_end)
        {
            return ref_line_points[min_start];
        }
        return ReferencePathMatcher::findProjectionPoint(ref_line_points[min_start],ref_line_points[min_end],x,y);
    }

    WaypointData ReferencePathMatcher::findProjectionPoint(const WaypointData& p0,
                                                           const WaypointData& p1,
                                                           const double x,
                                                           const double y)
    {
        double v0x=x-p0.x;
        double v0y=y-p0.y;

        double v1x=x-p1.x;
        double v1y=y-p1.y;

        double v1_norm=std::sqrt(v1x*v1x+v1y*v1y);
        double v_dot=v0x*v1x+v0y*v1y;

        double delta_s=v1_norm/v_dot;
        return interpolateUsingLinearApproximation(p0,p1,delta_s);
    }

WaypointData ReferencePathMatcher::interpolateUsingLinearApproximation(const WaypointData& p0,const WaypointData& p1,const double s)
{
        //首先計算出一個權重
        double weight=(s-p0.s)/(p1.s-p0.s);
        WaypointData matchedPoint;
        matchedPoint.x=(1-weight)*p0.x+weight*p1.x;
        matchedPoint.y=(1-weight)*p0.y+weight*p1.y;
        matchedPoint.s=(1-weight)*p0.s+weight*p1.s;
        matchedPoint.kappa=(1-weight)*p0.kappa_ds+weight*p1.kappa;
        matchedPoint.kappa_ds=(1-weight)*p0.kappa_ds+weight*p1.kappa_ds;
        matchedPoint.yaw= slerp(p0.yaw, p0.s, p1.yaw, p1.s, s);

        return matchedPoint;
}

double ReferencePathMatcher::slerp(const double a0, const double t0, const double a1, const double t1, const double t)
{
        if(t1-t0<0.00001)
        {
            std::cout<<"the time step is too small"<<std::endl;
            return calcNormalizedAngle(a0);
        }
        double angle_start= calcNormalizedAngle(a0);
        double angle_end= calcNormalizedAngle(a1);
        double angle_step=angle_end-angle_start;
        if(angle_step>M_PI)
        {
            angle_step-=2*M_PI;
        }else if(angle_step<-M_PI)
        {
            angle_step+=2*M_PI;
        }
        const double k=(t-t0)/(t1-t0);
        return angle_start+k*angle_step;
}

double ReferencePathMatcher::calcNormalizedAngle(const double angle)
{
        double angle_new=std::fmod(angle,2*M_PI);
        if(angle_new<0)
        {
            angle_new+=2*M_PI;
        }
        return angle_new-M_PI; //始終保持在(-pi,pi)
}

VehicleState CartesianFrenetConverter::convertCartesianToFrenetCoord(const WaypointData& matched_point,
                                          const WaypointData& cartesian_state){
        //frenet 坐標系上選取的點
        double rx=matched_point.x;
        double ry=matched_point.y;
        double rs=matched_point.s;
        double rkappa=matched_point.kappa;
        double rdkappa=matched_point.kappa_ds;
        double rtheta=matched_point.yaw;

        //汽車當前在地卡爾坐標下的點
        double x=cartesian_state.x;
        double y=cartesian_state.y;
        double theta=cartesian_state.yaw;
        double kappa=cartesian_state.kappa;
        double v=cartesian_state.d_x;
        double a=cartesian_state.d_y;

        //開始轉化
        VehicleState vehicle_state_sl;

        //l
        double dx=x-rx;
        double dy=y-ry;
        double l=dx*dx+dy*dy;
        l=std::copysign(l,(std::cos(rtheta)*dy-std::sin(theta)*dx));
        vehicle_state_sl.l=l;

        //dl
        double one_minus_kappa_l=(1-rkappa*l);
        double delta_theta=theta-rtheta;
        double tan_delta_theta=std::tan(delta_theta);
        vehicle_state_sl.dl=one_minus_kappa_l*tan_delta_theta;

        //ddl
        double cos_delta_theta=std::cos(delta_theta);
        double d_rkappa_l=rdkappa*l+rkappa*vehicle_state_sl.dl;
        vehicle_state_sl.ddl=-d_rkappa_l*tan_delta_theta+(one_minus_kappa_l)/cos_delta_theta/cos_delta_theta*
                (kappa*one_minus_kappa_l/cos_delta_theta-rkappa);

        //s
        vehicle_state_sl.s=rs;

        //ds
        vehicle_state_sl.ds=v*cos_delta_theta/one_minus_kappa_l;

        //dds
        vehicle_state_sl.dds=(a*cos_delta_theta-
                vehicle_state_sl.ds*vehicle_state_sl.ds*
                ((one_minus_kappa_l)*tan_delta_theta*(kappa*one_minus_kappa_l/cos_delta_theta-rkappa)-d_rkappa_l))/one_minus_kappa_l;

    return vehicle_state_sl;
    }

std::vector< TrajectoryAllInfo > TrajectoryStitcher::addSLInfoForTajectoryPoint(TrajectoryPoint& current_vehicle_pose)
{
    TrajectoryAllInfo all_info;
    std::vector<WaypointData> ref_line= ReferencePathMatcher::getStoredRefPoints();
    WaypointData matched_point=ReferencePathMatcher::matchToRefernecePathByXY(ref_line,current_vehicle_pose.x,current_vehicle_pose.y);
    WaypointData cartesian_state;
    cartesian_state.x=current_vehicle_pose.x;
    cartesian_state.y=current_vehicle_pose.y;
    cartesian_state.d_x=current_vehicle_pose.v;
    cartesian_state.d_y=current_vehicle_pose.a;
    cartesian_state.yaw=ReferencePathMatcher::calcNormalizedAngle(current_vehicle_pose.yaw);
    cartesian_state.kappa=current_vehicle_pose.kappa;
    VehicleState vehicle_state=CartesianFrenetConverter::convertCartesianToFrenetCoord(matched_point, cartesian_state);
    current_vehicle_pose.s=vehicle_state.s;
    current_vehicle_pose.d=vehicle_state.l;

    all_info.vehicle_state=vehicle_state;
    all_info.trajectory_point=current_vehicle_pose;
    return std::vector<TrajectoryAllInfo> (1,all_info);

}

void TrajectoryStitcher::show( std::vector< TrajectoryAllInfo > trajectory_all_info)
{
    for(unsigned int i=0;i<trajectory_all_info.size();i++)
    {
        VehicleState vehicle_state=trajectory_all_info[i].vehicle_state;
        TrajectoryPoint trajectory_point=trajectory_all_info[i].trajectory_point;
        cout<<"-----------vehicle state-----------\n"
        <<"x="<<vehicle_state.x<<",y="<<vehicle_state.y<<",yaw="<<vehicle_state.yaw
                <<",v="<<vehicle_state.v<<",a="<<vehicle_state.a<<",kappa="<<vehicle_state.kappa
                <<",dkappa="<<vehicle_state.dkappa<<",s="<<vehicle_state.s<<",ds="<<vehicle_state.ds
                <<",dds="<<vehicle_state.dds<<",l="<<vehicle_state.l<<",dl="<<vehicle_state.dl
                <<",ddl="<<vehicle_state.ddl<<std::endl;
        cout<<"\n-----------trajectory point-----------\n"
            <<"x="<<trajectory_point.x<<",y="<<trajectory_point.y<<",yaw="<<trajectory_point.yaw
            <<",v="<<trajectory_point.v<<",a="<<trajectory_point.a<<",kappa="<<trajectory_point.kappa
            <<",s="<<trajectory_point.s<<",d="<<trajectory_point.d<<std::endl;
    }
}

