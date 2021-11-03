//
// Created by youfa on 2021/11/3.
//

#ifndef UNTITLED_DATATYPE_H
#define UNTITLED_DATATYPE_H

struct WaypointData
{
    double x;
    double y;
    double s;
    double d_x;
    double d_y;
    double yaw;
    double kappa;
    double kappa_ds;  // The first derivative of curvature with respect to s

    WaypointData(double x = 0.0,
                 double y = 0.0,
                 double s = 0.0,
                 double d_x = 0.0,
                 double d_y = 0.0,
                 double yaw = 0.0,
                 double kappa = 0.0,
                 double kappa_ds = 0.0)
            : x(x), y(y), s(s), d_x(d_x), d_y(d_y), yaw(yaw), kappa(kappa), kappa_ds(kappa_ds)
    {
    }
};

struct TrajectoryPoint  //// x, y, yaw, v, a, t, kappa, s
{
    double x;
    double y;
    double yaw;
    double v;
    double a;
    double kappa;
    double s;
    double d;
    double t;

    TrajectoryPoint(double x = 0.0,
                    double y = 0.0,
                    double yaw = 0.0,
                    double v = 0.0,
                    double a = 0.0,
                    double kappa = 0.0,
                    double s = 0.0,
                    double d = 0.0,
                    double t = 0.0)
            : x(x), y(y), yaw(yaw), v(v), a(a), kappa(kappa), s(s), d(d), t(t) {
    }
};

struct  VehicleState
{
    double x;
    double y;
    double yaw;
    double v;
    double a;
    double kappa;
    double dkappa;
    double s;
    double ds;
    double dds;
    double l;
    double dl;
    double ddl;
    double t;
    VehicleState(double x = 0.0,
                    double y = 0.0,
                    double yaw = 0.0,
                    double v = 0.0,
                    double a = 0.0,
                    double kappa = 0.0,
                    double dkappa=0.0,
                    double s = 0.0,
                    double ds=0.0,
                    double dds=0.0,
                    double l = 0.0,
                    double dl=0.0,
                    double ddl=0.0,
                    double t = 0.0)
            : x(x), y(y), yaw(yaw), v(v), a(a), kappa(kappa), dkappa(dkappa),s(s), ds(ds),dds(dds),l(l),dl(dl),ddl(ddl), t(t) {
    }
};

struct TrajectoryAllInfo{
    TrajectoryPoint trajectory_point;
    VehicleState vehicle_state;

};

#endif //UNTITLED_DATATYPE_H
