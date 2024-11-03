#ifndef GLOBAL_DEFS_HPP
#define GLOBAL_DEFS_HPP

// include eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

// include c++
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <string>
#include <cstring>
#include <iomanip>
#include <chrono>
#include <bitset>
#include <limits>

// include yaml-cpp
#include <yaml-cpp/yaml.h>


typedef struct INS_MECH_STATE 
{
    double Time;               // Timestamp (GPS Time)
    double dt;                 // Interval(s)
    Eigen::Vector3d Pos;       // [lat;lon;h]
    Eigen::Vector3d Vel;       // [Vn; Ve; Vd]
    Eigen::Vector3d dVel;      // [dVn; dVe; dVd] velocity increment
    Eigen::Matrix3d C_bn;      // Rotation matrix from body frame to navigation frame
    Eigen::Quaterniond q_bn;   // Quaternion from body frame to navigation frame
    Eigen::Vector3d Eul_nb;    // Euler angle from navigation frame to body frame
    Eigen::Vector3d w_nb_b;    // body rotation rate with respect to navigation frame in body frame
    Eigen::Matrix3d C_vn;      // Rotation matrix from vehicle frame to navigation frame
    Eigen::Vector3d Eul_nv;    // Euler angle from navigation frame to vehicle frame
    Eigen::Vector3d g_n;       // normal gravity in navigation frame
    Eigen::Vector3d w_ie_n;    // earth rotation rate with respect to inertial frame in navigation frame
    Eigen::Vector3d w_en_n;    // transport rate of navigation frame with respect to earth frame in navigation frame
    Eigen::Quaterniond q_ne;   // Quaternion from navigation frame to earth frame
    Eigen::Vector3d w_ib_b;    // angular rate in body frame
    Eigen::Vector3d f_ib_b;    // specific force in body frame
    Eigen::Vector3d f_ib_n;    // specific force in navigation frame
    double Time_Start;         // Timestamp after alignment was finish
    double Time_End;           // Timestamp of navigation was finish
    double Time_Feedback;      // Timestamp of Feedback
    double Time_Output;        // Timestamp of Output
    double Rm;                 // radius of curvature in meridian
    double Rn;                 // radius of curvature in prime vertical
}InsState;

typedef struct GNSS
{
    double Timetag; // GNSS Second of week

    Eigen::Vector3d Pos;
    Eigen::Vector3d Vel;
    Eigen::Vector3d PosStd;
    Eigen::Vector3d VelStd; 

}GnssData;

typedef struct IMU
{
    double Timetag; // GNSS Second of week

    // 0:incremental angle and velocity; 1:angular rate and specific force 
    Eigen::Vector3d Gyro;
    Eigen::Vector3d Accel;

}ImuData;

typedef struct SENSOR_DATA
{
    GnssData GnssCur;
    GnssData GnssPrev;

    ImuData ImrCur;       // angular rate and specific force
    ImuData ImuPrev;      // angular rate and specific force 
    ImuData ImrMeasCur;   // incremental angle and velocity
    ImuData ImuMeasPrev;  // incremental angle and velocity
}SensorData;






#endif // GLOBAL_DEFS_HPP