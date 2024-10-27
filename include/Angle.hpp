#ifndef ANGLE_HPP
#define ANGLE_HPP

#include "Global_defs.hpp"

class Angle
{
public:
    // Angle transformation
    static double deg2rad(double deg);
    static double rad2deg(double rad);
    static Eigen::Vector3d latlon_deg2rad(const Eigen::Vector3d &Pos_LLA);
    static Eigen::Vector3d latlon_rad2deg(const Eigen::Vector3d &Pos_LLA);
    static double wrap2pi(double angle);
    static double dist_ang(double angle1, double angle2);
};





#endif // ANGLE_HPP