#include "Angle.hpp"

const double D2R = (M_PI / 180.0);
const double R2D = (180.0 / M_PI);

double Angle::deg2rad(double deg)
{
    return deg * D2R;
}

double Angle::rad2deg(double rad)
{
    return rad * R2D;
}

Eigen::Vector3d Angle::latlon_deg2rad(const Eigen::Vector3d &Pos_LLA)
{
    Eigen::Vector3d rad_Pos;
    rad_Pos[0] = Angle::deg2rad(Pos_LLA[0]);
    rad_Pos[1] = Angle::deg2rad(Pos_LLA[1]);
    rad_Pos[2] = Pos_LLA[2];

    return rad_Pos;
}

Eigen::Vector3d Angle::latlon_rad2deg(const Eigen::Vector3d &Pos_LLA)
{
    Eigen::Vector3d deg_Pos;
    deg_Pos[0] = Angle::rad2deg(Pos_LLA[0]);
    deg_Pos[1] = Angle::rad2deg(Pos_LLA[1]);
    deg_Pos[2] = Pos_LLA[2];

    return deg_Pos;
}

/* Function to wrap an angle to the range (-π, π] */
double Angle::wrap2pi(double angle)
{
    // Use fmod to bring the angle within the range (0, 2π]
    angle = fmod(angle + M_PI, 2 * M_PI);

    // Wrap to (0, 2π] if negative
    if (angle < 0) {
        angle += 2 * M_PI;
    }

    // Shift to (-π, π]
    angle -= M_PI;

    // Handle the edge case where angle is exactly -π
    if (angle == -M_PI) {
        angle = M_PI;
    }

    return angle;
}

double Angle::dist_ang(double angle1, double angle2)
{
    double angle = Angle::wrap2pi(Angle::wrap2pi(angle2) - Angle::wrap2pi(angle1));
    return angle;
}
