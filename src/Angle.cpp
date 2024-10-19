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

Vector3d Angle::latlon_deg2rad(const Vector3d &Pos_LLA)
{
    Vector3d rad_Pos;
    rad_Pos[0] = Angle::deg2rad(Pos_LLA[0]);
    rad_Pos[1] = Angle::deg2rad(Pos_LLA[1]);
    rad_Pos[2] = Pos_LLA[2];

    return rad_Pos;
}

Vector3d Angle::latlon_rad2deg(const Vector3d &Pos_LLA)
{
    Vector3d deg_Pos;
    deg_Pos[0] = Angle::rad2deg(Pos_LLA[0]);
    deg_Pos[1] = Angle::rad2deg(Pos_LLA[1]);
    deg_Pos[2] = Pos_LLA[2];

    return deg_Pos;
}