#include "Rotation.hpp"

/* Transformation matrix of navigation frame -> earth frame */
Matrix3d Rotation::C_ne(const Vector3d &Pos_LLA)
{
    double s_lat = std::sin(Pos_LLA[0]);
    double c_lat = std::cos(Pos_LLA[0]);
    double s_lon = std::sin(Pos_LLA[1]);
    double c_lon = std::cos(Pos_LLA[1]);

    Matrix3d C_ne;
    C_ne << -s_lat * c_lon, -s_lon, -c_lat * c_lon,
            -s_lat * s_lon,  c_lon, -c_lat * s_lon,
                     c_lat,      0,         -s_lat;

    return C_ne;
}