#include "Rotation.hpp"

/* Transformation matrix of navigation frame -> earth frame */
Matrix3d Rotation::C_ne(const Vector3d &Pos_LLA) //pos2dcm
{
    double lat = Pos_LLA[0];
    double lon = Pos_LLA[1];

    double s_lat = std::sin(lat);
    double c_lat = std::cos(lat);
    double s_lon = std::sin(lon);
    double c_lon = std::cos(lon);

    Matrix3d C_ne;
    C_ne(0, 0) = -s_lat * c_lon;
    C_ne(0, 1) = -s_lon;
    C_ne(0, 2) = -c_lat * c_lon;

    C_ne(1, 0) = -s_lat * s_lon;
    C_ne(1, 1) = c_lon;
    C_ne(1, 2) = -c_lat * s_lon;

    C_ne(2, 0) = c_lat;
    C_ne(2, 1) = 0;
    C_ne(2, 2) = -s_lat;

    return C_ne;
}

/* Function to convert Euler angles (roll, pitch, heading) to Direction Cosine Matrix (DCM) */
Matrix3d Rotation::euler2dcm(double roll, double pitch, double heading) //C_bn
{
    double cr = std::cos(roll);
    double sr = std::sin(roll);
    double cp = std::cos(pitch);
    double sp = std::sin(pitch);
    double ch = std::cos(heading);
    double sh = std::sin(heading);

    Matrix3d dcm;
    dcm(0, 0) = ch * cp;
    dcm(0, 1) = ch * sp * sr - sh * cr;
    dcm(0, 2) = ch * sp * cr + sh * sr;

    dcm(1, 0) = sh * cp;
    dcm(1, 1) = sh * sp * sr + ch * cr;
    dcm(1, 2) = sh * sp * cr - ch * sr;

    dcm(2, 0) = -sp;
    dcm(2, 1) = cp * sr;
    dcm(2, 2) = cp * cr;

    return dcm;
}

Vector3d Rotation::dcm2euler(const Matrix3d &dcm)
{
    Vector3d euler;

    euler[1] = std::atan(-dcm(2, 0) / std::sqrt(dcm(2, 1) * dcm(2, 1) + dcm(2, 2) * dcm(2, 2)));

    if(dcm(2, 0) <= -0.999)
    {
        euler[0] = 0;
        euler[2] = std::atan2((dcm(1, 2) - dcm(0, 1)), (dcm(0, 2) + dcm(1, 1)));
    }
    else if(dcm(2, 0) >= 0.999)
    {
        euler[0] = 0;
        euler[2] = M_PI + atan2((dcm(1, 2) + dcm(0, 1)), (dcm(0, 2) - dcm(1, 1)));
    }
    else
    {
        euler[0] = atan2(dcm(2, 1), dcm(2, 2));
        euler[2] = atan2(dcm(1, 0), dcm(0, 0));
    }

    // heading 0 ~ 2PI
    euler[2] = std::fmod(euler[2] + 2 * M_PI, 2 * M_PI);        

    return euler;
}
