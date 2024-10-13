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
    dcm(0, 0) = cp * ch;
    dcm(0, 1) = -cr * sh + sr * sp * ch; 
    dcm(0, 2) = sr * sh + cr * sp * ch;

    dcm(1, 0) = cp * sh;
    dcm(1, 1) = cr * ch + sr * sp * sh; 
    dcm(1, 2) = -sr * ch + cr * sp * sh;

    dcm(2, 0) = -sp;
    dcm(2, 1) = sr * cp;
    dcm(2, 2) = cr * cp;

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

Matrix3d Rotation::quat2dcm(const Quaterniond &q)
{
    Matrix3d C;
    C(0, 0) = q.w() * q.w() + q.x() * q.x() - q.y() * q.y() - q.z() * q.z();
    C(0, 1) = 2 * (q.x() * q.y() - q.w() * q.z());
    C(0, 2) = 2 * (q.w() * q.y() + q.x() * q.z());

    C(1, 0) = 2 * (q.x() * q.y() + q.w() * q.z());
    C(1, 1) = q.w() * q.w() - q.x() * q.x() + q.y() * q.y() - q.z() * q.z();
    C(1, 2) = 2 * (q.y() * q.z() - q.w() * q.x());

    C(2, 0) = 2 * (q.x() * q.z() - q.w() * q.y());
    C(2, 1) = 2 * (q.w() * q.x() + q.y() * q.z());
    C(2, 2) = q.w() * q.w() - q.x() * q.x() - q.y() * q.y() + q.z() * q.z();

    return C;
}

Quaterniond Rotation::dcm2quat(const Matrix3d &C)
{
    Quaterniond q;
    double trace = C(0, 0) + C(1, 1) + C(2, 2);
    if (trace > 0.0)
    {
        double s = std::sqrt(trace + 1.0) * 2.0;
        q.w() = 0.25 * s;
        q.x() = (C(2, 1) - C(1, 2)) / s;
        q.y() = (C(0, 2) - C(2, 0)) / s;
        q.z() = (C(1, 0) - C(0, 1)) / s;
    }
    else if ((C(0, 0) > C(1, 1)) && (C(0, 0) > C(2, 2)))
    {
        double s = std::sqrt(1.0 + C(0, 0) - C(1, 1) - C(2, 2)) * 2.0;
        q.w() = (C(2, 1) - C(1, 2)) / s;
        q.x() = 0.25 * s;
        q.y() = (C(0, 1) + C(1, 0)) / s;
        q.z() = (C(0, 2) + C(2, 0)) / s;
    }
    else if (C(1, 1) > C(2, 2))
    {
        double s = std::sqrt(1.0 + C(1, 1) - C(0, 0) - C(2, 2)) * 2.0;
        q.w() = (C(0, 2) - C(2, 0)) / s;
        q.x() = (C(0, 1) + C(1, 0)) / s;
        q.y() = 0.25 * s;
        q.z() = (C(1, 2) + C(2, 1)) / s;
    }
    else
    {
        double s = std::sqrt(1.0 + C(2, 2) - C(0, 0) - C(1, 1)) * 2.0;
        q.w() = (C(1, 0) - C(0, 1)) / s;
        q.x() = (C(0, 2) + C(2, 0)) / s;
        q.y() = (C(1, 2) + C(2, 1)) / s;
        q.z() = 0.25 * s;
    }

    if (q.w() < 0) {
        q.coeffs() *= -1;
    }

    return q;
}


