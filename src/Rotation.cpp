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

/* Transformation quaternion of navigation frame -> earth frame */
Quaterniond Rotation::q_ne(const Vector3d &Pos_LLA) //pos2quat
{
    Quaterniond q;
    double lat = Pos_LLA[0];
    double lon = Pos_LLA[1];

    double s_lat = std::sin(-M_PI * 0.25 - lat * 0.5);
    double c_lat = std::cos(-M_PI * 0.25 - lat * 0.5);
    double s_lon = std::sin(lon * 0.5);
    double c_lon = std::cos(lon * 0.5);

    q.w() = c_lat * c_lon;
    q.x() = -s_lat * s_lon;
    q.y() = s_lat * c_lon;
    q.z() = c_lat * s_lon;

    return q;
}

/* Quaternion corrresponding to position */
RowVector2d Rotation::quat2pos(const Quaterniond &q_ne)
{
    RowVector2d Pos;

    Pos[0] = -2 * std::atan(q_ne.y() / q_ne.w()) - M_PI / 2;    
    Pos[1] = 2 * std::atan2(q_ne.z(), q_ne.w());

    return Pos;
}

/* Function to convert Euler angles (roll, pitch, heading) to Direction Cosine Matrix (DCM) */
Matrix3d Rotation::euler2dcm(const Vector3d &euler) //C_bn
{
    double cr = std::cos(euler[0]);
    double sr = std::sin(euler[0]);
    double cp = std::cos(euler[1]);
    double sp = std::sin(euler[1]);
    double ch = std::cos(euler[2]);
    double sh = std::sin(euler[2]);

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
    Vector3d euler; // [row; pitch; heading]

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

    if (q.w() < 0) 
    {
        q.coeffs() *= -1;
    }

    return q;
}

Quaterniond Rotation::rvec2quat(const Vector3d &rot_vec)
{
    Quaterniond q;
    double mag2 = rot_vec.squaredNorm();

    if(mag2 < (M_PI * M_PI))
    {
        // Approximate solution
        double mag2_scaled = 0.25 * mag2;

        // Using Taylor expansion approximation for cosine and sine
        double c = 1.0 - mag2_scaled / 2.0 * (1.0 - mag2_scaled / 12.0 * (1.0 - mag2_scaled / 30.0));
        double s = 1.0 - mag2_scaled / 6.0 * (1.0 - mag2_scaled / 20.0 * (1.0 - mag2_scaled / 42.0));

        q.w() = c;
        q.x() = s * 0.5 * rot_vec[0];
        q.y() = s * 0.5 * rot_vec[1];
        q.z() = s * 0.5 * rot_vec[2];
    }
    else
    {
        // Analytical solution
        double mag = std::sqrt(mag2);
        double s_mag = std::sin(mag / 2.0);
        double c_mag = std::cos(mag / 2.0);

        q.w() = c_mag;
        q.x() = rot_vec[0] * s_mag / mag;
        q.y() = rot_vec[1] * s_mag / mag;
        q.z() = rot_vec[2] * s_mag / mag;

        if (q.w() < 0) 
        {
            q.coeffs() *= -1;
        }    
    }

    return q;
}

Vector3d Rotation::dpos2rvec(const double lat,
                             const double delta_lat, const double delta_lon)
{
    Vector3d rot_vec;
    
    rot_vec[0] = delta_lon * std::cos(lat);
    rot_vec[1] = -delta_lat;
    rot_vec[2] = -delta_lon * std::sin(lat);

    return rot_vec;
}

Quaterniond Rotation::quatprod(const Quaterniond& q, const Quaterniond& p) {
    // Quaternion multiplication using Eigen's overloaded operator
    Quaterniond result = q * p;

    // Ensure the scalar component is positive
    if (result.w() < 0) {
        result.coeffs() *= -1;
    }

    return result;
}

