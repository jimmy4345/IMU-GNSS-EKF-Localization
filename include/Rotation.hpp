#ifndef ROTATION_HPP
#define ROTATION_HPP

#include "Global_defs.hpp"

class Rotation
{
public:
    // Rotation matrix
    static Matrix3d C_ne(const Vector3d &Pos_LLA); //pos2dcm
    static Quaterniond q_ne(const Vector3d &Pos_LLA); //pos2quat
    static RowVector2d quat2pos(const Quaterniond &q_ne);
    static Matrix3d euler2dcm(const Vector3d &euler);
    static Vector3d dcm2euler(const Matrix3d &dcm);
    static Matrix3d quat2dcm(const Quaterniond &q);
    static Quaterniond dcm2quat(const Matrix3d &C);
    static Quaterniond rvec2quat(const Vector3d & rot_vec);
    static Vector3d dpos2rvec(const double lat,
                              const double delta_lat, const double delta_lon);
    static Quaterniond quatprod(const Quaterniond& q, const Quaterniond& p);
    static Matrix3d skew_symmetric(const Vector3d &vector); //cp_form

};













#endif // ROTATION_HPP
