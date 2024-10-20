#ifndef ROTATION_HPP
#define ROTATION_HPP

#include "Global_defs.hpp"

class Rotation
{
public:
    // Rotation matrix
    static Matrix3d C_ne(const Vector3d &Pos_LLA); //pos2dcm
    static Quaterniond q_ne(const Vector3d &Pos_LLA); //pos2quat
    static Matrix3d euler2dcm(const Vector3d &euler);
    static Vector3d dcm2euler(const Matrix3d &dcm);
    static Matrix3d quat2dcm(const Quaterniond &q);
    static Quaterniond dcm2quat(const Matrix3d &C);
    static Quaterniond rvec2quat(const Vector3d & rot_vec);

};













#endif // ROTATION_HPP
