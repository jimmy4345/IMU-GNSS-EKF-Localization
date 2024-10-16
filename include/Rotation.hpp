#ifndef ROTATION_HPP
#define ROTATION_HPP

#include "Global_defs.hpp"

class Rotation
{
public:
    // Rotation matrix
    static Matrix3d C_ne(const Vector3d &Pos_LLA);
    static Matrix3d euler2dcm(double roll, double pitch, double heading);
    static Vector3d dcm2euler(const Matrix3d &dcm);
    static Matrix3d quat2dcm(const Quaterniond &q);
    static Quaterniond dcm2quat(const Matrix3d &C);

};













#endif // ROTATION_HPP
