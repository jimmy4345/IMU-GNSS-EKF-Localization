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

};













#endif // ROTATION_HPP
