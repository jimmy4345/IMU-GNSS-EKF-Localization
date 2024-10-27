#ifndef ROTATION_HPP
#define ROTATION_HPP

#include "Global_defs.hpp"

class Rotation
{
public:
    // Rotation matrix
    static Eigen::Matrix3d C_ne(const Eigen::Vector3d &Pos_LLA); //pos2dcm
    static Eigen::Quaterniond q_ne(const Eigen::Vector3d &Pos_LLA); //pos2quat
    static Eigen::RowVector2d quat2pos(const Eigen::Quaterniond &q_ne);
    static Eigen::Matrix3d euler2dcm(const Eigen::Vector3d &euler);
    static Eigen::Vector3d dcm2euler(const Eigen::Matrix3d &dcm);
    static Eigen::Matrix3d quat2dcm(const Eigen::Quaterniond &q);
    static Eigen::Quaterniond dcm2quat(const Eigen::Matrix3d &C);
    static Eigen::Quaterniond rvec2quat(const Eigen::Vector3d & rot_vec);
    static Eigen::Vector3d dpos2rvec(const double lat,
                              const double delta_lat, const double delta_lon);
    static Eigen::Quaterniond quatprod(const Eigen::Quaterniond& q, const Eigen::Quaterniond& p);
    static Eigen::Matrix3d skew_symmetric(const Eigen::Vector3d &vector); //cp_form

};













#endif // ROTATION_HPP
