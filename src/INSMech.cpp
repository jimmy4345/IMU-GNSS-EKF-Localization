#include "INSMech.hpp"

void INSMech::insMech(InsState &InsStateCur, InsState &InsStatePrev, 
                      const ImuData &ImuCur, const ImuData &ImuPrev)
{
    VelInterg(InsStateCur, InsStateCur, ImuCur, ImuPrev);
    PosInterg(InsStateCur, InsStateCur, ImuCur, ImuPrev);
    AttInterg(InsStateCur, InsStateCur, ImuCur, ImuPrev);
}


void INSMech::VelInterg(InsState &InsStateCur, InsState &InsStatePrev, 
                        const ImuData &ImuCur, const ImuData &ImuPrev)
{
    double dt, d_lat, d_lon;
    Eigen::Vector3d midPos, midVel, d_theta;
    Eigen::Quaterniond q_ne, q_ee, q_nn, midQuat;

    dt = ImuCur.Timetag - ImuPrev.Timetag;

    // Altitude
    midPos[2] = InsStatePrev.Pos[2] - 0.5 *  InsStatePrev.Vel[2] * dt;

    // Latitude and Lontitude
    d_lat = 0.5 * InsStatePrev.Vel[0] * dt / (InsStatePrev.Rm + midPos[2]);
    d_lon = 0.5 * InsStatePrev.Vel[1] * dt / (InsStatePrev.Rn + midPos[2]) / std::cos(midPos[0]);
    d_theta = Rotation::dpos2rvec(InsStatePrev.Pos[0], d_lat, d_lon);
    q_nn = Rotation::rvec2quat(d_theta);
    q_ne = Rotation::quatprod(InsStatePrev.q_ne, q_nn);

    Eigen::Vector3d we_dt = {0, 0, Earth::WGS84_we * dt};
    q_ee = Rotation::rvec2quat(0.5 * we_dt);
    midQuat = Rotation::quatprod(q_ee, q_ne);

    Eigen::RowVector2d tempPos;
    tempPos = Rotation::quat2pos(midQuat);
    midPos[0] = tempPos[0];
    midPos[1] = tempPos[1];

    // Gravity at midway
    InsStateCur.g_n = {0.0, 0.0, Earth::NormalGravity(midPos)};

    // Extrapolate velocity
    midVel = InsStateCur.Vel + 0.5 * InsStateCur.dVel;

    // Computes Earth's rotation rates in n-frame
    InsStateCur.w_ie_n[0] = Earth::WGS84_we * std::cos(midPos[0]);
    InsStateCur.w_ie_n[1] = 0.0;
    InsStateCur.w_ie_n[2] = -Earth::WGS84_we * std::sin(midPos[0]);
    InsStateCur.w_en_n = Earth::w_en_n(midPos, midVel, InsStateCur.Rn, InsStateCur.Rm);

    // Navigation frame rotation vector
    Eigen::Vector3d Zeta;
    Zeta = (InsStateCur.w_en_n + InsStateCur.w_ie_n) * dt;
    Eigen::Matrix3d C_nn;
    C_nn = Eigen::Matrix3d::Identity() - Rotation::skew_symmetric(0.5 * Zeta);

    // Integrated specific force in navigation frame
    Eigen::Vector3d rotaMotion,scullMotion1, scullMotion2;
    rotaMotion = ImuCur.Gyro.cross(ImuCur.Accel) / 2;
    scullMotion1 = ImuPrev.Gyro.cross(ImuCur.Accel) / 12;
    scullMotion2 = ImuPrev.Accel.cross(ImuCur.Gyro) / 12;

    Eigen::Vector3d dv_f_ib_n;
    dv_f_ib_n = C_nn * InsStateCur.C_bn * (ImuCur.Accel + rotaMotion + scullMotion1 + scullMotion2);
    InsStateCur.f_ib_n = dv_f_ib_n / dt;

    // Correction including gravity, Coriolis term and Centrifugal term
    Eigen::Vector3d r_eb_e, C_en, r_eb_n, Coriolis, Centrifugal, dVGraCor;
    r_eb_e = Earth::geo2ecef(midPos);
    C_en = Rotation::C_ne(midPos).transpose();
    r_eb_n = C_en * r_eb_e;
    Centrifugal = InsStateCur.w_ie_n.cross(InsStateCur.w_ie_n.cross(r_eb_n));
    Coriolis = (2 * InsStateCur.w_ie_n + InsStateCur.w_en_n).cross(midVel);
    dVGraCor = (InsStateCur.g_n - Coriolis - Centrifugal) * dt;

    // Velocity increment
    InsStateCur.dVel = dv_f_ib_n + dVGraCor;

    // Update velocity
    InsStateCur.Vel += InsStateCur.dVel;

}

void INSMech::PosInterg(InsState &InsStateCur, InsState &InsStatePrev, 
                        const ImuData &ImuCur, const ImuData &ImuPrev)
{
    
}

void INSMech::AttInterg(InsState &InsStateCur, InsState &InsStatePrev, 
                        const ImuData &ImuCur, const ImuData &ImuPrev)
{
    
}