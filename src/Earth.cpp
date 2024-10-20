#include "Earth.hpp"
#include "Rotation.hpp"

/* Normal gravity*/
double Earth::NormalGravity(const Vector3d &Pos_LLA) {
    // Constants
    constexpr double a1 = 9.7803267715;
    constexpr double a2 = 0.0052790414;
    constexpr double a3 = 0.0000232718;
    constexpr double a4 = -0.000003087691089;
    constexpr double a5 = 0.000000004397731;
    constexpr double a6 = 0.000000000000721;

    // Latitude (Pos_LLA(0)) in radians
    double s_lat = std::sin(Pos_LLA[0]);
    double s_lat_squared = s_lat * s_lat;
    double s_lat_fourth = s_lat_squared * s_lat_squared;  // sin^4(lat)

    // Altitude (Pos_LLA(2))
    double altitude = Pos_LLA[2];

    // Compute Normal Gravity
    double NG = a1 * (1 + a2 * s_lat_squared + a3 * s_lat_fourth) +
                (a4 + a5 * s_lat_squared) * altitude +
                a6 * altitude * altitude;

    return NG;
}

/* prime vertical radius of curvature (N) */
double Earth::rc_PrimeVertical(double lat) 
{
    double s_lat = std::sin(lat);  
    double s_lat_squared = s_lat * s_lat;
    return Earth::WGS84_a / std::sqrt(1.0 - Earth::WGS84_e1 * s_lat_squared); 
}

/* meridian radius of curvature (M) */
double Earth::rc_meridian(double lat) 
{
    double s_lat = std::sin(lat);  
    double s_lat_squared = s_lat * s_lat;
    double denom = std::sqrt(1 - Earth::WGS84_e1 * s_lat_squared);  // Square root once
    return Earth::WGS84_a * (1 - Earth::WGS84_e1) / (denom * denom * denom);  // Cube manually
}

/* Geodetic coordinates to ECEF coordinates */
Vector3d Earth::geo2ecef(const Vector3d &Pos_LLA) {
    // Latitude and longitude in radians
    double lat = Pos_LLA[0];
    double lon = Pos_LLA[1];
    double alt = Pos_LLA[2];

    // Precompute trigonometric values
    double c_lat = std::cos(lat);
    double s_lat = std::sin(lat);
    double c_lon = std::cos(lon);
    double s_lon = std::sin(lon);

    // Prime vertical radius of curvature
    double Rn = Earth::rc_PrimeVertical(lat);  // Static access
    double Rn_h = Rn + alt;                    // Rn + height (altitude)

    // Compute ECEF coordinates
    Vector3d Pos_XYZ;
    Pos_XYZ << Rn_h * c_lat * c_lon,                        // X-coordinate
               Rn_h * c_lat * s_lon,                        // Y-coordinate
               (Rn * (1 - Earth::WGS84_e1) + alt) * s_lat;  // Z-coordinate

    return Pos_XYZ;
}

/* ECEF coordinates to Geodetic coordinates */
Vector3d Earth::ecef2geo(const Vector3d &Pos_XYZ)
{
    double rho = std::hypot(Pos_XYZ[0], Pos_XYZ[1]); // Horizontal distance from the Z-axis
    double phi, lambda, h;

    if (Earth::WGS84_f == 0)
    {
        // Spherical Earth case
        phi = std::atan2(Pos_XYZ[2], rho);
        h = std::hypot(Pos_XYZ[2], rho) - Earth::WGS84_a;
    }
    else
    {
        // Ellipsoidal Earth case: Using Bowring's method
        double ae2 = Earth::WGS84_a * Earth::WGS84_e1;
        double bep2 = Earth::WGS84_b * Earth::WGS84_e2;

        // Initial value for parametric latitude (beta)
        double r = std::hypot(rho, Pos_XYZ[2]);  // Distance from the origin
        double u = Earth::WGS84_a * rho;
        double v = Earth::WGS84_b * Pos_XYZ[2] * (1 + bep2 / r);
        double cosbeta = std::copysign(1.0, u) / std::hypot(1.0, v / u);
        double sinbeta = std::copysign(1.0, v) / std::hypot(1.0, u / v);

        // Iterative solution with Bowring's method (max 5 iterations)
        for (int count = 0; count < 5; ++count) {
            double cosprev = cosbeta;
            double sinprev = sinbeta;

            u = rho - ae2 * std::pow(cosbeta, 3);
            v = Pos_XYZ[2] + bep2 * std::pow(sinbeta, 3);
            double au = Earth::WGS84_a * u;
            double bv = Earth::WGS84_b * v;

            cosbeta = std::copysign(1.0, au) / std::hypot(1.0, bv / au);
            sinbeta = std::copysign(1.0, bv) / std::hypot(1.0, au / bv);

            // Check for convergence
            if (std::hypot(cosbeta - cosprev, sinbeta - sinprev) <= std::numeric_limits<double>::epsilon()) {
                break;
            }
        }

        // Final geodetic latitude
        phi = std::atan2(v, u);

        // Final altitude (h)
        double sinphi = std::sin(phi);
        double cosphi = std::cos(phi);
        double N = Earth::rc_PrimeVertical(phi);  // Prime vertical radius of curvature
        h = rho * cosphi + (Pos_XYZ[2] + Earth::WGS84_e1 * N * sinphi) * sinphi - N;
    }

    // Longitude
    lambda = std::atan2(Pos_XYZ[1], Pos_XYZ[0]);

    // Return geodetic coordinates [latitude, longitude, altitude] in radians and meters
    return {phi, lambda, h};       
}

/* Geodetic coordinates to NED coordinates relative to a reference point */
Vector3d Earth::geo2ned(const Vector3d &ref_Pos_LLA, const Vector3d &Pos_LLA)
{
    // Convert reference and target geodetic coordinates to ECEF
    Vector3d ref_Pos_XYZ = Earth::geo2ecef(ref_Pos_LLA);
    Vector3d Pos_XYZ = Earth::geo2ecef(Pos_LLA);

    // Calculate the difference in ECEF coordinates
    Vector3d delta_XYZ = Pos_XYZ - ref_Pos_XYZ;

    // Compute the rotation matrix from ECEF to NED
    Matrix3d C_en = Rotation::C_ne(ref_Pos_LLA).transpose();

    // Convert the delta ECEF vector to NED coordinates
    Vector3d Pos_NED = C_en * delta_XYZ;
    
    return Pos_NED;
}

/* Calculates transport rate of navigation frame with respect to Earth frame - NED system */
Vector3d Earth::w_en_n(const Vector3d &Pos_LLA, const Vector3d &Vel_NED, 
                       const double Rn, const double Rm) // TransRate
{
    Vector3d w_en_n;
    w_en_n[0] = Vel_NED[1] / (Rn + Pos_LLA[2]);
    w_en_n[1] = -Vel_NED[0] / (Rm + Pos_LLA[2]); 
    w_en_n[2] = -Vel_NED[0] * std::tan(Pos_LLA[0]) / (Rn +Pos_LLA[2]);

   return w_en_n;
}

Vector3d Earth::LeverArmPos(const Vector3d &GNSS_Pos, const Vector3d &LeverArm,
                            const Matrix3d C_bn, const double Rn, const double Rm)
{
    Vector3d dr = C_bn * LeverArm;
    
    Vector3d dr1;
    dr1[0] = dr[0] / (Rm + GNSS_Pos[2]);
    dr1[1] = dr[1] / (Rn + GNSS_Pos[2]) / std::cos(GNSS_Pos[0]);
    dr1[2] = -dr[2];

    Vector3d Imu_Pos = GNSS_Pos - dr1;

    return Imu_Pos;
}
