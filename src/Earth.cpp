#include "Earth.hpp"


/* Normal gravity*/
double Earth::NormalGravity(Vector3d Pos_LLA) {
    // Constants
    constexpr double a1 = 9.7803267715;
    constexpr double a2 = 0.0052790414;
    constexpr double a3 = 0.0000232718;
    constexpr double a4 = -0.000003087691089;
    constexpr double a5 = 0.000000004397731;
    constexpr double a6 = 0.000000000000721;

    // Latitude (Pos_LLA(0)) in radians
    double sinLat = std::sin(Pos_LLA(0));
    double sinLatSquared = sinLat * sinLat;
    double sinLatFourth = sinLatSquared * sinLatSquared;  // sin^4(lat)

    // Altitude (Pos_LLA(2))
    double altitude = Pos_LLA(2);

    // Compute Normal Gravity
    double NG = a1 * (1 + a2 * sinLatSquared + a3 * sinLatFourth) +
                (a4 + a5 * sinLatSquared) * altitude +
                a6 * altitude * altitude;

    return NG;
}

/* prime vertical radius of curvature (N) */
double Earth::rc_PrimeVertical(double lat) 
{
    double sinlat = std::sin(lat);  
    double sinlat_squared = sinlat * sinlat;
    return Earth::WGS84_a / std::sqrt(1.0 - Earth::WGS84_e1 * sinlat_squared); 
}

/* meridian radius of curvature (M) */
double Earth::rc_meridian(double lat) 
{
    double sinlat = std::sin(lat);  
    double sinlat_squared = sinlat * sinlat;
    double denom = std::sqrt(1 - Earth::WGS84_e1 * sinlat_squared);  // Square root once
    return Earth::WGS84_a * (1 - Earth::WGS84_e1) / (denom * denom * denom);  // Cube manually
}

/* Geodetic coordinates to ECEF coordinates */
Vector3d Earth::geo2ecef(Vector3d Pos_LLA) {
    // Latitude and longitude in radians
    double lat = Pos_LLA(0);
    double lon = Pos_LLA(1);
    double alt = Pos_LLA(2);

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
