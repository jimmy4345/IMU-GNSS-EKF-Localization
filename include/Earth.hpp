#ifndef EARTH_HPP
#define EARTH_HPP

#include "Global_defs.hpp"

class Earth
{
public:
    // WGS84 model parameters
    static constexpr double WGS84_a = 6378137.0;                     /* Semi-major axis a */
    static constexpr double WGS84_b = 6356752.3142;                  /* Semi-minor axis b */
    static constexpr double WGS84_f = 1 / 298.257223563;             /* Flattening, 1.0/298.2572236 */
    static constexpr double WGS84_we = 7.2921151467E-5;              /* Angular velocity of the Earth rotation */
    static constexpr double WGS84_e1 = WGS84_f * (2 - WGS84_f);      /* First eccentricity squared */
    static constexpr double WGS84_e2  = WGS84_e1 / (1 - WGS84_e1);   /* Second eccentricity squared */
    static constexpr double WGS84_GM0 = 3986004.418E+8;              /* Gravitational constant */
    
    // Earth relative function
    static double NormalGravity(const Vector3d &Pos_LLA);
    static double rc_PrimeVertical(double lat);
    static double rc_meridian(double lat);
    static Vector3d geo2ecef(const Vector3d &Pos_LLA);
    static Vector3d ecef2geo(const Vector3d &Pos_XYZ);


    
};



#endif // EARTH_HPP