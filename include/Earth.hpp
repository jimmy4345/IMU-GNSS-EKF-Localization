#ifndef EARTH_H
#define EARTH_H

#include <Eigen/Dense>
#include <cmath>

using Eigen::Vector3d;

class Earth
{
public:
    // WGS84 model parameters
    static constexpr double WGS84_a = 6378137.0000000000;        /* Semi-major axis a */
    static constexpr double WGS84_b = 6356752.3142451793;        /* Semi-minor axis b */
    static constexpr double WGS84_f = 0.0033528106647474805;     /* Flattening, 1.0/298.2572236 */
    static constexpr double WGS84_we = 7.2921151467E-5;          /* Angular velocity of the Earth rotation */
    static constexpr double WGS84_e1 = 0.0066943799901413156;    /* 第一偏心率平方 */
    static constexpr double WGS84_e2  = 0.0067394967422764341;   /* 第二偏心率平方 */
    static constexpr double WGS84_GM0 = 3986004.418E+8;          /* Gravitational constant */
    
    double NormalGravity(Vector3d Pos_LLA);
    double rc_PrimeVertical(double lat);
    double rc_meridian(double lat);
    Vector3d Earth::geo2ecef(Vector3d Pos_LLA);


    
};



#endif // EARTH_H