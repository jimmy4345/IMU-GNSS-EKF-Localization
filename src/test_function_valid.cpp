#include "Global_defs.hpp"
#include "Earth.hpp"
#include "Angle.hpp"

int main()
{
    // test eigen
    MatrixXd m(2,2);
    m(0,0) = 3;
    m(1,0) = 2.5;
    m(0,1) = -1;
    m(1,1) = m(1,0) + m(0,1);
    std::cout << m << std::endl;

    std::cout << std::endl;

    // test yaml-cpp
    YAML::Node node;
    node["number"] = 255;
    node["string"] = "sample str";

    YAML::Node subNode;
    subNode.push_back("element 1");
    subNode.push_back("element 2");
    node["sub"] = subNode;

    std::cout << node << std::endl;

    std::cout << std::endl;

    // test ecef2geo
    Eigen::Vector3d Pos_XYZ(-3028314.09, 4924120.63, 2686003.31);
    Eigen::Vector3d Pos_XYZ_2(0, 0, 6356782.3142);
    Earth earth;
    Angle angle;

    // Convert ECEF to Geodetic coordinates
    Eigen::Vector3d Pos_LLA = earth.ecef2geo(Pos_XYZ);
    Eigen::Vector3d Pos_LLA_2 = earth.ecef2geo(Pos_XYZ_2);




    // Output the results (latitude, longitude in degrees, and altitude in meters) 
    std::cout << "Latitude: " << Pos_LLA(0) * 180.0 / M_PI << " degrees" << std::endl;
    std::cout << "Longitude: " << Pos_LLA(1) * 180.0 / M_PI << " degrees" << std::endl;
    std::cout << "Altitude: " << Pos_LLA(2) << " meters" << std::endl;
    std::cout << "Latitude: " << Pos_LLA_2(0) * 180.0 / M_PI << " degrees" << std::endl;
    std::cout << "Longitude: " << Pos_LLA_2(1) * 180.0 / M_PI << " degrees" << std::endl;
    std::cout << "Altitude: " << Pos_LLA_2(2) << " meters" << std::endl;

    std::cout << std::endl;

    // Define a Vector3d representing latitude, longitude, height in degrees
    Vector3d pos_deg(45.0, 90.0, 100.0);  // 45° latitude, 90° longitude, 100 meters height

    // Convert degrees to radians
    Vector3d pos_rad = Angle::latlon_deg2rad(pos_deg);

    // Expected radian values
    Vector3d expected_rad(M_PI / 4, M_PI / 2, 100.0);

    // Check conversion from degrees to radians
    std::cout << "Degrees to Radians:" << std::endl;
    std::cout << "Input: " << pos_deg.transpose() << std::endl;
    std::cout << "Output: " << pos_rad.transpose() << std::endl;
    std::cout << "Expected: " << expected_rad.transpose() << std::endl;
    if ((pos_rad - expected_rad).norm() < 1e-6) {
        std::cout << "latlon_deg2rad test passed!" << std::endl;
    } else {
        std::cout << "latlon_deg2rad test failed!" << std::endl;
    }

    // Convert radians back to degrees
    Vector3d pos_deg_converted = Angle::latlon_rad2deg(pos_rad);

    // Check conversion from radians back to degrees
    std::cout << "\nRadians to Degrees:" << std::endl;
    std::cout << "Input: " << pos_rad.transpose() << std::endl;
    std::cout << "Output: " << pos_deg_converted.transpose() << std::endl;
    std::cout << "Expected: " << pos_deg.transpose() << std::endl;
    if ((pos_deg_converted - pos_deg).norm() < 1e-6) {
        std::cout << "latlon_rad2deg test passed!" << std::endl;
    } else {
        std::cout << "latlon_rad2deg test failed!" << std::endl;
    }    

}