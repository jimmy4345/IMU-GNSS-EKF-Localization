#include "Global_defs.hpp"
#include "Earth.hpp"

int main()
{
    // test eigen
    MatrixXd m(2,2);
    m(0,0) = 3;
    m(1,0) = 2.5;
    m(0,1) = -1;
    m(1,1) = m(1,0) + m(0,1);
    std::cout << m << std::endl;

    // test yaml-cpp
    YAML::Node node;
    node["number"] = 255;
    node["string"] = "sample str";

    YAML::Node subNode;
    subNode.push_back("element 1");
    subNode.push_back("element 2");
    node["sub"] = subNode;

    std::cout << node << std::endl;

    // test ecef2geo
    Eigen::Vector3d Pos_XYZ(-3028314.09, 4924120.63, 2686003.31);
    Eigen::Vector3d Pos_XYZ_2(0, 0, 6356782.3142);
    Earth earth;

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

}