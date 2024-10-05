#include <iostream>
#include "Eigen/Dense"
#include "yaml-cpp/yaml.h"
 
using Eigen::MatrixXd;
 
int main()
{
  MatrixXd m(2,2);
  m(0,0) = 3;
  m(1,0) = 2.5;
  m(0,1) = -1;
  m(1,1) = m(1,0) + m(0,1);
  std::cout << m << std::endl;

  YAML::Node node;
  node["number"] = 255;
  node["string"] = "sample str";

  YAML::Node subNode;
  subNode.push_back("element 1");
  subNode.push_back("element 2");
  node["sub"] = subNode;

  std::cout << node << std::endl;

}