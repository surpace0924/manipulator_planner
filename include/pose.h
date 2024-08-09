#pragma once
#include <Eigen/Dense>

template <typename T>
class Pose
{
public:
  Eigen::Matrix<T, 3, 1> position;
  Eigen::Quaternion<T> orientation;

  Pose() : position(Eigen::Vector3d::Zero()), orientation(Eigen::Quaterniond::Identity()) {}

  Pose(const Eigen::Vector3d & position, const Eigen::Quaterniond & orientation) : position(position), orientation(orientation) {}

  void print()
  {
    std::cout << "position(x,y,z): " << position.transpose() << std::endl;
    std::cout << "orientation(w,x,y,z): " << orientation.coeffs().transpose() << std::endl;
  }

  template <typename Char>
  inline std::basic_ostream<Char> & operator<<(std::basic_ostream<Char> & os) const
  {
    os << "position(x,y,z): " << position.transpose() << std::endl;
    os << "orientation(w,x,y,z): " << orientation.coeffs().transpose() << std::endl;
    return os;
  }
};
