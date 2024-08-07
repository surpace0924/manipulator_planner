#include <iostream>
#include <Eigen/Dense>
#include "../third_party/matplotlib-cpp/matplotlibcpp.h"

int main()
{
  Eigen::Matrix2d mat;
  mat << 1, 2, 3, 4;
  std::cout << "Here is the matrix mat:\n" << mat << std::endl;

  Eigen::Vector2d vec;
  vec << 5, 6;
  std::cout << "Here is the vector vec:\n" << vec << std::endl;

  Eigen::Vector2d result = mat * vec;
  std::cout << "The result of mat * vec is:\n" << result << std::endl;

  // Plotting example
  namespace plt = matplotlibcpp;
  plt::plot({1, 3, 2, 4});
  plt::show();

  return 0;
}
