#include <iostream>
#include <sstream>
#include <Eigen/Dense>
#include <vector>
#include "../third_party/matplotlib-cpp/matplotlibcpp.h"

namespace plt = matplotlibcpp;

#include <Eigen/Dense>
#include <cmath>

std::string get_shape(const Eigen::MatrixXd & mat)
{
  std::stringstream ss;
  ss << mat.rows() << "x" << mat.cols();
  return ss.str();
}

/**
 * @brief DHパラメータからDH変換行列を計算する関数
 * 
 * @param alpha リンクの前後の回転角度
 * @param a リンクの前後の距離
 * @param theta リンクの回転角度
 * @param d リンクの前後の距離
 * @return Eigen::Matrix4d DH変換行列
 */
Eigen::Matrix4d dh_transform(double alpha, double a, double theta, double d)
{
  std::cout << "alpha: " << alpha << ", a: " << a << ", theta: " << theta << ", d: " << d << std::endl;
  // clang-format off
  Eigen::Matrix4d T;
  T << cos(theta),  -sin(theta) * cos(alpha), sin(theta) * sin(alpha),  a * cos(theta),
       sin(theta),  cos(theta) * cos(alpha),  -cos(theta) * sin(alpha), a * sin(theta),
       0,           sin(alpha),               cos(alpha),               d,
       0,           0,                        0,                        1;
  // clang-format on
  return T;
}

/**
 * @brief DHパラメータ行列から関節位置を計算する関数
 * 
 * @param dh_params DHパラメータ行列
 * @return Eigen::MatrixXd 関節位置
 */
Eigen::MatrixXd compute_joint_positions(const Eigen::MatrixXd & dh_params)
{
  int num_joints = dh_params.rows();
  Eigen::MatrixXd joint_pos(num_joints + 1, 3);  // 関節数+1行、3列

  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();  // 初期位置（原点）

  // 初期位置を設定
  joint_pos.row(0) = T.block<1, 3>(3, 0);

  for (int i = 0; i < num_joints; ++i) {
    double alpha = dh_params(i, 0);
    double a = dh_params(i, 1);
    double theta = dh_params(i, 2);
    double d = dh_params(i, 3);

    Eigen::Matrix4d Ti = dh_transform(alpha, a, theta, d);
    T *= Ti;             // 累積変換
    double x = T(0, 3);  // x座標
    double y = T(1, 3);  // y座標
    double z = T(2, 3);  // z座標

    joint_pos.row(i + 1) = T.block(0, 3, 3, 1).transpose();
    std::cout << T.block(0, 3, 3, 1).transpose() << std::endl;
    std::cout << x << ", " << y << ", " << z << std::endl;
    std::cout << "T" << i << ":\n" << T << std::endl;
  }

  return joint_pos;
}

int main()
{
  double boom_offset_x = 0.3;
  double boom_offset_y = 0.1;
  double boom_offset_z = 1.0;
  double boom_length = 5.0;
  double arm_length = 3.0;
  double bucket_length = 1.0;
  double tip_width = 1.0;

  Eigen::VectorXd joint_states(4);
  joint_states << 30.0 * 180 / M_PI, 0.0 * 180 / M_PI, 0.0 * 180 / M_PI, 0.0 * 180 / M_PI;

  // DHパラメータ行列の設定
  // 4行（関節数）, 4列（α, a, θ, d）
  // clang-format off
  Eigen::MatrixXd dh_params(4, 4);
  dh_params << 0.0, 0.0, joint_states[0], boom_offset_z,
               M_PI_2, boom_offset_x, joint_states[1], boom_offset_y,
               0.0, boom_length, joint_states[2], 0.0,
               0.0, arm_length, joint_states[3], 0.0;
  // clang-format on
  std::cout << "dh_params:\n" << dh_params << std::endl;

  Eigen::MatrixXd joint_pos = compute_joint_positions(dh_params);

  std::cout << "Joint positions:\n" << joint_pos << std::endl;

  // matplotlibで描画
  std::vector<double> x, y, z;
  for (int i = 0; i < joint_pos.rows(); ++i) {
    x.push_back(joint_pos(i, 0));
    y.push_back(joint_pos(i, 1));
    z.push_back(joint_pos(i, 2));
  }

  // プロットラインを引く
  plt::figure();
  plt::scatter(x, y, z, 100, {{"label", "Joint"}, {"color", "r"}});
  plt::plot3(x, y, z, {{"label", "Link"}, {"color", "b"}});

  // 軸のラベル設定
  plt::xlabel("X");
  plt::ylabel("Y");
  plt::set_zlabel("Z");

  // 凡例を表示
  plt::legend();

  // グラフを表示
  plt::show();
  // plt::save("./basic.png");

  return 0;
}