#pragma once
#include <Eigen/Dense>
#include <backhoe_config.h>

template <typename T>
class BackhoeKinematics
{
public:
  BackhoeKinematics()
  {
    // パラメータの設定
    config_.boom_offset_x = 0.3;
    config_.boom_offset_y = 0.1;
    config_.boom_offset_z = 1.0;
    config_.boom_length = 5.0;
    config_.arm_length = 3.0;
    config_.bucket_length = 1.0;
    config_.tip_width = 1.0;
  }

  BackhoeKinematics(const BackhoeConfig<T> & config) : config_(config) {}

  /**
   * @brief バックホウの順運動学を解く関数
   * 
   * @param joint_states 関節角度
   * @return std::vector<Pose<T>> 関節姿勢の配列
   */
  std::vector<Pose<T>> solve_fk(const Eigen::Matrix<T, 4, 1> & joint_states)
  {
    // DHパラメータ行列の設定
    // 6行, 4列（α, a, θ, d）
    // clang-format off
    Eigen::MatrixXd dh_param_matrix(6, 4);
    dh_param_matrix << 0.0,     0.0,                   joint_states[0], config_.boom_offset_z,
                       M_PI_2,  config_.boom_offset_x, joint_states[1], config_.boom_offset_y,
                       0.0,     config_.boom_length,   joint_states[2], 0.0,
                       0.0,     config_.arm_length,    joint_states[3], 0.0,
                       -M_PI_2, config_.bucket_length, -M_PI_2,         0.0,
                       -M_PI_2, 0.0,                   0.0,             0.0;
    // clang-format on

    // 計算
    auto tmp_joint_pose = compute_joint_positions(dh_param_matrix);

    // 必要なジョイントだけ取り出す
    std::vector<Pose<T>> joint_pose;
    joint_pose.push_back(tmp_joint_pose[0]);  // base_link
    joint_pose.push_back(tmp_joint_pose[2]);  // boom_link
    joint_pose.push_back(tmp_joint_pose[3]);  // arm_link
    joint_pose.push_back(tmp_joint_pose[4]);  // bucket_link
    joint_pose.push_back(tmp_joint_pose[6]);  // tip_link

    return joint_pose;
  }

private:
  BackhoeConfig<T> config_;

  /**
   * @brief DHパラメータからDH変換行列を計算する関数
   * 
   * @param alpha リンクの前後の回転角度
   * @param a リンクの前後の距離
   * @param theta リンクの回転角度
   * @param d リンクの前後の距離
   * @return Eigen::Matrix4d DH変換行列
   */
  Eigen::Matrix<T, 4, 4> generate_transform_matrix(T alpha, T a, T theta, T d)
  {
    // clang-format off
    Eigen::Matrix<T, 4, 4> trans_mat;
    trans_mat << cos(theta),              -sin(theta),             0,           a,
                 cos(alpha) * sin(theta), cos(alpha) * cos(theta), -sin(alpha), -d * sin(alpha),
                 sin(alpha) * sin(theta), sin(alpha) * cos(theta), cos(alpha),  d * cos(alpha),
                 0,                       0,                       0,           1;
    // clang-format on
    return trans_mat;
  }

  /**
   * @brief DHパラメータ行列から関節位置を計算する関数
   * 
   * @param dh_params DHパラメータ行列
   * @return std::vector<Pose<T>> 関節姿勢の配列
   */
  std::vector<Pose<T>> compute_joint_positions(const Eigen::Matrix<T, Eigen::Dynamic, 4> & dh_params)
  {
    // 関節数
    int num_joints = dh_params.rows();

    // 結果格納用 関節数+1行、3列
    std::vector<Pose<T>> joint_pos(num_joints + 1);

    // 初期位置（原点）
    Eigen::Matrix4d trans_mat = Eigen::Matrix4d::Identity();

    // 初期位置を設定
    joint_pos[0].position = trans_mat.block<1, 3>(3, 0);

    for (int i = 0; i < num_joints; ++i) {
      T alpha = dh_params(i, 0);
      T a = dh_params(i, 1);
      T theta = dh_params(i, 2);
      T d = dh_params(i, 3);

      Eigen::Matrix4d trans_mat_i = generate_transform_matrix(alpha, a, theta, d);
      trans_mat *= trans_mat_i;  // 累積変換
      T x = trans_mat(0, 3);     // x座標
      T y = trans_mat(1, 3);     // y座標
      T z = trans_mat(2, 3);     // z座標

      // 関節位置
      joint_pos[i + 1].position = trans_mat.block(0, 3, 3, 1);

      // 関節姿勢
      Eigen::Matrix<T, 3, 3> rot_mat = trans_mat.block<3, 3>(0, 0);
      joint_pos[i + 1].orientation = Eigen::Quaternion<T>(rot_mat);
    }
    return joint_pos;
  }
};