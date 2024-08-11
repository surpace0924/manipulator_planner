#pragma once
#include <Eigen/Dense>
#include <backhoe_config.h>
#include <angles.h>

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

  Eigen::Matrix<T, 4, 1> solve_ik(const Pose<T> & pose)
  {
    // step1 旋回角を求める
    T body_angle = 0;
    for (int i = 0; i < 100; ++i) {
      // 仮の角度を用いてboomの位置を計算
      T boom_pos_x = config_.boom_offset_x * std::cos(body_angle) + config_.boom_offset_y * std::sin(body_angle);
      T boom_pos_y = config_.boom_offset_x * std::sin(body_angle) - config_.boom_offset_y * std::cos(body_angle);

      // 目標位置との差分を計算
      T dx = pose.position.x() - boom_pos_x;
      T dy = pose.position.y() - boom_pos_y;

      // 仮の角度を計算
      T tmp_body_angle = Angles::normalize(atan2(dy, dx));
      T diff = std::abs(Angles::shortest_angle(tmp_body_angle, body_angle));

      // 収束判定
      if (diff < 1e-9) {
        break;
      }

      // 更新
      body_angle = tmp_body_angle;
      body_angle = tmp_body_angle;
    }

    // step2 boom軸位置が原点、目標位置方向がx軸、上向きがy軸となるような座標系を定義して、目標位置と角度を計算
    T x = pose.position.x();
    T y = pose.position.y();
    T z = pose.position.z();
    T target_x = std::sqrt(x * x + y * y) - config_.boom_offset_x;
    T target_y = z - config_.boom_offset_z;

    // 四元数をEuler角に変換
    // ZYX順 (Yaw, Pitch, Roll)
    Eigen::Vector3d eulerAngles = pose.orientation.toRotationMatrix().eulerAngles(2, 1, 0);
    T target_angle = eulerAngles[2] + M_PI / 2;

    // step3 bucket軸の位置を計算
    T bucket_x = config_.bucket_length * std::cos(target_angle) + target_x;
    T bucket_y = config_.bucket_length * std::sin(target_angle) + target_y;

    // step4 arm軸の位置を計算
    // boom軸原点でboom長さの半径の円と、bucket軸位置でarm長さの半径の円の交点を求める
    T arm_x, arm_y;
    Eigen::Matrix<T, 2, 1> center1(0, 0);
    Eigen::Matrix<T, 2, 1> center2(bucket_x, bucket_y);
    std::pair<Eigen::Matrix<T, 2, 1>, Eigen::Matrix<T, 2, 1>> points =
      find_circle_intersections(center1, config_.boom_length, center2, config_.arm_length);

    // Step5 ジョイント角度の計算
    T boom_angle, arm_angle, bucket_angle;
    if (std::isnan(points.first.x())) {
      // 交点がない場合
      // step5 ジョイント角度の計算
      boom_angle = std::atan2(bucket_y, bucket_x);
      arm_angle = 0;  // armの角度範囲のうち最も0に近い角度
      bucket_angle = std::atan2(target_y, target_x) - arm_angle - boom_angle;
    } else {
      // 交点のうち、y座標が大きい方を選択
      Eigen::Matrix<T, 2, 1> p1 = points.first;
      Eigen::Matrix<T, 2, 1> p2 = points.second;
      if (p1.y() > p2.y()) {
        arm_x = p1.x();
        arm_y = p1.y();
      } else {
        arm_x = p2.x();
        arm_y = p2.y();
      }

      // step5 ジョイント角度の計算
      boom_angle = std::atan2(arm_y, arm_x);
      arm_angle = std::atan2(bucket_y - arm_y, bucket_x - arm_x) - boom_angle;
      bucket_angle = std::atan2(target_y - bucket_y, target_x - bucket_x) - arm_angle - boom_angle;
    }

    // パッキング
    Eigen::Matrix<T, 4, 1> joint_states;
    joint_states[0] = body_angle;
    joint_states[1] = boom_angle;
    joint_states[2] = arm_angle;
    joint_states[3] = bucket_angle;

    // 角度を正規化
    for (int i = 0; i < 4; ++i) {
      joint_states[i] = Angles::normalize(joint_states[i]);
    }
    return joint_states;
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

  std::pair<Eigen::Matrix<T, 2, 1>, Eigen::Matrix<T, 2, 1>> find_circle_intersections(
    Eigen::Matrix<T, 2, 1> center1, T r1, Eigen::Matrix<T, 2, 1> center2, T r2)
  {
    // 2つの円の中心間の距離
    T d = (center2 - center1).norm();

    // 交点がない場合
    if (d > r1 + r2 || d < std::abs(r1 - r2) || d == 0) {
      return {
        Eigen::Matrix<T, 2, 1>(std::numeric_limits<T>::quiet_NaN(), std::numeric_limits<T>::quiet_NaN()),
        Eigen::Matrix<T, 2, 1>(std::numeric_limits<T>::quiet_NaN(), std::numeric_limits<T>::quiet_NaN())};
    }

    T a = (r1 * r1 - r2 * r2 + d * d) / (2 * d);
    T h = std::sqrt(r1 * r1 - a * a);

    Eigen::Matrix<T, 2, 1> p2 = center1 + a * (center2 - center1) / d;

    Eigen::Matrix<T, 2, 1> intersection1 = p2 + h * Eigen::Matrix<T, 2, 1>(center2.y() - center1.y(), center1.x() - center2.x()) / d;
    Eigen::Matrix<T, 2, 1> intersection2 = p2 - h * Eigen::Matrix<T, 2, 1>(center2.y() - center1.y(), center1.x() - center2.x()) / d;

    return {intersection1, intersection2};
  }
};