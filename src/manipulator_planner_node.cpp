#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Dense>
#include <vector>
#include <cmath>

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

template <typename T>
struct BackhoeConfig
{
  T boom_offset_x;
  T boom_offset_y;
  T boom_offset_z;
  T boom_length;
  T arm_length;
  T bucket_length;
  T tip_width;
};

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

template <typename T>
struct TiltrotatorConfig
{
  T boom_offset_x;
  T boom_offset_y;
  T boom_offset_z;
  T boom_length;
  T arm_length;
  T bucket_length;
  T tip_width;
  T a5;
  T d5;
  T d6;
  T h7;
};

template <typename T>
class TiltrotatorKinematics
{
public:
  TiltrotatorKinematics()
  {
    // パラメータの設定
    config_.boom_offset_x = 0.3;
    config_.boom_offset_y = 0.1;
    config_.boom_offset_z = 1.0;
    config_.boom_length = 5.0;
    config_.arm_length = 3.0;
    config_.bucket_length = 1.0;
    config_.tip_width = 1.0;
    config_.a5 = 1.0;
    config_.d5 = 1.0;
    config_.d6 = 1.0;
    config_.h7 = 1.0;
  }

  TiltrotatorKinematics(const TiltrotatorConfig<T> & config) : config_(config) {}

  /**
   * @brief バックホウの順運動学を解く関数
   * 
   * @param joint_states 関節角度
   * @return std::vector<Pose<T>> 関節姿勢の配列
   */
  std::vector<Pose<T>> solve_fk(const Eigen::Matrix<T, 6, 1> & joint_states)
  {
    // DHパラメータ行列の設定
    // 5行, 4列（α, a, θ, d）
    // clang-format off
    Eigen::MatrixXd dh_param_matrix(6, 4);
    dh_param_matrix << 0.0,     0.0,                   joint_states[0], config_.boom_offset_z,
                       M_PI_2,  config_.boom_offset_x, joint_states[1], config_.boom_offset_y,
                       0.0,     config_.boom_length,   joint_states[2], 0.0,
                       0.0,     config_.arm_length,    joint_states[3], 0.0,
                       -M_PI_2, config_.a5,            joint_states[4]-M_PI_2, config_.d5,
                       -M_PI_2, 0,                     joint_states[5], config_.d6;
    // clang-format on

    return compute_joint_positions(dh_param_matrix);
  }

private:
  TiltrotatorConfig<T> config_;

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

class ManipulatorPlanner
{
public:
  ManipulatorPlanner()
  {
    ros::NodeHandle nh;
    ros::NodeHandle pn("~");

    pn.param("/backhoe_configs/link_lengths/boom_offset_x", backhoe_config_.boom_offset_x, 0.3);
    pn.param("/backhoe_configs/link_lengths/boom_offset_y", backhoe_config_.boom_offset_y, 0.1);
    pn.param("/backhoe_configs/link_lengths/boom_offset_z", backhoe_config_.boom_offset_z, 1.0);
    pn.param("/backhoe_configs/link_lengths/boom_length", backhoe_config_.boom_length, 5.0);
    pn.param("/backhoe_configs/link_lengths/arm_length", backhoe_config_.arm_length, 3.0);
    pn.param("/backhoe_configs/link_lengths/bucket_length", backhoe_config_.bucket_length, 1.0);
    pn.param("/backhoe_configs/link_lengths/tip_width", backhoe_config_.tip_width, 1.0);

    pn.param("/tiltrotator_configs/link_lengths/boom_offset_x", tiltrotator_config_.boom_offset_x, 0.3);
    pn.param("/tiltrotator_configs/link_lengths/boom_offset_y", tiltrotator_config_.boom_offset_y, 0.1);
    pn.param("/tiltrotator_configs/link_lengths/boom_offset_z", tiltrotator_config_.boom_offset_z, 1.0);
    pn.param("/tiltrotator_configs/link_lengths/boom_length", tiltrotator_config_.boom_length, 5.0);
    pn.param("/tiltrotator_configs/link_lengths/arm_length", tiltrotator_config_.arm_length, 3.0);
    pn.param("/tiltrotator_configs/link_lengths/bucket_length", tiltrotator_config_.bucket_length, 1.0);
    pn.param("/tiltrotator_configs/link_lengths/tip_width", tiltrotator_config_.tip_width, 1.0);
    pn.param("/tiltrotator_configs/link_lengths/a5", tiltrotator_config_.a5, 1.0);
    pn.param("/tiltrotator_configs/link_lengths/d5", tiltrotator_config_.d5, 1.0);
    pn.param("/tiltrotator_configs/link_lengths/d6", tiltrotator_config_.d6, 1.0);
    pn.param("/tiltrotator_configs/link_lengths/h7", tiltrotator_config_.h7, 1.0);

    // 関節角度の初期化
    joint_states_.resize(4);
    joint_states_ << 0.0, 0.0, 0.0, 0.0;

    // サブスクライバ
    sub_joint_states_ = nh.subscribe("joint_states", 1, &ManipulatorPlanner::cb_joint_states, this);

    // パブリッシャ
    pub_marker_ = nh.advertise<visualization_msgs::Marker>("marker", 1);
    pub_debug_pose_array_ = nh.advertise<geometry_msgs::PoseArray>("debug_pose_array", 1);

    // タイマー
    timer_ = nh.createTimer(ros::Duration(0.1), &ManipulatorPlanner::cb_timer, this);
  }

private:
  // マニピュレータのパラメータ
  BackhoeConfig<double> backhoe_config_;
  TiltrotatorConfig<double> tiltrotator_config_;

  // 関節角度
  Eigen::VectorXd joint_states_;

  ros::Subscriber sub_joint_states_;
  ros::Publisher pub_marker_;
  ros::Publisher pub_debug_pose_array_;
  ros::Timer timer_;

  void cb_timer(const ros::TimerEvent & event)
  {
    // 画面をクリアしてカーソルをホームポジションに移動
    std::cout << "\033[2J\033[H" << std::flush;

    std::cout << "joint_states_: \n" << joint_states_ << std::endl;

    BackhoeKinematics<double> backhoe_kinematics(backhoe_config_);
    auto joint_pos = backhoe_kinematics.solve_fk(joint_states_);

    // TiltrotatorKinematics<double> tiltrotator_kinematics(tiltrotator_config_);
    // auto joint_pos = tiltrotator_kinematics.solve_fk(joint_states_);

    std::cout << "Joint positions:" << std::endl;
    for (size_t i = 0; i < joint_pos.size(); ++i) {
      std::cout << "Joint " << i << std::endl;
      joint_pos[i].print();
    }

    visualization_msgs::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = ros::Time::now();
    marker.ns = "manipulator";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.01;
    marker.color.r = 1.0;
    marker.color.a = 1.0;
    for (int i = 0; i < joint_pos.size() - 1; ++i) {
      geometry_msgs::Point p1, p2;
      p1.x = joint_pos[i].position.x();
      p1.y = joint_pos[i].position.y();
      p1.z = joint_pos[i].position.z();
      p2.x = joint_pos[i + 1].position.x();
      p2.y = joint_pos[i + 1].position.y();
      p2.z = joint_pos[i + 1].position.z();
      marker.points.push_back(p1);
      marker.points.push_back(p2);
    }
    pub_marker_.publish(marker);

    geometry_msgs::PoseArray pose_array_msg;
    pose_array_msg.header.frame_id = "base_link";
    pose_array_msg.header.stamp = ros::Time::now();
    for (int i = 0; i < joint_pos.size(); ++i) {
      geometry_msgs::Pose pose;
      pose.position.x = joint_pos[i].position.x();
      pose.position.y = joint_pos[i].position.y();
      pose.position.z = joint_pos[i].position.z();
      pose.orientation.w = joint_pos[i].orientation.w();
      pose.orientation.x = joint_pos[i].orientation.x();
      pose.orientation.y = joint_pos[i].orientation.y();
      pose.orientation.z = joint_pos[i].orientation.z();
      pose_array_msg.poses.push_back(pose);
    }
    pub_debug_pose_array_.publish(pose_array_msg);
  }

  std::string get_shape(const Eigen::MatrixXd & mat)
  {
    std::stringstream ss;
    ss << mat.rows() << "x" << mat.cols();
    return ss.str();
  }

  /////////////////////////////
  // ROSコールバック関数
  /////////////////////////////
  void cb_joint_states(const sensor_msgs::JointState::ConstPtr & msg)
  {
    // 関節数が4でない場合はエラーを出力して終了
    constexpr int joint_num = 4;
    if (msg->position.size() != joint_num) {
      ROS_ERROR("Joint states size mismatch");
      return;
    }

    // 関節角度をEigen::VectorXdとして格納
    joint_states_.resize(msg->position.size());
    for (size_t i = 0; i < msg->position.size(); ++i) {
      joint_states_[i] = msg->position[i];
    }
  }
};

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "manipulator_planner");
  ManipulatorPlanner planner;
  ros::spin();
  return 0;
}
