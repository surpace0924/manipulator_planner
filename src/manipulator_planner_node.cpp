#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <Eigen/Dense>
#include <vector>
#include <cmath>

#include <pose.h>
#include <backhoe_config.h>
#include <backhoe_kinematics.h>

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

    // 関節角度の初期化
    joint_states_.resize(4);
    joint_states_ << 0.0, 0.0, 0.0, 0.0;

    // 始点と終点のjoint_statesを定義
    Eigen::Matrix<double, 4, 1> start_joint_states;
    start_joint_states << 0.3, 0.38, -1.25, -0.63;
    Eigen::Matrix<double, 4, 1> end_joint_states;
    end_joint_states << 0.3, 0.26, -1.76, -1.5;

    // 100ステップで関節角度を補間
    for (int i = 0; i < 100; ++i) {
      double t = static_cast<double>(i) / 100.0;
      Eigen::Matrix<double, 4, 1> joint_states = start_joint_states + t * (end_joint_states - start_joint_states);
      joint_trajecotry_.push_back(joint_states);
    }

    height_map_.resize(200, 200);

    for (auto joint_state : joint_trajecotry_) {
      BackhoeKinematics<double> backhoe_kinematics(backhoe_config_);
      auto joint_poses = backhoe_kinematics.solve_fk(joint_state);
      Pose<double> tip_pose = joint_poses[4];
      double r = backhoe_config_.tip_width / 2;

      // 四元数をEuler角に変換
      // ZYX順 (Yaw, Pitch, Roll)
      Eigen::Vector3d eulerAngles = tip_pose.orientation.toRotationMatrix().eulerAngles(2, 1, 0);
      double yaw = eulerAngles[0];

      Pose<double> tip_left_pose;
      tip_left_pose.position = Eigen::Vector3d(
        tip_pose.position.x() + r * std::sin(M_PI / 2 + yaw), tip_pose.position.y() - r * std::cos(M_PI / 2 + yaw), tip_pose.position.z());
      tip_left_pose.orientation = tip_pose.orientation;

      Pose<double> tip_right_pose;
      tip_right_pose.position = Eigen::Vector3d(
        tip_pose.position.x() - r * std::sin(M_PI / 2 + yaw), tip_pose.position.y() + r * std::cos(M_PI / 2 + yaw), tip_pose.position.z());
      tip_right_pose.orientation = tip_pose.orientation;

      double x = joint_poses[4].position.x();
      double y = joint_poses[4].position.y();
      double z = joint_poses[4].position.z();

      for (int i = 0; i < 10; ++i) {
        double t = static_cast<double>(i) / 10.0;
        double x = tip_left_pose.position.x() + t * (tip_right_pose.position.x() - tip_left_pose.position.x());
        double y = tip_left_pose.position.y() + t * (tip_right_pose.position.y() - tip_left_pose.position.y());
        double z = tip_left_pose.position.z() + t * (tip_right_pose.position.z() - tip_left_pose.position.z());
        height_map_ = update_height_map(height_map_, x, y, z);
      }
    }

    // サブスクライバ
    sub_joint_states_ = nh.subscribe("joint_states", 1, &ManipulatorPlanner::cb_joint_states, this);

    // パブリッシャ
    pub_marker_ = nh.advertise<visualization_msgs::Marker>("marker", 1);
    pub_debug_pose_array_ = nh.advertise<geometry_msgs::PoseArray>("debug_pose_array", 1);
    pub_joint_targets_ = nh.advertise<sensor_msgs::JointState>("joint_targets", 1);
    pub_height_map_ = nh.advertise<grid_map_msgs::GridMap>("height_map", 1);

    // タイマー
    timer_ = nh.createTimer(ros::Duration(0.1), &ManipulatorPlanner::cb_timer, this);
  }

private:
  // マニピュレータのパラメータ
  BackhoeConfig<double> backhoe_config_;

  // 関節角度の軌道
  std::vector<Eigen::Matrix<double, 4, 1>> joint_trajecotry_;

  Eigen::MatrixXd height_map_;

  // 関節角度
  Eigen::VectorXd joint_states_;

  int counter_ = 0;

  ros::Subscriber sub_joint_states_;
  ros::Publisher pub_marker_;
  ros::Publisher pub_debug_pose_array_;
  ros::Publisher pub_joint_targets_;
  ros::Publisher pub_height_map_;
  ros::Timer timer_;

  Eigen::MatrixXd update_height_map(Eigen::MatrixXd height_map, double x, double y, double z)
  {
    int x_id = -10 * x + 100;
    int y_id = -10 * y + 100;

    // 現在より低ければ更新
    height_map(x_id, y_id) = std::min(height_map(x_id, y_id), z);

    return height_map;
  }

  void cb_timer(const ros::TimerEvent & event)
  {
    int count = counter_ % joint_trajecotry_.size();

    // 画面をクリアしてカーソルをホームポジションに移動
    // std::cout << "\033[2J\033[H" << std::flush;

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

    for (int i = 0; i < 20; ++i) {
      int idx = joint_trajecotry_.size() * i / 20;
      Eigen::Matrix<double, 4, 1> joint_states = joint_trajecotry_[idx];
      // FKを解く
      BackhoeKinematics<double> backhoe_kinematics(backhoe_config_);
      auto joint_poses = backhoe_kinematics.solve_fk(joint_states);
      for (int j = 0; j < joint_poses.size() - 1; ++j) {
        geometry_msgs::Point p1, p2;
        p1.x = joint_poses[j].position.x();
        p1.y = joint_poses[j].position.y();
        p1.z = joint_poses[j].position.z();
        p2.x = joint_poses[j + 1].position.x();
        p2.y = joint_poses[j + 1].position.y();
        p2.z = joint_poses[j + 1].position.z();
        marker.points.push_back(p1);
        marker.points.push_back(p2);
      }
    }

    pub_marker_.publish(marker);

    sensor_msgs::JointState joint_targets_msg;
    joint_targets_msg.header.stamp = ros::Time::now();
    joint_targets_msg.name = {"body", "boom", "arm", "bucket"};
    joint_targets_msg.position = {joint_trajecotry_[count][0], joint_trajecotry_[count][1], joint_trajecotry_[count][2], joint_trajecotry_[count][3]};
    pub_joint_targets_.publish(joint_targets_msg);

    // // FK
    // BackhoeKinematics<double> backhoe_kinematics(backhoe_config_);
    // auto joint_poses = backhoe_kinematics.solve_fk(joint_trajecotry_[count]);
    // auto tip_pose = joint_poses[4];
    // double r = backhoe_config_.tip_width / 2;

    // // 四元数をEuler角に変換
    // // ZYX順 (Yaw, Pitch, Roll)
    // Eigen::Vector3d eulerAngles = tip_pose.orientation.toRotationMatrix().eulerAngles(2, 1, 0);
    // double yaw = eulerAngles[0];

    // geometry_msgs::PoseArray pose_array_msg;
    // pose_array_msg.header.frame_id = "base_link";
    // pose_array_msg.header.stamp = ros::Time::now();
    // geometry_msgs::Pose pose;
    // pose.position.x = tip_pose.position.x();
    // pose.position.y = tip_pose.position.y();
    // pose.position.z = tip_pose.position.z();
    // pose.orientation.w = tip_pose.orientation.w();
    // pose.orientation.x = tip_pose.orientation.x();
    // pose.orientation.y = tip_pose.orientation.y();
    // pose.orientation.z = tip_pose.orientation.z();
    // pose_array_msg.poses.push_back(pose);
    // pose.position.x = tip_left_pose.position.x();
    // pose.position.y = tip_left_pose.position.y();
    // pose.position.z = tip_left_pose.position.z();
    // pose.orientation.w = tip_left_pose.orientation.w();
    // pose.orientation.x = tip_left_pose.orientation.x();
    // pose.orientation.y = tip_left_pose.orientation.y();
    // pose.orientation.z = tip_left_pose.orientation.z();
    // pose_array_msg.poses.push_back(pose);
    // pose.position.x = tip_right_pose.position.x();
    // pose.position.y = tip_right_pose.position.y();
    // pose.position.z = tip_right_pose.position.z();
    // pose.orientation.w = tip_right_pose.orientation.w();
    // pose.orientation.x = tip_right_pose.orientation.x();
    // pose.orientation.y = tip_right_pose.orientation.y();
    // pose.orientation.z = tip_right_pose.orientation.z();
    // pose_array_msg.poses.push_back(pose);
    // pub_debug_pose_array_.publish(pose_array_msg);

    // マップのパラメータ
    double resolution = 0.1;  // マップの解像度 (メートル)
    double lengthX = 20.0;    // マップの横幅 (メートル)
    double lengthY = 20.0;    // マップの高さ (メートル)
    double originX = 0.0;     // マップの原点X座標
    double originY = 0.0;     // マップの原点Y座標

    // 新しいGridMapオブジェクトを作成
    grid_map::GridMap gridMap({"elevation"});
    gridMap.setGeometry(grid_map::Length(lengthX, lengthY), resolution, grid_map::Position(originX, originY));

    // Eigen行列のデータをコピー
    for (int i = 0; i < height_map_.rows(); ++i) {
      for (int j = 0; j < height_map_.cols(); ++j) {
        gridMap.at("elevation", grid_map::Index(i, j)) = height_map_(i, j);
      }
    }

    // GridMapをGridMapメッセージに変換
    grid_map_msgs::GridMap height_map_msg;
    grid_map::GridMapRosConverter::toMessage(gridMap, height_map_msg);

    // メッセージをパブリッシュ
    gridMap.setTimestamp(ros::Time::now().toNSec());
    height_map_msg.info.header.frame_id = "base_link";

    pub_height_map_.publish(height_map_msg);

    counter_++;
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
