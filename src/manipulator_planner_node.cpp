#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>
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

    // 順運動学計算
    BackhoeKinematics<double> backhoe_kinematics(backhoe_config_);
    auto joint_pos = backhoe_kinematics.solve_fk(joint_states_);

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
