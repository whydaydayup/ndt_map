/**
 * @file ndt_map.h
 * @author heng zhang (you@domain.com)
 * @brief 包含基于里程计的回环检测的 ndt_map，参考 autoware 的 ndt_mapping 和 lego_loam
 * @version 0.1
 * @date 2019-05-17
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#ifndef __NDT_MAP__
#define __NDT_MAP__

#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <deque>
#include <array>
#include <thread>
#include <memory>
#include <pthread.h>
#include <chrono>
#include <mutex>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/Empty.h>

#include <ndt_cpu/NormalDistributionsTransform.h>
#include <ndt_map/ground_filter.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/ISAM2.h>


struct pose
{
    double x;
    double y;
    double z;
    double roll;
    double pitch;
    double yaw;
};

using namespace gtsam;
// 添加新的point类型，跟lego_loam中一样
struct PointXYZIRPYT
{
  PCL_ADD_POINT4D    // 添加pcl里xyz+padding
  PCL_ADD_INTENSITY; // 添加pcl中的intensity
  float roll;
  float pitch;
  float yaw;
  double time;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW // 确保new操作符对齐操作,确保定义新类型点云内存与SSE对齐
} EIGEN_ALIGN16; // 强制SSE填充以正确对齐内存
// xyz,intensity, rpy, time
// 注册点类型宏，新的类型包含了xyz,intensity,roll,pitch,yaw,time
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRPYT, (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(float, roll, roll)(float, pitch, pitch)(float, yaw, yaw)(double, time, time))

using PointTPose = PointXYZIRPYT; // 包含了时间戳time，旋转rpy位姿xyz的点云类型

class NDTMap
{
public:
  NDTMap(ros::NodeHandle nh, ros::NodeHandle pnh);
  ~NDTMap();
  void run();
  void visualThread();
  void loopClosureThread();

private:
  using PointT = pcl::PointXYZI;

    pose previous_pose, guess_pose, current_pose, ndt_pose, added_pose;
    // 初值的设定
    double diff = 0.0;
    double diff_x = 0.0, diff_y = 0.0, diff_z = 0.0, diff_yaw;  // current_pose - previous_pose

    double current_velocity_x = 0.0;
    double current_velocity_y = 0.0;
    double current_velocity_z = 0.0;

    ros::Time current_scan_time;
    ros::Time previous_scan_time;
    ros::Duration scan_duration;

    bool _incremental_voxel_update = false;


  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber sub_pc_;
  ros::Subscriber sub_odom_;
  ros::Subscriber sub_imu_;

  ros::Publisher pub_keyposes_;
  ros::Publisher pub_laser_cloud_surround_;

  ros::Publisher pub_undistorted_pc_;
  ros::Publisher pub_predict_pose_;
  ros::Publisher pub_updated_pose_;

  ros::Publisher pub_history_keyframes_;
  ros::Publisher pub_icp_keyframes_;
  ros::Publisher pub_recent_keyframes_;

  ros::ServiceServer srv_save_map_;
  utils::RayGroundFilter ground_filter;

  tf::TransformBroadcaster tf_broadcaster_;
  tf::TransformListener tf_listener_;
  tf::Transform tf_m2o_;// map-->odom
  tf::Transform tf_o2b_;// odom-->base_link

  // gtsam 相关
  NonlinearFactorGraph gtSAMgraph_;//定义因子图
  Values initial_estimate_;//定义初始值容器
  ISAM2 *isam;
  Values isam_current_estimate_;
  // 三个对角线噪声模型，通过指定一个向量的方差
  noiseModel::Diagonal::shared_ptr prior_noise_;
  noiseModel::Diagonal::shared_ptr odom_noise_;
  noiseModel::Diagonal::shared_ptr constraint_noise_;

  // imu 相关
  static const int imu_queue_len_ = 100;

  int imu_ptr_front_, imu_ptr_last_, imu_ptr_last_iter_;
  Eigen::Vector3f rpy_cur_, velo_xyz_cur_, shift_xyz_cur_;
  Eigen::Vector3f rpy_start_, velo_xyz_start_, shift_xyz_start_;
  Eigen::Vector3f shift_from_start_;

  std::array<double, imu_queue_len_> imu_time_;
  std::array<float, imu_queue_len_> imu_roll_;
  std::array<float, imu_queue_len_> imu_pitch_;
  std::array<float, imu_queue_len_> imu_yaw_;

  std::array<float, imu_queue_len_> imu_acc_x_;
  std::array<float, imu_queue_len_> imu_acc_y_;
  std::array<float, imu_queue_len_> imu_acc_z_;
  std::array<float, imu_queue_len_> imu_velo_x_;
  std::array<float, imu_queue_len_> imu_velo_y_;
  std::array<float, imu_queue_len_> imu_velo_z_;
  std::array<float, imu_queue_len_> imu_shift_x_;
  std::array<float, imu_queue_len_> imu_shift_y_;
  std::array<float, imu_queue_len_> imu_shift_z_;

  std::array<float, imu_queue_len_> imu_angular_velo_x_;
  std::array<float, imu_queue_len_> imu_angular_velo_y_;
  std::array<float, imu_queue_len_> imu_angular_velo_z_;
  std::array<float, imu_queue_len_> imu_angular_rot_x_;
  std::array<float, imu_queue_len_> imu_angular_rot_y_;
  std::array<float, imu_queue_len_> imu_angular_rot_z_;

  // 里程计相关
  int odom_ptr_front_, odom_ptr_last_, odom_ptr_last_iter_;
  std::array<nav_msgs::Odometry, imu_queue_len_> odom_queue_;
  nav_msgs::Odometry pre_odom_, cur_odom_;

  // ndt 相关
  geometry_msgs::PoseWithCovarianceStamped pre_pose_ndt_, cur_pose_ndt_;
  geometry_msgs::PoseWithCovarianceStamped pre_keypose_;
  geometry_msgs::PoseWithCovarianceStamped predict_pose_;
  Eigen::Matrix4f pre_pose_m_, cur_pose_m_, pre_pose_o_, cur_pose_o_;
  Eigen::Matrix4f final_transformation_;
  double fitness_score_;
  bool has_converged_;
  int final_iters_;

  cpu::NormalDistributionsTransform<PointT, PointT> cpu_ndt_;

  pcl::VoxelGrid<PointT> voxel_filter_;

  pcl::PointCloud<PointT>::Ptr pc_source_;
  pcl::PointCloud<PointT>::Ptr pc_target_;

  pcl::PointCloud<PointT>::Ptr cloud_keyposes_3d_;// keypose_3d关键位姿
  // using PointTPose = PointXYZIRPYT; // 包含了时间戳time，旋转rpy位姿xyz的点云类型
  pcl::PointCloud<PointTPose>::Ptr cloud_keyposes_6d_;// keypose_6d关键位姿

  std::vector<pcl::PointCloud<PointT>::Ptr> cloud_keyframes_;// 关键帧点云

  pcl::KdTreeFLANN<PointT>::Ptr kdtree_poses_;

  std::vector<int> search_idx_;
  std::vector<float> search_dist_;

  pcl::VoxelGrid<PointT> ds_source_;
  pcl::VoxelGrid<PointT> ds_history_keyframes_;

  // 回环检测相关
  bool loop_closed_;// 是否检测出回环
  int latest_history_frame_id_;
  int closest_history_frame_id_;
  pcl::PointCloud<PointT>::Ptr latest_keyframe_;
  pcl::PointCloud<PointT>::Ptr near_history_keyframes_;
  // deque双端队列double-ended queue
  std::deque<pcl::PointCloud<PointT>::Ptr> recent_keyframes_;
  std::mutex mtx_;

  // 参数相关
  float scan_period_;
  bool use_odom_, use_imu_;
  float keyframe_dist_; // 移动距离作为关键帧提取参考
  bool loop_closure_enabled_;
  Eigen::Matrix4f tf_b2l_; // base_link --> localizer
  int surround_search_num_;      // 提取附近点云
  float surround_search_radius_; // kdtree 搜索参数
  float voxel_leaf_size_;        // 对地图点云进行降采样
  float min_scan_range_, max_scan_range_;

  double trans_eps_, step_size, ndt_res_; // ndt 配准参数
  int max_iters_;

  float history_search_radius_; // 回环检测参数
  int history_search_num_;
  float history_fitness_score_;
  float ds_history_size_;

  std::string save_dir_;

  bool init();

  void adjustDistortion(pcl::PointCloud<PointT>::Ptr &cloud, double scan_time);

  void extractSurroundKeyframes();

  bool saveKeyframesAndFactor();

  void publishKeyposesAndFrames();

  void odomCB(const nav_msgs::OdometryConstPtr &msg);
  void pcCB(const sensor_msgs::PointCloud2ConstPtr &msg);
  void imuCB(const sensor_msgs::ImuConstPtr &msg);
  bool saveMapCB(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

  void performLoopClosure();
  bool detectLoopClosure();
  void correctPoses();

  // 工具
  void setGeometryOrient(geometry_msgs::PoseWithCovarianceStamped &msg, float w, float x, float y, float z)
  {
    msg.pose.pose.orientation.w = w;
    msg.pose.pose.orientation.x = x;
    msg.pose.pose.orientation.y = y;
    msg.pose.pose.orientation.z = z;
  }
  void setGeometryPosition(geometry_msgs::PoseWithCovarianceStamped &msg, float x, float y, float z)
  {
    msg.pose.pose.position.x = x;
    msg.pose.pose.position.y = y;
    msg.pose.pose.position.z = z;
  }
  // tmp = transformPointCloud(cloud_keyframes_[i], cloud_keyposes_6d_->points[i]);
  pcl::PointCloud<PointT>::Ptr transformPointCloud(const pcl::PointCloud<PointT>::ConstPtr cloud_in, const PointTPose &trans)
  {
    Eigen::Matrix4f this_transformation(Eigen::Matrix4f::Identity());// 单位矩阵
    // ZYX旋转，然后toRotationMatrix转为矩阵
    this_transformation.block<3, 3>(0, 0) = (Eigen::AngleAxisf(trans.yaw, Eigen::Vector3f::UnitZ()) *
                                             Eigen::AngleAxisf(trans.pitch, Eigen::Vector3f::UnitY()) *
                                             Eigen::AngleAxisf(trans.roll, Eigen::Vector3f::UnitX()))
                                                .toRotationMatrix();
    this_transformation(0, 3) = trans.x;
    this_transformation(1, 3) = trans.y;
    this_transformation(2, 3) = trans.z;
    pcl::PointCloud<PointT>::Ptr tf_cloud(new pcl::PointCloud<PointT>());
    // template <typename PointT> void transformPointCloud (const pcl::PointCloud<PointT> &cloud_in, pcl::PointCloud<PointT> &cloud_out, const Eigen::Matrix4f &transform)
    // 对输入点云cloud_in应用transformation变换转换，保存到tf_cloud中
    pcl::transformPointCloud(*cloud_in, *tf_cloud, this_transformation);
    return tf_cloud;
  }
};

#endif