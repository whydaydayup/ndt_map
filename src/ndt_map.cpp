/**
 * @file ndt_map.cpp
 * @author heng zhang (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2019-05-20
 * 
 * @copyright Copyright (c) 2019
 * 
 */

/*
[pcl::IterativeClosestPoint::setInputTarget] Invalid or empty point cloud dataset given!
[pcl::registration::IterativeClosestPoint::compute] No input target dataset was given!
 */

#include "ndt_map/ndt_map.h"

// 计算相邻帧航向角yaw的变化函数 diff_yaw = calcDiffForRadian(current_pose.yaw, previous_pose.yaw);
static double calcDiffForRadian(const double lhs_rad, const double rhs_rad)
{
    double diff_rad = lhs_rad - rhs_rad; // 当前帧 航向角 - 上一帧 航向角
    if (diff_rad >= M_PI)
        diff_rad = diff_rad - 2 * M_PI;
    else if (diff_rad < -M_PI)
        diff_rad = diff_rad + 2 * M_PI;
    return diff_rad;
}


// 正常的建图的线程
NDTMap::NDTMap(ros::NodeHandle nh, ros::NodeHandle pnh) : nh_(nh), pnh_(pnh)
{
  pnh_.param<float>("scan_period", scan_period_, 0.2);
  pnh_.param<float>("keyframe_dist", keyframe_dist_, 0.3);
  keyframe_dist_ *= keyframe_dist_;
  pnh_.param<float>("surround_search_radius", surround_search_radius_, 20.);
  pnh_.param<int>("surround_search_num", surround_search_num_, 50);
  pnh_.param<float>("voxel_leaf_size", voxel_leaf_size_, 1.);// voxel降采样，默认2
  pnh_.param<float>("min_scan_range", min_scan_range_, 2);// 最小检测距离，默认2
  min_scan_range_ *= min_scan_range_;
  pnh_.param<float>("max_scan_range", max_scan_range_, 100);// 最大检测距离默认70
  max_scan_range_ *= max_scan_range_;
  pnh_.param<bool>("use_odom", use_odom_, true);
  pnh_.param<bool>("use_imu", use_imu_, true); // 基于LOAM的匀速运动假设，使用IMU去畸变
  pnh_.param<bool>("loop_closure_enabled", loop_closure_enabled_, true);// 回环开关
  // NDT参数
  pnh_.param<double>("trans_eps", trans_eps_, 0.01);
  pnh_.param<double>("step_size", step_size, 0.1);
  pnh_.param<double>("ndt_res", ndt_res_, 1.);
  pnh_.param<int>("max_iters", max_iters_, 30);
  // 回环参数
  pnh_.param<float>("history_search_radius", history_search_radius_, 10.);
  pnh_.param<int>("history_search_num", history_search_num_, 20);
  pnh_.param<float>("history_fitness_score", history_fitness_score_, 0.3);
  pnh_.param<float>("ds_history_size", ds_history_size_, 1.);// voxel降采样
  // 保存的目录
  pnh_.param<std::string>("save_dir", save_dir_, "");

  // Eigen::Matrix4f tf_b2l_;
  tf_b2l_ = Eigen::Matrix4f::Identity();// 单位矩阵
  float roll, pitch, yaw;
  // /base_link --> /localizer的转换，
  if (!nh_.getParam("tf_b2l_x", tf_b2l_(0, 3)) || !nh_.getParam("tf_b2l_y", tf_b2l_(1, 3)) || !nh_.getParam("tf_b2l_z", tf_b2l_(2, 3)) || !nh_.getParam("tf_b2l_roll", roll) || !nh_.getParam("tf_b2l_pitch", pitch) || !nh_.getParam("tf_b2l_yaw", yaw))
  {
    ROS_ERROR("transform between /base_link to /laser not set.");
    exit(-1);
  }
  // 轴角，绕着XYZ轴旋转Roll，Pitch，yaw角度
  Eigen::AngleAxisf rx(roll, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf ry(pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf rz(yaw, Eigen::Vector3f::UnitZ());
  // 将0,0开始的3x3的矩阵 赋值
  tf_b2l_.block(0, 0, 3, 3) = (rz * ry * rx).matrix();

  if (!init()) // 初始化
  {
    exit(-1);
  }
	// 发布的消息
  pub_keyposes_ = nh_.advertise<sensor_msgs::PointCloud2>("/keyposes", 1);
  pub_laser_cloud_surround_ = nh_.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surround", 1);
  pub_undistorted_pc_ = nh_.advertise<sensor_msgs::PointCloud2>("/undistorted_pc", 1);
  pub_predict_pose_ = nh_.advertise<nav_msgs::Odometry>("/predict_pose", 1);
  pub_updated_pose_ = nh_.advertise<nav_msgs::Odometry>("/updated_pose", 1);

  pub_history_keyframes_ = nh_.advertise<sensor_msgs::PointCloud2>("/history_keyframes", 1);
  pub_recent_keyframes_ = nh_.advertise<sensor_msgs::PointCloud2>("/recent_keyframes", 1);
  pub_icp_keyframes_ = nh_.advertise<sensor_msgs::PointCloud2>("/icp_keyframes", 1);
	// service服务函数，来保存地图数据
  srv_save_map_ = nh_.advertiseService("/save_map", &NDTMap::saveMapCB, this);
	// 订阅的消息
  sub_pc_ = nh_.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 5, boost::bind(&NDTMap::pcCB, this, _1));
  sub_imu_ = nh_.subscribe<sensor_msgs::Imu>("/imu/data", 5, boost::bind(&NDTMap::imuCB, this, _1));
  sub_odom_ = nh_.subscribe<nav_msgs::Odometry>("/gps/imu", 5, boost::bind(&NDTMap::odomCB, this, _1));
}
// ndtmap初始化
bool NDTMap::init()
{
    previous_pose.x = 0.0;
    previous_pose.y = 0.0;
    previous_pose.z = 0.0;
    previous_pose.roll = 0.0;
    previous_pose.pitch = 0.0;
    previous_pose.yaw = 0.0;
    //
    ndt_pose.x = 0.0;
    ndt_pose.y = 0.0;
    ndt_pose.z = 0.0;
    ndt_pose.roll = 0.0;
    ndt_pose.pitch = 0.0;
    ndt_pose.yaw = 0.0;
    // 当前位姿  current_pose = ndt_pose;
    current_pose.x = 0.0;
    current_pose.y = 0.0;
    current_pose.z = 0.0;
    current_pose.roll = 0.0;
    current_pose.pitch = 0.0;
    current_pose.yaw = 0.0;

    // 只使用激光雷达时，进行NDT估计的初始位姿
    guess_pose.x = 0.0;
    guess_pose.y = 0.0;
    guess_pose.z = 0.0;
    guess_pose.roll = 0.0;
    guess_pose.pitch = 0.0;
    guess_pose.yaw = 0.0;
    // 判断是否更新地图时，上一次更新地图时的位姿
    added_pose.x = 0.0;
    added_pose.y = 0.0;
    added_pose.z = 0.0;
    added_pose.roll = 0.0;
    added_pose.pitch = 0.0;
    added_pose.yaw = 0.0;
    // 前后两帧之间的变化
    diff_x = 0.0;
    diff_y = 0.0;
    diff_z = 0.0;
    diff_yaw = 0.0;


  // 用于闭环图优化的参数设置，使用gtsam库,    iSAM2参数
  ISAM2Params params;//在定义ISAM2实例的时候存储参数的。
  params.relinearizeThreshold = 0.01; // 默认0.1
  params.relinearizeSkip = 1;/// < Only relinearize any variables every relinearizeSkip calls to ISAM2::update (default: 10)
  isam = new ISAM2(params);// 创建一个ISAM2实例
  gtsam::Vector vector6(6);
  vector6 << 1e-6, 1e-6, 1e-6, 1e-8, 1e-8, 1e-6;
  // 噪声定义，对角线噪声模型，通过指定一个向量的方差
  prior_noise_ = noiseModel::Diagonal::Variances(vector6);// 对角线噪声
  odom_noise_ = noiseModel::Diagonal::Variances(vector6);
  // ndt设置
  cpu_ndt_.setTransformationEpsilon(trans_eps_);
  cpu_ndt_.setResolution(ndt_res_);
  cpu_ndt_.setStepSize(step_size);
  cpu_ndt_.setMaximumIterations(max_iters_);
  // 设置voxel降采样参数
  voxel_filter_.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
  ds_source_.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
  ds_history_keyframes_.setLeafSize(ds_history_size_, ds_history_size_, ds_history_size_);

  // int imu_ptr_front_, imu_ptr_last_, imu_ptr_last_iter_;
  // int odom_ptr_front_, odom_ptr_last_, odom_ptr_last_iter_;
  imu_ptr_front_ = odom_ptr_front_ = 0;
  imu_ptr_last_ = odom_ptr_last_ = -1;
  imu_ptr_last_iter_ = odom_ptr_last_iter_ = 0;

  pre_pose_m_ = cur_pose_m_ = pre_pose_o_ = cur_pose_o_ = Eigen::Matrix4f::Identity();

  tf_m2o_.setIdentity();

  loop_closed_ = false;// ？？？？啥变量？？？

  ground_filter.setIfClipHeight(false);// 是否截取掉高于雷达自身xx米的点，默认是20米
  ground_filter.setMinDistance(1.0);// 移除给定xx距离的内的点

  pc_source_.reset(new pcl::PointCloud<PointT>());
  pc_target_.reset(new pcl::PointCloud<PointT>());

  cloud_keyposes_3d_.reset(new pcl::PointCloud<PointT>());
  cloud_keyposes_6d_.reset(new pcl::PointCloud<PointXYZIRPYT>());

  latest_keyframe_.reset(new pcl::PointCloud<PointT>());
  near_history_keyframes_.reset(new pcl::PointCloud<PointT>());

  kdtree_poses_.reset(new pcl::KdTreeFLANN<PointT>());

  // std::array<double, imu_queue_len_> imu_time_;
  imu_time_.fill(0);// fill(0)将整个array中的所有元素全部置为0
  imu_roll_.fill(0);
  imu_pitch_.fill(0);
  imu_yaw_.fill(0);

  imu_acc_x_.fill(0);
  imu_acc_y_.fill(0);
  imu_acc_z_.fill(0);
  imu_velo_x_.fill(0);
  imu_velo_y_.fill(0);
  imu_velo_z_.fill(0);
  imu_shift_x_.fill(0);
  imu_shift_y_.fill(0);
  imu_shift_z_.fill(0);

  imu_angular_velo_x_.fill(0);
  imu_angular_velo_y_.fill(0);
  imu_angular_velo_z_.fill(0);
  imu_angular_rot_x_.fill(0);
  imu_angular_rot_y_.fill(0);
  imu_angular_rot_z_.fill(0);

  ROS_INFO("init.");
  return true;
}

void NDTMap::run() {}

void NDTMap::visualThread()
{
  ros::Duration duration(2.5);
  while (ros::ok())
  {
    publishKeyposesAndFrames();
    duration.sleep();// 每2.5秒发布一次关键位姿和帧数据
  }
}

/**
 * @brief 参考 loam 的点云去运动畸变（基于匀速运动假设）这里使用的是匀加速运动模型，若不适用IMU应该用的是匀速运动模型去畸变
 * 
 */
void NDTMap::adjustDistortion(pcl::PointCloud<PointT>::Ptr &cloud, double scan_time)
{
    // loam中把所有的点云数据全部映射到每个线号上，然后去每个点云的畸变
  bool half_passed = false;
  int cloud_size = cloud->points.size();

  float start_ori = -std::atan2(cloud->points[0].y, cloud->points[0].x);
  float end_ori = -std::atan2(cloud->points[cloud_size - 1].y, cloud->points[cloud_size - 1].x);
  if (end_ori - start_ori > 3 * M_PI)
  {
    end_ori -= 2 * M_PI;
  }
  else if (end_ori - start_ori < M_PI)
  {
    end_ori += 2 * M_PI;
  }
  float ori_diff = end_ori - start_ori;

  Eigen::Vector3f rpy_start, shift_start, velo_start, rpy_cur, shift_cur, velo_cur;
  Eigen::Vector3f shift_from_start;
  Eigen::Matrix3f r_s_i, r_c;
  Eigen::Vector3f adjusted_p;
  float ori_h;
  for (int i = 0; i < cloud_size; ++i)
  {
    PointT &p = cloud->points[i];
    ori_h = -std::atan2(p.y, p.x);
    if (!half_passed)
    {
      if (ori_h < start_ori - M_PI * 0.5)
      {
        ori_h += 2 * M_PI;
      }
      else if (ori_h > start_ori + M_PI * 1.5)
      {
        ori_h -= 2 * M_PI;
      }

      if (ori_h - start_ori > M_PI)
      {
        half_passed = true;
      }
    }
    else
    {
      ori_h += 2 * M_PI;
      if (ori_h < end_ori - 1.5 * M_PI)
      {
        ori_h += 2 * M_PI;
      }
      else if (ori_h > end_ori + 0.5 * M_PI)
      {
        ori_h -= 2 * M_PI;
      }
    }

    float rel_time = (ori_h - start_ori) / ori_diff * scan_period_;

    if (imu_ptr_last_ > 0)
    {
      imu_ptr_front_ = imu_ptr_last_iter_;
      while (imu_ptr_front_ != imu_ptr_last_)
      {
        if (scan_time + rel_time < imu_time_[imu_ptr_front_])
        {
          break;
        }
        imu_ptr_front_ = (imu_ptr_front_ + 1) % imu_queue_len_;
      }
      if (std::abs(scan_time + rel_time - imu_time_[imu_ptr_front_]) > scan_period_)
      {
        ROS_WARN_COND(i < 10, "unsync imu and pc msg");
        continue;
      }

      if (scan_time + rel_time > imu_time_[imu_ptr_front_])
      {
        rpy_cur(0) = imu_roll_[imu_ptr_front_];
        rpy_cur(1) = imu_pitch_[imu_ptr_front_];
        rpy_cur(2) = imu_yaw_[imu_ptr_front_];
        shift_cur(0) = imu_shift_x_[imu_ptr_front_];
        shift_cur(1) = imu_shift_y_[imu_ptr_front_];
        shift_cur(2) = imu_shift_z_[imu_ptr_front_];
        velo_cur(0) = imu_velo_x_[imu_ptr_front_];
        velo_cur(1) = imu_velo_y_[imu_ptr_front_];
        velo_cur(2) = imu_velo_z_[imu_ptr_front_];
      }
      else
      {
        int imu_ptr_back = (imu_ptr_front_ - 1 + imu_queue_len_) % imu_queue_len_;
        float ratio_front = (scan_time + rel_time - imu_time_[imu_ptr_back]) / (imu_time_[imu_ptr_front_] - imu_time_[imu_ptr_back]);
        float ratio_back = 1. - ratio_front;
        rpy_cur(0) = imu_roll_[imu_ptr_front_] * ratio_front + imu_roll_[imu_ptr_back] * ratio_back;
        rpy_cur(1) = imu_pitch_[imu_ptr_front_] * ratio_front + imu_pitch_[imu_ptr_back] * ratio_back;
        rpy_cur(2) = imu_yaw_[imu_ptr_front_] * ratio_front + imu_yaw_[imu_ptr_back] * ratio_back;
        shift_cur(0) = imu_shift_x_[imu_ptr_front_] * ratio_front + imu_shift_x_[imu_ptr_back] * ratio_back;
        shift_cur(1) = imu_shift_y_[imu_ptr_front_] * ratio_front + imu_shift_y_[imu_ptr_back] * ratio_back;
        shift_cur(2) = imu_shift_z_[imu_ptr_front_] * ratio_front + imu_shift_z_[imu_ptr_back] * ratio_back;
        velo_cur(0) = imu_velo_x_[imu_ptr_front_] * ratio_front + imu_velo_x_[imu_ptr_back] * ratio_back;
        velo_cur(1) = imu_velo_y_[imu_ptr_front_] * ratio_front + imu_velo_y_[imu_ptr_back] * ratio_back;
        velo_cur(2) = imu_velo_z_[imu_ptr_front_] * ratio_front + imu_velo_z_[imu_ptr_back] * ratio_back;
      }

      r_c = (Eigen::AngleAxisf(rpy_cur(2), Eigen::Vector3f::UnitZ()) * Eigen::AngleAxisf(rpy_cur(1), Eigen::Vector3f::UnitY()) * Eigen::AngleAxisf(rpy_cur(0), Eigen::Vector3f::UnitX())).toRotationMatrix();

      if (i == 0)
      {
        rpy_start = rpy_cur;
        shift_start = shift_cur;
        velo_start = velo_cur;
        r_s_i = r_c.inverse();
      }
      else
      {
        shift_from_start = shift_cur - shift_start - velo_start * rel_time;
        adjusted_p = r_s_i * (r_c * Eigen::Vector3f(p.x, p.y, p.z) + shift_from_start);
        p.x = adjusted_p.x();
        p.y = adjusted_p.y();
        p.z = adjusted_p.z();
      }
    }
    imu_ptr_last_iter_ = imu_ptr_front_;
  }

  if (pub_undistorted_pc_.getNumSubscribers() > 0) // 发布去畸变后的点云数据"/undistorted_pc"
  {
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header.stamp.fromSec(scan_time);
    msg.header.frame_id = "/laser";
    pub_undistorted_pc_.publish(msg);
  }
}

/**
 * @brief 提取附近点云作为 target map进行NDT匹配。
 * 在 loop_closure_enabled_ = false 的情况下目前不需要提取，因为可以通过 updateVoxelGrid 更新 target map
 *
 */
void NDTMap::extractSurroundKeyframes()
{
    // if(cloudKeyPoses3D为空) return；
    // cloudKeyPoses3D是保存了机器人在建图过程中轨迹，这里一般是刚开始建图，没有轨迹
  if (cloud_keyframes_.empty()) // 关键帧点云为空
  {
    return;
  }

  bool target_updated = false;
  if (loop_closure_enabled_)// 开启回环检测
  {
      // std::deque<pcl::PointCloud<PointT>::Ptr> recent_keyframes_;
      // 1.recent_keyframes_保存的点云数量太少，则清空后重新塞入新的点云直至数量够
      // surround_search_num_ = 50;
    if (recent_keyframes_.size() < surround_search_num_)
    {
      recent_keyframes_.clear();
      for (int i = cloud_keyposes_3d_->points.size() - 1; i >= 0; --i)
      {
          // cloudKeyPoses3D的intensity中存的是索引值,保存的索引值从0开始编号；
        int this_key_id = int(cloud_keyposes_3d_->points[i].intensity);
        pcl::PointCloud<PointT>::Ptr tf_cloud(new pcl::PointCloud<PointT>());
        // 对输入点云cloud_keyframes_应用cloud_keyposes_6d_变换转换，保存到tf_cloud中
        tf_cloud = transformPointCloud(cloud_keyframes_[this_key_id], cloud_keyposes_6d_->points[this_key_id]);
        recent_keyframes_.push_back(tf_cloud);// push_back放在最后面，   lego_loam中是push_front放在最前面，在容器的开头添加元素，后面的else中是pop_front弹出最最前面的元素
        if (recent_keyframes_.size() >= surround_search_num_)
        {
          break;
        }
      }
      target_updated = true;
    }
    //将最新的点云叠加
    // 2.否则pop_back()队列recent_keyframes_最后面的一个，再往队列前部（头部）push_front一个；
    else
    {
        // static静态全局变量
      static int latest_frame_id = cloud_keyframes_.size() - 1;
        // recent_keyframes_中点云保存的数量较多(>=surround_search_num_(50))
        // pop队列最后面的一个，再push前面一个
      if (latest_frame_id != cloud_keyframes_.size() - 1)
      {
        latest_frame_id = cloud_keyframes_.size() - 1;
        recent_keyframes_.pop_back();// pop_back()
        pcl::PointCloud<PointT>::Ptr tf_cloud(new pcl::PointCloud<PointT>());
        tf_cloud = transformPointCloud(cloud_keyframes_[latest_frame_id], cloud_keyposes_6d_->points[latest_frame_id]);
        recent_keyframes_.push_front(tf_cloud);
        target_updated = true;
      }
    }
  }
  // 没有回环的情况，在这里是寻找附近点之后进行降维处理，
  // 目的是让地图点云不要太过密集，最终将合适的点云累加起来，这是一个将新点云在线调整并叠加的过程。
  else// 没有开启回环检测
  {
  }

  if (target_updated)
  {
//    pc_target_->clear();
    // 遍历所有的recent_keyframes_，将其添加到pc_target中
//    for (auto keyframe : recent_keyframes_)
//    {
//        *pc_target_ += *keyframe;
//    }
//    cpu_ndt_.setInputTarget(pc_target_);
    ROS_INFO("new ndt target set");
  }

  if (pub_recent_keyframes_.getNumSubscribers() > 0)// 发布最近的关键帧点云"/recent_keyframes"
  {
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*pc_target_, msg);
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";
    pub_recent_keyframes_.publish(msg);
  }
}

/**
 * @brief 保存关键帧及其对应位姿，更新位姿图
 * 移动距离作为关键帧选取标准
 * 
 */
bool NDTMap::saveKeyframesAndFactor()
{
  // 此处的当前位姿(cur_pose_ndt_)为 ndt 匹配后的 final_transformation
  double roll, pitch, yaw;
  tf::Matrix3x3(tf::Quaternion(cur_pose_ndt_.pose.pose.orientation.x, cur_pose_ndt_.pose.pose.orientation.y, cur_pose_ndt_.pose.pose.orientation.z, cur_pose_ndt_.pose.pose.orientation.w)).getRPY(roll, pitch, yaw);
  if (cloud_keyposes_3d_->points.empty())// 第一帧点云数据输入
  {
    // gtSAMgraph_.add(PriorFactor<Pose3>(0, Pose3(Rot3::Quaternion(cur_pose_ndt_.pose.pose.orientation.w, cur_pose_ndt_.pose.pose.orientation.x, cur_pose_ndt_.pose.pose.orientation.y, cur_pose_ndt_.pose.pose.orientation.z), Point3(cur_pose_ndt_.pose.pose.position.x, cur_pose_ndt_.pose.pose.position.y, cur_pose_ndt_.pose.pose.position.z)), prior_noise_));
    gtSAMgraph_.add(PriorFactor<Pose3>(0, Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(cur_pose_ndt_.pose.pose.position.x, cur_pose_ndt_.pose.pose.position.y, cur_pose_ndt_.pose.pose.position.z)), prior_noise_));
    initial_estimate_.insert(0, Pose3(Rot3::Quaternion(cur_pose_ndt_.pose.pose.orientation.w, cur_pose_ndt_.pose.pose.orientation.x, cur_pose_ndt_.pose.pose.orientation.y, cur_pose_ndt_.pose.pose.orientation.z), Point3(cur_pose_ndt_.pose.pose.position.x, cur_pose_ndt_.pose.pose.position.y, cur_pose_ndt_.pose.pose.position.z)));
    pre_keypose_ = cur_pose_ndt_;
  }
  else
  {
      pcl::PointCloud<PointT>::Ptr pc_m(new pcl::PointCloud<PointT>());
      pcl::transformPointCloud(*pc_source_, *pc_m, cur_pose_m_);
      // Update voxel grid
      // cpu_ndt_.updateVoxelGrid(pc_m);

      // 加入到地图中
      *pc_target_ += *pc_m;
      // 加入地图的 帧 的位姿
      if (_incremental_voxel_update == true)
          cpu_ndt_.updateVoxelGrid(pc_m);// Update voxel grid
      else
          cpu_ndt_.setInputTarget(pc_target_);

      double shift = sqrt(pow(current_pose.x - added_pose.x, 2.0) + pow(current_pose.y - added_pose.y, 2.0));
      // 给定两帧距离
      if (shift <= keyframe_dist_)// 0.3
      {
          // too close
          return false;
      }
      added_pose.x = current_pose.x;
      added_pose.y = current_pose.y;
      added_pose.z = current_pose.z;
      added_pose.roll = current_pose.roll;
      added_pose.pitch = current_pose.pitch;
      added_pose.yaw = current_pose.yaw;

      PointTPose pre_pose;
      pre_pose.x = added_pose.x;
      pre_pose.y = added_pose.y;
      pre_pose.z = added_pose.z;
      pre_pose.roll = added_pose.roll;
      pre_pose.pitch = added_pose.pitch;
      pre_pose.yaw = added_pose.yaw;
    // const auto &pre_pose = cloud_keyposes_6d_->points[cloud_keyposes_3d_->points.size() - 1];
     std::cout << "cloud_keyposes_3d_->points.size(): " << cloud_keyposes_3d_->points.size() << std::endl;
//    if (std::pow(cur_pose_ndt_.pose.pose.position.x - pre_pose.x, 2) +
//            std::pow(cur_pose_ndt_.pose.pose.position.y - pre_pose.y, 2) +
//            std::pow(cur_pose_ndt_.pose.pose.position.z - pre_pose.z, 2) <
//        keyframe_dist_) // keyframe_dist_:0.3
//    {
//      // too close
//      return false;
//    }

    // gtsam::Pose3 pose_from = Pose3(Rot3::Quaternion(pre_keypose_.pose.pose.orientation.w, pre_keypose_.pose.pose.orientation.x, pre_keypose_.pose.pose.orientation.y, pre_keypose_.pose.pose.orientation.z), Point3(pre_keypose_.pose.pose.position.x, pre_keypose_.pose.pose.position.y, pre_keypose_.pose.pose.position.z));
    gtsam::Pose3 pose_from = Pose3(Rot3::RzRyRx(pre_pose.roll, pre_pose.pitch, pre_pose.yaw), Point3(pre_pose.x, pre_pose.y, pre_pose.z));
    // gtsam::Pose3 pose_to = Pose3(Rot3::Quaternion(cur_pose_ndt_.pose.pose.orientation.w, cur_pose_ndt_.pose.pose.orientation.x, cur_pose_ndt_.pose.pose.orientation.y, cur_pose_ndt_.pose.pose.orientation.z), Point3(cur_pose_ndt_.pose.pose.position.x, cur_pose_ndt_.pose.pose.position.y, cur_pose_ndt_.pose.pose.position.z));
    gtsam::Pose3 pose_to = Pose3(Rot3::RzRyRx(roll, pitch * 0, yaw), Point3(cur_pose_ndt_.pose.pose.position.x, cur_pose_ndt_.pose.pose.position.y, cur_pose_ndt_.pose.pose.position.z * 0));
    gtSAMgraph_.add(BetweenFactor<Pose3>(cloud_keyposes_3d_->points.size() - 1, cloud_keyposes_3d_->points.size(), pose_from.between(pose_to), odom_noise_));
    initial_estimate_.insert(cloud_keyposes_3d_->points.size(), Pose3(Rot3::RzRyRx(roll, pitch * 0, yaw), Point3(cur_pose_ndt_.pose.pose.position.x, cur_pose_ndt_.pose.pose.position.y, cur_pose_ndt_.pose.pose.position.z * 0)));
  }

  isam->update(gtSAMgraph_, initial_estimate_);
  isam->update();

  gtSAMgraph_.resize(0);
  initial_estimate_.clear();

  PointT this_pose_3d;
  PointXYZIRPYT this_pose_6d;
  Pose3 latest_estimate;
  isam_current_estimate_ = isam->calculateEstimate();
  latest_estimate = isam_current_estimate_.at<Pose3>(isam_current_estimate_.size() - 1);

  this_pose_6d.x = this_pose_3d.x = latest_estimate.translation().x();
  this_pose_6d.y = this_pose_3d.y = latest_estimate.translation().y();
  this_pose_6d.z = this_pose_3d.z = latest_estimate.translation().z();
  // intensity表示的是关键帧的
  this_pose_6d.intensity = this_pose_3d.intensity = cloud_keyposes_3d_->points.size();// intensity直接等于size()大小
  this_pose_6d.roll = latest_estimate.rotation().roll();
  this_pose_6d.pitch = latest_estimate.rotation().pitch();
  this_pose_6d.yaw = latest_estimate.rotation().yaw();
  this_pose_6d.time = cur_pose_ndt_.header.stamp.toSec();
  cloud_keyposes_3d_->points.push_back(this_pose_3d);
  cloud_keyposes_6d_->points.push_back(this_pose_6d);

  // std::cout << "pre_keypose: (" << pre_keypose_.pose.pose.position.x << ", " << pre_keypose_.pose.pose.position.y << ", " << pre_keypose_.pose.pose.position.z << "; " << std::endl;
  std::cout << "cur_pose_ndt: (" << cur_pose_ndt_.pose.pose.position.x << ", " << cur_pose_ndt_.pose.pose.position.y << ", " << cur_pose_ndt_.pose.pose.position.z << "; "
            << roll << ", " << pitch << ", " << yaw << ")" << std::endl;
  std::cout << "this_pose_6d: (" << this_pose_6d.x << ", " << this_pose_6d.y << ", " << this_pose_6d.z << "; "
            << this_pose_6d.roll << ", " << this_pose_6d.pitch << ", " << this_pose_6d.yaw << ")" << std::endl;

  if (cloud_keyposes_3d_->points.size() > 1)
  {
    pre_keypose_.pose.pose.position.x = this_pose_3d.x;
    pre_keypose_.pose.pose.position.y = this_pose_3d.y;
    pre_keypose_.pose.pose.position.z = this_pose_3d.z;
    pre_keypose_.pose.pose.orientation.w = latest_estimate.rotation().toQuaternion().w();
    pre_keypose_.pose.pose.orientation.x = latest_estimate.rotation().toQuaternion().x();
    pre_keypose_.pose.pose.orientation.y = latest_estimate.rotation().toQuaternion().y();
    pre_keypose_.pose.pose.orientation.z = latest_estimate.rotation().toQuaternion().z();
    pre_keypose_.header.stamp = cur_pose_ndt_.header.stamp;
  }

  cur_pose_m_.block<3, 3>(0, 0) = Eigen::Quaternionf(pre_keypose_.pose.pose.orientation.w, pre_keypose_.pose.pose.orientation.x, pre_keypose_.pose.pose.orientation.y, pre_keypose_.pose.pose.orientation.z).toRotationMatrix();
  cur_pose_m_(0, 3) = pre_keypose_.pose.pose.position.x;
  cur_pose_m_(1, 3) = pre_keypose_.pose.pose.position.y;
  cur_pose_m_(2, 3) = pre_keypose_.pose.pose.position.z;

  pcl::PointCloud<PointT>::Ptr cur_keyframe(new pcl::PointCloud<PointT>());
  pcl::copyPointCloud(*pc_source_, *cur_keyframe);
  for (auto &p : cur_keyframe->points)
  {
    p.intensity = this_pose_3d.intensity;
  }

  cloud_keyframes_.push_back(cur_keyframe);

  ROS_INFO("saveKeyframesAndFactor: %d points", cur_keyframe->points.size());

  return true;
}

void NDTMap::publishKeyposesAndFrames()
{
    // "/keyposes"
  if (pub_keyposes_.getNumSubscribers() > 0)
  {
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*cloud_keyposes_3d_, msg);
    msg.header.frame_id = "map";
    msg.header.stamp = ros::Time::now();
    pub_keyposes_.publish(msg);
  }
  // "/laser_cloud_surround"
  if (pub_laser_cloud_surround_.getNumSubscribers() > 0)
  {
    int num_points = 0;
    sensor_msgs::PointCloud2 msg;
    pcl::PointCloud<PointT>::Ptr map(new pcl::PointCloud<PointT>());
    Eigen::Matrix4f T(Eigen::Matrix4f::Identity());
    for (int i = 0; i < cloud_keyframes_.size(); ++i)
    {
      pcl::PointCloud<PointT>::Ptr tmp(new pcl::PointCloud<PointT>());
      tmp = transformPointCloud(cloud_keyframes_[i], cloud_keyposes_6d_->points[i]);
      *map += *tmp;
      num_points += tmp->points.size();
    }
    pcl::toROSMsg(*map, msg);
    msg.header.frame_id = "map";
    msg.header.stamp = ros::Time::now();
    pub_laser_cloud_surround_.publish(msg);
    ROS_INFO("Map Size: %d points", num_points);
  }
}

void NDTMap::odomCB(const nav_msgs::OdometryConstPtr &msg)
{
  odom_ptr_last_ = (odom_ptr_last_ + 1) % imu_queue_len_;
  if ((odom_ptr_last_ + 1) % imu_queue_len_ == odom_ptr_front_)
  {
    odom_ptr_front_ = (odom_ptr_front_ + 1) % imu_queue_len_;
  }
  odom_queue_[odom_ptr_last_] = *msg;
}
// 激光雷达数据/velodyne_points回调函数  pointCloud_CallBack
void NDTMap::pcCB(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    // lock_guard 自动加锁、解锁。原理是 RAII，和智能指针类似。
  // std::lock_guard<std::mutex> lock(mtx_);// 线程锁
  auto start = std::chrono::system_clock::now();// 开始的时间

  ros::Time current_scan_time = msg->header.stamp;
  static ros::Time previous_scan_time = current_scan_time;

    Eigen::Matrix4f t_base_link(Eigen::Matrix4f::Identity());

    // 抽取周围关键帧
  // extractSurroundKeyframes();// 提取附近点云作为 target map进行NDT匹配。 *pc_target_ += *keyframe (recent_keyframes_所有点云)

  // 去运动畸变，距离和 voxel 滤波
  pcl::PointCloud<PointT>::Ptr tmp_cloud(new pcl::PointCloud<PointT>());

  pc_source_->clear();
  pcl::fromROSMsg(*msg, *tmp_cloud);
  // 点云由/localizer坐标系 转换到 /base_link 坐标系中处理，
  // 进来的点云数据直接转换到/base_link坐标系下面
  pcl::transformPointCloud(*tmp_cloud, *pc_source_, tf_b2l_);

  if (use_imu_)// 使用IMU数据去点云畸变
  {
    adjustDistortion(pc_source_, msg->header.stamp.toSec());// 基于LOAM的匀加速运动假设，使用IMU去畸变
  }

  // volex降采样
  tmp_cloud->clear();
  ROS_INFO("before ds: %d points", pc_source_->points.size());
  voxel_filter_.setInputCloud(pc_source_);
  voxel_filter_.filter(*tmp_cloud);
  pc_source_->clear();

  float r;
  for (const auto &p : tmp_cloud->points) // 保留指定距离内的点
  {
    r = p.x * p.x + p.y * p.y;
    // 最大最小距离是100, 2米
    if (r > min_scan_range_ && r < max_scan_range_)
    {
      pc_source_->points.push_back(p);
    }
  }
  ROS_INFO("after ds: %d points", pc_source_->points.size());

  // 第一帧点云，存为 target，初始化起始位置
  if (cloud_keyframes_.empty()) // 也可以用一个flag表示initial_scan_loaded, is_first_map
  {
    ROS_INFO("first laser frame.");
    *pc_target_ += *pc_source_;
    // 第一帧的时候，直接设置为InputTarget
    cpu_ndt_.setInputTarget(pc_target_);// ndt输入的target点云数据
  }



    // nav_msgs::Odometry predict_msg;// 预测的位姿数据
  // 配准初始位姿估计， 使用odometer的情况下
 // 不使用odometer的情况下
  {
    // TODO: 改进
//    predict_pose_ = pre_pose_ndt_;
//    cur_pose_m_ = pre_pose_m_;

      // 初始pose,不使用odometer，的预测值
      guess_pose.x = previous_pose.x + diff_x; // 初始位姿 = 上一帧t-1的位姿 + (t-1  -  t-2)的位姿差
      guess_pose.y = previous_pose.y + diff_y;
      guess_pose.z = previous_pose.z + diff_z;
      guess_pose.roll = previous_pose.roll; // 横摆角
      guess_pose.pitch = previous_pose.pitch;// 俯仰角
      guess_pose.yaw = previous_pose.yaw + diff_yaw;// 平面上运动，近似只有偏航角变化

      pose guess_pose_for_ndt;
      guess_pose_for_ndt = guess_pose;
    //  欧拉角->旋转向量
      Eigen::AngleAxisf init_rotation_x(guess_pose_for_ndt.roll, Eigen::Vector3f::UnitX());//沿X轴转roll
      Eigen::AngleAxisf init_rotation_y(guess_pose_for_ndt.pitch, Eigen::Vector3f::UnitY());
      Eigen::AngleAxisf init_rotation_z(guess_pose_for_ndt.yaw, Eigen::Vector3f::UnitZ());

      Eigen::Translation3f init_translation(guess_pose_for_ndt.x, guess_pose_for_ndt.y, guess_pose_for_ndt.z);
    //  旋转向量->初始的变换矩阵 角度*模长
      Eigen::Matrix4f init_guess = // (translation(x,y,z) * rz * ry * rx)*上一帧的激光雷达lidar到世界坐标系map的变换
              //(init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix() * tf_btol;
              (init_translation * init_rotation_z * init_rotation_y * init_rotation_x).matrix();

      cur_pose_m_ = init_guess;
  }

  // ndt 匹配迭代
  cpu_ndt_.setInputSource(pc_source_); // 这里的pc_source是/base_link坐标系下的
  cpu_ndt_.align(cur_pose_m_);//开始配准
  fitness_score_ = cpu_ndt_.getFitnessScore();//配准得分
  // T_{map, base_link}最开始就进行了localizer到base_link的转换
  final_transformation_ = cpu_ndt_.getFinalTransformation();//配准成功得到的变换矩阵
  has_converged_ = cpu_ndt_.hasConverged();// 是否收敛
  final_iters_ = cpu_ndt_.getFinalNumIteration();//最终迭代次数

    t_base_link = final_transformation_;
    tf::Matrix3x3 mat_b;//相邻帧、相对于全局的旋转矩阵
    mat_b.setValue(static_cast<double>(t_base_link(0, 0)), static_cast<double>(t_base_link(0, 1)),
                   static_cast<double>(t_base_link(0, 2)), static_cast<double>(t_base_link(1, 0)),
                   static_cast<double>(t_base_link(1, 1)), static_cast<double>(t_base_link(1, 2)),
                   static_cast<double>(t_base_link(2, 0)), static_cast<double>(t_base_link(2, 1)),
                   static_cast<double>(t_base_link(2, 2)));
    // Update ndt_pose.   T_{map, base_link} base_link到map的转换
    ndt_pose.x = t_base_link(0, 3);
    ndt_pose.y = t_base_link(1, 3);
    ndt_pose.z = t_base_link(2, 3);
    mat_b.getRPY(ndt_pose.roll, ndt_pose.pitch, ndt_pose.yaw, 1);

    current_pose.x = ndt_pose.x;
    current_pose.y = ndt_pose.y;
    current_pose.z = ndt_pose.z;
    current_pose.roll = ndt_pose.roll;
    current_pose.pitch = ndt_pose.pitch;
    current_pose.yaw = ndt_pose.yaw;

    //  计算两帧时间
    scan_duration = current_scan_time - previous_scan_time;
    double secs = scan_duration.toSec();

    // Calculate the offset (curren_pos - previous_pos)计算两帧之间的位移、速度和偏航角变化
    diff_x = current_pose.x - previous_pose.x;
    diff_y = current_pose.y - previous_pose.y;
    diff_z = current_pose.z - previous_pose.z;
    diff_yaw = calcDiffForRadian(current_pose.yaw, previous_pose.yaw); // 前后偏航角差
    diff = sqrt(diff_x * diff_x + diff_y * diff_y + diff_z * diff_z);

    // Update position and posture. current_pos -> previous_pos 更新当前位姿
    previous_pose.x = current_pose.x;
    previous_pose.y = current_pose.y;
    previous_pose.z = current_pose.z;
    previous_pose.roll = current_pose.roll;
    previous_pose.pitch = current_pose.pitch;
    previous_pose.yaw = current_pose.yaw;

    previous_scan_time.sec = current_scan_time.sec;
    previous_scan_time.nsec = current_scan_time.nsec;




  Eigen::Quaternionf tmp_q(final_transformation_.block<3, 3>(0, 0));// ndt最终的最终的旋转位姿
  double roll, pitch, yaw;
  tf::Matrix3x3(tf::Quaternion(tmp_q.x(), tmp_q.y(), tmp_q.z(), tmp_q.w())).getRPY(roll, pitch, yaw);

  cur_pose_ndt_.pose.pose.position.x = final_transformation_(0, 3);
  cur_pose_ndt_.pose.pose.position.y = final_transformation_(1, 3);
  cur_pose_ndt_.pose.pose.position.z = final_transformation_(2, 3);
  cur_pose_ndt_.pose.pose.orientation.w = tmp_q.w();
  cur_pose_ndt_.pose.pose.orientation.x = tmp_q.x();
  cur_pose_ndt_.pose.pose.orientation.y = tmp_q.y();
  cur_pose_ndt_.pose.pose.orientation.z = tmp_q.z();
  cur_pose_ndt_.header.stamp = msg->header.stamp;
  cur_pose_m_ = final_transformation_;
  nav_msgs::Odometry updated_msg;
  updated_msg.header.stamp = msg->header.stamp;
  updated_msg.header.frame_id = "map";
  updated_msg.pose = cur_pose_ndt_.pose;
  pub_updated_pose_.publish(updated_msg);

  tf::Transform tf_m2b;// map-->base_link
  // T_{map, base_link}
  tf_m2b.setOrigin(tf::Vector3(final_transformation_(0, 3), final_transformation_(1, 3), final_transformation_(2, 3)));
  tf_m2b.setRotation(tf::Quaternion(tmp_q.x(), tmp_q.y(), tmp_q.z(), tmp_q.w()));
//  if (use_odom_ && odom_ptr_last_ != -1)//使用odometer
//  {
//    tf_o2b_.setOrigin(tf::Vector3(odom_queue_[odom_ptr_front_].pose.pose.position.x, odom_queue_[odom_ptr_front_].pose.pose.position.y, odom_queue_[odom_ptr_front_].pose.pose.position.z));
//    tf_o2b_.setRotation(tf::Quaternion(odom_queue_[odom_ptr_front_].pose.pose.orientation.x, odom_queue_[odom_ptr_front_].pose.pose.orientation.y, odom_queue_[odom_ptr_front_].pose.pose.orientation.z, odom_queue_[odom_ptr_front_].pose.pose.orientation.w));
//    // map-->odom = map-->base_link * base_link-->odom = map-->base_link * odom-->base_link.inverse()
//    tf_m2o_ = tf_m2b * tf_o2b_.inverse();
//    // tf::StampedTransform tmp;
//    // tf_listener_.waitForTransform("/odom", "/base_link", ros::Time(0), ros::Duration(0.05));
//    // tf_listener_.lookupTransform("/odom", "/base_link", ros::Time(0), tmp);
//    // tf_m2o_ = tf_m2b * tmp.inverse();
//    tf_broadcaster_.sendTransform(tf::StampedTransform(tf_m2o_, msg->header.stamp, "map", "/odom"));
//  }
//  else
  {
  	// 发布map-->base_link的
    tf_broadcaster_.sendTransform(tf::StampedTransform(tf_m2b, msg->header.stamp, "map", "/base_link"));
  }


    if (saveKeyframesAndFactor())
    {

    }
    else
    {
        std::cout << "too close" << std::endl;
    }

  auto end = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsed = end - start;
  std::cout << "-----------------------------------------------------------------" << std::endl;
  std::cout << "Sequence number: " << msg->header.seq << "; time elapsed: " << elapsed.count() << std::endl;
  std::cout << "Number of scan points: " << pc_source_->size() << " points." << std::endl;
  std::cout << "map: " << pc_target_->points.size() << " points." << std::endl;
  std::cout << "NDT has converged: " << has_converged_ << std::endl;
  std::cout << "Fitness score: " << fitness_score_ << std::endl;
  std::cout << "Number of iteration: " << final_iters_ << std::endl;
  std::cout << "(x,y,z,roll,pitch,yaw):" << std::endl;
  std::cout << "(" << final_transformation_(0, 3) << ", " << final_transformation_(1, 3) << ", " << final_transformation_(2, 3) << ", " << roll
            << ", " << pitch << ", " << yaw << ")" << std::endl;
  std::cout << "Transformation Matrix:" << std::endl;
  std::cout << final_transformation_ << std::endl;
  std::cout << "shift: " << std::sqrt(std::pow(final_transformation_(0, 3) - pre_pose_m_(0, 3), 2) + std::pow(final_transformation_(1, 3) - pre_pose_m_(1, 3), 2) + std::pow(final_transformation_(2, 3) - pre_pose_m_(2, 3), 2)) << std::endl;
  std::cout << "-----------------------------------------------------------------" << std::endl;

  // 4. 每n米更新一次全局地图，同时使用单独的线程,按照一定的频率进行地图的保存。// 这里是0.3米
//    pcl::PointCloud<PointT>::Ptr pc_map(new pcl::PointCloud<PointT>());
//    pcl::transformPointCloud(*pc_source_, *pc_map, cur_pose_m_);
//    *pc_target_ += *pc_map;
//    cpu_ndt_.setInputTarget(pc_target_);



  pre_pose_m_ = cur_pose_m_;
  pre_pose_ndt_ = cur_pose_ndt_;

  // 通过判断loop_closed_标志来检测是否有回环出现
  correctPoses();// 检测出回环后，更新位姿及 target map
}
// IMU消息回调函数
void NDTMap::imuCB(const sensor_msgs::ImuConstPtr &msg)
{
  double roll, pitch, yaw;
  tf::Quaternion orientation;
  tf::quaternionMsgToTF(msg->orientation, orientation);
  tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
  float acc_x = msg->linear_acceleration.x + 9.81 * sin(pitch);
  float acc_y = msg->linear_acceleration.y - 9.81 * cos(pitch) * sin(roll);
  float acc_z = msg->linear_acceleration.z - 9.81 * cos(pitch) * cos(roll);

  imu_ptr_last_ = (imu_ptr_last_ + 1) % imu_queue_len_;

  if ((imu_ptr_last_ + 1) % imu_queue_len_ == imu_ptr_front_)
  {
    imu_ptr_front_ = (imu_ptr_front_ + 1) % imu_queue_len_;
  }

  imu_time_[imu_ptr_last_] = msg->header.stamp.toSec();
  imu_roll_[imu_ptr_last_] = roll;
  imu_pitch_[imu_ptr_last_] = pitch;
  imu_yaw_[imu_ptr_last_] = yaw;
  imu_acc_x_[imu_ptr_last_] = acc_x;
  imu_acc_y_[imu_ptr_last_] = acc_y;
  imu_acc_z_[imu_ptr_last_] = acc_z;
  imu_angular_velo_x_[imu_ptr_last_] = msg->angular_velocity.x;
  imu_angular_velo_y_[imu_ptr_last_] = msg->angular_velocity.y;
  imu_angular_velo_z_[imu_ptr_last_] = msg->angular_velocity.z;

  // 转换到 imu 的全局坐标系中
  Eigen::Matrix3f rot = Eigen::Quaternionf(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z).toRotationMatrix();
  Eigen::Vector3f acc = rot * Eigen::Vector3f(acc_x, acc_y, acc_z);
  // TODO: lego_loam 里没有对角速度转换，是否需要尚且存疑
  // Eigen::Vector3f angular_velo = rot * Eigen::Vector3f(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
  Eigen::Vector3f angular_velo(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);

  int imu_ptr_back = (imu_ptr_last_ - 1 + imu_queue_len_) % imu_queue_len_;
  double time_diff = imu_time_[imu_ptr_last_] - imu_time_[imu_ptr_back];
  if (time_diff < 1.)
  {
    imu_shift_x_[imu_ptr_last_] = imu_shift_x_[imu_ptr_back] + imu_velo_x_[imu_ptr_back] * time_diff + acc(0) * time_diff * time_diff * 0.5;
    imu_shift_y_[imu_ptr_last_] = imu_shift_y_[imu_ptr_back] + imu_velo_y_[imu_ptr_back] * time_diff + acc(1) * time_diff * time_diff * 0.5;
    imu_shift_z_[imu_ptr_last_] = imu_shift_z_[imu_ptr_back] + imu_velo_z_[imu_ptr_back] * time_diff + acc(2) * time_diff * time_diff * 0.5;

    imu_velo_x_[imu_ptr_last_] = imu_velo_x_[imu_ptr_back] + acc(0) * time_diff;
    imu_velo_y_[imu_ptr_last_] = imu_velo_y_[imu_ptr_back] + acc(1) * time_diff;
    imu_velo_z_[imu_ptr_last_] = imu_velo_z_[imu_ptr_back] + acc(2) * time_diff;

    imu_angular_rot_x_[imu_ptr_last_] = imu_angular_rot_x_[imu_ptr_back] + angular_velo(0) * time_diff;
    imu_angular_rot_y_[imu_ptr_last_] = imu_angular_rot_y_[imu_ptr_back] + angular_velo(1) * time_diff;
    imu_angular_rot_z_[imu_ptr_last_] = imu_angular_rot_z_[imu_ptr_back] + angular_velo(2) * time_diff;
  }
}

void NDTMap::loopClosureThread()
{
  if (!loop_closure_enabled_)
  {
    return;
  }
  ros::Duration duration(1);
  while (ros::ok())
  {
    performLoopClosure();
    duration.sleep();
  }
}

/**
 * @brief 回环检测及位姿图更新
 * ICP 匹配添加回环约束
 */
void NDTMap::performLoopClosure()
{
  if (cloud_keyposes_3d_->points.empty())
  {
    return;
  }

  if (!detectLoopClosure())
  {
    return;
  }
  else
  {
    ROS_WARN("detected loop closure");
  }

  auto start = std::chrono::system_clock::now();

  pcl::IterativeClosestPoint<PointT, PointT> icp;
  icp.setMaxCorrespondenceDistance(100);
  icp.setMaximumIterations(100);
  icp.setTransformationEpsilon(1e-6);
  icp.setEuclideanFitnessEpsilon(1e-6);
  icp.setRANSACIterations(0);

  icp.setInputSource(latest_keyframe_);
  icp.setInputTarget(near_history_keyframes_);
  pcl::PointCloud<PointT>::Ptr unused_result(new pcl::PointCloud<PointT>());
  Eigen::Matrix4f initial_guess(Eigen::Matrix4f::Identity());
  initial_guess.block<3, 3>(0, 0) = (Eigen::AngleAxisf(cloud_keyposes_6d_->points[latest_history_frame_id_].yaw, Eigen::Vector3f::UnitZ()) *
                                     Eigen::AngleAxisf(cloud_keyposes_6d_->points[latest_history_frame_id_].pitch, Eigen::Vector3f::UnitY()) *
                                     Eigen::AngleAxisf(cloud_keyposes_6d_->points[latest_history_frame_id_].roll, Eigen::Vector3f::UnitX()))
                                        .toRotationMatrix();
  initial_guess(0, 3) = cloud_keyposes_6d_->points[latest_history_frame_id_].x;
  initial_guess(1, 3) = cloud_keyposes_6d_->points[latest_history_frame_id_].y;
  initial_guess(2, 3) = cloud_keyposes_6d_->points[latest_history_frame_id_].z;
  icp.align(*unused_result);

  bool has_converged = icp.hasConverged();
  float fitness_score = icp.getFitnessScore();
  Eigen::Matrix4f correction_frame = icp.getFinalTransformation();
  Eigen::Quaternionf tmp_q(correction_frame.block<3, 3>(0, 0));
  double roll, pitch, yaw;
  tf::Matrix3x3(tf::Quaternion(tmp_q.x(), tmp_q.y(), tmp_q.z(), tmp_q.w())).getRPY(roll, pitch, yaw);

  auto end = std::chrono::system_clock::now();
  std::chrono::duration<double> elapsed = end - start;

  if (has_converged == false || fitness_score > history_fitness_score_)
  {
    ROS_WARN("loop cannot closed");
    return;
  }
  else
  {
    ROS_WARN("loop closed");
    if (pub_icp_keyframes_.getNumSubscribers() > 0)
    {
      pcl::PointCloud<PointT>::Ptr closed_cloud(new pcl::PointCloud<PointT>());
      pcl::transformPointCloud(*latest_keyframe_, *closed_cloud, correction_frame);
      sensor_msgs::PointCloud2 msg;
      pcl::toROSMsg(*closed_cloud, msg);
      msg.header.stamp.fromSec(cloud_keyposes_6d_->points[latest_history_frame_id_].time);
      msg.header.frame_id = "map";
      pub_icp_keyframes_.publish(msg);
    }
  }

  Eigen::Matrix4f t_wrong = initial_guess;
  Eigen::Matrix4f t_correct = correction_frame * t_wrong;
  // Eigen::Matrix4f t_correct = correction_frame;
  Eigen::Quaternionf r_correct(t_correct.block<3, 3>(0, 0));
  gtsam::Pose3 pose_from = Pose3(Rot3::Quaternion(r_correct.w(), r_correct.x(), r_correct.y(), r_correct.z()),
                                 Point3(t_correct(0, 3), t_correct(1, 3), t_correct(2, 3)));
  gtsam::Pose3 pose_to = Pose3(Rot3::RzRyRx(cloud_keyposes_6d_->points[closest_history_frame_id_].roll, cloud_keyposes_6d_->points[closest_history_frame_id_].pitch, cloud_keyposes_6d_->points[closest_history_frame_id_].yaw),
                               Point3(cloud_keyposes_6d_->points[closest_history_frame_id_].x, cloud_keyposes_6d_->points[closest_history_frame_id_].y, cloud_keyposes_6d_->points[closest_history_frame_id_].z));
  float noise_score = fitness_score;
  gtsam::Vector vector6(6);
  vector6 << noise_score, noise_score, noise_score, noise_score, noise_score, noise_score;
  constraint_noise_ = noiseModel::Diagonal::Variances(vector6);

  std::lock_guard<std::mutex> lock(mtx_);
  gtSAMgraph_.add(BetweenFactor<Pose3>(latest_history_frame_id_, closest_history_frame_id_, pose_from.between(pose_to), constraint_noise_));
  isam->update(gtSAMgraph_);
  isam->update();
  gtSAMgraph_.resize(0);

  std::cout << "-----------------------------------------------------------------" << std::endl;
  std::cout << "Time elapsed: " << elapsed.count() << std::endl;
  std::cout << "Number of source points: " << latest_keyframe_->size() << " points." << std::endl;
  std::cout << "target: " << near_history_keyframes_->points.size() << " points." << std::endl;
  std::cout << "ICP has converged: " << has_converged << std::endl;
  std::cout << "Fitness score: " << fitness_score << std::endl;
  std::cout << "initial (x,y,z,roll,pitch,yaw):" << std::endl;
  std::cout << "(" << initial_guess(0, 3) << ", " << initial_guess(1, 3) << ", " << initial_guess(2, 3) << ", "
            << cloud_keyposes_6d_->points[latest_history_frame_id_].roll << ", " << cloud_keyposes_6d_->points[latest_history_frame_id_].pitch << ", " << cloud_keyposes_6d_->points[latest_history_frame_id_].yaw << ")" << std::endl;
  std::cout << "final (x,y,z,roll,pitch,yaw):" << std::endl;
  std::cout << "(" << t_correct(0, 3) << ", " << t_correct(1, 3) << ", " << t_correct(2, 3) << ", "
            << roll << ", " << pitch << ", " << yaw << ")" << std::endl;
  std::cout << "Transformation Matrix:" << std::endl;
  std::cout << t_correct << std::endl;
  std::cout << "shift: " << std::sqrt(std::pow(correction_frame(0, 3) - initial_guess(0, 3), 2) + std::pow(correction_frame(1, 3) - initial_guess(1, 3), 2) + std::pow(correction_frame(2, 3) - initial_guess(2, 3), 2)) << std::endl;
  std::cout << "-----------------------------------------------------------------" << std::endl;

  loop_closed_ = true;
}

/**
 * @brief 基于里程计的回环检测，直接提取当前位姿附近(radiusSearch)的 keyframe 作为 icp 的 target
 * 
 */
bool NDTMap::detectLoopClosure()
{
  latest_keyframe_->clear();
  near_history_keyframes_->clear();

  std::lock_guard<std::mutex> lock(mtx_);

  PointT cur_pose;
  cur_pose.x = cur_pose_m_(0, 3);
  cur_pose.y = cur_pose_m_(1, 3);
  cur_pose.z = cur_pose_m_(2, 3);
  kdtree_poses_->setInputCloud(cloud_keyposes_3d_);
  kdtree_poses_->radiusSearch(cur_pose, history_search_radius_, search_idx_, search_dist_);

  latest_history_frame_id_ = cloud_keyframes_.size() - 1;
  closest_history_frame_id_ = -1;
  for (int i = 0; i < search_idx_.size(); ++i)
  {
    if (cur_pose_ndt_.header.stamp.toSec() - cloud_keyposes_6d_->points[search_idx_[i]].time > 30.)
    {
      closest_history_frame_id_ = search_idx_[i];
      break;
    }
  }
  // 时间太短不做回环
  if (closest_history_frame_id_ == -1)
  {
    return false;
  }

  pcl::copyPointCloud(*transformPointCloud(cloud_keyframes_[latest_history_frame_id_], cloud_keyposes_6d_->points[latest_history_frame_id_]), *latest_keyframe_);

  pcl::PointCloud<PointT>::Ptr tmp_cloud(new pcl::PointCloud<PointT>());
  for (int i = -history_search_num_, j; i <= history_search_num_; ++i)
  {
    j = closest_history_frame_id_ + i;
    if (j < 0 || j >= latest_history_frame_id_)
    {
      continue;
    }
    *tmp_cloud += *transformPointCloud(cloud_keyframes_[j], cloud_keyposes_6d_->points[j]);
  }

  ds_history_keyframes_.setInputCloud(tmp_cloud);
  ds_history_keyframes_.filter(*near_history_keyframes_);

  if (pub_history_keyframes_.getNumSubscribers() > 0)
  {
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*near_history_keyframes_, msg);
    msg.header.stamp = cur_pose_ndt_.header.stamp;
    msg.header.frame_id = "map";
    pub_history_keyframes_.publish(msg);
  }

  return true;
}

/**
 * @brief 检测出回环后，更新位姿及 target map
 * 
 */
void NDTMap::correctPoses()
{
  if (loop_closed_)
  {
    recent_keyframes_.clear();
    ROS_WARN("correctPoses");
    int num_poses = isam_current_estimate_.size();
    for (int i = 0; i < num_poses; ++i)
    {
      // if (std::abs(cloud_keyposes_3d_->points[i].z - isam_current_estimate_.at<Pose3>(i).translation().z()) > 1)
      // {
      //   ROS_WARN("aaaa");
      // }
      // else
      // {
      //   ROS_WARN("bbbb");
      // }
      cloud_keyposes_6d_->points[i].x = cloud_keyposes_3d_->points[i].x = isam_current_estimate_.at<Pose3>(i).translation().x();
      cloud_keyposes_6d_->points[i].y = cloud_keyposes_3d_->points[i].y = isam_current_estimate_.at<Pose3>(i).translation().y();
      cloud_keyposes_6d_->points[i].z = cloud_keyposes_3d_->points[i].z = isam_current_estimate_.at<Pose3>(i).translation().z();
      cloud_keyposes_6d_->points[i].roll = isam_current_estimate_.at<Pose3>(i).rotation().roll();
      cloud_keyposes_6d_->points[i].pitch = isam_current_estimate_.at<Pose3>(i).rotation().pitch();
      cloud_keyposes_6d_->points[i].yaw = isam_current_estimate_.at<Pose3>(i).rotation().yaw();
    }

    loop_closed_ = false;
  }
}
// "/save_map"保存地图的service
bool NDTMap::saveMapCB(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  std::stringstream ss;
  ss << int(ros::Time::now().toSec());
  std::string stamp = ss.str();// 保存当前ROS时间的string字符串

  int num_points = 0, num_points1 = 0;
  pcl::PointCloud<PointT>::Ptr map(new pcl::PointCloud<PointT>());// 有地面的map地图
  pcl::PointCloud<PointT>::Ptr map_no_ground(new pcl::PointCloud<PointT>());// 没有地面的地图map
  for (int i = 0; i < cloud_keyframes_.size(); ++i)
  {
    pcl::PointCloud<PointT>::Ptr tmp(new pcl::PointCloud<PointT>());
    // cloud_keyframes_关键帧点云，及其位姿xyz,rpy,time
    // 根据位姿，将点云进行转换，转换后的点云保存到tmp中
    tmp = transformPointCloud(cloud_keyframes_[i], cloud_keyposes_6d_->points[i]);
    *map += *tmp;
    num_points += tmp->points.size();// 统计map点云的大小

    pcl::PointCloud<PointT>::Ptr tmp1(new pcl::PointCloud<PointT>());
    // utils::RayGroundFilter ground_filter;去除clip_height高度以上的点云
    ground_filter.convert(cloud_keyframes_[i], tmp1);
    tmp1 = transformPointCloud(tmp1, cloud_keyposes_6d_->points[i]);
    *map_no_ground += *tmp1;
    num_points1 += tmp1->points.size();// 统计map_no_ground点云的大小
  }

  map->width = map->points.size(); // map的点云数量
  map->height = 1;
  map->is_dense = false;

  map_no_ground->width = map_no_ground->points.size();// map_no_ground的大小
  map_no_ground->height = 1;
  map_no_ground->is_dense = false;

  pcl::PointCloud<PointT>::Ptr poses(new pcl::PointCloud<PointT>());
  pcl::copyPointCloud(*cloud_keyposes_3d_, *poses);// 位姿，xyz
  poses->width = poses->points.size();
  poses->height = 1;
  poses->is_dense = false;

  pcl::io::savePCDFile(save_dir_ + "pose3d_" + stamp + ".pcd", *poses);
  pcl::io::savePCDFile(save_dir_ + "frames_" + stamp + ".pcd", *map);
  pcl::io::savePCDFile(save_dir_ + "frames_no_ground_" + stamp + ".pcd", *map_no_ground);

  ROS_WARN("Save map. pose size: %d, cloud size: %d, cloud no ground size: %d", poses->points.size(), num_points, num_points1);
}