/**
 * @file voxelslam.hpp
 * @brief VoxelSLAM header file containing main class definitions and includes
 * 
 * Defines the core data structures and interfaces for the VoxelSLAM
 * LiDAR-inertial SLAM system.
 */

#pragma once

#include "tools.hpp"
#include "ekf_imu.hpp"
#include "voxel_map.hpp"
#include "feature_point.hpp"
#include "loop_refine.hpp"
#include <mutex>
#include <Eigen/Eigenvalues>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>
#include <malloc.h>
#include <geometry_msgs/PoseArray.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <malloc.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <Eigen/Sparse>
#include <Eigen/SparseQR>
#include "BTC.h"
#include <fstream>
#include <iomanip>
#include <sys/stat.h>
#include <sys/types.h>

using namespace std;

ros::Publisher pub_scan, pub_cmap, pub_init, pub_pmap;
ros::Publisher pub_test, pub_prev_path, pub_curr_path;
ros::Subscriber sub_imu, sub_pcl;

// Global variables for original resolution saving
extern bool is_save_original_resolution;
extern bool use_full_resolution_odometry;

template <typename T>
void pub_pl_func(T &pl, ros::Publisher &pub)
{
  pl.height = 1; pl.width = pl.size();
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(pl, output);
  output.header.frame_id = "camera_init";
  output.header.stamp = ros::Time::now();
  pub.publish(output);
}

mutex mBuf;
Features feat;
deque<sensor_msgs::Imu::Ptr> imu_buf;
deque<pcl::PointCloud<PointType>::Ptr> pcl_buf;
deque<double> time_buf;

double imu_last_time = -1;
int point_notime = 0;
double last_pcl_time = -1;

// Timestamp logging variables
int frame_counter = 0;
bool enable_timestamp_logging = true;
string timestamp_log_dir = "/tmp/voxelslam_timestamps";

// Raw LiDAR data logging variables (for pcl_handler)
int raw_frame_counter = 0;
bool enable_raw_timestamp_logging = true;
string raw_timestamp_log_dir = "/media/yjcho/MT_009/data/voxel_slam/raw_timestamp";

void imu_handler(const sensor_msgs::Imu::ConstPtr &msg_in)
{
  static int flag = 1;
  if(flag)
  {
    flag = 0;
    printf("Time0: %lf\n", msg_in->header.stamp.toSec());
  }

  sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));

  // For Hilti 2022 exp03
  // double t0 = 1646320760 + 255.5;
  // double t1 = 1646320760 + 256.2;
  // double tc = msg->header.stamp.toSec();
  // if(tc > t0 && tc < t1)
  //   msg->linear_acceleration.z = -9.7;

  mBuf.lock();
  imu_last_time = msg->header.stamp.toSec();
  imu_buf.push_back(msg);
  mBuf.unlock();
}

// Function to create directory if it doesn't exist
void create_directory_if_not_exists(const string& dir_path) {
    struct stat st = {0};
    if (stat(dir_path.c_str(), &st) == -1) {
        mkdir(dir_path.c_str(), 0755);
    }
}

// Function to save LiDAR point timestamps
void save_lidar_timestamps(const pcl::PointCloud<PointType>::Ptr &pl_ptr, int frame_id) {
    if (!enable_timestamp_logging) return;
    
    create_directory_if_not_exists(timestamp_log_dir);
    
    string filename = timestamp_log_dir + "/lidar_timestamps_frame_" + to_string(frame_id) + ".txt";
    ofstream file(filename);
    
    if (file.is_open()) {
        file << fixed << setprecision(9);
        file << "# LiDAR Point Timestamps for Frame " << frame_id << endl;
        file << "# Format: point_index timestamp_seconds" << endl;
        file << "# Total points: " << pl_ptr->size() << endl;
        
        for (size_t i = 0; i < pl_ptr->size(); i++) {
            file << i << " " << pl_ptr->points[i].curvature << endl;
        }
        
        file.close();
        printf("LiDAR timestamps saved to: %s (%zu points)\n", filename.c_str(), pl_ptr->size());
    } else {
        printf("Failed to open file for writing: %s\n", filename.c_str());
    }
}

// Function to save IMU timestamps
void save_imu_timestamps(const deque<sensor_msgs::Imu::Ptr> &imus, int frame_id) {
    if (!enable_timestamp_logging) return;
    
    create_directory_if_not_exists(timestamp_log_dir);
    
    string filename = timestamp_log_dir + "/imu_timestamps_frame_" + to_string(frame_id) + ".txt";
    ofstream file(filename);
    
    if (file.is_open()) {
        file << fixed << setprecision(9);
        file << "# IMU Timestamps for Frame " << frame_id << endl;
        file << "# Format: imu_index timestamp_seconds angular_vel_x angular_vel_y angular_vel_z linear_acc_x linear_acc_y linear_acc_z" << endl;
        file << "# Total IMU samples: " << imus.size() << endl;
        
        for (size_t i = 0; i < imus.size(); i++) {
            file << i << " " 
                 << imus[i]->header.stamp.toSec() << " "
                 << imus[i]->angular_velocity.x << " "
                 << imus[i]->angular_velocity.y << " "
                 << imus[i]->angular_velocity.z << " "
                 << imus[i]->linear_acceleration.x << " "
                 << imus[i]->linear_acceleration.y << " "
                 << imus[i]->linear_acceleration.z << endl;
        }
        
        file.close();
        printf("IMU timestamps saved to: %s (%zu samples)\n", filename.c_str(), imus.size());
    } else {
        printf("Failed to open file for writing: %s\n", filename.c_str());
    }
}

// Function to save motion compensation details
void save_motion_compensation_details(int frame_id, const vector<pair<double, double>> &dt_head_pairs) {
    if (!enable_timestamp_logging) return;
    
    create_directory_if_not_exists(timestamp_log_dir);
    
    string filename = timestamp_log_dir + "/motion_compensation_frame_" + to_string(frame_id) + ".txt";
    ofstream file(filename);
    
    if (file.is_open()) {
        file << fixed << setprecision(9);
        file << "# Motion Compensation Details for Frame " << frame_id << endl;
        file << "# Format: point_index dt_seconds head_t_seconds" << endl;
        file << "# dt = point_timestamp - head_timestamp" << endl;
        file << "# Total compensated points: " << dt_head_pairs.size() << endl;
        
        for (size_t i = 0; i < dt_head_pairs.size(); i++) {
            file << i << " " << dt_head_pairs[i].first << " " << dt_head_pairs[i].second << endl;
        }
        
        file.close();
        printf("Motion compensation details saved to: %s (%zu points)\n", filename.c_str(), dt_head_pairs.size());
    } else {
        printf("Failed to open file for writing: %s\n", filename.c_str());
    }
}

// Function to save raw LiDAR timestamps (from pcl_handler)
void save_raw_lidar_timestamps(const pcl::PointCloud<PointType>::Ptr &pl_ptr, int frame_id) {
    if (!enable_raw_timestamp_logging) return;
    
    create_directory_if_not_exists(raw_timestamp_log_dir);
    
    string filename = raw_timestamp_log_dir + "/raw_lidar_timestamps_frame_" + to_string(frame_id) + ".txt";
    ofstream file(filename);
    
    if (file.is_open()) {
        file << fixed << setprecision(9);
        file << "# Raw LiDAR Point Timestamps for Frame " << frame_id << endl;
        file << "# Format: point_index timestamp_seconds" << endl;
        file << "# Total points: " << pl_ptr->size() << endl;
        file << "# Note: This is the raw data from pcl_handler before any processing" << endl;
        
        for (size_t i = 0; i < pl_ptr->size(); i++) {
            file << i << " " << pl_ptr->points[i].curvature << endl;
        }
        
        file.close();
        printf("Raw LiDAR timestamps saved to: %s (%zu points)\n", filename.c_str(), pl_ptr->size());
    } else {
        printf("Failed to open file for writing: %s\n", filename.c_str());
    }
}

template<class T>
void pcl_handler(T &msg)
{
  pcl::PointCloud<PointType>::Ptr pl_ptr(new pcl::PointCloud<PointType>());
  double t0 = feat.process(msg, *pl_ptr);

  if(pl_ptr->empty())
  {
    PointType ap; 
    ap.x = 0; ap.y = 0; ap.z = 0; 
    ap.intensity = 0; ap.curvature = 0;
    pl_ptr->push_back(ap);
    ap.curvature = 0.09;
    pl_ptr->push_back(ap);
  }

  // Save raw LiDAR timestamps before any processing
  raw_frame_counter++;
  save_raw_lidar_timestamps(pl_ptr, raw_frame_counter);

  sort(pl_ptr->begin(), pl_ptr->end(), [](PointType &x, PointType &y)
  {
    return x.curvature < y.curvature;
  });
  // Store original data before curvature filtering
  if(is_save_original_resolution) {
    pcl::PointCloud<PointType>::Ptr pl_orig_before_filter(new pcl::PointCloud<PointType>(*pl_ptr));
    feat.original_pointclouds.push_back(pl_orig_before_filter);
  }
  while(pl_ptr->back().curvature > 0.11)
    pl_ptr->points.pop_back();

  mBuf.lock();
  time_buf.push_back(t0);
  pcl_buf.push_back(pl_ptr);
  mBuf.unlock();
}

bool sync_packages(pcl::PointCloud<PointType>::Ptr &pl_ptr, deque<sensor_msgs::Imu::Ptr> &imus, IMUEKF &p_imu)
{
  static bool pl_ready = false;
  static int sync_call_count = 0;
  sync_call_count++;

  //printf("SYNC_ENTRY[%d]: pl_ready=%s, pcl_buf.size()=%zu, imu_buf.size()=%zu\n", 
  //       sync_call_count, pl_ready ? "true" : "false", pcl_buf.size(), imu_buf.size());

  if(!pl_ready)
  {
    if(pcl_buf.empty()) {
      //printf("SYNC_EXIT[%d]: PCL buffer empty, returning false\n", sync_call_count);
      return false;
    }

    mBuf.lock();
    pl_ptr = pcl_buf.front();
    double frame_stamp = time_buf.front();    //frame 중간 시간
    //p_imu.pcl_beg_time = time_buf.front();
    pcl_buf.pop_front(); time_buf.pop_front();
    mBuf.unlock();

    //printf("SYNC_PCL[%d]: Retrieved pointcloud with %zu points, frame_stamp=%.6f\n", 
    //       sync_call_count, pl_ptr->size(), frame_stamp);

    p_imu.pcl_beg_time = frame_stamp + pl_ptr->front().curvature;  // 첫 포인트 실제 시간
    p_imu.pcl_end_time = frame_stamp + pl_ptr->back().curvature;   // 마지막 포인트 실제 시간
    //p_imu.pcl_end_time = p_imu.pcl_beg_time + pl_ptr->back().curvature;
    
    // DEBUG: 시간 동기화 확인
    //printf("SYNC_TIME[%d]: frame_stamp=%.6f, front_curvature=%.6f, back_curvature=%.6f\n", 
    //       sync_call_count, frame_stamp, pl_ptr->front().curvature, pl_ptr->back().curvature);
    //printf("SYNC_TIME[%d]: pcl_beg_time=%.6f, pcl_end_time=%.6f, scan_duration=%.6f\n", 
    //       sync_call_count, p_imu.pcl_beg_time, p_imu.pcl_end_time, p_imu.pcl_end_time - p_imu.pcl_beg_time);

    if(point_notime)
    {
      //printf("SYNC_NOTIME[%d]: point_notime mode active\n", sync_call_count);
      if(last_pcl_time < 0)
      {
        last_pcl_time = p_imu.pcl_beg_time;
        //printf("SYNC_EXIT[%d]: First frame in point_notime mode, returning false\n", sync_call_count);
        return false;
      }

      p_imu.pcl_end_time = p_imu.pcl_beg_time;
      p_imu.pcl_beg_time = last_pcl_time;
      last_pcl_time = p_imu.pcl_end_time;
    }

    pl_ready = true;
    //printf("SYNC_READY[%d]: Point cloud ready, pl_ready=true\n", sync_call_count);
  }

  //printf("SYNC_IMU_CHECK[%d]: imu_last_time=%.6f, pcl_end_time=%.6f, condition=%s\n", 
  //       sync_call_count, imu_last_time, p_imu.pcl_end_time, 
  //       (imu_last_time > p_imu.pcl_end_time) ? "PASS" : "FAIL");

  if(!pl_ready || imu_last_time <= p_imu.pcl_end_time) {
    //printf("SYNC_EXIT[%d]: IMU condition failed, returning false\n", sync_call_count);
    return false;
  }

  mBuf.lock();
  
  // 1. 안전한 시간 범위 계산 (여유분 포함)
  double margin = 0.06;  // 60ms 여유분 (라이다 시간 범위보다 큰 값)
  double required_start_time = p_imu.pcl_beg_time - margin;
  double required_end_time = p_imu.pcl_end_time + margin;
  double current_time = ros::Time::now().toSec();  // 절대 시간 기준
  
  //printf("SYNC_IMU_RANGE[%d]: pcl_beg_time=%.6f, pcl_end_time=%.6f, scan_duration=%.6f\n", 
  //       sync_call_count, p_imu.pcl_beg_time, p_imu.pcl_end_time, p_imu.pcl_end_time - p_imu.pcl_beg_time);
  //printf("SYNC_IMU_TARGET[%d]: required_range=[%.6f, %.6f], margin=%.6f\n", 
  //       sync_call_count, required_start_time, required_end_time, margin);
  
  // 2. 필요한 IMU 데이터만 수집 (제거하지 않고 복사만)
  int imu_collected = 0;
  double first_imu_time = -1, last_imu_time = -1;
  
  for(auto imu_ptr : imu_buf)
  {
    double imu_time = imu_ptr->header.stamp.toSec();
    
    // 필요한 시간 범위 내의 IMU만 수집
    if(imu_time >= required_start_time && imu_time <= required_end_time)
    {
      if(first_imu_time < 0) first_imu_time = imu_time;
      last_imu_time = imu_time;
      imus.push_back(imu_ptr);
      imu_collected++;
    }
    
    // 필요 범위를 넘으면 종료
    if(imu_time > required_end_time) break;
  }
  
  // 3. 안전한 제거 (두 조건 모두 만족하는 데이터만 제거)
  double cleanup_threshold = std::min(p_imu.pcl_beg_time - 0.15,     // 상대적 기준 (150ms)
                                     current_time - 0.5);            // 절대적 기준 (500ms)
  int removed_count = 0;
  while(!imu_buf.empty() && imu_buf.front()->header.stamp.toSec() < cleanup_threshold)
  {
    imu_buf.pop_front();
    removed_count++;
  }
  
  mBuf.unlock();
  
  //printf("SYNC_IMU_RESULT[%d]: Collected %d IMU messages, time_range=[%.6f, %.6f], coverage=%.6f\n", 
  //       sync_call_count, imu_collected, first_imu_time, last_imu_time, last_imu_time - first_imu_time);
  //printf("SYNC_IMU_COVERAGE[%d]: Historical coverage=%.6f, Future coverage=%.6f\n", 
  //       sync_call_count, p_imu.pcl_beg_time - first_imu_time, last_imu_time - p_imu.pcl_end_time);
  //printf("SYNC_IMU_CLEANUP[%d]: cleanup_threshold=%.6f, removed=%d, remaining_buf=%zu\n", 
  //       sync_call_count, cleanup_threshold, removed_count, imu_buf.size());

  //printf("SYNC_IMU_RESULT[%d]: Collected %d IMU messages, remaining_imu_buf=%zu\n", 
  //       sync_call_count, imu_collected, imu_buf.size());

  if(imu_buf.empty())
  {
    printf("imu buf empty\n"); exit(0);
  }

  pl_ready = false;

  bool success = (imus.size() > 4);
  //printf("SYNC_FINAL[%d]: imus.size()=%zu, threshold=4, SUCCESS=%s\n", 
  //       sync_call_count, imus.size(), success ? "YES" : "NO");

  if(success) {
    //printf("SYNC_SUCCESS[%d]: Returning true - synchronization successful!\n", sync_call_count);
    return true;
  } else {
    //printf("SYNC_FAIL[%d]: Returning false - insufficient IMU data\n", sync_call_count);
    return false;
  }
}

double dept_err, beam_err;
void calcBodyVar(Eigen::Vector3d &pb, const float range_inc, const float degree_inc, Eigen::Matrix3d &var) 
{
  if (pb[2] == 0)
    pb[2] = 0.0001;
  float range = sqrt(pb[0] * pb[0] + pb[1] * pb[1] + pb[2] * pb[2]);
  float range_var = range_inc * range_inc;
  Eigen::Matrix2d direction_var;
  direction_var << pow(sin(DEG2RAD(degree_inc)), 2), 0, 0, pow(sin(DEG2RAD(degree_inc)), 2);
  Eigen::Vector3d direction(pb);
  direction.normalize();
  Eigen::Matrix3d direction_hat;
  direction_hat << 0, -direction(2), direction(1), direction(2), 0, -direction(0), -direction(1), direction(0), 0;
  Eigen::Vector3d base_vector1(1, 1, -(direction(0) + direction(1)) / direction(2));
  base_vector1.normalize();
  Eigen::Vector3d base_vector2 = base_vector1.cross(direction);
  base_vector2.normalize();
  Eigen::Matrix<double, 3, 2> N;
  N << base_vector1(0), base_vector2(0), base_vector1(1), base_vector2(1), base_vector1(2), base_vector2(2);
  Eigen::Matrix<double, 3, 2> A = range * direction_hat * N;
  var = direction * range_var * direction.transpose() + A * direction_var * A.transpose();
};

// Compute the variance of the each point
void var_init(IMUST &ext, pcl::PointCloud<PointType> &pl_cur, PVecPtr pptr, double dept_err, double beam_err)
{
  int plsize = pl_cur.size();
  pptr->clear();
  pptr->resize(plsize);
  for(int i=0; i<plsize; i++)
  {
    PointType &ap = pl_cur[i];
    pointVar &pv = pptr->at(i);
    pv.pnt << ap.x, ap.y, ap.z;
    pv.intensity = ap.intensity; // Copy intensity information
    calcBodyVar(pv.pnt, dept_err, beam_err, pv.var);
    pv.pnt = ext.R * pv.pnt + ext.p;
    pv.var = ext.R * pv.var * ext.R.transpose();
  }
}

// Original function - preserves algorithm integrity
void pvec_update(PVecPtr pptr, IMUST &x_curr, PLV(3) &pwld)
{
  Eigen::Matrix3d rot_var = x_curr.cov.block<3, 3>(0, 0);
  Eigen::Matrix3d tsl_var = x_curr.cov.block<3, 3>(3, 3);

  for(pointVar &pv: *pptr)
  {
    Eigen::Matrix3d phat = hat(pv.pnt);
    pv.var = x_curr.R * pv.var * x_curr.R.transpose() + phat * rot_var * phat.transpose() + tsl_var;
    pwld.push_back(x_curr.R * pv.pnt + x_curr.p);
  }
}

// Overloaded function - for visualization with intensity
void pvec_update(PVecPtr pptr, IMUST &x_curr, PLV(3) &pwld, vector<float> &intensity_buf)
{
  Eigen::Matrix3d rot_var = x_curr.cov.block<3, 3>(0, 0);
  Eigen::Matrix3d tsl_var = x_curr.cov.block<3, 3>(3, 3);

  for(pointVar &pv: *pptr)
  {
    Eigen::Matrix3d phat = hat(pv.pnt);
    pv.var = x_curr.R * pv.var * x_curr.R.transpose() + phat * rot_var * phat.transpose() + tsl_var;
    pwld.push_back(x_curr.R * pv.pnt + x_curr.p);
    intensity_buf.push_back(pv.intensity); // Store intensity information
  }
}

// Read the alidarstate.txt
void read_lidarstate(string filename, vector<ScanPose*> &bl_tem)
{
  ifstream file(filename);
  if(!file.is_open())
  {
    printf("Error: %s not found\n", filename.c_str());
    exit(0);
  }

  string lineStr, str;
  vector<double> nums;
  while(getline(file, lineStr))
  {
    nums.clear();
    stringstream ss(lineStr);
    while(getline(ss, str, ' '))
      nums.push_back(stod(str));
    
    IMUST xx;
    xx.t = nums[0];
    xx.p << nums[1], nums[2], nums[3];
    xx.R = Eigen::Quaterniond(nums[7], nums[4], nums[5], nums[6]).matrix();

    if(nums.size() >= 20)
    {
      xx.v << nums[8], nums[9], nums[10];
      xx.bg << nums[11], nums[12], nums[13];
      xx.ba << nums[14], nums[15], nums[16];
      xx.g << nums[17], nums[18], nums[19];
    }

    ScanPose* blp = new ScanPose(xx, nullptr);
    bl_tem.push_back(blp);

    if(nums.size() >= 26)
      for(int i=0; i<6; i++) 
        blp->v6[i] = nums[i + 20];
  }
}

double get_memory()
{
  ifstream infile("/proc/self/status");
  double mem = -1;
  string lineStr, str;
  while(getline(infile, lineStr))
  {
    stringstream ss(lineStr);
    bool is_find = false;
    while(ss >> str)
    {
      if(str == "VmRSS:")
      {
        is_find = true; continue;
      }

      if(is_find) mem = stod(str);
      break;
    }
    if(is_find) break;
  }
  return mem / (1048576);
}

void icp_check(pcl::PointCloud<PointType> &pl_src, pcl::PointCloud<PointType> &pl_tar, ros::Publisher &pub_src, ros::Publisher &pub_tar, pair<Eigen::Vector3d, Eigen::Matrix3d> &loop_transform, IMUST &xx)
{
  pcl::PointCloud<PointType> pl1, pl2;
  for(PointType ap: pl_src.points)
  {
    Eigen::Vector3d v(ap.x, ap.y, ap.z);
    v = loop_transform.second * v + loop_transform.first;
    v = xx.R * v + xx.p;
    ap.x = v[0]; ap.y = v[1]; ap.z = v[2];
    pl1.push_back(ap);
  }
  for(PointType ap: pl_tar.points)
  {
    Eigen::Vector3d v(ap.x, ap.y, ap.z);
    v = xx.R * v + xx.p;
    ap.x = v[0]; ap.y = v[1]; ap.z = v[2];
    pl2.push_back(ap);
  }
  pub_pl_func(pl1, pub_src); pub_pl_func(pl2, pub_tar);
}

