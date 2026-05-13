/**
 * @file feature_point.hpp
 * @brief LiDAR point cloud processing and feature extraction
 * 
 * This file contains point cloud data structures and processing functions
 * for multiple LiDAR sensor types including Livox, Velodyne, Ouster, Hesai,
 * RoboSense, and TartanAir datasets. Provides unified interface for different
 * LiDAR data formats and point cloud preprocessing.
 */

#ifndef FEATURE_POINT_HPP
#define FEATURE_POINT_HPP

#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <livox_ros_driver/CustomMsg.h>

typedef pcl::PointXYZINormal PointType;
using namespace std;

/** @brief Enumeration of supported LiDAR sensor types */
enum LID_TYPE{LIVOX, VELODYNE, OUSTER, HESAI, ROBOSENSE, TARTANAIR, HDL32E};

/**
 * @namespace velodyne_ros
 * @brief Point cloud structure for Velodyne LiDAR sensors
 */
namespace velodyne_ros {
  /**
   * @struct Point
   * @brief Velodyne point structure with timing and ring information
   */
  struct EIGEN_ALIGN16 Point {
      PCL_ADD_POINT4D;              ///< 3D coordinates + homogeneous coordinate
      float intensity;
      float time;                   ///< Timestamp within scan
      std::uint16_t ring;          ///< Laser ring number
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}  // namespace velodyne_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (float, time, time)
    (std::uint16_t, ring, ring)
)

/**
 * @namespace hdl32e_ros
 * @brief Point cloud structure for HDL-32E LiDAR sensor
 * 
 * HDL-32E specific implementation for optimized processing of 32-channel
 * Velodyne HDL-32E LiDAR data with enhanced intensity handling and
 * channel-based filtering capabilities.
 */
namespace hdl32e_ros {
  /**
   * @struct Point
   * @brief HDL-32E point structure with HDL-32E specific optimizations
   * 
   * Enhanced point structure for HDL-32E with optimized intensity processing,
   * 32-channel ring identification, and precise timing for better SLAM performance.
   */
  struct EIGEN_ALIGN16 Point {
      PCL_ADD_POINT4D;              ///< 3D coordinates + homogeneous coordinate
      float intensity;              ///< HDL-32E optimized intensity (0-255 range)
      float time;                   ///< HDL-32E time within scan (rosbag field name)
      std::uint16_t ring;          ///< Laser ring number (0-31 for HDL-32E)
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}  // namespace hdl32e_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(hdl32e_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (float, time, time)
    (std::uint16_t, ring, ring)
)

namespace ouster_ros 
{
  struct EIGEN_ALIGN16 Point 
  {
    PCL_ADD_POINT4D;
    float intensity;
    uint32_t t;
    uint16_t reflectivity;
    uint8_t  ring;
    // uint16_t ambient;
    uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point,
  (float, x, x)
  (float, y, y)
  (float, z, z)
  (float, intensity, intensity)
  // use std::uint32_t to avoid conflicting with pcl::uint32_t
  (std::uint32_t, t, t)
  // (std::uint16_t, reflectivity, reflectivity)
  // (std::uint8_t, ring, ring)
  // (std::uint16_t, ambient, ambient)
  // (std::uint32_t, range, range)
)

namespace xt32_ros {
  struct EIGEN_ALIGN16 Point {
      PCL_ADD_POINT4D;
      float intensity;
      float time;
      uint16_t ring;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}  // namespace velodyne_ros
POINT_CLOUD_REGISTER_POINT_STRUCT(xt32_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (float, time, time)
    (std::uint16_t, ring, ring)
)


namespace rslidar_ros {
  struct EIGEN_ALIGN16 Point {
      PCL_ADD_POINT4D;
      float intensity;
      std::uint16_t ring;
      double timestamp;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
}
POINT_CLOUD_REGISTER_POINT_STRUCT(rslidar_ros::Point,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, intensity, intensity)
    (std::uint16_t, ring, ring)
    (double, timestamp, timestamp)
)

/**
 * @class Features
 * @brief Multi-sensor LiDAR point cloud processing and feature extraction
 * 
 * Unified interface for processing point clouds from different LiDAR sensors.
 * Handles data format conversion, filtering, and feature extraction for
 * Livox, Velodyne, Ouster, Hesai, RoboSense, and TartanAir datasets.
 */
class Features
{
public:
  int lidar_type;              ///< Type of LiDAR sensor (from LID_TYPE enum)
  int point_filter_num;        ///< Point sampling rate (process every Nth point)
  double blind = 1;            ///< Minimum range threshold (m²)
  double omega_l = 3610;       ///< Angular velocity parameter
  
  // HDL32E timestamp offset settings
  bool enable_hdl32e_timestamp_offset = false;  ///< Enable timestamp offset for HDL32E
  double hdl32e_timestamp_offset = 0.05;        ///< Offset value to add to timestamps (default: 0.05)
  
  // HDL32E timing validation settings
  bool is_valid_hdl32e_timing = true;           ///< Direct setting for is_valid_hdl32e_timing from YAML
  
  // Original resolution point cloud storage (FIFO)
  std::deque<pcl::PointCloud<PointType>::Ptr> original_pointclouds;  ///< Store truly original point clouds in arrival order

  /**
   * @brief Process Livox custom message format
   * @param msg Livox custom message pointer
   * @param pl_full Output point cloud
   * @return Message timestamp in seconds
   */
  double process(const livox_ros_driver::CustomMsg::ConstPtr &msg, pcl::PointCloud<PointType> &pl_full)
  {
    livox_handler(msg, pl_full);
    return msg->header.stamp.toSec();
  }

  /**
   * @brief Process standard ROS PointCloud2 message
   * @param msg PointCloud2 message pointer
   * @param pl_full Output point cloud
   * @return Message timestamp in seconds
   * 
   * Routes processing to appropriate sensor-specific handler based on lidar_type.
   */
  double process(const sensor_msgs::PointCloud2::ConstPtr &msg, pcl::PointCloud<PointType> &pl_full)
  {
    double t0 = msg->header.stamp.toSec();
    switch(lidar_type)
    {
    case VELODYNE:
      velodyne_handler(msg, pl_full);
      break;

    case OUSTER:
      ouster_handler(msg, pl_full);
      break;

    case HESAI:
      hesai_handler(msg, pl_full);
      break;
    
    case ROBOSENSE:
      t0 = robosense_handler(msg, pl_full);
      break;
    
    case TARTANAIR:
      tartanair_handler(msg, pl_full);
      break;
    
    case HDL32E:
      hdl32e_handler(msg, pl_full);
      break;

    default:
      printf("Lidar Type Error\n");
      exit(0);
    }

    return t0;
  }

  void livox_handler(const livox_ros_driver::CustomMsg::ConstPtr &msg, pcl::PointCloud<PointType> &pl_full)
  { 
    int plsize = msg->point_num;
    pl_full.reserve(plsize);

    for(int i=0; i<plsize; i++)
    {
      PointType ap;
      ap.x = msg->points[i].x;
      ap.y = msg->points[i].y;
      ap.z = msg->points[i].z;
      ap.intensity = msg->points[i].reflectivity;
      // ap.curvature = msg->points[i].offset_time / float(1000000); // ms
      ap.curvature = msg->points[i].offset_time / float(1000000000); // s

      if(i % point_filter_num == 0)
      {
        if(ap.x*ap.x + ap.y*ap.y + ap.z*ap.z > blind)
        {
          pl_full.push_back(ap);
        }
      }

    }

  }

  void velodyne_handler(const sensor_msgs::PointCloud2::ConstPtr &msg, pcl::PointCloud<PointType> &pl_full)
  {
    pcl::PointCloud<velodyne_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);

    int plsize = pl_orig.size();
    if(plsize == 0) return;
    // Velodyne: 두 가지 시간 범위 지원
    // 1) 기존 Velodyne rosbag: 0.01~0.12 (양수만)
    // 2) HDL-32E triggerTime: -0.05~+0.051 (음수 포함)
    bool is_legacy_velodyne = (pl_orig.back().time > 0.01 && pl_orig.back().time < 0.12);

    // DEBUG: 시간 범위 분석
    static int vel_call_count = 0;
    vel_call_count++;
    //printf("VELODYNE_DEBUG[%d]: time_range=[%.6f, %.6f], legacy=%s,", 
    //       vel_call_count, pl_orig[0].time, pl_orig.back().time,
    //      is_legacy_velodyne ? "YES" : "NO");
    //if(is_legacy_velodyne)
    if(0)
    {
      printf("VELODYNE_DEBUG[%d]: Using TIME-BASED processing path\n", vel_call_count);
      
      // DEBUG: Sample input point analysis
      //printf("VELODYNE_DEBUG[%d]: Input analysis - plsize=%d, point_filter_num=%d, blind=%.2f\n", 
      //       vel_call_count, plsize, point_filter_num, blind);
      
      if(plsize > 0) {
        printf("VELODYNE_DEBUG[%d]: Sample input points:[%d]\n", vel_call_count, plsize);
        for(int sample_i = 0; sample_i < std::min(3, plsize); sample_i++) {
          //printf("  Input[%d]: x=%.3f y=%.3f z=%.3f intensity=%.1f ring=%d time=%.6f\n", 
          //      sample_i, pl_orig[sample_i].x, pl_orig[sample_i].y, pl_orig[sample_i].z,
          //       pl_orig[sample_i].intensity, pl_orig[sample_i].ring, pl_orig[sample_i].time);
        }
      }
      
      int filtered_count = 0;
      int blind_filtered_count = 0;
      int output_count = 0;
      
      // for(velodyne_ros::Point &iter : pl_orig.points)
      for(int i=0; i<plsize; i++)
      {
        velodyne_ros::Point &iter = pl_orig[i];
        PointType ap;
        ap.x = iter.x; ap.y = iter.y; ap.z = iter.z;
        
        ap.intensity = iter.intensity;
        // ap.curvature = iter.time * 1e-3; // ms
        // ap.curvature = iter.time * 1e-6;
        ap.curvature = iter.time;

        if(i % point_filter_num == 0)
        {
          filtered_count++;
          double point_range_sq = ap.x*ap.x + ap.y*ap.y + ap.z*ap.z;
          if(point_range_sq > blind)
          {
            pl_full.push_back(ap);
            output_count++;
            
            // DEBUG: Sample output points
            //if(output_count <= 3) {
            //  printf("  Output[%d]: x=%.3f y=%.3f z=%.3f intensity=%.1f range=%.2f time=%.6f\n", 
            //         output_count, ap.x, ap.y, ap.z, ap.intensity, sqrt(point_range_sq), ap.curvature);
            //}
          }
          else {
            blind_filtered_count++;
          }
        }
      }
      
      //printf("VELODYNE_DEBUG[%d]: Processing summary - input:%d -> filtered:%d -> blind_removed:%d -> output:%d\n", 
      //       vel_call_count, plsize, filtered_count, blind_filtered_count, output_count);

    }
    else
    {
      //printf("VELODYNE_DEBUG[%d]: Using ANGLE-BASED processing path\n", vel_call_count);
      
      // lidar clockwise rotate
      bool first_point = true;
      double yaw_first = 0;
      double yaw_last = 0;
      double yaw_bias = 0;
      int cool = 0;
      float max_ang = 0;
      for(int i=0; i<plsize; i++)
      {
        cool--;
        velodyne_ros::Point &iter = pl_orig[i];
        PointType ap;
        ap.x = iter.x; ap.y = iter.y; ap.z = iter.z;
        ap.intensity = iter.intensity;

        if(fabs(ap.x) < 0.1)
          continue;
        
        double yaw_angle = atan2(ap.y, ap.x) * 57.2957 - yaw_bias;
        if(first_point)
        {
          yaw_first = yaw_angle;
          yaw_last  = yaw_angle;
          first_point = false;
        }

        if(ap.x*ap.x + ap.y*ap.y + ap.z*ap.z < blind)
          continue;

        if(yaw_angle - yaw_last > 180 && cool <= 0)
        {
          yaw_bias += 360; yaw_angle-= 360; cool = 1000;
        }

        if(fabs(yaw_angle - yaw_last) > 180)
        {
          yaw_angle += 360;
        }

        ap.curvature = (yaw_first - yaw_angle) / omega_l;
        yaw_last = yaw_angle;

        if(ap.curvature > max_ang)
          max_ang = ap.curvature;

        if(ap.curvature >= 0 && ap.curvature < 0.1)
          if(i % point_filter_num == 0)
            pl_full.push_back(ap);
      }

      // printf("maxang: %f\n", max_ang);
    }

  }

  void ouster_handler(const sensor_msgs::PointCloud2::ConstPtr &msg, pcl::PointCloud<PointType> &pl_full)
  {
    pcl::PointCloud<ouster_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);

    int plsize = pl_orig.points.size();
    pl_full.reserve(plsize);
    for(int i=0; i<plsize; i++)
    {
      PointType ap;
      ap.x = pl_orig.points[i].x;
      ap.y = pl_orig.points[i].y;
      ap.z = pl_orig.points[i].z;
      ap.intensity = pl_orig[i].intensity;
      // ap.curvature = pl_orig[i].t / float(1e6); // ms
      ap.curvature = pl_orig[i].t / float(1e9); // s

      if(i % point_filter_num == 0)
      {
        if(ap.x*ap.x + ap.y*ap.y + ap.z*ap.z > blind)
        {
          pl_full.points.push_back(ap);
        }
      }

    }

  }

  void hesai_handler(const sensor_msgs::PointCloud2::ConstPtr &msg, pcl::PointCloud<PointType> &pl_full)
  { 
    pcl::PointCloud<xt32_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);

    int plsize = pl_orig.points.size();
    pl_full.reserve(plsize);
    float time_head = pl_orig.points[0].time;
    for(int i=0; i<plsize; i++)
    {
      PointType added_pt;

      added_pt.normal_x = 0;
      added_pt.normal_y = 0;
      added_pt.normal_z = 0;
      added_pt.x = pl_orig.points[i].x;
      added_pt.y = pl_orig.points[i].y;
      added_pt.z = pl_orig.points[i].z;
      added_pt.intensity = pl_orig.points[i].intensity;
      added_pt.curvature = (pl_orig.points[i].time - time_head);

      if (i % point_filter_num == 0)
      {
        if (added_pt.x*added_pt.x+added_pt.y*added_pt.y+added_pt.z*added_pt.z > blind)
        {
          pl_full.points.push_back(added_pt);
        }
      }


    }

  }

  double robosense_handler(const sensor_msgs::PointCloud2::ConstPtr &msg, pcl::PointCloud<PointType> &pl_full)
  {
    pcl::PointCloud<rslidar_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);

    int plsize = pl_orig.points.size();
    pl_full.reserve(plsize);
    double t0 = pl_orig[0].timestamp;
    for(int i=0; i<plsize; i++)
    {
      PointType ap;
      ap.x = pl_orig.points[i].x;
      ap.y = pl_orig.points[i].y;
      ap.z = pl_orig.points[i].z;
      ap.intensity = pl_orig.points[i].intensity;
      // ap.curvature = (pl_orig[i].timestamp - t0) * float(1e3); //
      ap.curvature = (pl_orig[i].timestamp - t0);

      if(i % point_filter_num == 0)
      {
        if(ap.x*ap.x + ap.y*ap.y + ap.z*ap.z > blind)
        {
          pl_full.points.push_back(ap);
        }
      }

    }

    return t0;
  }

  void tartanair_handler(const sensor_msgs::PointCloud2::ConstPtr &msg, pcl::PointCloud<PointType> &pl_full)
  {
    pcl::PointCloud<pcl::PointXYZ> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    pl_full.reserve(pl_orig.size());

    PointType pp; pp.curvature = 0; pp.intensity = 0;
    for(pcl::PointXYZ &ap: pl_orig.points)
    {
      pp.x = ap.x;
      pp.y = ap.y;
      pp.z = ap.z; 
      pl_full.push_back(pp);
    }

    return;
  }

  /**
   * @brief Process HDL-32E LiDAR point cloud data with GPS time handling
   * @param msg PointCloud2 message from HDL-32E sensor
   * @param pl_full Output processed point cloud
   * 
   * HDL-32E specific processing that handles:
   * - GPS time conversion to relative scan time
   * - High-precision timestamp differences between points
   * - Ring field ignored (dummy data, all zeros)
   * - Optimized intensity processing for HDL-32E characteristics
   */
  void hdl32e_handler(const sensor_msgs::PointCloud2::ConstPtr &msg, pcl::PointCloud<PointType> &pl_full)
  {
    // DEBUG: HDL-32E handler called
    static int call_count = 0;
    call_count++;
    printf("HDL32E_DEBUG[%d]: Handler called, msg size: %zu bytes\n", call_count, msg->data.size());
    
    // DEBUG: Check raw ROS message fields first
    printf("HDL32E_DEBUG[%d]: ROS msg - width:%d height:%d point_step:%d row_step:%d\n", 
           call_count, msg->width, msg->height, msg->point_step, msg->row_step);
    printf("HDL32E_DEBUG[%d]: Fields count: %zu\n", call_count, msg->fields.size());
    for(size_t i = 0; i < msg->fields.size(); i++) {
      printf("HDL32E_DEBUG[%d]: Field[%zu]: name='%s' offset=%d datatype=%d count=%d\n", 
             call_count, i, msg->fields[i].name.c_str(), msg->fields[i].offset, 
             msg->fields[i].datatype, msg->fields[i].count);
    }
    
    // HDL32E: use hdl32e_ros::Point with corrected field names (x,y,z,intensity,ring,timestamp)
    pcl::PointCloud<hdl32e_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);

    int plsize = pl_orig.size();
    printf("HDL32E_DEBUG[%d]: Point cloud size: %d points\n", call_count, plsize);
    
    if(plsize == 0) {
      printf("HDL32E_DEBUG[%d]: Empty point cloud, returning\n", call_count);
      return;
    }
    
    // Original point cloud storage moved to pcl_handler
    
    // HDL32E: 전체 포인트의 시간 범위 분석 (triggerTime 기준 상대적 오프셋)
    double min_time = std::numeric_limits<double>::max();
    double max_time = std::numeric_limits<double>::lowest();
    
    for(const auto& point : pl_orig.points) {
      if(std::isfinite(point.time)) {  // NaN/Inf 체크
        min_time = std::min(min_time, (double)point.time);
        max_time = std::max(max_time, (double)point.time);
      }
    }
    
    double time_span = max_time - min_time;

    std::cout << "time span: " << time_span << ", min_time: " << min_time << ", max_time: " << max_time << std::endl;
    
    // DEBUG: HDL32E 시간 분석 결과
    printf("HDL32E_DEBUG[%d]: Time analysis - min:%.6f max:%.6f span:%.6f\n", 
           call_count, min_time, max_time, time_span);
    printf("HDL32E_DEBUG[%d]: Sample times - [0]:%.6f [%d]:%.6f [%d]:%.6f [%d]:%.6f\n", 
           call_count, pl_orig.points[0].time, plsize/4, pl_orig.points[plsize/4].time,
           plsize/2, pl_orig.points[plsize/2].time, plsize-1, pl_orig.points[plsize-1].time);
    
    // HDL32E 특화 조건: YAML에서 직접 설정
    // is_valid_hdl32e_timing은 YAML에서 설정된 값을 그대로 사용
      
    printf("HDL32E_DEBUG[%d]: Timing validation - valid:%s (span:%.3f, range:[%.3f,%.3f])\n", 
           call_count, is_valid_hdl32e_timing ? "YES" : "NO", time_span, min_time, max_time);
    
    if(is_valid_hdl32e_timing)
    {
      printf("HDL32E_DEBUG[%d]: Using TIME-BASED processing path\n", call_count);
      
      // Time-based processing path for HDL-32E
      int processed_points = 0;
      for(int i = 0; i < plsize; i++)
      {
        hdl32e_ros::Point &iter = pl_orig[i];
        
        PointType ap;
        ap.x = iter.x; 
        ap.y = iter.y; 
        ap.z = iter.z;
        
        // HDL-32E intensity processing (preserve full range)
        ap.intensity = iter.intensity;
        
        // HDL32E: triggerTime 기준 상대적 시간 직접 사용 (음수 포함)
        ap.curvature = iter.time;  // rosbag 'time' field 사용
        
        // Apply timestamp offset if enabled (convert -0.05~0.05 to 0~0.1)
        if(enable_hdl32e_timestamp_offset) {
            double original_time = ap.curvature;
            ap.curvature += hdl32e_timestamp_offset;
            
            // Debug output for first few points
            if(processed_points <= 3) {
                printf("HDL32E_TIMESTAMP_OFFSET[%d]: Point[%d] - original: %.6f -> offset: %.6f (offset=%.6f)\n", 
                       call_count, processed_points, original_time, ap.curvature, hdl32e_timestamp_offset);
            }
        }

        // Original points are stored in pcl_handler

        if(i % point_filter_num == 0)
        {
          double point_range = ap.x*ap.x + ap.y*ap.y + ap.z*ap.z;
          if(point_range > blind)
          {
            pl_full.push_back(ap);
            processed_points++;
            
            // Sample debug output (음수 시간 포함)
            if(processed_points <= 5) {
              printf("HDL32E_DEBUG[%d]: Point[%d] - intensity: %.2f, range: %.2f, rel_time: %.6f\n", 
                     call_count, processed_points, ap.intensity, sqrt(point_range), ap.curvature);
            }
          }
        }
      }
      printf("HDL32E_DEBUG[%d]: TIME-BASED path processed %d points\n", call_count, processed_points);
    }
    else
    {
      printf("HDL32E_DEBUG[%d]: Using ANGLE-BASED processing path\n", call_count);
      
      // Angle-based processing path for HDL-32E (safer for GPS time)
      bool first_point = true;
      double yaw_first = 0;
      double yaw_last = 0;
      double yaw_bias = 0;
      int cool = 0;
      int processed_points = 0;
      
      for(int i = 0; i < plsize; i++)
      {
        cool--;
        hdl32e_ros::Point &iter = pl_orig[i];
        
        PointType ap;
        ap.x = iter.x; 
        ap.y = iter.y; 
        ap.z = iter.z;
        
        // HDL-32E intensity processing
        ap.intensity = iter.intensity;

        // Original points are stored in pcl_handler

        if(fabs(ap.x) < 0.1) continue;
        
        double yaw_angle = atan2(ap.y, ap.x) * 57.2957 - yaw_bias;
        if(first_point)
        {
          yaw_first = yaw_angle;
          yaw_last  = yaw_angle;
          first_point = false;
        }

        double point_range = ap.x*ap.x + ap.y*ap.y + ap.z*ap.z;
        if(point_range < blind) continue;

        if(yaw_angle - yaw_last > 180 && cool <= 0)
        {
          yaw_bias += 360; 
          yaw_angle -= 360; 
          cool = 1000;
        }

        if(fabs(yaw_angle - yaw_last) > 180)
        {
          yaw_angle += 360;
        }

        // Angular-based curvature calculation for HDL-32E
        ap.curvature = (yaw_first - yaw_angle) / omega_l;
        yaw_last = yaw_angle;

        if(ap.curvature >= 0 && ap.curvature < 0.1) {
          if(i % point_filter_num == 0) {
            pl_full.push_back(ap);
            processed_points++;
            
            // Sample intensity debug output
            if(processed_points <= 5) {
              printf("HDL32E_DEBUG[%d]: Point[%d] - intensity: %.2f, range: %.2f, curvature: %.6f\n", 
                     call_count, processed_points, ap.intensity, sqrt(point_range), ap.curvature);
            }
          }
        }
      }
      printf("HDL32E_DEBUG[%d]: ANGLE-BASED path processed %d points\n", call_count, processed_points);
    }
    
    printf("HDL32E_DEBUG[%d]: Handler completed, output size: %zu points\n", call_count, pl_full.size());
    // Original point cloud size is now tracked in pcl_handler
    printf("HDL32E_DEBUG[%d]: Input ROS message size: %d points\n", call_count, plsize);
    
    // Original point cloud is now stored in pcl_handler before curvature filtering
  }

};

#endif
