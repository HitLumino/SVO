// This file is part of SVO - Semi-direct Visual Odometry.
//
// Copyright (C) 2014 Christian Forster <forster at ifi dot uzh dot ch>
// (Robotics and Perception Group, University of Zurich, Switzerland).
//
// SVO is free software: you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the Free Software
// Foundation, either version 3 of the License, or any later version.
//
// SVO is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#ifndef SVO_CONFIG_H_
#define SVO_CONFIG_H_

#include <string>
#include <stdint.h>
#include <stdio.h>

namespace svo {

using std::string;

/// Global configuration file of SVO.
/// Implements the Singleton design pattern to allow global access and to ensure
/// that only one instance exists.

/// svo的配置文件， 使用了单件的方式
class Config
{
public:
	/// 单件实例
  static Config& getInstance();

  /// Base-name of the tracefiles.
  /// 轨迹文件的base名字
  static string& traceName() { return getInstance().trace_name; }

  /// Directory where the tracefiles are saved.
  ///  轨迹文件存储的目录
  static string& traceDir() { return getInstance().trace_dir; }

  /// Number of pyramid levels used for features.
  //// 金字塔的层数，默认为3层
  static size_t& nPyrLevels() { return getInstance().n_pyr_levels; }

  /// 是否使用IMU得到相对的旋转
  /// Use the IMU to get relative rotations.
  static bool& useImu() { return getInstance().use_imu; }

  /// coreKfs中关键帧的数目，会进行BA
  /// Number of keyframes in the core. The core-kfs are optimized through bundle adjustment.
  static size_t& coreNKfs() { return getInstance().core_n_kfs; }

  /// 地图的初始尺度
  /// Initial scale of the map. Depends on the distance the camera is moved for the initialization.
  static double& mapScale() { return getInstance().map_scale; }

  /// 特征点栅格的大小，单位为像素,默认为25
  /// Feature grid size of a cell in [px].
  static size_t& gridSize() { return getInstance().grid_size; }

  /// 设置文件， 初始化时两帧图像需要的最小的视差
  /// Initialization: Minimum required disparity between the first two frames.
  static double& initMinDisparity() { return getInstance().init_min_disparity; }

  /// 初始化需要的最少的特征点的数目
  /// Initialization: Minimum number of tracked features.
  static size_t& initMinTracked() { return getInstance().init_min_tracked; }

  /// 初始化时经过RANSAC以后需要的最少的内点数目
  /// Initialization: Minimum number of inliers after RANSAC.
  static size_t& initMinInliers() { return getInstance().init_min_inliers; }

  /// KLT 最大的层数
  /// Maximum level of the Lucas Kanade tracker.
  static size_t& kltMaxLevel() { return getInstance().klt_max_level; }

  /// KLT最小的层数
  /// Minimum level of the Lucas Kanade tracker.
  static size_t& kltMinLevel() { return getInstance().klt_min_level; }

  /// 重投影的阈值，像素????
  /// Reprojection threshold [px].
  static double& reprojThresh() { return getInstance().reproj_thresh; }

  /// 位姿优化以后重投影的阈值
  /// Reprojection threshold after pose optimization.
  static double& poseOptimThresh() { return getInstance().poseoptim_thresh; }

  /// 局部BA中需要的迭代的次数
  /// Number of iterations in local bundle adjustment.
  static size_t& poseOptimNumIter() { return getInstance().poseoptim_num_iter; }

  /// 优化每次迭代需要的点数
  /// Maximum number of points to optimize at every iteration.
  static size_t& structureOptimMaxPts() { return getInstance().structureoptim_max_pts; }

  /// 优化地图点需要的次数
  /// Number of iterations in structure optimization.
  static size_t& structureOptimNumIter() { return getInstance().structureoptim_num_iter; }

  /// 进行局部BA以后的重投影误差阈值
  /// Reprojection threshold after bundle adjustment.
  static double& lobaThresh() { return getInstance().loba_thresh; }

  /// 局部BA鲁棒Huber核函数阈值
  /// Threshold for the robust Huber kernel of the local bundle adjustment.
  static double& lobaRobustHuberWidth() { return getInstance().loba_robust_huber_width; }

  /// 局部BA中迭代的次数
  /// Number of iterations in the local bundle adjustment.
  static size_t& lobaNumIter() { return getInstance().loba_num_iter; }

  /// 两个关键帧之间的最小距离，相对于地图的平均高度？？？因为地图尺度不同，所以移动不同的距离对应视野的变化是不一样的
  /// Minimum distance between two keyframes. Relative to the average height in the map.
  static double& kfSelectMinDist() { return getInstance().kfselect_mindist; }

  /// 三角测量的时候只选择最小的Harris角点得分
  /// Select only features with a minimum Harris corner score for triangulation.
  static double& triangMinCornerScore() { return getInstance().triang_min_corner_score; }

  /// 亚像素级别的重投影和三角测量，如果不需要亚像素级别，设置为0
  /// Subpixel refinement of reprojection and triangulation. Set to 0 if no subpix refinement required!
  static size_t& subpixNIter() { return getInstance().subpix_n_iter; }

  /// 限制在地图中关键帧的数目，其实是slam，是指为0则为不限制关键帧的数目，最小的数目为3
  /// Limit the number of keyframes in the map. This makes nslam essentially.
  /// a Visual Odometry. Set to 0 if unlimited number of keyframes are allowed.
  /// Minimum number of keyframes is 3.
  static size_t& maxNKfs() { return getInstance().max_n_kfs; }

  /// 相机相对于IMU的延迟，单位为ms
  /// How much (in milliseconds) is the camera delayed with respect to the imu.
  static double& imgImuDelay() { return getInstance().img_imu_delay; }

  /// 跟踪时特征点最多的数目
  /// Maximum number of features that should be tracked.
  static size_t& maxFts() { return getInstance().max_fts; }

  /// 如果跟踪的特征点的数目低于这个值，则认为跟踪质量比较差
  /// If the number of tracked features drops below this threshold. Tracking quality is bad.
  static size_t& qualityMinFts() { return getInstance().quality_min_fts; }

  /// 如果在一帧中，丢弃的特征点的数量达到这个值，则认为跟踪的质量比较差
  /// If within one frame, this amount of features are dropped. Tracking quality is bad.
  static int& qualityMaxFtsDrop() { return getInstance().quality_max_drop_fts; }

private:
	/// 配置
  Config();
  Config(Config const&);
  void operator=(Config const&);
  string trace_name;
  string trace_dir;
  size_t n_pyr_levels;
  bool use_imu;
  size_t core_n_kfs;
  double map_scale;
  size_t grid_size;
  double init_min_disparity;
  size_t init_min_tracked;
  size_t init_min_inliers;
  size_t klt_max_level;
  size_t klt_min_level;
  double reproj_thresh;
  double poseoptim_thresh;
  size_t poseoptim_num_iter;
  size_t structureoptim_max_pts;
  size_t structureoptim_num_iter;
  double loba_thresh;
  double loba_robust_huber_width;
  size_t loba_num_iter;
  double kfselect_mindist;
  double triang_min_corner_score;
  size_t triang_half_patch_size;
  size_t subpix_n_iter;
  size_t max_n_kfs;
  double img_imu_delay;
  size_t max_fts;
  size_t quality_min_fts;
  int quality_max_drop_fts;
};

} // namespace svo

#endif // SVO_CONFIG_H_
