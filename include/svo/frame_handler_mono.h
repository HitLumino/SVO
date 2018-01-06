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

#ifndef SVO_FRAME_HANDLER_H_
#define SVO_FRAME_HANDLER_H_

#include <set>
#include <boost/concept_check.hpp>
#include <vikit/abstract_camera.h>
#include <svo/frame_handler_base.h>
#include <svo/reprojector.h>
#include <svo/initialization.h>
#include <svo/map_drawer.h>
#include <svo/IMU_handler.h>

namespace svo {

/// Monocular Visual Odometry Pipeline as described in the SVO paper.
class FrameHandlerMono : public FrameHandlerBase
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  /// 构造函数，传入已经构建好的相机实例
  FrameHandlerMono(vk::AbstractCamera* cam,  MapDrawer* map_drawer);
  /// 析构函数
  virtual ~FrameHandlerMono();

  /// 处理新到的图像，传入的是图像的引用以及图像的时间戳
  /// Provide an image.
  void addImage(const cv::Mat& img, double timestamp);

  /// Provide an IMU data
  void addIMU(const Vector3d& acc_measure,  const Vector3d& gyro_measure, double time_stamp,  double delta_t);
  
  /// 将图像设置为第一帧图像，传入的是帧的指针
  /// Set the first frame (used for synthetic datasets in benchmark node)
  void setFirstFrame(const FramePtr& first_frame);

  /// 得到上一帧图像， 返回的是对象的指针
  /// Get the last frame that has been processed.
  FramePtr lastFrame() const { return last_frame_; }

  /// 得到空间上最相近的关键帧
  /// Get the set of spatially closest keyframes of the last frame.
  const set<FramePtr>& coreKeyframes() { return core_kfs_; }

  /// 返回KLT初始化的第一帧和第二帧的关键点
  /// Return the feature track to visualize the KLT tracking during initialization.
  const vector<cv::Point2f>& initFeatureTrackRefPx() const { return klt_homography_init_.px_ref_; }
  const vector<cv::Point2f>& initFeatureTrackCurPx() const { return klt_homography_init_.px_cur_; }

  /// 返回深度滤波器的指针
  /// Access the depth filter.
  DepthFilter* depthFilter() const { return depth_filter_; }

  /// 在给定位姿处进行重定位
  /// An external place recognition module may know where to relocalize.
  bool relocalizeFrameAtPose(
      const int keyframe_id,
      const SE3& T_kf_f,
      const cv::Mat& img,
      const double timestamp);

protected:
  vk::AbstractCamera* cam_;                     //!< Camera model, can be ATAN, Pinhole or Ocam (see vikit).相机模型， ATAN模型、针孔模型或者是全景模型
  Reprojector reprojector_;                     //!< Projects points from other keyframes into the current frame 将其他帧的点投影到当前帧
  FramePtr new_frame_;                          //!< Current frame.当前帧图像
  FramePtr last_frame_;                         //!< Last frame, not necessarily a keyframe.上一帧图像
  set<FramePtr> core_kfs_;                      //!< Keyframes in the closer neighbourhood.距离当前帧最近的关键帧，文章中说最近的意思就是observation angle最近
  vector< pair<FramePtr,size_t> > overlap_kfs_; 
  
  IMUhandler imu_handler_;
  
  //!< All keyframes with overlapping field of view. the paired number specifies how many common mappoints are observed TODO: why vector!?
  //! 所有有重叠区域的关键帧， 第二个值是公共地图点的数目？
  
  MapDrawer * map_drawer_;
  
  initialization::KltHomographyInit klt_homography_init_; 
  //!< Used to estimate pose of the first two keyframes by estimating a homography.
  //! 用前两帧图像估计单应性矩阵并进行初始化
  
  DepthFilter* depth_filter_;                   
  //!< Depth estimation algorithm runs in a parallel thread and is used to initialize new 3D points.
  //! 深度滤波器， 用来初始化新的3D点
  
  /// Initialize the visual odometry algorithm.
  // 初始化整个算法
  virtual void initialize();

  /// Processes the first frame and sets it as a keyframe.
  /// 处理第一帧图像并将第一帧图像设置为关键帧
  virtual UpdateResult processFirstFrame();

  /// Processes all frames after the first frame until a keyframe is selected.
  /// 处理第一帧图像以后的所有图像，并从中选择一个关键帧
  virtual UpdateResult processSecondFrame();

  /// Processes all frames after the first two keyframes.
	/// 处理两个关键帧以后的所有帧
  virtual UpdateResult processFrame();

  /// Try relocalizing the frame at relative position to provided keyframe.
  /// 重定位相机的位置
  virtual UpdateResult relocalizeFrame(
      const SE3& T_cur_ref,
      FramePtr ref_keyframe);

  /// Reset the frame handler. Implement in derived class.
  /// 复位整个系统，在派生类中实现
  virtual void resetAll();

  /// Keyframe selection criterion.
  /// 关键帧选择策略
  virtual bool needNewKf(double scene_depth_mean);

  /// 设置核心的关键帧，也就是观测地图点的角度比较相近
  void setCoreKfs(size_t n_closest);
};

} // namespace svo

#endif // SVO_FRAME_HANDLER_H_
