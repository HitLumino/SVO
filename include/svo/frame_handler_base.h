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

#ifndef SVO_FRAME_HANDLER_BASE_H_
#define SVO_FRAME_HANDLER_BASE_H_

#include <queue>
#include <vikit/timer.h>
#include <vikit/ringbuffer.h>
#include <boost/noncopyable.hpp>
#include <boost/function.hpp>
#include <boost/thread.hpp>
#include <svo/global.h>
#include <svo/map.h>

namespace vk
{
class AbstractCamera;
class PerformanceMonitor;
}

namespace svo
{
class Point;
class Matcher;
class DepthFilter;

/// Base class for various VO pipelines. Manages the map and the state machine.
class FrameHandlerBase : boost::noncopyable
{
public:
	
	/// 系统的状态
  enum Stage {
    STAGE_PAUSED,
    STAGE_FIRST_FRAME,
    STAGE_SECOND_FRAME,
    STAGE_DEFAULT_FRAME,
    STAGE_RELOCALIZING
  };
  
  /// 跟踪的质量
  enum TrackingQuality {
    TRACKING_INSUFFICIENT,
    TRACKING_BAD,
    TRACKING_GOOD
  };
  
  /// 更新的结果
  enum UpdateResult {
    RESULT_NO_KEYFRAME,
    RESULT_IS_KEYFRAME,
    RESULT_FAILURE
  };

  /// 构造函数
  FrameHandlerBase();

  /// 析构函数
  virtual ~FrameHandlerBase();

  /// Get the current map.
  /// 得到当前的地图
  const Map* map() const { return &map_; }

  /// Will reset the map as soon as the current frame is finished processing.
  /// 在当前帧处理完以后会复位地图？？？为什么？
  void reset() { set_reset_ = true; }

  /// Start processing.
  /// 开始处理图像
  void start() { set_start_ = true; }

  /// 得到当前系统的状态
  /// Get the current stage of the algorithm.
  Stage stage() const { return stage_; }

  /// 得到跟踪的质量
  /// Get tracking quality.
  TrackingQuality trackingQuality() const { return tracking_quality_; }

  /// Get the processing time of the previous iteration.
  /// 得到上一帧？的处理时间
  double lastProcessingTime() const { return timer_.getTime(); }

  /// 得到上一帧图像的特征点数量
  /// Get the number of feature observations of the last frame.
  size_t lastNumObservations() const { return num_obs_last_; }

protected:
  Stage stage_;                 //!< Current stage of the algorithm.当前算法的状态
  bool set_reset_;              //!< Flag that the user can set. Will reset the system before the next iteration.在下一次迭代以前复位整个系统
  bool set_start_;              //!< Flag the user can set to start the system when the next image is received.当下一个图像接受到的时候启动系统
  Map map_;                     //!< Map of keyframes created by the slam system. 整个slam系统由关键帧构成的地图
  vk::Timer timer_;             //!< Stopwatch to measure time to process frame. 
  vk::RingBuffer<double> acc_frame_timings_;    
  //!< Total processing time of the last 10 frames, used to give some user feedback on the performance. 
  //! 最近10帧图像的处理时间
  vk::RingBuffer<size_t> acc_num_obs_;          
  //!< Number of observed features of the last 10 frames, used to give some user feedback on the tracking performance.
  //! 最近10帧图像观测到的特征点数量
  size_t num_obs_last_;                         //!< Number of observations in the previous frame.上一帧图像观测到的特征点数目
  TrackingQuality tracking_quality_;            
  //!< An estimate of the tracking quality based on the number of tracked features.根据跟踪的特征点的数量来股基跟踪的质量

  /// Before a frame is processed, this function is called.
  /// 每次得到一帧图像都会先设置整个系统
  bool startFrameProcessingCommon(const double timestamp);

  /// 在图像处理完以后会设置系统的状态
  /// When a frame is finished processing, this function is called.
  int finishFrameProcessingCommon(
      const size_t update_id,
      const UpdateResult dropout,
      const size_t num_observations);

  /// 复位地图和跟踪部分，从新开始
  /// Reset the map and frame handler to start from scratch.
  void resetCommon();

  /// Reset the frame handler. Implement in derived class.
  /// 复位帧处理的handler，在派生类中实现
  virtual void resetAll() { resetCommon(); }

  /// Set the tracking quality based on the number of tracked features.
  /// 设置跟踪的质量
  virtual void setTrackingQuality(const size_t num_observations);

  /// 优化观测到的3D的位置
  /// Optimize some of the observed 3D points.
  virtual void optimizeStructure(FramePtr frame, size_t max_n_pts, int max_iter);
};

} // namespace nslam

#endif // SVO_FRAME_HANDLER_BASE_H_
