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
  
  /// ���캯���������Ѿ������õ����ʵ��
  FrameHandlerMono(vk::AbstractCamera* cam,  MapDrawer* map_drawer);
  /// ��������
  virtual ~FrameHandlerMono();

  /// �����µ���ͼ�񣬴������ͼ��������Լ�ͼ���ʱ���
  /// Provide an image.
  void addImage(const cv::Mat& img, double timestamp);

  /// Provide an IMU data
  void addIMU(const Vector3d& acc_measure,  const Vector3d& gyro_measure, double time_stamp,  double delta_t);
  
  /// ��ͼ������Ϊ��һ֡ͼ�񣬴������֡��ָ��
  /// Set the first frame (used for synthetic datasets in benchmark node)
  void setFirstFrame(const FramePtr& first_frame);

  /// �õ���һ֡ͼ�� ���ص��Ƕ����ָ��
  /// Get the last frame that has been processed.
  FramePtr lastFrame() const { return last_frame_; }

  /// �õ��ռ���������Ĺؼ�֡
  /// Get the set of spatially closest keyframes of the last frame.
  const set<FramePtr>& coreKeyframes() { return core_kfs_; }

  /// ����KLT��ʼ���ĵ�һ֡�͵ڶ�֡�Ĺؼ���
  /// Return the feature track to visualize the KLT tracking during initialization.
  const vector<cv::Point2f>& initFeatureTrackRefPx() const { return klt_homography_init_.px_ref_; }
  const vector<cv::Point2f>& initFeatureTrackCurPx() const { return klt_homography_init_.px_cur_; }

  /// ��������˲�����ָ��
  /// Access the depth filter.
  DepthFilter* depthFilter() const { return depth_filter_; }

  /// �ڸ���λ�˴������ض�λ
  /// An external place recognition module may know where to relocalize.
  bool relocalizeFrameAtPose(
      const int keyframe_id,
      const SE3& T_kf_f,
      const cv::Mat& img,
      const double timestamp);

protected:
  vk::AbstractCamera* cam_;                     //!< Camera model, can be ATAN, Pinhole or Ocam (see vikit).���ģ�ͣ� ATANģ�͡����ģ�ͻ�����ȫ��ģ��
  Reprojector reprojector_;                     //!< Projects points from other keyframes into the current frame ������֡�ĵ�ͶӰ����ǰ֡
  FramePtr new_frame_;                          //!< Current frame.��ǰ֡ͼ��
  FramePtr last_frame_;                         //!< Last frame, not necessarily a keyframe.��һ֡ͼ��
  set<FramePtr> core_kfs_;                      //!< Keyframes in the closer neighbourhood.���뵱ǰ֡����Ĺؼ�֡��������˵�������˼����observation angle���
  vector< pair<FramePtr,size_t> > overlap_kfs_; 
  
  IMUhandler imu_handler_;
  
  //!< All keyframes with overlapping field of view. the paired number specifies how many common mappoints are observed TODO: why vector!?
  //! �������ص�����Ĺؼ�֡�� �ڶ���ֵ�ǹ�����ͼ�����Ŀ��
  
  MapDrawer * map_drawer_;
  
  initialization::KltHomographyInit klt_homography_init_; 
  //!< Used to estimate pose of the first two keyframes by estimating a homography.
  //! ��ǰ��֡ͼ����Ƶ�Ӧ�Ծ��󲢽��г�ʼ��
  
  DepthFilter* depth_filter_;                   
  //!< Depth estimation algorithm runs in a parallel thread and is used to initialize new 3D points.
  //! ����˲����� ������ʼ���µ�3D��
  
  /// Initialize the visual odometry algorithm.
  // ��ʼ�������㷨
  virtual void initialize();

  /// Processes the first frame and sets it as a keyframe.
  /// �����һ֡ͼ�񲢽���һ֡ͼ������Ϊ�ؼ�֡
  virtual UpdateResult processFirstFrame();

  /// Processes all frames after the first frame until a keyframe is selected.
  /// �����һ֡ͼ���Ժ������ͼ�񣬲�����ѡ��һ���ؼ�֡
  virtual UpdateResult processSecondFrame();

  /// Processes all frames after the first two keyframes.
	/// ���������ؼ�֡�Ժ������֡
  virtual UpdateResult processFrame();

  /// Try relocalizing the frame at relative position to provided keyframe.
  /// �ض�λ�����λ��
  virtual UpdateResult relocalizeFrame(
      const SE3& T_cur_ref,
      FramePtr ref_keyframe);

  /// Reset the frame handler. Implement in derived class.
  /// ��λ����ϵͳ������������ʵ��
  virtual void resetAll();

  /// Keyframe selection criterion.
  /// �ؼ�֡ѡ�����
  virtual bool needNewKf(double scene_depth_mean);

  /// ���ú��ĵĹؼ�֡��Ҳ���ǹ۲��ͼ��ĽǶȱȽ����
  void setCoreKfs(size_t n_closest);
};

} // namespace svo

#endif // SVO_FRAME_HANDLER_H_
