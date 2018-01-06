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

#ifndef SVO_FRAME_H_
# define SVO_FRAME_H_

# include <sophus/se3.h>
# include <vikit/math_utils.h>
# include <vikit/abstract_camera.h>
# include <boost/noncopyable.hpp>
# include <svo/global.h>

namespace g2o {
class VertexSE3Expmap;
}
typedef g2o::VertexSE3Expmap g2oFrameSE3;

namespace svo {

class Point;
struct Feature;

typedef list<Feature*> Features;
typedef vector<cv::Mat> ImgPyr;

/// 图像帧保存图像，相应的特征点以及估计的位姿
/// A frame saves the image, the associated features and the estimated pose.
class Frame : boost::noncopyable
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    static int                    frame_counter_;         
    //!< Counts the number of created frames. Used to set the unique id.帧的数目
    int                           id_;                    
    //!< Unique id of the frame. 帧的编号
    double                        timestamp_;             
    //!< Timestamp of when the image was recorded.图像的时间戳
    vk::AbstractCamera*           cam_;                   
    //!< Camera model. 相机模型
    Sophus::SE3                   T_f_w_;                 
    //!< Transform (f)rame from (w)orld. 相机到世界坐标系的位姿矩阵
    Matrix<double, 6, 6>          Cov_;                   
    //!< Covariance.协方差？干啥用的？？
    ImgPyr                        img_pyr_;               
    //!< Image Pyramid.图像金字塔

    Features                      fts_;
    //!< List of features in the image.图像中的特征点，这里使用的是对象类型

    vector<Feature*>              key_pts_;               
    //!< Five features and associated 3D points which are used to detect if two frames have overlapping field of view.
    // 5个特征点已经对应的3d点，用来检测两个是否有重叠的视角

    bool                          is_keyframe_;           
    //!< Was this frames selected as keyframe? 当前帧是否为关键帧的标记

    g2oFrameSE3*                  v_kf_;                  
    //!< Temporary pointer to the g2o node object of the keyframe. 

    int                           last_published_ts_;     
    //!< Timestamp of last publishing.

    //构造函数
    Frame(vk::AbstractCamera* cam, const cv::Mat& img, double timestamp);
    ~Frame();

    //初始化帧并创建图像金字塔
    /// Initialize new frame and create image pyramid.
    void initFrame(const cv::Mat& img);

    ///将当前帧设置为关键帧
    /// Select this frame as keyframe.
    void setKeyframe();

    /// 图像添加特征点
    /// Add a feature to the image
    void addFeature(Feature* ftr);

    /// 关键点位最靠近角落的四个点以及图像中心的有对应的3D点的特征点，从而快速的检测两个帧是否有重叠的视野
    /// The KeyPoints are those five features which are closest to the 4 image corners
    /// and to the center and which have a 3D point assigned. These points are used
    /// to quickly check whether two frames have overlapping field of view.
    void setKeyPoints();

    /// 检查是否能够使用5个更好的关键点
    /// Check if we can select five better key-points.
    void checkKeyPoints(Feature* ftr);

    /// 如果删除一个点，如果这个点为关键点，则需要重新选择关键点
    /// If a point is deleted, we must remove the corresponding key-point.
    void removeKeyPoint(Feature* ftr);

    // 返回特征点的数目
    /// Return number of point observations.
    inline size_t nObs() const { return fts_.size(); }

    /// 检查图像中的一个点在当前帧是否可见
    /// Check if a point in (w)orld coordinate frame is visible in the image.
    bool isVisible(const Vector3d& xyz_w) const;

    /// 金字塔第一层
    /// Full resolution image stored in the frame.
    inline const cv::Mat& img() const { return img_pyr_[0]; }

    /// 判断图像是否被选择为关键帧
    /// Was this frame selected as keyframe?
    inline bool isKeyframe() const { return is_keyframe_; }

    /// 将世界坐标系中的点转换成相机像素坐标
    /// Transforms point coordinates in world-frame (w) to camera pixel coordinates (c).
    inline Vector2d w2c(const Vector3d& xyz_w) const { return cam_->world2cam( T_f_w_ * xyz_w ); }

    /// 将像素坐标转换成
    /// Transforms pixel coordinates (c) to frame unit sphere coordinates (f).
    inline Vector3d c2f(const Vector2d& px) const { return cam_->cam2world(px[0], px[1]); }

    /// Transforms pixel coordinates (c) to frame unit sphere coordinates (f).
    inline Vector3d c2f(const double x, const double y) const { return cam_->cam2world(x, y); }

    /// Transforms point coordinates in world-frame (w) to camera-frams (f).
    inline Vector3d w2f(const Vector3d& xyz_w) const { return T_f_w_ * xyz_w; }

    /// Transforms point from frame unit sphere (f) frame to world coordinate frame (w).
    inline Vector3d f2w(const Vector3d& f) const { return T_f_w_.inverse() * f; }

    /// Projects Point from unit sphere (f) in camera pixels (c).
    inline Vector2d f2c(const Vector3d& f) const { return cam_->world2cam( f ); }

    /// Return the pose of the frame in the (w)orld coordinate frame.
    inline Vector3d pos() const { return T_f_w_.inverse().translation(); }

    /// Frame jacobian for projection of 3D point in (f)rame coordinate to
    /// unit plane coordinates uv (focal length = 1).
    inline static void jacobian_xyz2uv(
    const Vector3d& xyz_in_f,
    Matrix<double,2,6>& J)
    {
        const double x = xyz_in_f[0];
        const double y = xyz_in_f[1];
        const double z_inv = 1./xyz_in_f[2];
        const double z_inv_2 = z_inv*z_inv;

        J(0,0) = -z_inv;                                        // -1/z
        J(0,1) = 0.0;                                           // 0
        J(0,2) = x*z_inv_2;                                     // x/z^2
        J(0,3) = y*J(0,2);                                      // x*y/z^2
        J(0,4) = -(1.0 + x*J(0,2));                             // -(1.0 + x^2/z^2)
        J(0,5) = y*z_inv;                                       // y/z

        J(1,0) = 0.0;                                           // 0
        J(1,1) = -z_inv;                                        // -1/z
        J(1,2) = y*z_inv_2;                                     // y/z^2
        J(1,3) = 1.0 + y*J(1,2);                                // 1.0 + y^2/z^2
        J(1,4) = -J(0,3);                                       // -x*y/z^2
        J(1,5) = -x*z_inv;                                      // x/z
    }
};


/// Some helper functions for the frame object.
namespace frame_utils {

/// Creates an image pyramid of half-sampled images.
void createImgPyramid(const cv::Mat& img_level_0, int n_levels, ImgPyr& pyr);

/// Get the average depth of the features in the image.
bool getSceneDepth(const Frame& frame, double& depth_mean, double& depth_min);

}                                                           // namespace frame_utils
}                                                           // namespace svo

#endif                                                      // SVO_FRAME_H_
