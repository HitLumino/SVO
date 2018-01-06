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

#include <svo/config.h>
#include <svo/frame.h>
#include <svo/point.h>
#include <svo/feature.h>
#include <svo/initialization.h>
#include <svo/feature_detection.h>
#include <vikit/math_utils.h>
#include <vikit/homography.h>

#include <opencv2/calib3d/calib3d.hpp>
#include <boost/concept_check.hpp>

namespace svo {
namespace initialization {

/// 初始化增加第一帧图像，如果特征点的数目小于100，则添加失败
InitResult KltHomographyInit::addFirstFrame(FramePtr frame_ref)
{
    reset();
    // 特征点的坐标为原始图像（金字塔第0层）的像素坐标
    detectFeatures(frame_ref, px_ref_, f_ref_);
    if(px_ref_.size() < 100)
    {
        SVO_WARN_STREAM_THROTTLE(2.0, "First image has less than 100 features. Retry in more textured environment.");
        return FAILURE;
    }
    frame_ref_ = frame_ref;
    px_cur_.insert(px_cur_.begin(), px_ref_.begin(), px_ref_.end());
    return SUCCESS;
}

InitResult KltHomographyInit::addSecondFrame(FramePtr frame_cur)
{
    /* KLT跟踪算法
     * http://blog.sina.com.cn/s/blog_4f3a049701014w91.html
     */
    // 第二帧图像并没有提取特征点
    trackKlt(frame_ref_, frame_cur, px_ref_, px_cur_, f_ref_, f_cur_, disparities_);
    SVO_INFO_STREAM("Init: KLT tracked "<< disparities_.size() <<" features");

    if(disparities_.size() < Config::initMinTracked())
    return FAILURE;

    double disparity = vk::getMedian(disparities_);
    SVO_INFO_STREAM("Init: KLT " << disparity << "px average disparity.");
    if(disparity < Config::initMinDisparity())
    return NO_KEYFRAME;

    // 根据光流跟踪的结果计算单应性矩阵
    computeHomography(
        f_ref_, f_cur_, 
        frame_ref_->cam_->errorMultiplier2(), Config::poseOptimThresh(),
        inliers_, xyz_in_cur_, T_cur_from_ref_);
        
    SVO_INFO_STREAM("Init: Homography RANSAC " << inliers_.size() << " inliers.");

    if(inliers_.size() < Config::initMinInliers())
    {
        SVO_WARN_STREAM("Init WARNING: " << Config::initMinInliers() << " inliers minimum required.");
        return FAILURE;
    }

    // Rescale the map such that the mean scene depth is equal to the specified scale
    vector<double> depth_vec;
    for(size_t i=0; i<xyz_in_cur_.size(); ++i)
    depth_vec.push_back((xyz_in_cur_[i]).z());

    double scene_depth_median = vk::getMedian(depth_vec);
    double scale = Config::mapScale()/scene_depth_median;

    // 当前坐标系的位姿
    // quaternion: [0, 0, 0, 1],  translation:[0, 0, 0]
    // 左乘矩阵：说明为点坐标变换矩阵
    frame_cur->T_f_w_ = T_cur_from_ref_ * frame_ref_->T_f_w_;

    // 设置slam的尺度
    frame_cur->T_f_w_.translation() =
    -frame_cur->T_f_w_.rotation_matrix()*(frame_ref_->pos() + scale*(frame_cur->pos() - frame_ref_->pos()));

    // For each inlier create 3D point and add feature in both frames
    SE3 T_world_cur = frame_cur->T_f_w_.inverse();

    for(vector<int>::iterator it=inliers_.begin(); it!=inliers_.end(); ++it)
    {
        Vector2d px_cur(px_cur_[*it].x, px_cur_[*it].y);
        Vector2d px_ref(px_ref_[*it].x, px_ref_[*it].y);

        if(frame_ref_->cam_->isInFrame(px_cur.cast<int>(), 10) && frame_ref_->cam_->isInFrame(px_ref.cast<int>(), 10) && xyz_in_cur_[*it].z() > 0)
        {
            Vector3d pos = T_world_cur * (xyz_in_cur_[*it]*scale);
            Point* new_point = new Point(pos);

            Feature* ftr_cur(new Feature(frame_cur.get(), new_point, px_cur, f_cur_[*it], 0));
            frame_cur->addFeature(ftr_cur);
            new_point->addFrameRef(ftr_cur);

            Feature* ftr_ref(new Feature(frame_ref_.get(), new_point, px_ref, f_ref_[*it], 0));
            frame_ref_->addFeature(ftr_ref);
            new_point->addFrameRef(ftr_ref);
        }
    }	

    return SUCCESS;
}

void KltHomographyInit::reset()
{
    px_cur_.clear();
    frame_ref_.reset();
}

void detectFeatures(
FramePtr frame,
vector<cv::Point2f>& px_vec,
vector<Vector3d>& f_vec)
{
	/// 检测fast角点
    Features new_features;
    feature_detection::FastDetector detector(
        frame->img().cols, frame->img().rows, Config::gridSize(), Config::nPyrLevels());

    detector.detect(frame.get(), frame->img_pyr_, Config::triangMinCornerScore(), new_features);

    // now for all maximum corners, initialize a new seed
    px_vec.clear(); px_vec.reserve(new_features.size());
    f_vec.clear(); f_vec.reserve(new_features.size());
    std::for_each(new_features.begin(), new_features.end(), [&](Feature* ftr){
        px_vec.push_back(cv::Point2f(ftr->px[0], ftr->px[1]));
        f_vec.push_back(ftr->f);
        delete ftr;
      });
}

void trackKlt(
FramePtr frame_ref,
FramePtr frame_cur,
vector<cv::Point2f>& px_ref,
vector<cv::Point2f>& px_cur,
vector<Vector3d>& f_ref,                                    // 传入ref只是为了移除没有跟踪的点的方向
vector<Vector3d>& f_cur,
vector<double>& disparities)
{
    const double klt_win_size = 30.0;
    const int klt_max_iter = 30;
    const double klt_eps = 0.001;
    vector<uchar> status;
    vector<float> error;
    vector<float> min_eig_vec;
    cv::TermCriteria termcrit(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, klt_max_iter, klt_eps);
    
    // 计算光流，根据参考帧和当前帧图像计算参考帧中点的光流
    // 输出为参考帧中的点在当前帧中的坐标
    // status表示是否计算出来了光流
    cv::calcOpticalFlowPyrLK(frame_ref->img_pyr_[0], frame_cur->img_pyr_[0],
        px_ref, px_cur,
        status, error,
        cv::Size2i(klt_win_size, klt_win_size),
        4, termcrit, cv::OPTFLOW_USE_INITIAL_FLOW);

        // px_ref为参考帧的坐标
        // px_cur为光流计算的在当前帧中的像素坐标
    vector<cv::Point2f>::iterator px_ref_it = px_ref.begin();
    vector<cv::Point2f>::iterator px_cur_it = px_cur.begin();
    
    vector<Vector3d>::iterator f_ref_it = f_ref.begin();
    f_cur.clear(); f_cur.reserve(px_cur.size());
    disparities.clear(); disparities.reserve(px_cur.size());
    
    for(size_t i=0; px_ref_it != px_ref.end(); ++i)
    {
        // 如果参考帧中的图像没有计算出来光流，则丢弃参考帧的点和当前帧的点
        if(!status[i])
        {
            px_ref_it = px_ref.erase(px_ref_it);
            px_cur_it = px_cur.erase(px_cur_it);
            f_ref_it = f_ref.erase(f_ref_it);
            continue;
        }
        // 计算当前帧中特征点在当前帧相机坐标系中的方向
        f_cur.push_back(frame_cur->c2f(px_cur_it->x, px_cur_it->y));
        // 计算光流法得到的视差
        disparities.push_back(Vector2d(px_ref_it->x - px_cur_it->x, px_ref_it->y - px_cur_it->y).norm());
        ++px_ref_it;
        ++px_cur_it;
        ++f_ref_it;
    }
}

void computeHomography(
const vector<Vector3d>& f_ref,
const vector<Vector3d>& f_cur,
double focal_length,
double reprojection_threshold,
vector<int>& inliers,
vector<Vector3d>& xyz_in_cur,
SE3& T_cur_from_ref)
{
    vector<Vector2d, aligned_allocator<Vector2d> > uv_ref(f_ref.size());
    vector<Vector2d, aligned_allocator<Vector2d> > uv_cur(f_cur.size());
    
    for(size_t i=0, i_max=f_ref.size(); i<i_max; ++i)
    {
        uv_ref[i] = vk::project2d(f_ref[i]);
        uv_cur[i] = vk::project2d(f_cur[i]);
    }
    
    vk::Homography Homography(uv_ref, uv_cur, focal_length, reprojection_threshold);
    
    Homography.computeSE3fromMatches();
    vector<int> outliers;
    vk::computeInliers(f_cur, f_ref,
        Homography.T_c2_from_c1.rotation_matrix(), Homography.T_c2_from_c1.translation(),
        reprojection_threshold, focal_length,
        xyz_in_cur, inliers, outliers);
    T_cur_from_ref = Homography.T_c2_from_c1;
}

void computeEssential(
const vector<Vector3d>& f_ref,
const vector<Vector3d>& f_cur,
double focal_length,
double reprojection_threshold,
vector<int>& inliers,
vector<Vector3d>& xyz_in_cur,
SE3& T_cur_from_ref)
{
    /*
    vector<cv::Point2f> points1(f_ref.size());
    vector<cv::Point2f> points2(f_cur.size());

    Vector2d uv_ref,  uv_cur;
    for(size_t i=0, i_max = f_ref.size(); i<i_max; ++i)
    {
        uv_ref = vk::project2d(f_ref[i]);
        points1[i] = cv::Point2f(uv_ref(0), uv_ref(1));
        
        uv_cur = vk::project2d(f_cur[i]);
        points2[i] = cv::Point2f(uv_cur(0), uv_cur(1));        
    }

    double focal = 1.0;
    cv::Point2d pp(0.0,  0.0);
    cv::Mat E, R, t, mask;
    
    E = cv::findEssentialMat(points1, points2, focal, pp, cv::RANSAC, 0.999, 1.0, mask);
    cv::recoverPose(E, points1, R, t, focal, pp, mask);
    
    Matrix3d rotation_matrix;
    rotation_matrix <<  R.at<float>(0, 0),  R.at<float>(0, 1),  R.at<float>(0, 2), 
                        R.at<float>(1, 0),  R.at<float>(1, 1),  R.at<float>(1, 2), 
                        R.at<float>(2, 0),  R.at<float>(2, 1),  R.at<float>(2, 2);
    
    Vector3d translation;
    translation <<  t.at<float>(0, 0),  t.at<float>(0, 1),  t.at<float>(0, 2);
    
    T_cur_from_ref = SE3(rotation_matrix, translation);
    */
    
}

}                                                           // namespace initialization
}                                                           // namespace svo
