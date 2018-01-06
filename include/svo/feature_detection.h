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

#ifndef SVO_FEATURE_DETECTION_H_
#define SVO_FEATURE_DETECTION_H_

#include <svo/global.h>
#include <svo/frame.h>

namespace svo {

/// Implementation of various feature detectors.
namespace feature_detection {

/// Temporary container used for corner detection. Features are initialized from these.
/// 角点检测的临时容器， 所有的特征点根据该结构体进行初始化
/// 角点并不都是两条直线的交点，而是具有一定特征的点
struct Corner
{
  int x;        //!< x-coordinate of corner in the image.
  int y;        //!< y-coordinate of corner in the image.
  int level;    //!< pyramid level of the corner. 角点所在金字塔的层数
  float score;  //!< shi-tomasi score of the corner. 角点的shi-tomasi打分， 如果
  float angle;  //!< for gradient-features: dominant gradient angle. 对于梯度特征， 显著的梯度角度
  Corner(int x, int y, float score, int level, float angle) :
    x(x), y(y), level(level), score(score), angle(angle)
  {}
};
typedef vector<Corner> Corners;

/// All detectors should derive from this abstract class.
class AbstractDetector
{
public:
	
	/// Abstract Detector的构造函数
  AbstractDetector(
      const int img_width,
      const int img_height,
      const int cell_size,
      const int n_pyr_levels);

  /// AbstractDetector的析构函数
  virtual ~AbstractDetector() {};

  /// 检测特征点，检测的预制
  virtual void detect(
      Frame* frame,
      const ImgPyr& img_pyr,
      const double detection_threshold,
      Features& fts) = 0;

  /// Flag the grid cell as occupied
  /// 设置网格为占据的？？？
  void setGridOccpuancy(const Vector2d& px);

  /// 将存在特征的网格设置为占据的
  /// Set grid cells of existing features as occupied
  void setExistingFeatures(const Features& fts);

protected:

	/// 特征点应该在图像边缘的8个像素以内
  static const int border_ = 8; //!< no feature should be within 8px of border.
	/// 栅格的大小
  const int cell_size_;
  /// 金字塔的层数
  const int n_pyr_levels_;
  /// 栅格的列数
  const int grid_n_cols_;
  /// 栅格的行数
  const int grid_n_rows_;
  /// 栅格的占据情况
  vector<bool> grid_occupancy_;

  ///将栅格复位
  void resetGrid();
 /// 得到栅格的索引
  inline int getCellIndex(int x, int y, int level)
  {
	 /// 尺度为层数的倒数
    const int scale = (1<<level);
	///  在level层中栅格的索引
    return (scale*y)/cell_size_*grid_n_cols_ + (scale*x)/cell_size_;
  }
};
typedef boost::shared_ptr<AbstractDetector> DetectorPtr;

/// FAST detector by Edward Rosten.
class FastDetector : public AbstractDetector
{
public:
	
	/// Fast 检测方法初始化， 图像的宽度和图像的高度， cell_size的大小， 金字塔的层数
  FastDetector(
      const int img_width,
      const int img_height,
      const int cell_size,
      const int n_pyr_levels);

  /// Fast 析构函数
  virtual ~FastDetector() {}

  /// 
  virtual void detect(
      Frame* frame,
      const ImgPyr& img_pyr,
      const double detection_threshold,
      Features& fts);
};

} // namespace feature_detection
} // namespace svo

#endif // SVO_FEATURE_DETECTION_H_
