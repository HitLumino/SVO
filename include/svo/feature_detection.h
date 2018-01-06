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
/// �ǵ������ʱ������ ���е���������ݸýṹ����г�ʼ��
/// �ǵ㲢����������ֱ�ߵĽ��㣬���Ǿ���һ�������ĵ�
struct Corner
{
  int x;        //!< x-coordinate of corner in the image.
  int y;        //!< y-coordinate of corner in the image.
  int level;    //!< pyramid level of the corner. �ǵ����ڽ������Ĳ���
  float score;  //!< shi-tomasi score of the corner. �ǵ��shi-tomasi��֣� ���
  float angle;  //!< for gradient-features: dominant gradient angle. �����ݶ������� �������ݶȽǶ�
  Corner(int x, int y, float score, int level, float angle) :
    x(x), y(y), level(level), score(score), angle(angle)
  {}
};
typedef vector<Corner> Corners;

/// All detectors should derive from this abstract class.
class AbstractDetector
{
public:
	
	/// Abstract Detector�Ĺ��캯��
  AbstractDetector(
      const int img_width,
      const int img_height,
      const int cell_size,
      const int n_pyr_levels);

  /// AbstractDetector����������
  virtual ~AbstractDetector() {};

  /// ��������㣬����Ԥ��
  virtual void detect(
      Frame* frame,
      const ImgPyr& img_pyr,
      const double detection_threshold,
      Features& fts) = 0;

  /// Flag the grid cell as occupied
  /// ��������Ϊռ�ݵģ�����
  void setGridOccpuancy(const Vector2d& px);

  /// ��������������������Ϊռ�ݵ�
  /// Set grid cells of existing features as occupied
  void setExistingFeatures(const Features& fts);

protected:

	/// ������Ӧ����ͼ���Ե��8����������
  static const int border_ = 8; //!< no feature should be within 8px of border.
	/// դ��Ĵ�С
  const int cell_size_;
  /// �������Ĳ���
  const int n_pyr_levels_;
  /// դ�������
  const int grid_n_cols_;
  /// դ�������
  const int grid_n_rows_;
  /// դ���ռ�����
  vector<bool> grid_occupancy_;

  ///��դ��λ
  void resetGrid();
 /// �õ�դ�������
  inline int getCellIndex(int x, int y, int level)
  {
	 /// �߶�Ϊ�����ĵ���
    const int scale = (1<<level);
	///  ��level����դ�������
    return (scale*y)/cell_size_*grid_n_cols_ + (scale*x)/cell_size_;
  }
};
typedef boost::shared_ptr<AbstractDetector> DetectorPtr;

/// FAST detector by Edward Rosten.
class FastDetector : public AbstractDetector
{
public:
	
	/// Fast ��ⷽ����ʼ���� ͼ��Ŀ�Ⱥ�ͼ��ĸ߶ȣ� cell_size�Ĵ�С�� �������Ĳ���
  FastDetector(
      const int img_width,
      const int img_height,
      const int cell_size,
      const int n_pyr_levels);

  /// Fast ��������
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
