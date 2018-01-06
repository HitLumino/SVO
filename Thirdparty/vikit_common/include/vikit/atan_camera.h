/*
 * atan_camera.h
 *
 *  Created on: Aug 21, 2012
 *      Author: cforster
 *
 *  This class implements the FOV distortion model of Deverneay and Faugeras,
 *  Straight lines have to be straight, 2001.
 *
 *  The code is an implementation of the ATAN class in PTAM by Georg Klein using Eigen.
 */

#ifndef ATAN_CAMERA_H_
#define ATAN_CAMERA_H_

#include <stdlib.h>
#include <string>
#include <Eigen/Eigen>
#include <vikit/abstract_camera.h>
#include <vikit/math_utils.h>

namespace vk {

using namespace std;
using namespace Eigen;

/// ATAN相机模型
class ATANCamera : public AbstractCamera {

private:
  double fx_, fy_;                      //!< focal length相机的归一化焦距
  double fx_inv_, fy_inv_;              //!< inverse focal length焦距的倒数
  double cx_, cy_;                      //!< projection center相机归一化的投影中心
  double s_, s_inv_;                    //!< distortion model coeff该项表示公式中的omega和omega^{-1}
  double tans_;                         //!< distortion model coeff该项表示公式中的2 * tan(omega/2) 
  double tans_inv_;                     //!< distortion model coeff该项表示公式中的1/(2 * tan(omega/2))
  bool distortion_;                     //!< use distortion model?是否使用畸变模型

  //! Radial distortion transformation factor: returns ration of distorted / undistorted radius.
  //径向畸变转换因子： r_d / r_u，其中r_d = 1/omega * arctan(2 * r_u * tan(omega/2))
  inline double rtrans_factor(double r) const
  {
    if(r < 0.001 || s_ == 0.0)
      return 1.0;
    else
      return (s_inv_* atan(r * tans_) / r);
  };

  //! Inverse radial distortion: returns un-distorted radius from distorted.
  // 逆径向畸变 r_u = tan(r_d * omega) / (2 * tan(omega/2))
  inline double invrtrans(double r) const
  {
    if(s_ == 0.0)
      return r;
    return (tan(r * s_) * tans_inv_);
  };

public:

  ATANCamera(double width, double height, double fx, double fy, double dx, double dy, double s);

  ~ATANCamera();

  virtual Vector3d
  cam2world(const double& x, const double& y) const;

  virtual Vector3d
  cam2world(const Vector2d& px) const;

  virtual Vector2d
  world2cam(const Vector3d& xyz_c) const;

  virtual Vector2d
  world2cam(const Vector2d& uv) const;

  const Vector2d focal_length() const
  {
    return Vector2d(fx_, fy_);
  }

  virtual double errorMultiplier2() const
  {
    return fx_;
  }

  virtual double errorMultiplier() const
  {
    return 4*fx_*fy_;
  }
};

} // end namespace vk

#endif /* ATAN_CAMERA_H_ */
