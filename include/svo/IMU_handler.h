#ifndef IMU_HANDLER_H
#define IMU_HANDLER_H

#include <Eigen/Dense>
#include <svo/global.h>

namespace svo {
using namespace Eigen;
struct IMUNominalState
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Vector3d pos_;
    Vector3d vel_;
    Quaterniond attiude_quat_;
    Matrix3d attitude_mat_;
    Vector3d gyro_bias_;
    Vector3d acc_bias_;
    Vector3d gravity_;
    
    inline void reset()
    {
        pos_ <<  0, 0, 0;
        vel_ <<  0, 0, 0;
        attitude_mat_ <<  1, 0, 0, 
                          0, 1, 0, 
                          0, 0, 1;
        attiude_quat_ = Quaterniond( 1, 0, 0, 0);
        gyro_bias_ <<  0, 0, 0;
        acc_bias_ <<  0, 0, 0;
        gravity_ <<  9.8, 0, 0;
    }
};

struct ImuData 
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    ImuData();
    ImuData(Vector3d acc_data,  Vector3d gyro_data, 
      double time_stamp, double delta_t);
    
    Vector3d acc_data_;
    Vector3d gyro_data_;
    double time_stamp_;                                     // unit is s
    double delta_t_;                                        // unis is s
// 这个属性用来判断当前的IMU数据和图像数据的同步关系
};

class IMUhandler
{
public:
    IMUhandler();
    ~IMUhandler();

    //  IMU related variables
    IMUNominalState last_state_;
    IMUNominalState cur_state_;
    ImuData imu_data_;

    list<ImuData> imu_datas_;
    
    void reset();

    // 
    void addImuData(ImuData imu_data);

    // 
    void propagateNominalState(IMUNominalState& last_state, IMUNominalState& cur_state, 
        ImuData& imu_data);

    // 
    void updateCovarianceMatrix();

protected:
    Matrix3d crossMat(const Vector3d& vec);

    Matrix3d omageMat(const Vector3d& vec);

    const Quaterniond vec2quat(const Vector3d& vec);
};
}

#endif