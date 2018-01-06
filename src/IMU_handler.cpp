#include <svo/IMU_handler.h>
#include <boost/concept_check.hpp>

namespace svo {

ImuData::ImuData()
{
}

ImuData::ImuData(Vector3d acc_data,  Vector3d gyro_data, 
    double time_stamp, double delta_t):
    acc_data_(acc_data), 
    gyro_data_(gyro_data), 
    time_stamp_(time_stamp), 
    delta_t_(delta_t)
{

}



IMUhandler::IMUhandler()
{
}

IMUhandler::~IMUhandler()
{

}

void IMUhandler::reset()
{
    last_state_.reset();
    cur_state_.reset();
}

void IMUhandler::addImuData(ImuData imu_data)
{
    propagateNominalState(last_state_, cur_state_, imu_data);
    last_state_ = cur_state_;
}

void IMUhandler::propagateNominalState(IMUNominalState &last_state, IMUNominalState &cur_state, ImuData& imu_data)
{
    // 更新位置
    cur_state.pos_ = last_state.pos_ + last_state.vel_ * imu_data.delta_t_
    + 0.5 * (last_state.attitude_mat_ * (imu_data.acc_data_ - last_state.acc_bias_)
      + last_state.gravity_) * imu_data.delta_t_ * imu_data.delta_t_;
      
   // 更新速度
   cur_state.vel_ = last_state.vel_ + (last_state.attitude_mat_ * (imu_data.acc_data_ - 
    last_state.acc_bias_) + last_state.gravity_) * imu_data.delta_t_;
    
    //更新姿态
    cur_state.attiude_quat_ = last_state.attiude_quat_ * 
        vec2quat((imu_data.gyro_data_ - last_state.gyro_bias_) * imu_data.delta_t_); 
    
    // 
    cur_state.attitude_mat_ = cur_state.attiude_quat_.toRotationMatrix();
    
    // 更新加速度计零偏
    cur_state.acc_bias_ = last_state.acc_bias_;
    
    // update gyro bias 
    cur_state.gyro_bias_ = last_state.gyro_bias_;
    
    // update gravity 
    cur_state.gravity_ = last_state.gravity_;
    
    std::cout << "Pose attitude increments: " << std::endl <<  cur_state.attitude_mat_ << std::endl
    << "Pose position increments: " << std::endl <<  cur_state.pos_ <<  std::endl;
     
}

Matrix3d IMUhandler::crossMat(const Vector3d &vec)
{
    Matrix3d cross_mat;
    cross_mat <<  0,  -1.0 * vec(2),  vec(1), 
                 vec(2),  0,  -1.0 * vec(0), 
                 -1.0 * vec(1),  vec(0),  0;
                 
    return cross_mat;
}

Matrix3d IMUhandler::omageMat(const Vector3d &vec)
{
    Matrix3d omega_mat;
    omega_mat.block<3, 3>(0, 0) = -crossMat(vec);
    omega_mat.block<3, 1>(0, 0) = vec;
    omega_mat.block<1, 3>(3, 0) = -vec.transpose();
    omega_mat(3, 3) = 0;
    
    return omega_mat;
}

const Quaterniond IMUhandler::vec2quat(const Vector3d& vec)
{
    return Quaterniond(0, vec(0), vec(1), vec(2));
}

}