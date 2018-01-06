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
#include <svo/frame_handler_mono.h>
#include <svo/map.h>
#include <svo/frame.h>
#include <vector>
#include <string>
#include <vikit/math_utils.h>
#include <vikit/vision.h>
#include <vikit/abstract_camera.h>
#include <vikit/atan_camera.h>
#include <vikit/pinhole_camera.h>
#include <opencv2/opencv.hpp>
#include <sophus/se3.h>
#include <iostream>
#include "test_utils.h"
#include "svo/viewer.h"
#include <thread>
#include "svo/map_drawer.h"
#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <cstdlib>
#include <fstream>
#include <assert.h>

namespace svo {

class BenchmarkNode
{
    vk::AbstractCamera* cam_;
    svo::FrameHandlerMono* vo_;
public:
    const svo::Map* Map() {return vo_->map();}
    BenchmarkNode();
    ~BenchmarkNode();
    void runFromFolder();
    void ReadEurocDataset();
};

BenchmarkNode::BenchmarkNode()
{
    /// 相机构造函数，生成针孔相机模型
    //cam_ = new vk::PinholeCamera(752, 480, 315.5, 315.5, 376.0, 240.0);

    cam_ =  new vk::PinholeCamera(752, 480, 458.654, 457.296, 367.215, 248.375);
    MapDrawer *map_drawer = new svo::MapDrawer(this->Map());

    // 构建vo系统，使用相机的指针初始化
    vo_ = new svo::FrameHandlerMono(cam_,  map_drawer);
    /// vo系统开始运行， 设置set_start_为true
    vo_->start();

    Viewer *viewer = new svo::Viewer(map_drawer);
    std::thread *mptviewer = new std::thread(&svo::Viewer::Run, viewer);
}

BenchmarkNode::~BenchmarkNode()
{
    delete vo_;
    delete cam_;
}

void BenchmarkNode::runFromFolder()
{
    //读取文件夹里面的图像，调用VO进行跟踪
    for(int img_id = 2; img_id < 188; ++img_id)
    {
        // load image
        std::stringstream ss;
        ss << svo::test_utils::getDatasetDir() << "/sin2_tex2_h1_v8_d/img/frame_"
        << std::setw( 6 ) << std::setfill( '0' ) << img_id << "_0.png";
        if(img_id == 2)
        std::cout << "reading image " << ss.str() << std::endl;

        /// 读取一帧图像
        cv::Mat img(cv::imread(ss.str().c_str(), 0));
        assert(!img.empty());

        cv::imshow("Image",  img);
        cv::waitKey(1);

        // process frame，处理图像
        vo_->addImage(img, 0.01*img_id);

        // display tracking quality
        // 跟踪质量包括跟踪的特征点的数目与跟踪用的时间
        if(vo_->lastFrame() != NULL)
        {
            std::cout << "Frame-Id: " << vo_->lastFrame()->id_ << " \t"
            << "#Features: " << vo_->lastNumObservations() << " \t"
            << "Proc. Time: " << vo_->lastProcessingTime()*1000 << "ms \n";

        // access the pose of the camera via vo_->lastFrame()->T_f_w_.
        }
    }
}

void BenchmarkNode::ReadEurocDataset()
{
    std::stringstream ss;
    //ss << svo::test_utils::getDatasetDir();
    ss<<"/home/lumino/Downloads";
    std::string path = ss.str() + "/mav0";
    //std::cout <<  path << std::endl;
    // read all imu data
    std::ifstream imu_file(path + "/imu0/data.csv");

    if (!imu_file.good())
    {
        std::cout <<  "no imu file found at" <<  path + "/imu0/data.csv";
        return;
    }

    int num_imu_data = 0;
    std::string line;
    while (std::getline(imu_file, line))
        num_imu_data++;

    std::cout <<  "IMU measurement number is " <<  num_imu_data - 1 <<  std::endl;

    if (num_imu_data <= 1)
    {
        std::cout <<  "no measurement data in " <<  path + "/imu0/data.csv";
        return;
    }
    // file pointer to file start
    imu_file.clear();
    imu_file.seekg(0, std::ios::beg);
    std::getline(imu_file, line);

    //
    std::vector<std::string> images_names;

    std::string folder(path + "/cam" + std::to_string(0) + "/data");

    int num_camera_images = 0;

    for (auto it = boost::filesystem::directory_iterator(folder);
        it !=  boost::filesystem::directory_iterator(); it++)
    {
            // elimate directories
            if (!boost::filesystem::is_directory(it->path()))
            {
                num_camera_images++;
                images_names.push_back(it->path().filename().string());
            }
            else
                continue;
    }

    assert(num_camera_images !=  0);

    // sort all the images
    std::sort(images_names.begin(),  images_names.end());

    std::vector<std::string>::iterator image_iterator;

    image_iterator = images_names.begin();

    std::string start_time_str_ns = image_iterator->substr(image_iterator->size() - 13, 9);
    std::string start_time_str_s = image_iterator->substr(0, image_iterator->size() - 13);
    long start_time_ns = std::stoi(start_time_str_ns) + std::stoi(start_time_str_s) * 1e9;

    size_t imu_index = 0;

    for (int i = 0; i < num_camera_images; i++)
    {
        cv::Mat filtered = cv::imread(
            path + "/cam" + std::to_string(0) + "/data/" + *image_iterator, cv::IMREAD_GRAYSCALE);

        std::string timestamp_str_ns = image_iterator->substr(image_iterator->size()- 13,  9);
        std::string timestamp_str_s = image_iterator->substr(0,  image_iterator->size() - 13);

        long timestamp_ns = std::stoi(timestamp_str_ns) + std::stoi(timestamp_str_s) * 1e9;
        timestamp_ns = timestamp_ns - start_time_ns;

        // the unit is us

        // read all imu data till now
        long last_imu_ts;

        long imu_timestamp_ns;
        do
        {
            // read imu data
            if (!std::getline(imu_file, line))
            {
                std::cout <<  std::endl << "Finished. Press any key to exit" <<  std::endl <<  std::flush;
                cv::waitKey();
                return;
            }

            std::stringstream stream(line);
            std::string imu_time_str;
            std::getline(stream, imu_time_str, ',');
            //std::cout <<  imu_time_str << std::endl;

            std::string imu_time_str_ns = imu_time_str.substr(imu_time_str.size() - 9,  9);
            std::string imu_time_str_s = imu_time_str.substr(0,  imu_time_str.size() - 9);

            imu_timestamp_ns = std::stoi(imu_time_str_ns) + std::stoi(imu_time_str_s) * 1e9;
            imu_timestamp_ns = imu_timestamp_ns - start_time_ns;


            //std::cout << imu_timestamp_ns <<  std::endl;

            Eigen::Vector3d gyro;
            std::string s;
            for (int j = 0; j < 3; j++)
            {
                std::getline(stream,  s,  ',');
                gyro[j] = std::stof(s);
                //std::cout <<  gyro[j] <<  " ";
            }
            //std::cout <<  std::endl;

            Eigen::Vector3d acc;
            for (int j = 0; j < 3; j++)
            {
                std::getline(stream, s, ',');
                acc[j] =  std::stof(s);
                //std::cout <<  acc[j] <<  " ";
            }

            //std::cout <<  std::endl;
            if (imu_index != 0)
            {
                vo_->addIMU(acc, gyro, imu_timestamp_ns, 1e-9 * (imu_timestamp_ns - last_imu_ts) );
            }
            imu_index++;

            last_imu_ts = imu_timestamp_ns;

        }while (imu_timestamp_ns <= timestamp_ns);

        //std::cout << images_names[i] <<  std::endl;
        //std::cout <<  timestamp_ns <<  std::endl;

        image_iterator++;

        imshow("image0",  filtered);
        cv::waitKey(1);

        vo_->addImage(filtered, timestamp_ns);

        if(vo_->lastFrame() != NULL)
        {
            std::cout << "Frame-Id: " << vo_->lastFrame()->id_ << " \t"
            << "#Features: " << vo_->lastNumObservations() << " \t"
            << "Proc. Time: " << vo_->lastProcessingTime()*1000 << "ms \n";

        // access the pose of the camera via vo_->lastFrame()->T_f_w_.
        }
    }
}                                                           // namespace svo
}

int main(int argc, char** argv)
{
    {
        /// benchmarkNode构造函数
        svo::BenchmarkNode benchmark;

        /// 从目录中读取图像，并进行跟踪
        benchmark.ReadEurocDataset();
    }

    printf("DataSet finished.\n");
    return 0;
}

