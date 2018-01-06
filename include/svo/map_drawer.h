/**
 * This file is part of SVO.
 * 
 * Copyright 2016 Technical University of Munich and Intel.
 * Developed by Jakob Engel <engelj at in dot tum dot de>,
 * for more information see <http://vision.in.tum.de/dso>.
 * If you use this code, please cite the respective publications as
 * listed on the above website.
 *
 * DSO is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * DSO is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with SVO. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef MAP_DRAWER_H
#define MAP_DRAWER_H
#include <svo/frame.h>
#include "svo/map.h"
#include "svo/point.h"
#include "svo/feature.h"
# include <pangolin/pangolin.h>
# include <mutex>

namespace svo
{
    class MapDrawer
    {
    public:
        MapDrawer(const Map* pMap);
        
        MapDrawer(const Map* pMap, const string &strSettingPath);

        const svo::Map* mpMap;

        void DrawMapPoints();
        
        void DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph);
        void DrawCurrentCamera(pangolin::OpenGlMatrix &Twc);
        void SetCurrentCameraPose(const cv::Mat &Tcw);
        //void SetReferenceKeyFrame(KeyFrame *pKF);
        void GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M);
       
    private:

        float mKeyFrameSize;
        float mKeyFrameLineWidth;
        float mGraphLineWidth;
        float mPointSize;
        float mCameraSize;
        float mCameraLineWidth;

        cv::Mat mCameraPose;

        std::mutex mMutexCamera;
    };

} 

#endif