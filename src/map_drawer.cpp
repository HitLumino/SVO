/**
 * This file is part of SVO.
 *
 * Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
 * For more information see <https://github.com/raulmur/ORB_SLAM2>
 *
 * SVO is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * SVO is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with SVO. If not, see <http://www.gnu.org/licenses/>.
 */

#include "svo/map_drawer.h"
#include <mutex>

namespace svo
{
    MapDrawer::MapDrawer(const Map* pMap):mpMap(pMap)
    {
        mPointSize = 2;
        mCameraSize = 0.08;
        mKeyFrameSize = 0.05;
        mCameraLineWidth = 3; 
    }
    
    MapDrawer::MapDrawer(const Map* pMap, const string &strSettingPath):
        mpMap(pMap)
    {
        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

        mKeyFrameSize = fSettings["Viewer.KeyFrameSize"];
        mKeyFrameLineWidth = fSettings["Viewer.KeyFrameLineWidth"];
        mGraphLineWidth = fSettings["Viewer.GraphLineWidth"];
        mPointSize = 2;                                      //fSettings["Viewer.PointSize"];
        mCameraSize = fSettings["Viewer.CameraSize"];
        mCameraLineWidth = fSettings["Viewer.CameraLineWidth"];
    }

    void MapDrawer::DrawMapPoints()
    {
        //  get all map points from the map
        list<Point*> all_map_points;
        
        /*
        list<Sophus::SE3> all_keyframes = mpMap->allKeyFramePose();
        
        for(auto frame_it = all_keyframes.begin(), ite = all_keyframes.end(); frame_it !=ite; ++frame_it)
        {
            std::cout <<  "Here OK" <<  std::endl;
            FramePtr frame_ptr = *frame_it;
            std::cout <<  "Here OK" <<  std::endl;
            for (auto feature_it = frame_ptr->fts_.begin(), feature_ite = frame_ptr->fts_.end(); 
                feature_it != feature_ite; ++feature_it)
            {
                Feature* ft = *feature_it;
                all_map_points.push_back(ft->point);
            }
        }
    
        
        //const vector<MapPoint*> &vpRefMPs = mpMap->GetReferenceMapPoints();

        //set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

        if(all_map_points.empty())
            return;

        glPointSize(mPointSize);
        glBegin(GL_POINTS);
        glColor3f(0.0,0.0,0.0);
        
        for(auto point_it = all_map_points.begin(), ite = all_map_points.end(); point_it !=ite; ++point_it)
        {
            Point* p = (*point_it);
            glVertex3f(p->pos_(0), p->pos_(1), p->pos_(2));
        }
        
        glEnd();
        
        */
        
        //std::cout <<  "end " <<  std::endl;
        
        /*
        glPointSize(mPointSize);
        glBegin(GL_POINTS);
        glColor3f(1.0,0.0,0.0);

        for(set<MapPoint*>::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++)
        {
            if((*sit)->isBad())
            continue;
            cv::Mat pos = (*sit)->GetWorldPos();
            glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));

        }

        glEnd();
        */
    }
    
    void MapDrawer::DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph)
    {
        const float &w = mKeyFrameSize;
        const float h = w*0.75;
        const float z = w*0.6;

        if(bDrawKF)
        {
            list<Sophus::SE3> all_keyframesPose;
            mpMap->allKeyFramePose(all_keyframesPose);
            
            std::cout <<  "Here OK" <<  std::endl;
            
            for(auto pose_it = all_keyframesPose.begin(), ite = all_keyframesPose.end();
                pose_it !=ite; ++pose_it)
            {
                // draw all keyframe 
                cv::Mat Twc = cv::Mat::zeros(4, 4, CV_32FC1);
                std::cout <<  "Here ok" <<  std::endl;
                Eigen::Matrix3d R = (*pose_it).rotation_matrix();
                
                Twc.at<float>(0, 0) = R(0, 0);
                Twc.at<float>(1, 0) = R(0, 1);
                Twc.at<float>(2, 0) = R(0, 2);
                Twc.at<float>(0, 1) = R(1, 0);
                Twc.at<float>(1, 1) = R(1, 1);
                Twc.at<float>(2, 1) = R(1, 2);
                Twc.at<float>(0, 2) = R(2, 0);
                Twc.at<float>(1, 2) = R(2, 1);
                Twc.at<float>(2, 2) = R(2, 2);
                Eigen::Vector3d t = (*pose_it).translation();
                Twc.at<float>(0, 3) = t(0);
                Twc.at<float>(1, 3) = t(1);
                Twc.at<float>(2, 3) = t(2);
                Twc.at<float>(3, 3) = 1;
                
                glPushMatrix();

                glMultMatrixf(Twc.ptr<GLfloat>(0));

                glLineWidth(mKeyFrameLineWidth);
                glColor3f(0.0f,0.0f,1.0f);
                glBegin(GL_LINES);
                glVertex3f(0,0,0);
                glVertex3f(w,h,z);
                glVertex3f(0,0,0);
                glVertex3f(w,-h,z);
                glVertex3f(0,0,0);
                glVertex3f(-w,-h,z);
                glVertex3f(0,0,0);
                glVertex3f(-w,h,z);

                glVertex3f(w,h,z);
                glVertex3f(w,-h,z);

                glVertex3f(-w,h,z);
                glVertex3f(-w,-h,z);

                glVertex3f(-w,h,z);
                glVertex3f(w,h,z);

                glVertex3f(-w,-h,z);
                glVertex3f(w,-h,z);
                glEnd();

                glPopMatrix();
            }
        }

        
        /*
        if(bDrawGraph)
        {
            glLineWidth(mGraphLineWidth);
            glColor4f(0.0f,1.0f,0.0f,0.6f);
            glBegin(GL_LINES);

            for(size_t i=0; i<vpKFs.size(); i++)
            {
                // Covisibility Graph
                const vector<KeyFrame*> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);
                cv::Mat Ow = vpKFs[i]->GetCameraCenter();
                if(!vCovKFs.empty())
                {
                    for(vector<KeyFrame*>::const_iterator vit=vCovKFs.begin(), vend=vCovKFs.end(); vit!=vend; vit++)
                    {
                        if((*vit)->mnId<vpKFs[i]->mnId)
                        continue;
                        cv::Mat Ow2 = (*vit)->GetCameraCenter();
                        glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                        glVertex3f(Ow2.at<float>(0),Ow2.at<float>(1),Ow2.at<float>(2));
                    }
                }

                // Spanning tree
                KeyFrame* pParent = vpKFs[i]->GetParent();
                if(pParent)
                {
                    cv::Mat Owp = pParent->GetCameraCenter();
                    glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                    glVertex3f(Owp.at<float>(0),Owp.at<float>(1),Owp.at<float>(2));
                }

                // Loops
                set<KeyFrame*> sLoopKFs = vpKFs[i]->GetLoopEdges();
                for(set<KeyFrame*>::iterator sit=sLoopKFs.begin(), send=sLoopKFs.end(); sit!=send; sit++)
                {
                    if((*sit)->mnId<vpKFs[i]->mnId)
                    continue;
                    cv::Mat Owl = (*sit)->GetCameraCenter();
                    glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                    glVertex3f(Owl.at<float>(0),Owl.at<float>(1),Owl.at<float>(2));
                }
            }

            glEnd();
        }
        */
    }
    
    
    void MapDrawer::DrawCurrentCamera(pangolin::OpenGlMatrix &Twc)
    {
        const float &w = mCameraSize;
        const float h = w*0.75;
        const float z = w*0.6;

        glPushMatrix();

#ifdef HAVE_GLES
        glMultMatrixf(Twc.m);
#else
        glMultMatrixd(Twc.m);
#endif

        glLineWidth(mCameraLineWidth);
        glColor3f(0.0f, 1.0f, 0.0f);
        glBegin(GL_LINES);
        glVertex3f(0,0,0);
        glVertex3f(w,h,z);
        glVertex3f(0,0,0);
        glVertex3f(w,-h,z);
        glVertex3f(0,0,0);
        glVertex3f(-w,-h,z);
        glVertex3f(0,0,0);
        glVertex3f(-w,h,z);

        glVertex3f(w,h,z);
        glVertex3f(w,-h,z);

        glVertex3f(-w,h,z);
        glVertex3f(-w,-h,z);

        glVertex3f(-w,h,z);
        glVertex3f(w,h,z);

        glVertex3f(-w,-h,z);
        glVertex3f(w,-h,z);
        glEnd();

        glPopMatrix();
    }

    void MapDrawer::SetCurrentCameraPose(const cv::Mat &Tcw)
    {
        std::unique_lock<std::mutex> lock(mMutexCamera);
        mCameraPose = Tcw.clone();
    }

    void MapDrawer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M)
    {
        if(!mCameraPose.empty())
        {
            cv::Mat Rwc(3,3,CV_32F);
            cv::Mat twc(3,1,CV_32F);
            {
                std::unique_lock<std::mutex> lock(mMutexCamera);
                Rwc = mCameraPose.rowRange(0,3).colRange(0,3).t();
                twc = -Rwc*mCameraPose.rowRange(0,3).col(3);
                
                //std::cout <<  mCameraPose << std::endl;
            }

            M.m[0] = Rwc.at<float>(0,0);
            M.m[1] = Rwc.at<float>(1,0);
            M.m[2] = Rwc.at<float>(2,0);
            M.m[3]  = 0.0;

            M.m[4] = Rwc.at<float>(0,1);
            M.m[5] = Rwc.at<float>(1,1);
            M.m[6] = Rwc.at<float>(2,1);
            M.m[7]  = 0.0;

            M.m[8] = Rwc.at<float>(0,2);
            M.m[9] = Rwc.at<float>(1,2);
            M.m[10] = Rwc.at<float>(2,2);
            M.m[11]  = 0.0;

            M.m[12] = twc.at<float>(0);
            M.m[13] = twc.at<float>(1);
            M.m[14] = twc.at<float>(2);
            M.m[15]  = 1.0;
        }
        else
        M.SetIdentity();
    }
}                                                           //namespace ORB_SLAM

