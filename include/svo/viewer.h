#ifndef VIEWER_H
# define VIEWER_H

//# include "FrameDrawer.h"
//# include "MapDrawer.h"
//# include "Tracking.h"
//# include "System.h"

# include <mutex>

namespace svo
{
    /*
    class Tracking;
    class FrameDrawer;
    class System;
    */
    class  MapDrawer;
    
    class Viewer
    {
    public:
        Viewer(MapDrawer* map_drawer);
        //Viewer(System* pSystem, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Tracking *pTracking, const string &strSettingPath);

        // Main thread function. Draw points, keyframes, the current camera pose and the last processed
        // frame. Drawing is refreshed according to the camera fps. We use Pangolin.
        void Run();

        void RequestFinish();

        void RequestStop();

        bool isFinished();

        bool isStopped();

        void Release();

    private:

        bool Stop();

        /*
        System* mpSystem;
        FrameDrawer* mpFrameDrawer;
        MapDrawer* mpMapDrawer;
        Tracking* mpTracker;
        */
       
        MapDrawer* map_drawer_;
        
        // 1/fps in ms
        double mT;
        float mImageWidth, mImageHeight;

        float mViewpointX, mViewpointY, mViewpointZ, mViewpointF;

        bool CheckFinish();
        void SetFinish();
        bool mbFinishRequested;
        bool mbFinished;
        std::mutex mMutexFinish;

        bool mbStopped;
        bool mbStopRequested;
        std::mutex mMutexStop;
    };
}


#endif                                                      // VIEWER_H