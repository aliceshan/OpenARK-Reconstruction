#ifndef ORB_SLAM2_BRIDGERSD435_H
#define ORB_SLAM2_BRIDGERSD435_H

#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <librealsense2/hpp/rs_pipeline.hpp>
#include <mutex>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>

#include <stdafx.h>
// #include <assert.h>
// #include "OpenARK/Version.h"

// // Boost
// #include <boost/filesystem.hpp>
// #include <boost/function.hpp>
// #include <boost/smart_ptr.hpp>
// #include <boost/bind.hpp>
// #include <boost/polygon/voronoi.hpp>
// #include <boost/polygon/point_data.hpp>
// #include <boost/polygon/segment_data.hpp>

// #include <vector>
// #include <map>

class BridgeRSD435{

    // // internal storage
    // std::shared_ptr<rs2::pipeline> pipe;
    // rs2::align align;
    // rs2::config config;

    // rs2::context mCtx;
    // rs2::device * mpDev; 
    // rs2_intrinsics mDepth_intrin;
    // rs2_extrinsics mDepth_to_color;
    // rs2_intrinsics mColor_intrin;
    
    // float mDepthScale;



        // internal storage
        std::shared_ptr<rs2::pipeline> pipe;
        rs2::align align;
        rs2::config config;
        rs2_stream align_to;
        rs2::pipeline_profile profile;

        // pointer to depth sensor intrinsics (RealSense C API: rs_intrinsics)
        void * depthIntrinsics = nullptr;
        // pointer to RGB/IR sensor intrinsics (RealSense C API: rs_intrinsics)
        void * rgbIntrinsics = nullptr;
        // pointer to depth-to-RGB extrinsics (RealSense C API: rs_extrinsics)
        void * d2rExtrinsics = nullptr;
        // pointer to RGB-to-depth extrinsics (RealSense C API: rs_extrinsics)
        void * r2dExtrinsics = nullptr;

        int width, height;
        bool badInputFlag;
        const int PREFERRED_FRAME_H = 480;
        const int PREFERRED_FRAME_W = 640;

public:
            double scale;

    BridgeRSD435();
    int Start();
    void GrabRGBDPair(cv::Mat& imRGB, cv::Mat& imD);
    void Stop();
    void query_intrinsics();

};

#endif //ORB_SLAM2_BRIDGERSD435_H
