#include <opencv2/opencv.hpp>
#include <libfreenect/libfreenect.h>
#include <libfreenect/libfreenect_sync.h>

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <opencv2/core/core.hpp>

#include <ORBSLAMSystem.h>
#include <PointCloudGenerator.h>

using namespace cv;
using namespace std;

void freenect_sync_get_depth_cv(Mat &imD, int index)
{
    imD = Mat::zeros(480,640,CV_16UC1);
    static char *data = nullptr;
    unsigned int timestamp;
    if (freenect_sync_get_depth((void**)&data, &timestamp, index, FREENECT_DEPTH_REGISTERED))
        return;
    memcpy(imD.data,data,640*480*2);
    return;
}

void freenect_sync_get_rgb_cv(Mat &imRGB, int index)
{
    imRGB = Mat::zeros(480,640,CV_8UC3);
    static char *data = nullptr;
    unsigned int timestamp;
    if (freenect_sync_get_video((void**)&data, &timestamp, index, FREENECT_VIDEO_RGB))
        return;
    memcpy(imRGB.data,data,640*480*3);
    return;
}


int main(int argc, char **argv)
{
    if (argc != 3) {
        cerr << endl << "Usage: ./rgbd_realsense path_to_vocabulary path_to_settings" << endl;
        return 1;
    }


    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ark::PointCloudGenerator pointCloudGenerator(argv[2]);
    ark::ORBSLAMSystem slam(argv[1], argv[2], ark::ORBSLAMSystem::RGBD, true);

    slam.AddKeyFrameAvailableHandler([&pointCloudGenerator](const ark::RGBDFrame& keyFrame){return pointCloudGenerator.OnKeyFrameAvailable(keyFrame);}, "PointCloudFusion");

    slam.Start();
    pointCloudGenerator.Start();

    // Main loop
    int tframe = 1;

    while (!slam.IsRunning()) {
        Mat imRGB;
        freenect_sync_get_rgb_cv(imRGB, 0);
        if (imRGB.cols==0) {
            printf("Error: Kinect not connected?\n");
            return -1;
        }
        Mat imD;
        freenect_sync_get_depth_cv(imD, 0);
        if (imD.cols==0) {
            printf("Error: Kinect not connected?\n");
            return -1;
        }

        imshow("Depth", imD*5);

        // Pass the image to the SLAM system
        slam.PushFrame(imRGB, imD, tframe);

        // check map changed
        if (slam.MapChanged()) {
            std::cout << "map changed" << std::endl;
        }

        if(waitKey(10)>=0)
        {
            slam.RequestStop();
            pointCloudGenerator.RequestStop();
        }
    }

    freenect_sync_stop();

    pointCloudGenerator.SavePly("tmp.ply");

    slam.ShutDown();
}
