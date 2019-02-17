#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
// #include <librealsense/rs.hpp>
#include <opencv2/core/core.hpp>
// #include <opencv2/ximgproc.hpp>

#include <ORBSLAMSystem.h>
#include <BridgeRSD435.h>

#include <SaveFrame.h>

using namespace cv;
using namespace std;

int main(int argc, char **argv) {
    if (argc != 3) {
        cerr << endl << "Usage: ./rgbd_realsense path_to_vocabulary path_to_settings" << endl;
        return 1;
    }


    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    // ark::PointCloudGenerator pointCloudGenerator(argv[2]);
    ark::SaveFrame saveFrame("./frames/");
    ark::ORBSLAMSystem slam(argv[1], argv[2], ark::ORBSLAMSystem::RGBD, true);
    BridgeRSD435 bridgeRSD435;

    // slam.AddKeyFrameAvailableHandler([&pointCloudGenerator](const ark::RGBDFrame& keyFrame){return pointCloudGenerator.OnKeyFrameAvailable(keyFrame);}, "PointCloudFusion");
    slam.AddKeyFrameAvailableHandler([&saveFrame](const ark::RGBDFrame& keyFrame){return saveFrame.frameWrite(keyFrame);}, "saveFrame");

    slam.Start();
//    saveFrame.Start();
    // pointCloudGenerator.Start();
    bridgeRSD435.Start();

    // Main loop
    int tframe = 1;

    while (!slam.IsRunning()) {
        cv::Mat imBGR, imD;
        cv::Mat imRGB;

        bridgeRSD435.GrabRGBDPair(imBGR, imD);
        // cv::cvtColor(imBGR, imRGB, cv::COLOR_BGR2RGB);
        imRGB = imBGR;
        // Pass the image to the SLAM system
        slam.PushFrame(imRGB, imD, tframe);

        // imshow("Depth", imD*5);

        // check map changed
        if (slam.MapChanged()) {
            std::cout << "map changed" << std::endl;
        }


        //stop the program
        // if(waitKey(1)>0)
        // {
        //     slam.RequestStop();
        //     // saveFrame.RequestStop();
        // }
    }

    // pointCloudGenerator.SavePly("tmp.ply");
//    slam.RequestStop();
    slam.ShutDown();
}