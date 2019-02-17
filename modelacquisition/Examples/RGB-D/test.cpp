#include <iostream>
#include <algorithm>
#include <thread>

#include <GL/glew.h>
#include <GL/glut.h>

#include <opencv2/opencv.hpp>

#include <ORBSLAMSystem.h>
#include <PointCloudGenerator.h>

#include <libfreenect/libfreenect.h>
#include <libfreenect/libfreenect_sync.h>


using namespace std;
ark::PointCloudGenerator *pointCloudGenerator;
ark::ORBSLAMSystem *slam;
	
void freenect_sync_get_depth_cv(cv::Mat &imD, int index)
{
    imD = cv::Mat::zeros(480,640,CV_16UC1);
    static char *data = nullptr;
    unsigned int timestamp;
    if (freenect_sync_get_depth((void**)&data, &timestamp, index, FREENECT_DEPTH_REGISTERED))
        return;
    memcpy(imD.data,data,640*480*2);
    return;
}

void freenect_sync_get_rgb_cv(cv::Mat &imRGB, int index)
{
    imRGB = cv::Mat::zeros(480,640,CV_8UC3);
    static char *data = nullptr;
    unsigned int timestamp;
    if (freenect_sync_get_video((void**)&data, &timestamp, index, FREENECT_VIDEO_RGB))
        return;
    memcpy(imRGB.data,data,640*480*3);
    return;
}

void thread1(){

	slam->Start();
    pointCloudGenerator->Start();

	int tframe = 1;
	int counter = 0;
	while (!slam->IsRunning()){

		cv::Mat imRGB;
		cv::Mat imD;
		freenect_sync_get_rgb_cv(imRGB, 0);
		freenect_sync_get_depth_cv(imD, 0);

		slam->PushFrame(imRGB,imD,tframe);
		//std::cout << pointCloudGenerator->GetPly() << std::endl;
		counter++;

		if (counter == 200){
			slam->RequestStop();
            pointCloudGenerator->RequestStop();
			pointCloudGenerator->SavePly("model.ply");
			break;
		}

	}
}

int main(int argc, char **argv){
	
	pointCloudGenerator = new ark::PointCloudGenerator(argv[2]);
	slam = new ark::ORBSLAMSystem(argv[1], argv[2], ark::ORBSLAMSystem::RGBD, true);


	slam->AddKeyFrameAvailableHandler([pointCloudGenerator](const ark::RGBDFrame &keyFrame) {
        return pointCloudGenerator->OnKeyFrameAvailable(keyFrame);
    }, "PointCloudFusion");


	thread1();

}
