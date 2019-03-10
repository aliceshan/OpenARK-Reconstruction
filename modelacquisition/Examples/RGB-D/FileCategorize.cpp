#include <iostream>
#include <set>
#include <opencv2/opencv.hpp>
#include <sys/types.h>
#include <sys/stat.h>

using namespace std;

string folderPath, depthPath, tcwPath;
float blockSize;
int width_, height_;

set<string>blocks;

struct stat info;



string convert(cv::Mat point) {
	string x = to_string((int)(floor(point.at<float>(0) / blockSize) * blockSize));
	string y = to_string((int)(floor(point.at<float>(1) / blockSize) * blockSize));
	string z = to_string((int)(floor(point.at<float>(2) / blockSize) * blockSize));
	return x + "_" + y + "_" + z;
}

void categorize(cv::Mat depthMat, cv::Mat projectionMat, cv::Mat cameraPos) {


	for (int i = 0; i < depthMat.rows; ++i) {
		for (int j = 0; j < depthMat.cols; ++j) {
			if (depthMat.at<float>(i,j) < 0.0001) {
				continue;
			}

			float imageX = (j - width_ / 2) / (width_ * 0.5f);
			float imageY = (height_ / 2 - i) / (height_ * 0.5f);
			float imageZ = 1;


			cv::Vec3f projectedVector(imageX, imageY, imageZ);

			cv::Mat projectedPoint = projectionMat.colRange(0, 3) * cv::Mat(projectedVector);

			projectedPoint = (projectedPoint - cameraPos) * depthMat.at<float>(i,j);

			//cout << projectedPoint << endl;

			string insideBlock = convert(projectedPoint);

			blocks.insert(insideBlock);


			//cout << insideBlock << endl;
		}
	}
	
}



int main(int argc, char **argv) {
	if (argc != 4) {
        cerr << endl << "Usage: ./FileCategorize path_to_frames size_of_block(m) settings_file" << endl;
        return 1;
    }

    folderPath = argv[1];
    depthPath = folderPath + "depth/";
    tcwPath = folderPath + "tcw/";


    if(stat(folderPath.c_str(), &info) != 0){
    	cout << "what is this" << endl;
    }
    else if( info.st_mode & S_IFDIR ){
        cout << folderPath << " is a directory" << endl;
    }
    else{
        cout << folderPath << " is no directory" << endl;
    }

    blockSize = atof(argv[2]);
    string strSettingsFile = argv[3];

    cv::FileStorage fSettings(strSettingsFile, cv::FileStorage::READ);

    float fx_ = fSettings["Camera.fx"];
    float fy_ = fSettings["Camera.fy"];
    float cx_ = fSettings["Camera.cx"];
    float cy_ = fSettings["Camera.cy"];
    width_ = fSettings["Camera.width"];
    height_ = fSettings["Camera.height"];



    float Karr[3][3] = {{fx_, 0, cx_}, {0, fy_, cy_}, {0, 0, 1}};
    cv::Mat K(3, 3, CV_32F, Karr);

    int empty = 0;
    int frame = 1;
    cv::Mat depthMat;
    cv::Mat tcwMat;


    while (true) {
    	depthMat = cv::imread(depthPath + to_string(frame) + ".png", -1);
    	depthMat.convertTo(depthMat, CV_32FC1);
        depthMat *= 0.001;


/*
        for (int i = 15; i < 16; ++i) {
			for (int j = 100; j < 200; ++j) {
				cout << depthMat.at<float>(i,j) << endl;
			}
		}
*/


    	frame++;
    	if (depthMat.rows == 0) {
    		cout << "no image found at frame_id: " << frame - 1<< endl;
    		empty++;
    		if (empty > 5) {
    			break;
    		}
    		continue;
    	} 

    	cout << "calculating frame: " << frame - 1 << endl;


    	cv::FileStorage fs2(tcwPath + to_string(frame) + ".xml", cv::FileStorage::READ);
    	fs2["tcw"] >> tcwMat;

    	cv::Mat projectionMat;
    	cv::Mat projectionMatOne = (K * tcwMat.rowRange(0,3));
    	cv::Mat projectionMatInv = projectionMatOne.colRange(0,3).inv();
    	cv::hconcat(projectionMatInv, projectionMatOne.col(3), projectionMat);

    	cv::Mat cameraPos = tcwMat.rowRange(0,3).colRange(0,3).t()*-1.f*tcwMat.rowRange(0,3).col(3);


    	empty = 0;


    	/*cv::namedWindow("Display window", cv::WINDOW_AUTOSIZE);
    	cv::imshow("Display window", depthMat);

    	cv::waitKey(0);*/


    	categorize(depthMat, projectionMat, cameraPos);


    	
    }
    for (string block : blocks) {
    	cout << block << endl;
    }

}