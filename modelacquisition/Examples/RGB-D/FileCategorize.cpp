//r*(d*(K^-1)*<x,y,1>) + T


#include <iostream>
#include <set>
#include <opencv2/opencv.hpp>
#include <sys/types.h>
#include <sys/stat.h>

using namespace std;

string folderPath, rgbPath, depthPath, tcwPath;
float blockSize; //size of reconstruction section size (meters)
float maxAccurateDistance = 10; //based on camera limits to exclude noisy measurements (meters)
int width_, height_;


struct stat info;

void createFolder(std::string folderPath){

	/*
    if(stat(folderPath.c_str(), &info) != 0 ) {
        if (-1 == mkdir(folderPath.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH))
        {
            std::cout<< "Error creating directory "<< folderPath<<" !" << std::endl;
            exit(1);
        }
        std::cout << folderPath << " is created" << folderPath << std::endl;
    }else if( info.st_mode & S_IFDIR )  // S_ISDIR() doesn't exist on my windows
        std::cout<<folderPath<<" is a directory"<<std::endl;
    else
        std::cout<<folderPath<<" is no directory"<<std::endl;
        */
	if( stat(folderPath.c_str(), &info) != 0){
		cout <<"creating folder: " << folderPath << endl;
        if (-1 == mkdir(folderPath.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH))
        {
            std::cout<< "Error creating directory "<< folderPath <<" !" << std::endl;
            exit(1);
        }
    }
    else{
    	//cout << folderPath << " already exists" << endl;
    }
}


string convert(cv::Mat point) {
	string x = to_string((int)(floor(point.at<float>(0) / blockSize) * blockSize));
	string y = to_string((int)(floor(point.at<float>(1) / blockSize) * blockSize));
	string z = to_string((int)(floor(point.at<float>(2) / blockSize) * blockSize));
	return x + "_" + y + "_" + z;
}

float magnitude(cv::Mat vectorPoint) {
	float x = 0;
	for (int i = 0; i < vectorPoint.rows; i++) {
		x += vectorPoint.at<float>(i, 0) * vectorPoint.at<float>(i, 0);
	}
	return sqrt(x);
}

set<string> categorize(cv::Mat depthMat, cv::Mat cameraIntrinsic, cv::Mat tcwMat) {


	set<string>blocks;


	for (int i = 0; i < depthMat.rows; ++i) {
		for (int j = 0; j < depthMat.cols; ++j) {
            float pointDepth = depthMat.at<float>(i, j);
			if (pointDepth < 0.0001 || pointDepth > maxAccurateDistance) {
				continue;
			}

			float imageX = j;
			float imageY = i;
			float imageZ = 1;


			cv::Vec3f projectedVector(imageX, imageY, imageZ);

			cv::Mat projectedPoint = cameraIntrinsic * cv::Mat(projectedVector);

			projectedPoint =  pointDepth * projectedPoint;

            projectedPoint = tcwMat.rowRange(0,3).colRange(0,3) * projectedPoint;

            projectedPoint = projectedPoint + tcwMat.rowRange(0,3).col(3);

			//cout << projectedPoint << endl;

			string insideBlock = convert(projectedPoint);

            if (blocks.find(insideBlock) == blocks.end()) {
                cout << "NEW POINT " << pointDepth << endl;
            }

			blocks.insert(insideBlock);

		}
	}
	

	return blocks;

	
}



int main(int argc, char **argv) {
	if (argc != 3) {
        cerr << endl << "Usage: ./FileCategorize path_to_frames settings_file" << endl;
        return 1;
    }

    folderPath = argv[1];
    rgbPath = folderPath + "/RGB/";
    depthPath = folderPath + "/depth/";
    tcwPath = folderPath + "/tcw/";


    stat(folderPath.c_str(), &info);
    if( info.st_mode & S_IFDIR ){https://docs.opencv.org/3.0-beta/doc/py_tutorials/py_core/py_basic_ops/py_basic_ops.html
        cout << folderPath << " is a directory" << endl;
    }
    else{
        cout << folderPath << " is not a valid directory" << endl;
        return 0;
    }


    createFolder(folderPath + "/frames_categorized/");


    string strSettingsFile = argv[2];

    cv::FileStorage fSettings(strSettingsFile, cv::FileStorage::READ);

    float fx_ = fSettings["Camera.fx"];
    float fy_ = fSettings["Camera.fy"];
    float cx_ = fSettings["Camera.cx"];
    float cy_ = fSettings["Camera.cy"];
    width_ = fSettings["Camera.width"];
    height_ = fSettings["Camera.height"];
    float voxSize = fSettings["Voxel.Size.Offline"];
    int voxDim = fSettings["Voxel.Dim.x"];

    blockSize = voxSize * voxDim;



    float Karr[3][3] = {{fx_, 0, cx_}, {0, fy_, cy_}, {0, 0, 1}};
    cv::Mat K(3, 3, CV_32F, Karr);

    K = K.inv();

    int empty = 0;
    int frame = 1;
    cv::Mat RGBMat;
    cv::Mat depthMat;
    cv::Mat tcwMat;


    while (true) {
    	RGBMat = cv::imread(rgbPath + std::to_string(frame) + ".png",cv::IMREAD_COLOR);
    	depthMat = cv::imread(depthPath + to_string(frame) + ".png", -1);
    	depthMat.convertTo(depthMat, CV_32FC1);
        depthMat *= 0.001;


    	if (depthMat.rows == 0) {
    		cout << "no image found at frame_id: " << frame << endl;
    		empty++;
    		frame++;
    		if (empty > 6) {
    			break;
    		}
    		continue;
    	} 

    	cout << "calculating frame: " << frame << endl;


    	cv::FileStorage fs2(tcwPath + to_string(frame) + ".xml", cv::FileStorage::READ);
    	fs2["tcw"] >> tcwMat;


    	empty = 0;

    	set<string> blocks = categorize(depthMat, K, tcwMat);

    	
    	for (string block : blocks) {
    		cout << "block to assign: " << block << endl;

    		string pathAssign = folderPath + "/frames_categorized/" + block + "/";
    		string rgbPathAssign = pathAssign + "RGB/";
    		string depthPathAssign = pathAssign + "depth/";
    		string tcwPathAssign = pathAssign + "tcw/";

    		createFolder(pathAssign);
    		createFolder(rgbPathAssign);
    		createFolder(depthPathAssign);
    		createFolder(tcwPathAssign);



    		cv::imwrite(rgbPathAssign + std::to_string(frame) + ".png", RGBMat);

    		cv::Mat depth255;
    		depthMat.convertTo(depth255, CV_16UC1, 1000);

    		cv::imwrite(depthPathAssign + std::to_string(frame) + ".png", depth255);

    		cv::FileStorage fs(tcwPathAssign + std::to_string(frame)+".xml",cv::FileStorage::WRITE);
    		fs << "tcw" << tcwMat;
    		fs.release();


    	}

    	frame++;

    	
    }
    

}