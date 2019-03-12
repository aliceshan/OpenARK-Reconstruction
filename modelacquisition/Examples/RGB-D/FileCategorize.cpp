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

set<string> categorize(cv::Mat RGBMat, cv::Mat depthMat, cv::Mat projectionMat, cv::Mat cameraPos, int frame) {


	set<string>blocks;


	for (int i = 0; i < depthMat.rows; ++i) {
		for (int j = 0; j < depthMat.cols; ++j) {
			if (depthMat.at<float>(i,j) < 0.0001 || depthMat.at<float>(i,j) > maxAccurateDistance) {
				continue;
			}

			float imageX = (j - width_ / 2) / (width_ * 0.5f);
			float imageY = (height_ / 2 - i) / (height_ * 0.5f);
			float imageZ = 1;


			cv::Vec3f projectedVector(imageX, imageY, imageZ);

			cv::Mat projectedPoint = projectionMat.colRange(0, 3) * cv::Mat(projectedVector);

			projectedPoint = (projectedPoint - cameraPos) * depthMat.at<float>(i,j);

			cv::multiply(projectedPoint.col(0), 1.0f / magnitude(projectedPoint), projectedPoint.col(0));

			projectedPoint *= depthMat.at<float>(i,j);

			//cout << projectedPoint << endl;

			string insideBlock = convert(projectedPoint);

			blocks.insert(insideBlock);

			if (insideBlock == "-24_-6_18") {
				cout << depthMat.at<float>(i,j) << endl;
			}

		}
	}
	

	return blocks;

	
}



int main(int argc, char **argv) {
	if (argc != 4) {
        cerr << endl << "Usage: ./FileCategorize path_to_frames size_of_block(m) settings_file" << endl;
        return 1;
    }

    folderPath = argv[1];
    rgbPath = folderPath + "/RGB/";
    depthPath = folderPath + "/depth/";
    tcwPath = folderPath + "/tcw/";


    stat(folderPath.c_str(), &info);
    if( info.st_mode & S_IFDIR ){
        cout << folderPath << " is a directory" << endl;
    }
    else{
        cout << folderPath << " is not a valid directory" << endl;
        return 0;
    }


    createFolder(folderPath + "/frames_categorized/");



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

    	//cout << tcwMat << endl;


    	cv::Mat projectionMat;
    	cv::Mat projectionMatOne = (K * tcwMat.rowRange(0,3));
    	cv::Mat projectionMatInv = projectionMatOne.colRange(0,3).inv();
    	cv::hconcat(projectionMatInv, projectionMatOne.col(3), projectionMat);

    	cv::Mat cameraPos = tcwMat.rowRange(0,3).colRange(0,3).t()*-1.f*tcwMat.rowRange(0,3).col(3);


    	empty = 0;


    	/*cv::namedWindow("Display window", cv::WINDOW_AUTOSIZE);
    	cv::imshow("Display window", depthMat);

    	cv::waitKey(0);*/


    	set<string> blocks = categorize(RGBMat, depthMat, projectionMat, cameraPos, frame);

    	
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