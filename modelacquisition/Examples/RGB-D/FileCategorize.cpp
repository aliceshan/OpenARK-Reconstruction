/*
Adam 
Categorizes frames into reconstruction blocks using 2D-3D projection
Requires RGB, Depth, and tcw frames
RGB: /frames/RGB/
Depth: /frames/depth/
TCW: /frames/tcw/
*/


#include <iostream>
#include <set>
#include <opencv2/opencv.hpp>
#include <sys/types.h>
#include <sys/stat.h>

#include <stdio.h>
#include <dirent.h>


#include <SaveFrame.h>

using namespace std;

string folderPath, rgbPath, depthPath, tcwPath;
float blockSize; //size of reconstruction section size (meters)
float maxAccurateDistance; //based on camera limits to exclude noisy measurements (meters)
int precision = 5; //projects every precision-th row and column (total pixels / precision^2 pixels projected)


struct stat info;

//Gets files in a directory and returns a set with file names

set<string> getFiles(string filename){
    cout << "getting names from: " << filename << endl;
    set<string> files;
    DIR *dp;
    int i = 0;
    struct dirent *ep;     
    dp = opendir (filename.c_str());

    if (dp != NULL) {
        while ((ep = readdir (dp))) {
            string name = ep -> d_name;
            if (name.length() > 2) {
                files.insert(name);
                i++;
            }
        }
        (void) closedir (dp);
    }
    else {
        perror ("Couldn't open the directory");
    }
    
    printf("There's %d files in the current directory.\n", i);
    return files;
}


// Creates folders using <sys/stat.h>

void createFolder(std::string folderPath){

	if( stat(folderPath.c_str(), &info) != 0){
		cout <<"creating folder: " << folderPath << endl;
        if (-1 == mkdir(folderPath.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH))
        {
            std::cout<< "Error creating directory "<< folderPath <<" !" << std::endl;
            exit(1);
        }
    }
}


//Converts an <x,y,z> cv::Mat to a string formatted "x_y_z"

string convert(cv::Mat point) {
	string x = to_string((int)(floor((point.at<float>(0) - 3.) / blockSize) * blockSize + 3));
	string y = to_string((int)(floor((point.at<float>(1) - 3.) / blockSize) * blockSize + 3));
	string z = to_string((int)(floor((point.at<float>(2) - 3.) / blockSize) * blockSize + 3));
	return x + "_" + y + "_" + z;
}


//Returns magnitude of a vector

float magnitude(cv::Mat vectorPoint) {
	float x = 0;
	for (int i = 0; i < vectorPoint.rows; i++) {
		x += vectorPoint.at<float>(i, 0) * vectorPoint.at<float>(i, 0);
	}
	return sqrt(x);
}


//Finds reconstruction blocks of any given image using 2D-3D Projection

set<string> categorize(cv::Mat depthMat, cv::Mat cameraIntrinsic, cv::Mat tcwMat) {

	set<string>blocks;

	for (int i = 0; i < depthMat.rows; i += precision) {
		for (int j = 0; j < depthMat.cols; j += precision) {
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


			string insideBlock = convert(projectedPoint);

			blocks.insert(insideBlock);
            
		}
	}

	return blocks;

	
}


//Write frames to folders given set of reconstruction blocks
void write_to_folders(set<string> blocks, ark::RGBDFrame frame) {

    for (string block : blocks) {

        string pathAssign = folderPath + "/frames_categorized/" + block + "/";

        ark::SaveFrame *saveFrame = new ark::SaveFrame(pathAssign);

        saveFrame->frameWrite(frame);

    }
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
    if( info.st_mode & S_IFDIR ){
        cout << folderPath << " is a directory" << endl;
    }
    else{
        cout << folderPath << " is not a valid directory" << endl;
        return 0;
    }
    createFolder(folderPath + "frames_categorized/");

    string strSettingsFile = argv[2];

    cv::FileStorage fSettings(strSettingsFile, cv::FileStorage::READ);


    //Get camera intrinsic's and reconstruction parameters

    float fx_ = fSettings["Camera.fx"];
    float fy_ = fSettings["Camera.fy"];
    float cx_ = fSettings["Camera.cx"];
    float cy_ = fSettings["Camera.cy"];
    float voxSize = fSettings["Voxel.Size.Offline"];
    int voxDim = fSettings["Voxel.Dim.x"];

    maxAccurateDistance = fSettings["MaxDepth"];
    blockSize = voxSize * voxDim;


    float Karr[3][3] = {{fx_, 0, cx_}, {0, fy_, cy_}, {0, 0, 1}};
    cv::Mat K(3, 3, CV_32F, Karr);

    //Inverse camera intrinsic matrix
    K = K.inv();

    ark::SaveFrame *saveFrame = new ark::SaveFrame(folderPath);

    set<string> frames = getFiles(tcwPath);

    set<int> tframes;
    for (string frameC: frames) {
        int tframe_ = atoi(frameC.substr(0, frameC.find(".")).c_str());
        tframes.insert(tframe_);
    }


    //Main loop, currently geared towards .png for RGB and Depth, .xml for tcw

    for (int tframe: tframes) {

        ark::RGBDFrame frame = saveFrame->frameLoad(tframe);


        //TCW is World to Camera Transform
        //TCW is Camera to World Trasnform
        cv::Mat Twc = frame.mTcw.inv();

    	set<string> blocks = categorize(frame.imDepth, K, Twc);

        write_to_folders(blocks, frame);
    }
    

}