/*
Adam
Offline version of 3D reconstruction with categorization
- Loads categorized key frames
- 3D dense model reconstruction through TSDF 
- Supports much larger and accurate reconstruction space than the uncatergorizaed offline version
- Camera calibration file required
- No GL support (see rgbd_realsense_load_gl.cpp)
*/

#include <iostream>
#include <algorithm>
#include <thread>
#include <set>
#include <vector>


#include <stdio.h>
#include <sys/types.h>
#include <dirent.h>



#include <opencv2/opencv.hpp>
#include <sys/types.h>
#include <sys/stat.h>

#include <SaveFrame.h>
#include <PointCloudGenerator.h>



std::thread *app;
std::string settingsFile;
std::string directoryName;

using namespace std;



void createFolder(struct stat &info, std::string folderPath){
    if(stat( folderPath.c_str(), &info ) != 0 ) {
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
}


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



vector<float> getOrigin(string origin) {
    cout << "THIS ORIGIN: " << origin << endl;
    size_t pos = 0;
    string delimiter = "_";
    vector<float> originF;
    int i = 0;
    while ((pos = origin.find(delimiter)) != string::npos) {
        originF.push_back(atof(origin.substr(0, pos).c_str()));
        origin.erase(0, pos + delimiter.length());
        i++;
    }
    originF.push_back(atof(origin.c_str()));
    return originF;
    
}


void application_thread() {

    set<string> blocks = getFiles(directoryName);


    //Main loop
    for (string origin: blocks) {  

        vector<float> originF = getOrigin(origin);

        if (abs(originF[0]) > 9) {
            continue;
        }
        if (abs(originF[1]) > 9) {
            continue;
        }
        if (abs(originF[2]) > 9) {
            continue;
        }
        
        // Create saveFrame. It loads from timestamp, RGB image, depth image folders to retrieve key frames in the current block
        ark::SaveFrame *saveFrame = new ark::SaveFrame(directoryName + origin + "/");
        // Create pointCloudGenerator (TSDF). It initializes all system threads and gets ready to process frames (RBG and Depth).
        ark::PointCloudGenerator *pointCloudGenerator = new ark::PointCloudGenerator(settingsFile, 
            originF.at(0), originF.at(1), originF.at(2));

        pointCloudGenerator->Start();

        set<string> frames = getFiles(directoryName + origin + "/RGB/");

        if (frames.size() < 3) {
            continue;
        }

        set<int> tframes;
        for (string frameC: frames) {
            int tframe_ = atoi(frameC.substr(0, frameC.find(".")).c_str());
            tframes.insert(tframe_);
        }


        for (int tframe: tframes) {


            ark::RGBDFrame frame = saveFrame->frameLoad(tframe);

            cv::cvtColor(frame.imRGB, frame.imRGB, cv::COLOR_BGR2RGB);

            pointCloudGenerator->PushFrame(frame);

        }

        pointCloudGenerator->RequestStop();
        // Save the model in the current clock in a ply file

        struct stat info;
        createFolder(info, "./meshes/");

        pointCloudGenerator->SavePly();
        pointCloudGenerator->ClearTSDF();
        delete pointCloudGenerator;
        delete saveFrame;
    }
}

int main(int argc, char **argv) {
    if (argc != 3) {
        cerr << endl << "Usage: ./rgbd_realsense_load_categorized path_to_frames path_to_settings" << endl;
        return 1;
    }

    settingsFile = argv[2];

    directoryName = argv[1];
    directoryName += "frames_categorized/";

    // Main loop
    application_thread();

    delete app;
    return EXIT_SUCCESS;
}