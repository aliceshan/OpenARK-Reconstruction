//
// Created by alice on 2/12/19.
//


#include <iostream>
#include <chrono>
#include <vector>
#include <Utils.h>
#include <opencv2/opencv.hpp>

#include "SaveFrame.h"

using namespace std;
using namespace std::chrono;

//TODO update CMAKE file to include this one

int main(int argc, char **argv) {
	// store timings
	vector<float> rgb_times;
	vector<float> xml_times; 

	str load_folder = "./frames/";
	str save_folder = "./frames2/";
	saveFrame = new ark::SaveFrame(load_folder);

	//load all the frames
    int tframe = 1;
    int empty = 0;
    while (true) {

        if(empty == 5)
            break;

        ark::RGBDFrame frame = saveFrame->frameLoad(tframe);
        tframe ++ ;

        if(frame.frameId == -1){
            empty ++;
            continue;
        }
        else{
        	//save the frame back
        	//saveFrame->frameWrite(frame);

        	//save png 
        	high_resolution_clock::time_point t1 = high_resolution_clock::now();
	        cv::imwrite(save_folder + "RGB/" + std::to_string(frame.frameId) + ".png", frame.imRGB);
	        high_resolution_clock::time_point t2 = high_resolution_clock::now();

	        auto duration = duration_cast<microseconds>( t2 - t1 ).count();
	        rgb_times.push_back(duration);

	        //save xml
	        t1 = high_resolution_clock::now();
	        cv::FileStorage fs(save_folder + "twc/" + std::to_string(frame.frameId)+".xml",cv::FileStorage::WRITE);
	        fs << "tcw" << frame.mTcw ;
	        fs << "depth" << frame.imDepths;
	        fs.release();
	        t2 = high_resolution_clock::now();

	        duration = duration_cast<microseconds>( t2 - t1 ).count();
	        xml_times.push_back(duration);
        }
    }

    cout << "Avg RGB time: " << accumulate(rgb_times.begin(), v.end(), 0)/rgb_times.size() << endl;
    cout << "Avg XML time: " << accumulate(xml_times.begin(), v.end(), 0)/xml_times.size() << endl;
	return 0;
}

