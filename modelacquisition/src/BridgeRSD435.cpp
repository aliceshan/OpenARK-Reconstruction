#include "BridgeRSD435.h"

BridgeRSD435::BridgeRSD435(): align(RS2_STREAM_COLOR){
    // Turn on logging. We can separately enable logging to console or to file, and use different severity filters for each.
    // rs2::log_to_console(rs2_log_severity::warn);
    //rs2::log_to_file(rs2::log_severity::debug, "librealsense.log");
}
 
int BridgeRSD435::Start() {
    // Create a context object. This object owns the handles to all connected realsense devices.
    // rs2::device_list list = mCtx.query_devices();
    // printf("There are %d connected RealSense devices.\n", list.size());
    // if(list.size() == 0) return EXIT_FAILURE;

    // // This tutorial will access only a single device, but it is trivial to extend to multiple devices
    // const rs2::device & dev = list.front();
    // mpDev = &dev;
    // printf("\nUsing device 0, an %s\n", mpDev->get_name());
    // printf("    Serial number: %s\n", mpDev->get_serial());
    // printf("    Firmware version: %s\n", mpDev->get_firmware_version());

    // // Configure depth and color to run with the device's preferred settings
    // mpDev->enable_stream(RS2_STREAM_COLOR, rs2::preset::best_quality);
    // mpDev->enable_stream(RS2_STREAM_DEPTH, rs2::preset::largest_image);
    // mpDev->start();

    // mDepth_intrin = mpDev->get_stream_intrinsics(RS2_STREAM_DEPTH);
    // mDepth_to_color = mpDev->get_extrinsics(RS2_STREAM_DEPTH, RS2_STREAM_COLOR);
    // mColor_intrin = mpDev->get_stream_intrinsics(RS2_STREAM_COLOR);
    // mDepthScale = mpDev->get_depth_scale();

    ///////////////////////////////
        pipe = std::make_shared<rs2::pipeline>();

        query_intrinsics();
        badInputFlag = false;
        profile = pipe->start(config);

        // align = rs2::align(align_to);
        // get updated intrinsics
        rgbIntrinsics = new rs2_intrinsics();
        d2rExtrinsics = new rs2_extrinsics();
        r2dExtrinsics = new rs2_extrinsics();
        const std::vector<rs2::stream_profile> & stream_profiles = profile.get_streams();

        rs2::stream_profile rgbProfile;
        rgbProfile = profile.get_stream(RS2_STREAM_COLOR);

        rs2::stream_profile depthProfile = profile.get_stream(RS2_STREAM_DEPTH);
        *reinterpret_cast<rs2_intrinsics *>(rgbIntrinsics) =
            rgbProfile.as<rs2::video_stream_profile>().get_intrinsics();
        *reinterpret_cast<rs2_intrinsics *>(depthIntrinsics) =
            depthProfile.as<rs2::video_stream_profile>().get_intrinsics();
        *reinterpret_cast<rs2_extrinsics *>(d2rExtrinsics) = depthProfile.get_extrinsics_to(rgbProfile);
        *reinterpret_cast<rs2_extrinsics *>(r2dExtrinsics) = rgbProfile.get_extrinsics_to(depthProfile);
        scale = profile.get_device().first<rs2::depth_sensor>().get_depth_scale();

        // std::cout<<

    return 0;
}

void BridgeRSD435::GrabRGBDPair(cv::Mat &imRGB, cv::Mat &imD) {
    // uint8_t * color_image = (uint8_t *)mpDev->get_frame_data(rs::stream::rectified_color);
    // uint16_t * depth_image = (uint16_t *)mpDev->get_frame_data(rs::stream::depth_aligned_to_rectified_color);

    // imRGB = cv::Mat(mColor_intrin.height, mColor_intrin.width, CV_8UC3);
    // imD =  cv::Mat(mDepth_intrin.height, mDepth_intrin.width, CV_16SC1);

    // std::memcpy(imRGB.data, color_image, mColor_intrin.height*mColor_intrin.width*3*sizeof(unsigned char));
    // std::memcpy(imD.data, depth_image, mDepth_intrin.height*mDepth_intrin.width*sizeof(short));


        rs2::frameset data;

        if (!depthIntrinsics || !rgbIntrinsics || !d2rExtrinsics) return;
        rs2_intrinsics * dIntrin = reinterpret_cast<rs2_intrinsics *>(depthIntrinsics);
    
        imRGB = cv::Mat(height, width, CV_8UC3);
        imD =  cv::Mat( dIntrin->height, dIntrin->width, CV_16SC1);

        try {
            data = pipe->wait_for_frames();

            // if (profile_changed(pipe.get_active_profile().get_streams(), profile.get_streams()))
            // {
            //             //If the profile was changed, update the align object, and also get the new device's depth scale
            //     profile = pipe.get_active_profile();
            //     // align_to = find_stream_to_align(profile.get_streams());
            //     align = rs2::align(align_to);
            //     scale = profile.get_device().first<rs2::depth_sensor>().get_depth_scale();
            // }

        //Get processed aligned frame
            auto processed = align.process(data);

        // Trying to get both other and aligned depth frames
            // rs2::video_frame rgb_frame = processed.first(align_to);
            // rs2::depth_frame aligned_depth_frame = processed.get_depth_frame();

        // //If one of them is unavailable, continue iteration
        //     if (!aligned_depth_frame || !rgb_frame)
        //     {
        //         continue;
        //     }
        //     imRGB = color.clone();

            rs2::frame depth_frame = processed.get_depth_frame();

            cv::Mat depth(cv::Size(width, height), CV_16SC1, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);

            imD = depth.clone();
            // Mat depth(Size(640, 480), CV_16SC1, (void*)depth_frame.get_data(), Mat::AUTO_STEP);

            // const uint16_t * depth_image = (const uint16_t *)depth.get_data();
            // std::memcpy(imD.data, depth_image, dIntrin->height*dIntrin->width*sizeof(short));

            // memcpy(imD.data, depth.get_data(), dIntrin->width * dIntrin->height);

            rs2::frame video_frame = processed.first(RS2_STREAM_COLOR);

            cv::Mat color(cv::Size(width, height), CV_8UC3, (void*)video_frame.get_data(), cv::Mat::AUTO_STEP);
            // const uint8_t * color_image = (const uint8_t *)color.get_data();
            // memcpy(imRGB.data, color_image, 3 * width * height *sizeof(unsigned char));

            imRGB = color.clone();
        }
        catch (std::runtime_error e) {
            // try reconnecting
            badInputFlag = true;
            pipe->stop();
            printf("Couldn't connect to camera, retrying in 0.5s...\n");
            boost::this_thread::sleep_for(boost::chrono::milliseconds(500));
            query_intrinsics();
            pipe->start(config);
            badInputFlag = false;
            return;
        }

}

void BridgeRSD435::Stop() {

        try {
            pipe->stop();
        }
        catch (...) {}

        if (depthIntrinsics) delete reinterpret_cast<rs2_intrinsics *>(depthIntrinsics);
        if (rgbIntrinsics) delete reinterpret_cast<rs2_intrinsics *>(rgbIntrinsics);
        if (d2rExtrinsics) delete reinterpret_cast<rs2_extrinsics *>(d2rExtrinsics);
        depthIntrinsics = rgbIntrinsics = d2rExtrinsics = nullptr;
}



void BridgeRSD435::query_intrinsics() {
        rs2_intrinsics * depthIntrinsics = new rs2_intrinsics();

        rs2::context ctx;
        rs2::device_list list = ctx.query_devices();

        if(list.size() == 0){
            cout<<"No camera detected."<<endl;
            return;
        }

        const rs2::device & dev = list.front();
        const std::vector<rs2::sensor> sensors = dev.query_sensors();

        for (unsigned i = 0; i < sensors.size(); ++i) {
            const rs2::sensor & sensor = sensors[i];
            const std::vector<rs2::stream_profile> & stream_profiles = sensor.get_stream_profiles();

            for (unsigned j = 0; j < stream_profiles.size(); ++j) {
                const rs2::stream_profile & stream_profile = stream_profiles[j];
                const rs2_stream & stream_data_type = stream_profile.stream_type();
                const rs2_format & stream_format = stream_profile.format();

                if (stream_profile.is<rs2::video_stream_profile>()) {
                    if (stream_data_type == RS2_STREAM_DEPTH && stream_format == RS2_FORMAT_Z16) {
                        const rs2::video_stream_profile & prof = stream_profile.as<rs2::video_stream_profile>();
                        *depthIntrinsics = prof.get_intrinsics();
                        this->depthIntrinsics = depthIntrinsics;
                        if (depthIntrinsics->height == PREFERRED_FRAME_H && depthIntrinsics->width == PREFERRED_FRAME_W) break;
                    }
                }
            }
            if (this->depthIntrinsics) break;
        }

        // ASSERT(this->depthIntrinsics, "FATAL: Camera has no depth stream!");


        if(this->depthIntrinsics == 0){
            cout<<"FATAL: Camera has no depth stream!"<<endl;
            return;
        }

        width = depthIntrinsics->width;
        height = depthIntrinsics->height;

        std::cout<<"camera shape = "<<width<<","<<height<<std::endl;

        config.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16);
        config.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_BGR8);
    }