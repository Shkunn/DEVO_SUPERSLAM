/**
 * @file main.cpp
 * @author Duncan Hamill (duncanrhamill@googlemail.com)
 * @brief OAK-D/ORB_SLAM3 Experiments main file.
 * 
 * @version 0.1
 * @date 2021-01-13
 * 
 * @copyright Copyright (c) Duncan Hamill 2021
 */

/* -------------------------------------------------------------------------
 * INCLUDES
 * ------------------------------------------------------------------------- */

#include <iostream>

#include <opencv2/opencv.hpp>

#include "System.h"

/* NEW */
#include <librealsense2/rs.hpp>
#include <iomanip>
#include <librealsense2/rsutil.h>
#include <array>
#include <cmath>
#include <iostream>

using namespace cv;

/* -------------------------------------------------------------------------
 * CONSTANTS
 * ------------------------------------------------------------------------- */

// WLS parameters, taken from the OpenCV WLS filter docs recommended values.
#define WLS_LAMBDA (8000)
#define WLS_SIGMA (1.0)

/* -------------------------------------------------------------------------
 * STRUCTS
 * ------------------------------------------------------------------------- */

// A simple pose structure containing position vector and rotation matrix.
typedef struct _Pose {
    cv::Mat position;
    cv::Mat rotation;
} Pose;

/* -------------------------------------------------------------------------
 * MAIN
 * ------------------------------------------------------------------------- */

int main(int argc, char *argv[]) {

    std::cout << "OAK-D/ORB_SLAM3 Experiment" << std::endl;
    
    cv::Mat R1, R2, P1, P2, Q;

    // Create the SLAM system. First argument is path to the ORB_SLAM3 vocab
    // file. The second is the path to the settings file for this particular
    // camera setup. The values in this file were taken from what's printed out
    // of `depthai_demo.py`.
    //
    // While the OAK-D does have an IMU we can't use it right now, but support
    // is coming soon! For now just use stereo mode for SLAM, which isn't as
    // accurate as IMU_STEREO.
    //
    // The last input tells the system to display it's UI.
    ORB_SLAM3::System SLAM(
        "Vocabulary/ORBvoc.txt",
        "Examples/Stereo/EuRoCD435.yaml",
        ORB_SLAM3::System::STEREO, 
        true
    );

    // We also want somewhere to store our pose data
    Pose pose;

    // The time of each frame is required for SLAM, so we take an epoch time
    // (i.e. our start time) now
    auto slam_epoch = std::chrono::steady_clock::now();

    // CAMERA.
    int width = 640;
	int height = 480;
	int fps = 30;
	rs2::config config;
	config.enable_stream(RS2_STREAM_INFRARED, 1, width, height, RS2_FORMAT_Y8, fps);
	config.enable_stream(RS2_STREAM_INFRARED, 2, width, height, RS2_FORMAT_Y8, fps);

	// start pipeline
	rs2::pipeline pipeline;
	rs2::pipeline_profile pipeline_profile = pipeline.start(config);
    rs2::device select_divise = pipeline_profile.get_device();
    auto depth_depth = select_divise.first<rs2::depth_sensor>();
	depth_depth.set_option(RS2_OPTION_EMITTER_ENABLED, 0);   

    // Start pipeline with chosen configuration

    cv::Mat imLeft; 
    cv::Mat imRight; 

    // Now for the main loop
    while (1) {

		rs2::frameset frameset = pipeline.wait_for_frames();

		rs2::video_frame ir_frame_left = frameset.get_infrared_frame(1);
		rs2::video_frame ir_frame_right = frameset.get_infrared_frame(2);

        imLeft = cv::Mat(cv::Size(width, height), CV_8UC1, (void*)ir_frame_left.get_data());
        imRight = cv::Mat(cv::Size(width, height), CV_8UC1, (void*)ir_frame_right.get_data());

        // Get the time between the epoch and now, allowing us to get a
        // timestamp (in seconds) to pass into the slam system.
        auto elapsed_time = std::chrono::steady_clock::now() - slam_epoch;
        double frame_timestamp_s = elapsed_time.count() / 1000000000.0;

        std::cout << std::setprecision(4) << frame_timestamp_s << ": ";

        // Into the SLAM system. This produces a matrix with
        // the pose information of the camera.
        cv::Mat raw_pose = SLAM.TrackStereo(
            imLeft,
            imRight,
            frame_timestamp_s  // rs2::frameset frameset = pipe.wait_for_frames();
        // Pass the images i
        );

        // The output pose may be empty if the system was unable to track the
        // movement, so only get position and rotation if pose isn't empty. We
        // also put this info an a localisation fix available flag for later
        // use. 
        bool loc_fix_available = !raw_pose.empty();
        if (loc_fix_available) {
            // The pose matrix is a 4x4 extrinsic matrix, with the form:
            // [R_3x3 T_3x1; [0 0 0 1]], we can find the camera position with 
            // C = -R'T (R' = R transpose).
            pose.rotation = raw_pose(cv::Rect(0, 0, 3, 3));
            cv::Mat T = raw_pose(cv::Rect(3, 0, 1, 3));
            pose.position = -pose.rotation.t()*T;

            // Print the updated position, but transpose it so that instead of
            // a column vector we have a row vector, which is easier to read.
            std::cout << 
                "position: " << 
                pose.position.t() << 
                std::endl;
        }
        else {
            // If we didn't get a pose update log it.
            std::cout << "no pose update" << std::endl;
        }

        // Apply a colormap to the filtered disparity map, but don't normalise
        // it. Normalising the map will mean that the color doesn't correspond
        // directly with disparity.
        cv::Mat colour_disp;
        /*cv::applyColorMap(imLeft, colour_disp, cv::COLORMAP_JET);*/
        cv::imshow("disparity", imLeft);

        // See if q pressed, if so quit
        if (cv::waitKey(1) == 'q') {
            break;
        }
    }

    // Stop all SLAM threads
    SLAM.Shutdown();

    return EXIT_SUCCESS;
}