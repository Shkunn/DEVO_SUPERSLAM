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


void thread_IMU(rs2::pipeline pipe, vector<ORB_SLAM3::IMU::Point> & vImuMeas, std::chrono::_V2::steady_clock::time_point & slam_epoch)
{
    // Main loop
    while (true)
    {
        // Wait for the next set of frames from the camera
        auto frames = pipe.wait_for_frames();

        auto elapsed_time = std::chrono::steady_clock::now() - slam_epoch;
        double frame_timestamp_s = elapsed_time.count() / 1000000000.0;
        
        rs2::motion_frame accel_frame = frames.first_or_default(RS2_STREAM_ACCEL);
        rs2::motion_frame gyro_frame = frames.first_or_default(RS2_STREAM_GYRO);

        rs2_vector accel_sample = accel_frame.get_motion_data();
        rs2_vector gyro_sample = gyro_frame.get_motion_data();

        vImuMeas.push_back(ORB_SLAM3::IMU::Point(accel_sample.x, accel_sample.y, accel_sample.y,
                                    gyro_sample.x, gyro_sample.y, gyro_sample.z,
                                    frame_timestamp_s));
    }
}


void thread_MAIN(rs2::pipeline pipeline,  vector<ORB_SLAM3::IMU::Point> & vImuMeas, std::chrono::_V2::steady_clock::time_point & slam_epoch, ORB_SLAM3::System & SLAM)
{
    cv::Mat R1, R2, P1, P2, Q;
    // We also want somewhere to store our pose data
    Pose pose;

    cv::Mat imLeft;
    cv::Mat imRight;

    int width = 640;
    int height = 480;

    // rs2::pipeline_profile pipeline_profile = pipeline.start(config);
    // rs2::device select_divise = pipeline_profile.get_device();
    // auto depth_depth = select_divise.first<rs2::depth_sensor>();
	// depth_depth.set_option(RS2_OPTION_EMITTER_ENABLED, 0);

    while (1) {
        // Read the output frames from the OAK-D. These are blocking calls, so
        // they will wait until there's data available.
        
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
            frame_timestamp_s,  // rs2::frameset frameset = pipe.wait_for_frames();
            vImuMeas
        // Pass the images i
        );

        vImuMeas.clear();

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

        cv::imshow("disparity", imLeft);
        if (cv::waitKey(1) == 'q') {
            break;
        }
    }

    // Stop all SLAM threads
    SLAM.Shutdown();
}

/* -------------------------------------------------------------------------
 * MAIN
 * ------------------------------------------------------------------------- */

int main(int argc, char *argv[]) {

    std::cout << "OAK-D/ORB_SLAM3 Experiment" << std::endl;
    
    // The last input tells the system to display it's UI.
    ORB_SLAM3::System SLAM(
        "Vocabulary/ORBvoc.txt",
        "Examples/Stereo-Inertial/EuRoC.yaml",
        ORB_SLAM3::System::IMU_STEREO, 
        true
    );

    // The time of each frame is required for SLAM, so we take an epoch time
    // (i.e. our start time) now
    std::chrono::_V2::steady_clock::time_point slam_epoch = std::chrono::steady_clock::now();

    rs2::context                          ctx;        // Create librealsense context for managing devices
    std::map<std::string, rs2::colorizer> colorizers; // Declare map from device serial number to colorizer (utility class to convert depth data RGB colorspace)
    std::vector<rs2::pipeline>            pipelines;

    // Capture serial numbers before opening streaming
    std::vector<std::string>              serials;
    for (auto&& dev : ctx.query_devices())
        serials.push_back(dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER));

    // Start a streaming pipe per each connected device
    for (auto&& serial : serials)
    {
        rs2::pipeline pipe(ctx);
        rs2::config cfg;
        cfg.enable_device(serial);
        pipe.start(cfg);
        pipelines.emplace_back(pipe);
        // Map from each device's serial number to a different colorizer
        colorizers[serial] = rs2::colorizer();
    }

    vector<ORB_SLAM3::IMU::Point> vImuMeas;

    std::thread thread1;
    std::thread thread2;

    for (auto &&pipe : pipelines) // loop over pipelines
    {
        // Wait for the next set of frames from the camera
        auto frames = pipe.wait_for_frames();

        auto color = frames.get_color_frame();

        // For cameras that don't have RGB sensor, we'll map the pointcloud to infrared instead of color
        if (!color) {
            auto thread1 = std::thread(&thread_IMU, pipe, std::ref(vImuMeas), std::ref(slam_epoch));
        }

        // For cameras that don't have RGB sensor, we'll map the pointcloud to infrared instead of color
        if (color) {
            auto thread2 = std::thread(&thread_MAIN, pipe, std::ref(vImuMeas), std::ref(slam_epoch), std::ref(SLAM));
        }
    }

    // CREATE THREAD
    thread1.join();
    thread2.join();

    return 0;
}
