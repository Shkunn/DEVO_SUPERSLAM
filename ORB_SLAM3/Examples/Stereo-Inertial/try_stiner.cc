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
#include <sl/Camera.hpp>
#include <thread>

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

struct TimestampHandler {

    // Compare the new timestamp to the last valid one. If it is higher, save it as new reference.
    inline bool isNew(sl::Timestamp& ts_curr, sl::Timestamp& ts_ref) {
        bool new_ = ts_curr > ts_ref;
        if (new_) ts_ref = ts_curr;
        return new_;
    }
    // Specific function for IMUData.
    inline bool isNew(sl::SensorsData::IMUData& imu_data) {
        return isNew(imu_data.timestamp, ts_imu);
    }
    // Specific function for MagnetometerData.
    inline bool isNew(sl::SensorsData::MagnetometerData& mag_data) {
        return isNew(mag_data.timestamp, ts_mag);
    }
    // Specific function for BarometerData.
    inline bool isNew(sl::SensorsData::BarometerData& baro_data) {
        return isNew(baro_data.timestamp, ts_baro);
    }

    sl::Timestamp ts_imu = 0, ts_baro = 0, ts_mag = 0; // Initial values
};

void thread_IMU(sl::Camera & zed,  vector<ORB_SLAM3::IMU::Point> & vImuMeas, std::chrono::_V2::steady_clock::time_point & slam_epoch)
{
    sl::SensorsData sensors_data;
    sl::SensorsData::IMUData imu_data;
    TimestampHandler ts;

    while(1)
    {
        while((zed.getSensorsData(sensors_data, sl::TIME_REFERENCE::CURRENT) == sl::ERROR_CODE::SUCCESS) && ts.isNew(sensors_data.imu))
        {
            auto elapsed_time = std::chrono::steady_clock::now() - slam_epoch;
            double frame_timestamp_s = elapsed_time.count() / 1000000000.0;
            //std::cout << sensors_data.imu.linear_acceleration << " " << sensors_data.imu.effective_rate << std::endl;
            //std::cout << "IMU NEW DATA " << sensors_data.imu.effective_rate << std::endl;
            //std::cout << std::setprecision(4) << frame_timestamp_s << ": ";
            vImuMeas.push_back(ORB_SLAM3::IMU::Point(sensors_data.imu.linear_acceleration[0],sensors_data.imu.linear_acceleration[1],sensors_data.imu.linear_acceleration[2],
                                        sensors_data.imu.angular_velocity[0],sensors_data.imu.angular_velocity[1],sensors_data.imu.angular_velocity[2],
                                        frame_timestamp_s));
        }
    }
}
void thread_MAIN(sl::Camera & zed,  vector<ORB_SLAM3::IMU::Point> & vImuMeas, std::chrono::_V2::steady_clock::time_point & slam_epoch, ORB_SLAM3::System & SLAM)
{
    cv::Mat R1, R2, P1, P2, Q;
    Pose pose;
    sl::Mat zed_image_G;
    sl::Mat zed_image_D;
    cv::Mat imLeft; 
    cv::Mat imRight; 

    while (1) {
        // Read the output frames from the OAK-D. These are blocking calls, so
        // they will wait until there's data available.
        /*auto rectif_left_frame = rectif_left_queue->get<dai::ImgFrame>();
        auto rectif_right_frame = rectif_left_queue->get<dai::ImgFrame>();
        auto disp_map_frame = disp_queue->get<dai::ImgFrame>();

        // Convert the frames into opencv images
        auto rectif_left = imgframe_to_mat(rectif_left_frame);
        auto rectif_right = imgframe_to_mat(rectif_right_frame);
        auto disp_map = imgframe_to_mat(disp_map_frame);*/
        zed.grab();

        zed.retrieveImage(zed_image_G, sl::VIEW::LEFT);
        zed.retrieveImage(zed_image_D, sl::VIEW::RIGHT);

        imLeft = cv::Mat((int) zed_image_G.getHeight(), (int) zed_image_G.getWidth(), CV_8UC4, zed_image_G.getPtr<sl::uchar1>(sl::MEM::CPU));
        imRight = cv::Mat((int) zed_image_D.getHeight(), (int) zed_image_D.getWidth(), CV_8UC4, zed_image_D.getPtr<sl::uchar1>(sl::MEM::CPU));

        // Get the time between the epoch and now, allowing us to get a
        // timestamp (in seconds) to pass into the slam system.
        auto elapsed_time = std::chrono::steady_clock::now() - slam_epoch;
        double frame_timestamp_s = elapsed_time.count() / 1000000000.0;

        //std::cout << std::setprecision(4) << frame_timestamp_s << ": ";

        /*sl::float3 linear_acceleration = imu_data.linear_acceleration;
        sl::float3 angular_velocity = imu_data.angular_velocity;*/

        /*vImuMeas.clear();
        vImuMeas.push_back(ORB_SLAM3::IMU::Point(sensors_data.imu.linear_acceleration[0],sensors_data.imu.linear_acceleration[1],sensors_data.imu.linear_acceleration[2],
                                        sensors_data.imu.angular_velocity[0],sensors_data.imu.angular_velocity[1],sensors_data.imu.angular_velocity[2],
                                        frame_timestamp_s));*/

        // Pass the images into the SLAM system. This produces a matrix with
        // the pose information of the camera.
        cv::Mat raw_pose = SLAM.TrackStereo(
            imLeft,
            imRight,
            frame_timestamp_s,
            vImuMeas
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
    /*
    // Create the pipeline that we're going to build. Pipelines are depthai's
    // way of chaining up different series or parallel process, sort of like
    // gstreamer. 
    //
    // Our pipeline is going to extract the left and right rectified images
    // from the cameras so we can pass these into the SLAM system, as well as
    // disparity maps for building a point cloud.
    dai::Pipeline pipeline;

    // We need to create all the nodes in our pipeline, which are:
    //  - the left and right monochrome (greyscale) stereo cameras of the OAK-D
    //  - a stereo depth node, which generates disparity maps and rectified
    //    images. The disparity map will be used in constructing the global 
    //    point cloud, and the rectified images for SLAM tracking.
    //  - output nodes, which allow us to get the rectified image data and
    //    disparity map to use outside the pipeline.
    auto mono_left = pipeline.create<dai::node::MonoCamera>();
    auto mono_right = pipeline.create<dai::node::MonoCamera>();
    auto stereo = pipeline.create<dai::node::StereoDepth>();
    auto xout_rectif_left = pipeline.create<dai::node::XLinkOut>();
    auto xout_rectif_right = pipeline.create<dai::node::XLinkOut>();
    auto xout_disp = pipeline.create<dai::node::XLinkOut>();

    // And we set the names of each output node, so we can access them later as
    // output queues
    xout_rectif_left->setStreamName("rectified_left");
    xout_rectif_right->setStreamName("rectified_right");
    xout_disp->setStreamName("disparity");

    // Now we set which cameras are actually connected to the left and right
    // nodes, and set their resolution and framerate
    mono_left->setBoardSocket(dai::CameraBoardSocket::LEFT);
    mono_left->setResolution(
        dai::MonoCameraProperties::SensorResolution::THE_720_P
    );
    mono_left->setFps(20.0);
    mono_right->setBoardSocket(dai::CameraBoardSocket::RIGHT);
    mono_right->setResolution(
        dai::MonoCameraProperties::SensorResolution::THE_720_P
    );
    mono_right->setFps(20.0);

    // Now we set the stereo node to output rectified images and disp maps. We
    // also set the rectify frames to not be mirrored, and to use black to fill
    // the edges of the rectified images. We need non-flipped images as we're
    // going to use them later down the line as input to the SLAM,
    // unfortunately this means our output disparity map will be flipped, so
    // we'll have to correct that later. We don't output depth as this would
    // disable the disparity map output.
    // 
    // We also enable extended disparity depth, which increases the maximum 
    // disparity and therefore provides a shorter minimum depth
    stereo->setOutputRectified(true);
    stereo->setOutputDepth(false);
    stereo->setRectifyEdgeFillColor(0);
    stereo->setRectifyMirrorFrame(false);
    stereo->setExtendedDisparity(true);

    // We now link the cameras up to the stereo node
    mono_left->out.link(stereo->left);
    mono_right->out.link(stereo->right);

    // And the stereo rectified and disp outputs to the output nodes
    stereo->rectifiedLeft.link(xout_rectif_left->input);
    stereo->rectifiedRight.link(xout_rectif_right->input);
    stereo->disparity.link(xout_disp->input);

    // Now we can connect to the OAK-D device and start our pipeline
    dai::Device device(pipeline);
    device.startPipeline();

    // Finally to actually see the outputs we need to get their output queues
    // We use a max buffer size of 8 frames and set it into non-blocking mode.
    auto rectif_left_queue = device.getOutputQueue("rectified_left", 8, false);
    auto rectif_right_queue = device.getOutputQueue("rectified_right", 8, false);
    auto disp_queue = device.getOutputQueue("disparity", 8, false);
    
    // Create the WLS (weighted least squares) filter, which we use to improve
    // the quality of our disparity map. Also set the lambda and sigma values
    auto wls_filter = cv::ximgproc::createDisparityWLSFilterGeneric(false);
    wls_filter->setLambda(WLS_LAMBDA);
    wls_filter->setSigmaColor(WLS_SIGMA);
    */

    // To use OpenCV's reprojectImageTo3D we need a Q matrix, which is obtained
    // from stereoRectify. This means we'll have to extract some data from the
    // device itself, which is why this is done here. 
    // TODO: actually calculate these
    //cv::Mat R1, R2, P1, P2, Q;

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
        "Examples/Stereo-Inertial/EuRoC.yaml",
        ORB_SLAM3::System::IMU_STEREO, 
        true
    );

    // Formatter, for printing out matrices in a reasonable way.
    /*
    cv::Ptr<cv::Formatter> fmt = cv::Formatter::get(cv::Formatter::FMT_DEFAULT);
    fmt->set64fPrecision(3);
    fmt->set32fPrecision(3);
    */

    // We also want somewhere to store our pose data
    // Pose pose;

    // The time of each frame is required for SLAM, so we take an epoch time
    // (i.e. our start time) now
    std::chrono::_V2::steady_clock::time_point slam_epoch = std::chrono::steady_clock::now();

    // CAMERA.
    sl::Camera zed;
    sl::InitParameters init_parameters;
    init_parameters.sdk_verbose = true;
    init_parameters.camera_resolution= sl::RESOLUTION::HD720;
    init_parameters.depth_mode = sl::DEPTH_MODE::NONE; 
    init_parameters.coordinate_system = sl::COORDINATE_SYSTEM::IMAGE;
    zed.open(init_parameters);        

    vector<ORB_SLAM3::IMU::Point> vImuMeas;

    // CREATE THREAD
    auto thread1 = std::thread(&thread_IMU, std::ref(zed), std::ref(vImuMeas), std::ref(slam_epoch));
    auto thread2 = std::thread(&thread_MAIN, std::ref(zed), std::ref(vImuMeas), std::ref(slam_epoch), std::ref(SLAM));
    thread1.join();
    thread2.join();

    return 0;

    // Now for the main loop
    /*while (1) {
        // Read the output frames from the OAK-D. These are blocking calls, so
        // they will wait until there's data available.
        /*auto rectif_left_frame = rectif_left_queue->get<dai::ImgFrame>();
        auto rectif_right_frame = rectif_left_queue->get<dai::ImgFrame>();
        auto disp_map_frame = disp_queue->get<dai::ImgFrame>();

        // Convert the frames into opencv images
        auto rectif_left = imgframe_to_mat(rectif_left_frame);
        auto rectif_right = imgframe_to_mat(rectif_right_frame);
        auto disp_map = imgframe_to_mat(disp_map_frame);

        zed.grab();
        zed.getSensorsData(sensors_data, sl::TIME_REFERENCE::IMAGE);

        zed.retrieveImage(zed_image_G, sl::VIEW::LEFT);
        zed.retrieveImage(zed_image_D, sl::VIEW::RIGHT);

        imLeft = cv::Mat((int) zed_image_G.getHeight(), (int) zed_image_G.getWidth(), CV_8UC4, zed_image_G.getPtr<sl::uchar1>(sl::MEM::CPU));
        imRight = cv::Mat((int) zed_image_D.getHeight(), (int) zed_image_D.getWidth(), CV_8UC4, zed_image_D.getPtr<sl::uchar1>(sl::MEM::CPU));

        // Get the time between the epoch and now, allowing us to get a
        // timestamp (in seconds) to pass into the slam system.
        auto elapsed_time = std::chrono::steady_clock::now() - slam_epoch;
        double frame_timestamp_s = elapsed_time.count() / 1000000000.0;

        std::cout << std::setprecision(4) << frame_timestamp_s << ": ";

        sl::float3 linear_acceleration = imu_data.linear_acceleration;
        sl::float3 angular_velocity = imu_data.angular_velocity;

        /*vImuMeas.clear();
        vImuMeas.push_back(ORB_SLAM3::IMU::Point(sensors_data.imu.linear_acceleration[0],sensors_data.imu.linear_acceleration[1],sensors_data.imu.linear_acceleration[2],
                                        sensors_data.imu.angular_velocity[0],sensors_data.imu.angular_velocity[1],sensors_data.imu.angular_velocity[2],
                                        frame_timestamp_s));

        // Pass the images into the SLAM system. This produces a matrix with
        // the pose information of the camera.
        cv::Mat raw_pose = SLAM.TrackStereo(
            imLeft,
            imRight,
            frame_timestamp_s,
            vImuMeas
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

        // Apply a colormap to the filtered disparity map, but don't normalise
        // it. Normalising the map will mean that the color doesn't correspond
        // directly with disparity.
        cv::Mat colour_disp;
        /*cv::applyColorMap(imLeft, colour_disp, cv::COLORMAP_JET);
        cv::imshow("disparity", imLeft);

        // See if q pressed, if so quit
        if (cv::waitKey(1) == 'q') {
            break;
        }
    }

    // Stop all SLAM threads
    SLAM.Shutdown();

    return EXIT_SUCCESS;*/
}