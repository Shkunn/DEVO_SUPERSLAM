#include <iostream>

#include <opencv2/opencv.hpp>

#include "System.h"

/* NEW */
#include <librealsense2/rs.hpp>
#include <iomanip>
#include <librealsense2/rsutil.h>
#include <array>
#include <cmath>
#include <ctime>
#include <iostream>
#include <mutex>
#include <sys/time.h>

using std::chrono::duration_cast;
using std::chrono::nanoseconds;
using std::chrono::system_clock;


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

// Global variable.
// std::mutex mon_mutex;
std::mutex data_mutex;

// vector<ORB_SLAM3::IMU::Point> vImuMeas_brut;
// vector<ORB_SLAM3::IMU::Point> vImuMeas_valid;
// vector<ORB_SLAM3::IMU::Point> vImuMeas_next;

/* -------------------------------------------------------------------------
 * NEW MAIN CHANGED
 * ------------------------------------------------------------------------- */

int main(int argc, char *argv[]) {

    ORB_SLAM3::System SLAM(
        "Vocabulary/ORBvoc.txt",
        "Examples/Stereo-Inertial/EuRocT265_IMU.yaml",
        ORB_SLAM3::System::IMU_STEREO, 
        true
    );

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe_t265;
    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;
    // Add pose stream
    // cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
    // Add motion stream
    cfg.enable_stream(RS2_STREAM_GYRO);
    cfg.enable_stream(RS2_STREAM_ACCEL);
    // Enable both image streams
    // Note: It is not currently possible to enable only one
    cfg.enable_stream(RS2_STREAM_FISHEYE, 1, RS2_FORMAT_Y8);
    cfg.enable_stream(RS2_STREAM_FISHEYE, 2, RS2_FORMAT_Y8);

    cv::Mat imLeft;
    cv::Mat imRight;

    vector<ORB_SLAM3::IMU::Point> vImuMeas_brut;
    vector<ORB_SLAM3::IMU::Point> vImuMeas_valid;
    vector<ORB_SLAM3::IMU::Point> vImuMeas_next;

    int width = 848;
	int height = 800;
    double current_time_stamp = 0;
    bool bNewFrame = false;

    // Acceleration float.
    float accel_data_x = 0;
    float accel_data_y = 0;
    float accel_data_z = 0;

    std::mutex data_mutex;

    uint64_t accel_counter = 0;
    uint64_t gyro_counter = 0;
    uint64_t motion_counter = 0;
    uint64_t frame_counter = 0;
    bool first_data = true;
    auto last_print = std::chrono::system_clock::now();
    auto callback = [&](const rs2::frame& frame)
    {
        std::lock_guard<std::mutex> lock(data_mutex);

        if (auto fp = frame.as<rs2::motion_frame>()) {
            if (fp && fp.get_profile().stream_type() == RS2_STREAM_GYRO && 
                fp.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F)
            {
                // Get the timestamp of the current frame
                double ts = fp.get_timestamp();
                
                auto millisec_since_epoch = duration_cast<nanoseconds>(system_clock::now().time_since_epoch()).count();
                double frame_timestamp_s = millisec_since_epoch / 1000000000.0;

                // Get gyro measurements
                rs2_vector gyro_data = fp.get_motion_data();

                gyro_counter++;

                vImuMeas_brut.push_back(ORB_SLAM3::IMU::Point(accel_data_x, accel_data_y, accel_data_z,
                                        gyro_data.x, gyro_data.y, gyro_data.z,
                                        frame_timestamp_s));

                // std::cout << "IMU TIMESTAMP:" << std::setprecision(20) << ts << std::endl;

            }
            // If casting succeeded and the arrived frame is from accelerometer stream
            if (fp && fp.get_profile().stream_type() == RS2_STREAM_ACCEL && 
                fp.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F)
            {
                double ts_accel = fp.get_timestamp();

                rs2_vector accel_data = fp.get_motion_data();

                accel_counter++;

                accel_data_x = accel_data.x;
                accel_data_y = accel_data.y;
                accel_data_z = accel_data.z;
            }
        }        
        else if (auto fs = frame.as<rs2::frameset>()) {
            double ts_frame = fs.get_timestamp();
            
            auto millisec_since_epoch = duration_cast<nanoseconds>(system_clock::now().time_since_epoch()).count();
            double frame_timestamp_s = millisec_since_epoch / 1000000000.0;

            if (frame_timestamp_s != current_time_stamp)
            {
                std::cout << "FRAME TIMESTAMP:" << std::setprecision(20) << frame_timestamp_s << std::endl;
                bNewFrame = true;
                current_time_stamp = frame_timestamp_s;
                //std::cout << "FRAME TIMESTAMP: " << std::setprecision(20) << ts_frame << std::endl;
                rs2::video_frame fisheye_frame_left = fs.get_fisheye_frame(1);
                rs2::video_frame fisheye_frame_right = fs.get_fisheye_frame(2);
                imLeft = cv::Mat(cv::Size(width, height), CV_8UC1, (void*)fisheye_frame_left.get_data());
                imRight = cv::Mat(cv::Size(width, height), CV_8UC1, (void*)fisheye_frame_right.get_data());
            }
        }
    };

    // Start streaming through the callback
    rs2::pipeline_profile profiles = pipe_t265.start(cfg, callback);

    // Sleep this thread until we are done
    while(true) {

        std::cout << "[WHILE LOOP]" << std::endl;

        if(bNewFrame){
            // mon_mutex.lock(); 
            std::lock_guard<std::mutex> lock(data_mutex);
            bNewFrame = false;

            std::cout << "CHECKIN' TWO: " << vImuMeas_brut.size() << std::endl;

            for (auto &&vector : vImuMeas_brut)
            {    
                std::cout << "I'M IN JAMES BOND" << std::endl;

                if(vector.t > current_time_stamp)
                {
                    std::cout << "[TIMESTAMP vImuMeas_next]: " << vector.t << std::endl;
                    vImuMeas_next.push_back(vector);

                }
                if(vector.t <= current_time_stamp)
                {
                    std::cout << "[TIMESTAMP vImuMeas_valid]: " << vector.t << std::endl;
                    vImuMeas_valid.push_back(vector);
                }
            }

            // std::cout << std::endl << std::endl;

            // exit(EXIT_FAILURE);

            
            std::cout << "TRACK STEREO STARTED" << std::endl;
            std::cout << "[FRAME TIMESTAMP inside while]: " << std::setprecision(20) << current_time_stamp << std::endl;

            cv::Mat raw_pose = SLAM.TrackStereo(
                imLeft,
                imRight,
                current_time_stamp,  
                vImuMeas_valid
            );

            std::cout << "TRACK STEREO ENDED" << std::endl;
            
            cv::imshow("disparity", imLeft);
            if (cv::waitKey(1) == 'q') {
                exit(1);
            }

            vImuMeas_brut.clear();

            for (int i=0; i<vImuMeas_next.size(); i++)
                vImuMeas_brut.push_back(vImuMeas_next[i]);

            vImuMeas_next.clear();

            vImuMeas_valid.clear();

            std::cout << std::endl;

            // mon_mutex.unlock();
        }
    }

    return 0;
}
