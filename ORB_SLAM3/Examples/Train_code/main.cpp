// ZED include
#include <stdio.h>
#include <string.h>
#include <sl/Camera.hpp>
#include <iostream>
#include <thread>

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

void thread2(sl::Camera & zed)
{
    sl::SensorsData sensors_data;
    sl::SensorsData::IMUData imu_data;
    TimestampHandler ts;

    while(1)
    {
        while((zed.getSensorsData(sensors_data, sl::TIME_REFERENCE::CURRENT) == sl::ERROR_CODE::SUCCESS) && ts.isNew(sensors_data.imu))
        {
            //std::cout << sensors_data.imu.linear_acceleration << " " << sensors_data.imu.effective_rate << std::endl;
            std::cout << "IMU NEW DATA " << sensors_data.imu.effective_rate << std::endl;
        }
    }
}

void thread1(sl::Camera & zed)
{
    sl::Mat image;
    while(1)
    {
        if(zed.grab() == sl::ERROR_CODE::SUCCESS)
            {
                if(zed.retrieveImage(image, sl::VIEW::LEFT) == sl::ERROR_CODE::SUCCESS)
                {
                    std::cout << ">>> CAMERA NEW DATA" << std::endl;
                }
            }
    }
}

int main(int argc, char** argv)
{
    sl::Camera zed;

    // Set configuration parameters
    sl::InitParameters init_parameters;
    // No depth computation required here
    init_parameters.depth_mode = sl::DEPTH_MODE::NONE;
    init_parameters.camera_fps = 15;
    sl::ERROR_CODE err = zed.open(init_parameters);
    if (err != sl::ERROR_CODE::SUCCESS) {
        std::cout << "Error " << err << ", exit program.\n";
        return -1;
    }

    std::thread first (&thread1, std::ref(zed));
    std::thread second (&thread2, std::ref(zed));

    first.join();
    second.join();

    while(1)
    {
        int a = 0;
    }

    /*
    // Open the camera
    sl::ERROR_CODE err = zed.open(init_parameters);
    if (err != sl::ERROR_CODE::SUCCESS) {
        std::cout << "Error " << err << ", exit program.\n";
        return -1;
    }


    sl::SensorsData sensors_data;
    sl::Mat image;
    sl::SensorsData::IMUData imu_data;
    TimestampHandler ts;*/

    /*
    while(0)
    {   
        if(zed.grab() == sl::ERROR_CODE::SUCCESS)
        {
            if(zed.retrieveImage(image, sl::VIEW::LEFT) == sl::ERROR_CODE::SUCCESS)
            {
                std::cout << "CAMERA NEW DATA" << std::endl;
            }
        }

        while((zed.getSensorsData(sensors_data, sl::TIME_REFERENCE::CURRENT) == sl::ERROR_CODE::SUCCESS) && ts.isNew(sensors_data.imu))
        {
        //if (ts.isNew(sensors_data.imu)){
            std::cout << sensors_data.imu.linear_acceleration << " " << sensors_data.imu.effective_rate << std::endl;
        //}
        }
    }

    while(1)
    {
        while((zed.getSensorsData(sensors_data, sl::TIME_REFERENCE::CURRENT) == sl::ERROR_CODE::SUCCESS) && ts.isNew(sensors_data.imu))
        {
        //if (ts.isNew(sensors_data.imu)){
            std::cout << sensors_data.imu.linear_acceleration << " " << sensors_data.imu.effective_rate << std::endl;
        }
    }*/

    // Grab new frames and retrieve sensors data
    /*while(zed.grab() == sl::ERROR_CODE::SUCCESS){
        zed.getSensorsData(sensors_data, sl::TIME_REFERENCE::IMAGE); // Retrieve only frame synchronized data

        // Extract IMU data
        imu_data = sensors_data.imu;

        // Retrieve linear acceleration and angular velocity
        sl::float3 linear_acceleration = imu_data.linear_acceleration;
        sl::float3 angular_velocity = imu_data.angular_velocity;

        if (ts.isNew(sensors_data.imu)) {
            std::cout << "IMU Orientation: {" << sensors_data.imu.pose.getEulerAngles() << "}" << std::endl;
            std::cout << "IMU Linear Acceleration: {" << sensors_data.imu.linear_acceleration[0] << "} [m/sec^2]"<< std::endl;
            std::cout << "IMU Angular Velocity: {" << sensors_data.imu.angular_velocity << "} [deg/sec]"<< std::endl << std::endl;
}*/
    

    return 0;
}
