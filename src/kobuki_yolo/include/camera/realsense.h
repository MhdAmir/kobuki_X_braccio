#ifndef CAMERA_REALSENSE_H
#define CAMERA_REALSENSE_H

#include <opencv2/opencv.hpp>
#include <thread>
#include <mutex>
#include <atomic>
#include <memory>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <image_transport/image_transport.h>

class RealSenseCamera
{
public:
    RealSenseCamera();
    RealSenseCamera(int width, int height, int fps);
    ~RealSenseCamera();

    void Start();
    void Stop();

    cv::Mat GetColorFrame();
    cv::Mat GetDepthFrame();
    float GetDepthValue(cv::Point mid_point);

    std::atomic<bool> Device_connected_{false};

private:
    // Thread management
    bool running_ = false;
    std::thread checking_thread_;
    std::thread ros_spinner_thread_;

    // ROS components
    std::shared_ptr<ros::NodeHandle> nh_;
    std::shared_ptr<image_transport::ImageTransport> it_;
    image_transport::Subscriber color_sub_;
    image_transport::Subscriber depth_sub_;
    ros::Subscriber pointcloud_sub_;

    // Frame data
    cv::Mat color_mat_;
    cv::Mat depth_mat_;
    cv::Mat raw_depth_mat_;  // Raw depth data for measurements
    sensor_msgs::PointCloud2ConstPtr latest_pointcloud_;

    // Timestamps for connection monitoring
    ros::Time last_color_timestamp_;
    ros::Time last_depth_timestamp_;

    // Configuration parameters
    int width_ = 640;
    int height_ = 480;
    int fps_ = 30;

    // Thread safety
    std::mutex mutex_color_frame_;
    std::mutex mutex_depth_frame_;
    std::mutex mutex_pointcloud_;
    std::mutex mutex_depth_object_;

    // Callback functions for ROS subscribers
    void colorCallback(const sensor_msgs::ImageConstPtr& msg);
    void depthCallback(const sensor_msgs::ImageConstPtr& msg);
    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);
    
    // Device connection monitoring
    void CheckingDevice();
};

#endif // CAMERA_REALSENSE_H