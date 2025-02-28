#include "camera/realsense.h"
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

RealSenseCamera::RealSenseCamera()
{
    try
    {
        // Initialize ROS node handle
        nh_ = std::make_shared<ros::NodeHandle>();
        it_ = std::make_shared<image_transport::ImageTransport>(*nh_);
        
        // Subscribe to the topics published by rs_camera.launch
        color_sub_ = it_->subscribe("/camera/color/image_raw", 1, &RealSenseCamera::colorCallback, this);
        depth_sub_ = it_->subscribe("/camera/depth/image_rect_raw", 1, &RealSenseCamera::depthCallback, this);
        
        Device_connected_.store(true);
        std::cout << "RealSense camera node initialized." << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cerr << "RealSense initialization error: " << e.what() << '\n';
        Device_connected_.store(false);
    }
}

RealSenseCamera::RealSenseCamera(int width, int height, int fps)
{
    try
    {
        // Initialize ROS node handle
        nh_ = std::make_shared<ros::NodeHandle>();
        it_ = std::make_shared<image_transport::ImageTransport>(*nh_);
        
        // Store the desired resolution and fps for potential parameter setting
        width_ = width;
        height_ = height;
        fps_ = fps;
        
        // Subscribe to the topics published by rs_camera.launch
        color_sub_ = it_->subscribe("/camera/color/image_raw", 1, &RealSenseCamera::colorCallback, this);
        depth_sub_ = it_->subscribe("/camera/depth/image_rect_raw", 1, &RealSenseCamera::depthCallback, this);
        
        // Subscribe to the point cloud for depth measurements
        pointcloud_sub_ = nh_->subscribe("/camera/depth/points", 1, &RealSenseCamera::pointCloudCallback, this);
        
        Device_connected_.store(true);
        std::cout << "RealSense camera node initialized with resolution " << width << "x" << height << " @ " << fps << "fps" << std::endl;
    }
    catch (const std::exception &e)
    {
        std::cerr << "RealSense initialization error: " << e.what() << '\n';
        Device_connected_.store(false);
    }
}

RealSenseCamera::~RealSenseCamera()
{
    if (running_)
    {
        Stop();
    }
}

void RealSenseCamera::Start()
{
    running_ = true;
    checking_thread_ = std::thread(&RealSenseCamera::CheckingDevice, this);
    ros_spinner_thread_ = std::thread([this]() {
        ros::Rate rate(30); // Check at 30Hz
        while (running_ && ros::ok())
        {
            ros::spinOnce();
            rate.sleep();
        }
    });
}

void RealSenseCamera::Stop()
{
    running_ = false;
    
    if (ros_spinner_thread_.joinable())
    {
        ros_spinner_thread_.join();
    }
    
    if (checking_thread_.joinable())
    {
        checking_thread_.join();
    }
    
    fprintf(stderr, "[realsense] camera stopped\n");
}

void RealSenseCamera::colorCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        std::lock_guard<std::mutex> lock(mutex_color_frame_);
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        color_mat_ = cv_ptr->image;
        last_color_timestamp_ = ros::Time::now();
    }
    catch (cv_bridge::Exception& e)
    {
        std::cerr << "Color frame conversion error: " << e.what() << std::endl;
    }
}

void RealSenseCamera::depthCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
        
        {
            std::lock_guard<std::mutex> lock(mutex_depth_frame_);
            // Convert the depth image to color map for visualization
            cv::Mat depth_colormap;
            cv::normalize(cv_ptr->image, depth_colormap, 0, 255, cv::NORM_MINMAX, CV_8UC1);
            cv::applyColorMap(depth_colormap, depth_mat_, cv::COLORMAP_JET);
            
            // Store raw depth data for depth measurements
            raw_depth_mat_ = cv_ptr->image;
        }
        
        last_depth_timestamp_ = ros::Time::now();
    }
    catch (cv_bridge::Exception& e)
    {
        std::cerr << "Depth frame conversion error: " << e.what() << std::endl;
    }
}

void RealSenseCamera::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    // Store the point cloud for potential distance measurements
    std::lock_guard<std::mutex> lock(mutex_pointcloud_);
    latest_pointcloud_ = cloud_msg;
}

cv::Mat RealSenseCamera::GetColorFrame()
{
    std::lock_guard<std::mutex> lock(mutex_color_frame_);
    return color_mat_.clone(); // Return a copy to avoid thread conflicts
}

cv::Mat RealSenseCamera::GetDepthFrame()
{
    std::lock_guard<std::mutex> lock(mutex_depth_frame_);
    return depth_mat_.clone(); // Return a copy to avoid thread conflicts
}

float RealSenseCamera::GetDepthValue(cv::Point mid_point)
{
    float distance = 0.0f;
    try
    {
        std::lock_guard<std::mutex> lock(mutex_depth_frame_);
        
        // Check if the point is within the image bounds
        if (mid_point.x >= 0 && mid_point.x < raw_depth_mat_.cols && 
            mid_point.y >= 0 && mid_point.y < raw_depth_mat_.rows)
        {
            // Get the depth value at the specified point
            uint16_t depth_value = raw_depth_mat_.at<uint16_t>(mid_point.y, mid_point.x);
            
            // Convert to meters (assuming the depth is in millimeters)
            distance = static_cast<float>(depth_value) / 1000.0f;
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error getting depth value: " << e.what() << '\n';
    }
    return distance;
}

void RealSenseCamera::CheckingDevice()
{
    ros::Rate rate(10); // Check connection at 10Hz
    
    while (running_ && ros::ok())
    {
        // Check if we've received recent frames
        ros::Time current_time = ros::Time::now();
        ros::Duration color_duration = current_time - last_color_timestamp_;
        ros::Duration depth_duration = current_time - last_depth_timestamp_;
        
        bool frames_received = (color_duration.toSec() < 1.0) && (depth_duration.toSec() < 1.0);
        bool was_connected = Device_connected_.load();
        
        if (!frames_received && was_connected)
        {
            std::cout << "RealSense device disconnected or not publishing frames!" << std::endl;
            Device_connected_.store(false);
        }
        else if (frames_received && !was_connected)
        {
            std::cout << "RealSense device reconnected and publishing frames." << std::endl;
            Device_connected_.store(true);
        }
        
        rate.sleep();
    }
}