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
        color_sub_ = it_->subscribe("/camera1/camera/color/image_raw", 1, &RealSenseCamera::colorCallback, this);
        depth_sub_ = it_->subscribe("/camera1/camera/aligned_depth_to_color/image_raw", 1, &RealSenseCamera::depthCallback, this);
        camera_info_sub_ = nh_->subscribe("/camera1/camera/aligned_depth_to_color/camera_info", 1, &RealSenseCamera::cameraInfoCallback, this);

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
        color_sub_ = it_->subscribe("/camera1/camera/color/image_raw", 1, &RealSenseCamera::colorCallback, this);
        depth_sub_ = it_->subscribe("/camera1/camera/aligned_depth_to_color/image_raw", 1, &RealSenseCamera::depthCallback, this);
        
        // Subscribe to the point cloud for depth measurements
        
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

// void RealSenseCamera::pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
// {
//     // Store the point cloud for potential distance measurements
//     std::lock_guard<std::mutex> lock(mutex_pointcloud_);
//     latest_pointcloud_ = cloud_msg;
// }

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

float RealSenseCamera::GetDepthValue(cv::Rect area)
{
    float distance = 0.0f;
    try
    {
        std::lock_guard<std::mutex> lock(mutex_depth_frame_);
        
        // Extract depth values within the area
        cv::Mat depth_roi = raw_depth_mat_(area);
        depth_roi.convertTo(depth_roi, CV_32F, 1.0 / 1000.0); // Convert to meters

        // Reshape the ROI into a single column of depth values
        cv::Mat depth_values = depth_roi.reshape(1, depth_roi.total());

        // Apply k-means clustering to separate object and background
        int k = 2; // Assume 2 clusters: object and background
        cv::Mat labels, centers;
        cv::kmeans(depth_values, k, labels, cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 10, 1.0), 3, cv::KMEANS_PP_CENTERS, centers);

        // Find the cluster with the smaller depth (assumed to be the object)
        int object_cluster = (centers.at<float>(0) < centers.at<float>(1)) ? 0 : 1;

        // Calculate the average depth of the object cluster
        float sum = 0.0f;
        int count = 0;
        for (int i = 0; i < labels.rows; i++)
        {
            if (labels.at<int>(i) == object_cluster)
            {
                sum += depth_values.at<float>(i);
                count++;
            }
        }

        if (count > 0)
        {
            distance = sum / static_cast<float>(count);
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error getting depth value: " << e.what() << '\n';
    }
    return distance;
}

float RealSenseCamera::GetDepthValue(cv::Point mid_point, int window_size)
{
    float distance = 0.0f;
    int valid_points = 0;
    std::vector<float> valid_depths;
    
    try
    {
        std::lock_guard<std::mutex> lock(mutex_depth_frame_);
        
        // Define the region window (square with mid_point at center)
        int half_window = window_size / 2;
        int start_x = std::max(0, mid_point.x - half_window);
        int end_x = std::min(raw_depth_mat_.cols - 1, mid_point.x + half_window);
        int start_y = std::max(0, mid_point.y - half_window);
        int end_y = std::min(raw_depth_mat_.rows - 1, mid_point.y + half_window);
        
        // Collect valid depth values in the window
        for (int y = start_y; y <= end_y; y++)
        {
            for (int x = start_x; x <= end_x; x++)
            {
                uint16_t depth_value = raw_depth_mat_.at<uint16_t>(y, x);
                
                // Filter out zero or invalid readings
                if (depth_value > 0 && depth_value < 65000)  // Filter max value for depth sensor
                {
                    float depth_meters = static_cast<float>(depth_value) / 1000.0f;
                    valid_depths.push_back(depth_meters);
                    valid_points++;
                }
            }
        }
        
        // Calculate result based on collected values
        if (valid_points > 0)
        {
            // Sort the depths to find median
            std::sort(valid_depths.begin(), valid_depths.end());
            
            // If we have enough points, remove outliers
            if (valid_points >= 5)
            {
                // Calculate median or trimmed mean (remove top and bottom 10%)
                int trim_count = valid_points / 10;  // 10% of points
                float sum = 0.0f;
                
                // Sum the middle 80% of values (trimmed mean)
                for (int i = trim_count; i < valid_points - trim_count; i++)
                {
                    sum += valid_depths[i];
                }
                
                distance = sum / (valid_points - 2 * trim_count);
            }
            else
            {
                // For few points, use median which is more robust than mean
                distance = valid_depths[valid_points / 2];
            }
        }
        else
        {
            // No valid depth points in window
            distance = 0.0f;
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error getting depth value: " << e.what() << '\n';
    }
    
    return distance;
}

rs2_vector RealSenseCamera::GetXYZDepthValues(cv::Point mid_point, int window_size)
{
    rs2_vector xyz_values = {0.0f, 0.0f, 0.0f}; // x, depth (y), z
    int valid_points = 0;
    std::vector<rs2_vector> valid_points_xyz;
    
    try
    {
        std::lock_guard<std::mutex> lock(mutex_depth_frame_);
        
        // Define the region window (square with mid_point at center)
        int half_window = window_size / 2;
        int start_x = std::max(0, mid_point.x - half_window);
        int end_x = std::min(raw_depth_mat_.cols - 1, mid_point.x + half_window);
        int start_y = std::max(0, mid_point.y - half_window);
        int end_y = std::min(raw_depth_mat_.rows - 1, mid_point.y + half_window);
        
        // Collect valid depth values in the window
        for (int y = start_y; y <= end_y; y++)
        {
            for (int x = start_x; x <= end_x; x++)
            {
                uint16_t depth_value = raw_depth_mat_.at<uint16_t>(y, x);
                
                // Filter out zero or invalid readings
                if (depth_value > 0 && depth_value < 65000) // Filter max value for depth sensor
                {
                    float depth_meters = static_cast<float>(depth_value) / 1000.0f;
                    
                    // Deproject from pixel to 3D point
                    rs2_vector point_3d = DeprojectPixelToPoint(x, y, depth_meters);
                    
                    valid_points_xyz.push_back(point_3d);
                    valid_points++;
                }
            }
        }
        
        // Calculate result based on collected values
        if (valid_points > 0)
        {
            // Sort the points by depth to find median
            std::sort(valid_points_xyz.begin(), valid_points_xyz.end(), 
                [](const rs2_vector& a, const rs2_vector& b) { return a.y < b.y; });
            
            // If we have enough points, remove outliers
            if (valid_points >= 5)
            {
                // Calculate median or trimmed mean (remove top and bottom 10%)
                int trim_count = valid_points / 10; // 10% of points
                float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
                
                // Sum the middle 80% of values (trimmed mean)
                for (int i = trim_count; i < valid_points - trim_count; i++)
                {
                    sum_x += valid_points_xyz[i].x;
                    sum_y += valid_points_xyz[i].y;
                    sum_z += valid_points_xyz[i].z;
                }
                
                int included_points = valid_points - 2 * trim_count;
                xyz_values.x = sum_x / included_points;
                xyz_values.y = sum_y / included_points;  // This is the depth
                xyz_values.z = sum_z / included_points;
            }
            else
            {
                // For few points, use median which is more robust than mean
                int median_idx = valid_points / 2;
                xyz_values.x = valid_points_xyz[median_idx].x;
                xyz_values.y = valid_points_xyz[median_idx].y;  // This is the depth
                xyz_values.z = valid_points_xyz[median_idx].z;
            }
        }
        else
        {
            xyz_values = {0.0f, 0.0f, 0.0f};
        }
    }
    catch (const std::exception &e)
    {
        std::cerr << "Error getting XYZ values: " << e.what() << '\n';
    }
    
    return xyz_values;
}

rs2_vector RealSenseCamera::DeprojectPixelToPoint(int pixel_x, int pixel_y, float depth)
{
    rs2_vector point_3d = {0.0f, 0.0f, 0.0f};
    
    // Get camera intrinsics
    rs2_intrinsics intrinsics = depth_intrinsics_;

    // std::cout << "Intrinsics - fx: " << intrinsics.fx << ", fy: " << intrinsics.fy 
    // << ", ppx: " << intrinsics.ppx << ", ppy: " << intrinsics.ppy << std::endl;
    
    // Convert from pixel coordinates to normalized coordinates
    float x = (pixel_x - intrinsics.ppx) / intrinsics.fx;
    float y = (pixel_y - intrinsics.ppy) / intrinsics.fy;
    
    // Calculate 3D coordinates

    point_3d.x = x * depth * 1000;
    point_3d.y = y * depth * 1000;
    point_3d.z = depth * 1000;
    
    // fprintf(stderr, "point x >> %f\n", point_3d.x);
    // fprintf(stderr, "point y >> %f\n", point_3d.y);
    // fprintf(stderr, "point z >> %f\n\n", point_3d.z);
    return point_3d;
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