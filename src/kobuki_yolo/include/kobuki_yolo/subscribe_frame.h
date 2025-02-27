#ifndef SUBS_FRAME_H_
#define SUBS_FRAME_H_

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <std_msgs/Header.h>
#include <iostream>
#include <sstream>

class RealsenseSubscriber
{
public:
    RealsenseSubscriber(ros::NodeHandle &nh) : it_(nh)
    {
        rgb_sub_.subscribe(nh, "/camera/color/image_raw", 1);
        depth_sub_.subscribe(nh, "/camera/aligned_depth_to_color/image_raw", 1);
        
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_policy;
        sync_.reset(new message_filters::Synchronizer<sync_policy>(sync_policy(10), rgb_sub_, depth_sub_));
        sync_->registerCallback(boost::bind(&RealsenseSubscriber::imageCallback, this, _1, _2));
        
        cv::namedWindow("RGB Image", cv::WINDOW_AUTOSIZE);
        cv::namedWindow("Depth Image", cv::WINDOW_AUTOSIZE);
        cv::namedWindow("Combined View", cv::WINDOW_AUTOSIZE);
        
        ROS_INFO("RealSense subscriber initialized. Waiting for images...");
        ROS_INFO("Press 's' to save current frame, 'q' to quit");
    }
    
    ~RealsenseSubscriber()
    {
        cv::destroyAllWindows();
    }

    cv::Mat rgb_image;
    cv::Mat depth_image;


private:
    void imageCallback(const sensor_msgs::Image::ConstPtr& rgb_msg,
                      const sensor_msgs::Image::ConstPtr& depth_msg);
    void addDepthInfo(cv::Mat& img, const cv::Mat& depth_img, int rgb_width);
    
    image_transport::ImageTransport it_;
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub_;
    message_filters::Subscriber<sensor_msgs::Image> depth_sub_;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;
    typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;
    boost::shared_ptr<Synchronizer> sync_;
};

#endif // SUBS_FRAME_H_
