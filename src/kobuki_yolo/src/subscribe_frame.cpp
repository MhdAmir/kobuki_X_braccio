#include "kobuki_yolo/subscribe_frame.h"


void RealsenseSubscriber::imageCallback(const sensor_msgs::Image::ConstPtr &rgb_msg,
                                        const sensor_msgs::Image::ConstPtr &depth_msg)
{
    cv_bridge::CvImagePtr cv_rgb_ptr, cv_depth_ptr;

    try
    {
        cv_rgb_ptr = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8);
        cv_depth_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);

        rgb_image = cv_rgb_ptr->image;
        depth_image = cv_depth_ptr->image;

        // Process the depth image for visualization
        cv::Mat depth_display;
        depth_image.convertTo(depth_display, CV_8UC1, 255.0 / 10000.0); // Assuming max depth is 10m
        cv::applyColorMap(depth_display, depth_display, cv::COLORMAP_JET);

        // Create a combined view
        cv::Mat combined;
        if (rgb_image.size() == depth_display.size())
        {
            cv::hconcat(rgb_image, depth_display, combined);
        }
        else
        {
            cv::resize(depth_display, depth_display, rgb_image.size());
            cv::hconcat(rgb_image, depth_display, combined);
        }

        // Add depth information at multiple points
        addDepthInfo(combined, depth_image, rgb_image.cols);

        cv::imshow("RGB Image", rgb_image);
        cv::imshow("Depth Image", depth_display);
        cv::imshow("Combined View", combined);

        int key = cv::waitKey(1);
        if (key == 'q' || key == 'Q')
        {
            ros::shutdown();
        }
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("CV bridge exception: %s", e.what());
    }
}

void RealsenseSubscriber::addDepthInfo(cv::Mat &img, const cv::Mat &depth_img, int rgb_width)
{
    std::vector<cv::Point> points = {
        cv::Point(img.cols / 4, img.rows / 4),         // Top-left quadrant
        cv::Point(img.cols / 4, 3 * img.rows / 4),     // Bottom-left quadrant
        cv::Point(3 * img.cols / 4, img.rows / 4),     // Top-right quadrant
        cv::Point(3 * img.cols / 4, 3 * img.rows / 4), // Bottom-right quadrant
        cv::Point(img.cols / 2, img.rows / 2)          // Center
    };

    for (const auto &pt : points)
    {
        cv::Point depth_pt = pt;
        if (pt.x > rgb_width)
        {
            depth_pt.x = pt.x - rgb_width;
        }

        cv::line(img, cv::Point(pt.x - 10, pt.y), cv::Point(pt.x + 10, pt.y), cv::Scalar(0, 255, 0), 2);
        cv::line(img, cv::Point(pt.x, pt.y - 10), cv::Point(pt.x, pt.y + 10), cv::Scalar(0, 255, 0), 2);

        if (depth_pt.x < depth_img.cols && depth_pt.y < depth_img.rows)
        {
            uint16_t depth_value = depth_img.at<uint16_t>(depth_pt.y, depth_pt.x);
            float depth_in_meters = depth_value; // mm

            // Display depth value
            std::stringstream ss;
            ss << std::fixed << std::setprecision(2) << depth_in_meters << "mm";
            cv::putText(img, ss.str(), cv::Point(pt.x + 15, pt.y), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
        }
    }
}
