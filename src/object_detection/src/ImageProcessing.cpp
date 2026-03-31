// ROS 2 Core
#include "rclcpp/rclcpp.hpp"

// ROS 2 Message types
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"

// The "Bridge" between ROS and OpenCV
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>

// OpenCV Libraries
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Standard C++ for math and placeholders
#include <memory>
#include <vector>
#include <functional>


#include "ImageProcessing.hpp" 

ImageProcessing::ImageProcessing() : Node("image_processing_node") {
    // Note: Ensure WEBCAM_IMAGE is defined in your header or use the string "/image"
    Image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/image", 10, 
        std::bind(&ImageProcessing::image_callback, this, std::placeholders::_1)
    );
    
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("/processed_image", 10);
    pixel_coords_publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/center_pos", 10);
}

void ImageProcessing::image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        cv::Mat frame = cv_ptr->image;

        // 1. locate_green now returns the small mask
        cv::Mat green_mask = locate_green(frame);

        // 2. find_centroid calculates average on small mask and scales up
        cv::Point center = find_centroid(green_mask);

        if (center.x != -1) {
            // Use the original frame.cols for normalization since center was scaled back up
            float normX = (float)(center.x - (frame.cols / 2)) / (frame.cols / 2);
            float normY = (float)((frame.rows / 2) - center.y) / (frame.rows / 2);
            
            RCLCPP_INFO(this->get_logger(), "Green Object Center: [%.2f, %.2f]", normX, normY);
            
            // Publish the pixel coordinates as a PointStamped message
            // 2. Create the Message object
    auto point_msg = geometry_msgs::msg::PointStamped();

    // 3. Set the Header (Very important for ROS 2)
    point_msg.header.stamp = this->now();
    point_msg.header.frame_id = "camera_link"; // Or whatever your frame is named

    // 4. Set the Point values
    // We put our normalized vision data into x and y
    point_msg.point.x = normX;
    point_msg.point.y = normY;
    point_msg.point.z = 0.0; // Vision is 2D, so Z is zero

    // 5. Publish!
    pixel_coords_publisher_->publish(point_msg);


            // Draw on the ORIGINAL high-res frame for the video stream
            cv::circle(frame, center, 10, cv::Scalar(0, 0, 255), -1);
        }

        // 3. Publish to see in showimage
        auto out_msg = cv_bridge::CvImage(msg->header, "bgr8", frame).toImageMsg();
        image_pub_->publish(*out_msg);

    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
}

cv::Mat ImageProcessing::locate_green(const cv::Mat &input_frame) {
    cv::Mat small_frame, hsv_frame, mask;
    
    // Resize FIRST
    cv::resize(input_frame, small_frame, cv::Size(), 0.5, 0.5, cv::INTER_NEAREST);

    // Convert the SMALL frame to HSV (This is where the speedup happens)
    cv::cvtColor(small_frame, hsv_frame, cv::COLOR_BGR2HSV);

    cv::Scalar lower_green(35, 100, 100);
    cv::Scalar upper_green(85, 255, 255);

    // Threshold the SMALL HSV frame
    cv::inRange(hsv_frame, lower_green, upper_green, mask);

    return mask; 
}

cv::Point ImageProcessing::find_centroid(const cv::Mat &mask) {
    long sumX = 0, sumY = 0;
    int count = 0;

    for (int y = 0; y < mask.rows; y++) {
        const uchar* row = mask.ptr<uchar>(y);
        for (int x = 0; x < mask.cols; x++) {
            if (row[x] == 255) {
                sumX += x;
                sumY += y;
                count++;
            }
        }
    }

    if (count > 0) {
        int avgX = static_cast<int>(sumX / count);
        int avgY = static_cast<int>(sumY / count);
        
        // Scale back up by 2 because the mask was 0.5x size
        return cv::Point(avgX * 2, avgY * 2);
    }
    
    return cv::Point(-1, -1);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageProcessing>());
  rclcpp::shutdown();
  return 0;
}