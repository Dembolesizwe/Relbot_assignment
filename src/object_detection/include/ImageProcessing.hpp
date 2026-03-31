#ifndef IMAGE_PROCESSING_HPP
#define IMAGE_PROCESSING_HPP

#include <rclcpp/rclcpp.hpp>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include "geometry_msgs/msg/point_stamped.hpp"

class ImageProcessing : public rclcpp::Node {
public:
    
    const std::string WEBCAM_IMAGE = "/image";
    
    //constructor
    ImageProcessing();
    
    //destructor
  ~ImageProcessing()=default;

private:
    // Takes the raw frame, returns a binary mask of green pixels
    cv::Mat locate_green(const cv::Mat &input_frame);

    // Takes the mask, returns the midpoint-based center
    cv::Point find_centroid(const cv::Mat &mask);

    // Callback function for the image subscriber
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);


    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr Image_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr Image_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pixel_coords_publisher_;
};


#endif // IMAGE_PROCESSING_HPP