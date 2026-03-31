#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>


class GreenObjectDetector : public rclcpp::Node
{
public:
  GreenObjectDetector()
  : Node("green_object_detector")
  {
    //Subscribe to the image topic
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/image", 10, std::bind(&GreenObjectDetector::compute_center, this, std::placeholders::_1));

    //Publish the coordinates of the detected object
    publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/object_center", 10);
    
    RCLCPP_INFO(this->get_logger(), "Green Object Detector Node has been started.");
  }

private:
  void compute_center(const sensor_msgs::msg::Image::SharedPtr msg)
{
    cv::Mat cv_image;
    try {
        cv_image = cv_bridge::toCvShare(msg, "bgr8")->image;
    } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "CV Bridge Error: %s", e.what());
        return;
    }

    // Convert BGR to HSV
    cv::Mat hsv;
    cv::cvtColor(cv_image, hsv, cv::COLOR_BGR2HSV);

    // Green color range
    cv::Scalar lower_green(40, 50, 50);
    cv::Scalar upper_green(90, 255, 255);

    // Create mask
    cv::Mat mask;
    cv::inRange(hsv, lower_green, upper_green, mask);

    // --- Debug visualization ---
    cv::imshow("Green Mask", mask);
    cv::waitKey(1);

    // Find contours
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (!contours.empty()) {
        auto largest = std::max_element(contours.begin(), contours.end(),
            [](const std::vector<cv::Point> &a, const std::vector<cv::Point> &b){
                return cv::contourArea(a) < cv::contourArea(b);
            });

        cv::Moments M = cv::moments(*largest);
        if (M.m00 > 0) {
            double cx = M.m10 / M.m00;
            double cy = M.m01 / M.m00;

            // Get image size
            int width = cv_image.cols;
            int height = cv_image.rows;

            // Normalize coordinates around center
            double cx_norm = (cx - width / 2.0) / (width / 2.0);
            double cy_norm_flip = (cy - height / 2.0) / (height / 2.0);

            // Optional: flip y-axis so up is positive (more intuitive for robotics)
            double cy_norm = -cy_norm_flip;

            geometry_msgs::msg::PointStamped point_msg;
            point_msg.header = msg->header;
            point_msg.point.x = cx_norm;
            point_msg.point.y = cy_norm;
            point_msg.point.z = 0.0;
            publisher_->publish(point_msg);

            RCLCPP_INFO(this->get_logger(), "Green object center: x=%.2f, y=%.2f", cx_norm, cy_norm);
        } else {
            RCLCPP_INFO(this->get_logger(), "No green object detected (empty moments)");
        }
    } else {
        RCLCPP_INFO(this->get_logger(), "No green object detected (no contours)");
    }
}

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GreenObjectDetector>());
  rclcpp::shutdown();
  return 0;
}