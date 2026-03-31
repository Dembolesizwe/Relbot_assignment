#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "example_interfaces/msg/float64.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>


class GreenObjectDetector : public rclcpp::Node
{
public:
  GreenObjectDetector()
  : Node("green_object_detector")
  {
    //Subscribe to the image topic
    subscription_camera_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/image", 10, std::bind(&GreenObjectDetector::compute_center, this, std::placeholders::_1));

    //Publish the coordinates of the detected object
    publisher_coordinates_ = this->create_publisher<geometry_msgs::msg::PointStamped>("/object_center", 10);

    publisher_radius_ = this->create_publisher<example_interfaces::msg::Float64>("/object_radius", 10);
    
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

    // Get image size
    int width = cv_image.cols;
    int height = cv_image.rows;

    //Gaussian blur
    cv::GaussianBlur(cv_image, cv_image, cv::Size(11, 11), 0);

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
    // Convert mask to color so we can draw on it
    cv::Mat debug_image;
    cv::cvtColor(mask, debug_image, cv::COLOR_GRAY2BGR);

    // Find contours
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (!contours.empty()) {
        auto largest = std::max_element(contours.begin(), contours.end(),
            [](const std::vector<cv::Point> &a, const std::vector<cv::Point> &b){
                return cv::contourArea(a) < cv::contourArea(b);
            });

        // Fit circle to contour
        cv::Point2f center;
        float radius;
        cv::minEnclosingCircle(*largest, center, radius);

        //Draw contour and circle on debug image to show what it detects
        //Draw contour (blue)
        cv::drawContours(debug_image, std::vector<std::vector<cv::Point>>{*largest}, -1, cv::Scalar(255, 0, 0), 2);
        //Draw enclosing circle (green)
        cv::circle(debug_image, center, (int)radius, cv::Scalar(0, 255, 0), 2);
        //Draw center (red)
        cv::circle(debug_image, center, 5, cv::Scalar(0, 0, 255), -1);
        //Show debug image
        cv::imshow("Mask + Contour Debug", debug_image);
        cv::waitKey(1); 

        //Publish radius
        example_interfaces::msg::Float64 radius_norm;
        //Normalize radius to height. If radius is half the height, then the circle is as big as image, so 1.
        radius_norm.data = radius/cv_image.rows * 2.0;
        publisher_radius_->publish(radius_norm);

        //Publish center
        //Normalize coordinates around center
        double cx_norm = (center.x - width / 2.0) / (width / 2.0);
        double cy_norm_flip = (center.y - height / 2.0) / (height / 2.0);
        //Flip y-axis so up is positive (more intuitive)
        double cy_norm = -cy_norm_flip;

        geometry_msgs::msg::PointStamped point_msg;
        point_msg.header = msg->header;
        point_msg.point.x = cx_norm;
        point_msg.point.y = cy_norm;
        point_msg.point.z = 0.0;
        publisher_coordinates_->publish(point_msg);

        RCLCPP_INFO(this->get_logger(), "GREEN OBJECT: center: x=%.2f, y=%.2f, radius: %.2f", cx_norm, cy_norm, radius_norm.data);
    } else {
        RCLCPP_INFO(this->get_logger(), "No green object detected");
    }
}

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_camera_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr publisher_coordinates_;
  rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr publisher_radius_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GreenObjectDetector>());
  rclcpp::shutdown();
  return 0;
}