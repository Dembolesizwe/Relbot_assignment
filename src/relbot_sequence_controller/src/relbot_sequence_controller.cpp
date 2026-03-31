#include "steering.hpp"


SteerRelbot::SteerRelbot() : Node("steer_relbot") {
    RCLCPP_INFO(this->get_logger(), "Init");

    // initialize topics
    create_topics();
    RCLCPP_INFO(this->get_logger(), "Created Topics");

    initial_time = this->get_clock()->now();
    RCLCPP_INFO(this->get_logger(), "Initial time: %f seconds", initial_time.seconds());
    // initialize timer
    timer_ = this->create_wall_timer(std::chrono::duration<double>(1/DEFAULT_SETPOINT_STREAM),
                                     std::bind(&SteerRelbot::timer_callback, this));
                                     
}

void SteerRelbot::create_topics() {
    left_wheel_topic_ = this->create_publisher<example_interfaces::msg::Float64>(
        "/input/left_motor/setpoint_vel", 1);

    right_wheel_topic_ = this->create_publisher<example_interfaces::msg::Float64>(
        "/input/right_motor/setpoint_vel", 1);

    green_object_position_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/object_center", 10, std::bind(&SteerRelbot::follow_green_object, this, std::placeholders::_1));

}

void SteerRelbot::follow_green_object(const geometry_msgs::msg::PointStamped::SharedPtr green) {
    double x = green->point.x;
    double y = green->point.y;
    double gain_velocity = 5.0; 
    double gain_steering = 2.0;  

    left_velocity = gain_velocity * y + gain_steering * x;
    right_velocity = -1 * gain_velocity * y + gain_steering * x; 

}

void SteerRelbot::timer_callback() {
    // publish velocity to simulator
    example_interfaces::msg::Float64 left_wheel;
    left_wheel.data = left_velocity;
    example_interfaces::msg::Float64 right_wheel;
    right_wheel.data = right_velocity;
    left_wheel_topic_->publish(left_wheel);
    right_wheel_topic_->publish(right_wheel);
}

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SteerRelbot>());
    rclcpp::shutdown();
    return 0;
}