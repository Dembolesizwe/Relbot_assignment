#ifndef STEER_RELBOT_HPP_
#define STEER_RELBOT_HPP_

// CPP library headers

// ROS Client Library CPP
#include "rclcpp/rclcpp.hpp"

// message type for velocity
#include "example_interfaces/msg/float64.hpp"
#include "geometry_msgs/msg/point_stamped.hpp" 

class SteerRelbot : public rclcpp::Node
{
public:
    /**
     * @brief Construct a new steering object
     */
    SteerRelbot();

    const double DEFAULT_SETPOINT_STREAM = 30;  // How often the velocities are published per second

private:
    // Topics
    rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr left_wheel_topic_;
    rclcpp::Publisher<example_interfaces::msg::Float64>::SharedPtr right_wheel_topic_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr green_object_position_;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // attributes
    double left_velocity;
    double right_velocity;

    rclcpp::Time initial_time; //finds the initial time
    //bool loop_started = true;
    bool L_finished = false;

    // methods
    void create_topics();
    void timer_callback();
    void follow_green_object(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    
};

#endif /*STEER_RELBOT_HPP_*/