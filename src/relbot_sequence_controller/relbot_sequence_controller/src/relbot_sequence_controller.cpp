#include "steering.hpp"
#include <iostream>

SteerRelbot::SteerRelbot() : Node("steer_relbot") {
    RCLCPP_INFO(this->get_logger(), "Init");

    // initialize topics
    create_topics();
    RCLCPP_INFO(this->get_logger(), "Created Topics");

    RCLCPP_INFO(this->get_logger(), "Change the integer value in the code to change the shape:");
    RCLCPP_INFO(this->get_logger(), "1. Straight Line");
    RCLCPP_INFO(this->get_logger(), "2. Circle");
    RCLCPP_INFO(this->get_logger(), "3. Straight then 90 degree left");
    RCLCPP_INFO(this->get_logger(), "4. Square");

    rclcpp::Time initial_time = this->get_clock()->now();
    // initialize timer
    timer_ = this->create_wall_timer(std::chrono::duration<double>(1/DEFAULT_SETPOINT_STREAM),
                                     std::bind(&SteerRelbot::timer_callback, this));
                                     
}

void SteerRelbot::create_topics() {
    left_wheel_topic_ = this->create_publisher<example_interfaces::msg::Float64>(
        "/input/left_motor/setpoint_vel", 1);

    right_wheel_topic_ = this->create_publisher<example_interfaces::msg::Float64>(
        "/input/right_motor/setpoint_vel", 1);
}

void SteerRelbot::calculate_velocity() {
    
    
    static int shape_choice = 1; // Change this integer to change the shape the robot will drive in
    
    /* Change the code here: */
    //static rclcpp::Time initial_time = this->get_clock()->now();
    rclcpp::Time current_time = this->get_clock()->now();
    
    double elapsed_time = (current_time - initial_time).seconds();
    RCLCPP_INFO(this->get_logger(), "Elapsed time: %f seconds", elapsed_time);

    if (!loop_started) {
        initial_time = current_time;
        loop_started = true;
    }
    if (loop_started){
        switch (shape_choice) {
            case 1:
                left_velocity = 5;
                right_velocity = -5;
                break;
            case 2:
                left_velocity = 5;
                right_velocity = -3;
                break;
            case 3:
                left_velocity = 0;
                right_velocity = 0;
                break;
            case 4:
                left_velocity = 0;
                right_velocity = 0;
                break;
            default:
                left_velocity = 5;
                right_velocity = -5;
        }
    }
    /* End of your algorithm */

    //left_velocity = 0;
    //right_velocity = 0;
    /*int shape_choice;
    std::cout << "Tell me what shape you want to drive in: " << std::endl;
    std::cout << "1. Straight Line" << std::endl;
    std::cout << "2. Circle" << std::endl;
    std::cout << "3. Straight then 90 degree left" << std::endl;
    std::cout << "4. Square" << std::endl;
    std::cin >> shape_choice;
    


    
    switch (shape_choice) {
        case 1:
            std::cout << "You chose straight line." << std::endl;
            left_velocity = 5;
            right_velocity = -5;
            break;
        case 2:
            std::cout << "You chose circle." << std::endl;
            left_velocity = 5;
            right_velocity = -3;
            break;
        case 3:
            std::cout << "You chose straight then 90 degree left." << std::endl;
            left_velocity = 0;
            right_velocity = 0;
            break;
        case 4:
            std::cout << "You chose square." << std::endl;
            left_velocity = 0;
            right_velocity = 0;
            break;
        default:
            std::cout << "Invalid choice, defaulting to straight line." << std::endl;
    }
    */
}

void SteerRelbot::timer_callback() {
    // calculate velocity
    calculate_velocity();

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