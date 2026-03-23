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
}

void SteerRelbot::draw_L(double time) {
    const float LINE_TIME = 3; // Change this float to change how long it takes to complete the shape in seconds
    const float CURVE_TIME = 6;
    const float EDGE_TIME = LINE_TIME + CURVE_TIME;

    if(time < LINE_TIME){
        left_velocity = 5;
        right_velocity = -5;
    }
    else if(time < EDGE_TIME && time >= LINE_TIME){
        left_velocity = 2.02;
        right_velocity = -1;
    }
    else if(time < LINE_TIME + EDGE_TIME && time >= EDGE_TIME){
        left_velocity = 5;
        right_velocity = -5;
    }
    else if(time >= LINE_TIME + EDGE_TIME){
        left_velocity = 0;
        right_velocity = 0;
        L_finished = true;
    }
     
}

void SteerRelbot::calculate_velocity() {
        
    /* Change the code here: */
    static int shape_choice = 4; // Change this integer to change the shape the robot will drive in
    rclcpp::Time current_time = this->get_clock()->now();
    
    double elapsed_time = (current_time - initial_time).seconds();
    //RCLCPP_INFO(this->get_logger(), "Elapsed time: %f seconds", elapsed_time);
    
    /*
    const float LINE_TIME = 5; // Change this float to change how long it takes to complete the shape in seconds
    const float CURVE_TIME = 6;
    const float EDGE_TIME = LINE_TIME + CURVE_TIME;
    */
    
    
    if(!loop_started){
        loop_started = true;
        initial_time = current_time;
    }
    
    if(loop_started){
        switch (shape_choice) {
            case 1:
                //RCLCPP_INFO(this->get_logger(), "This is a straight line.");
                left_velocity = 5;
                right_velocity = -5;
                break;
            case 2:
                //RCLCPP_INFO(this->get_logger(), "This is a circle.");
                left_velocity = 5;
                right_velocity = -4;
                break;
            case 3:
                //RCLCPP_INFO(this->get_logger(), "This is a straight then 90 degree left.");
                SteerRelbot::draw_L(elapsed_time);
                break;
            case 4:
                //RCLCPP_INFO(this->get_logger(), "This is a square");    
                SteerRelbot::draw_L(elapsed_time);
                if (L_finished){
                    loop_started = false;
                    L_finished = false;
                }
                break;
            default:
                RCLCPP_INFO(this->get_logger(), "Invalid choice");
                left_velocity = 0;
                right_velocity = 0;
        }
    }
    /* End of your algorithm */

    
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