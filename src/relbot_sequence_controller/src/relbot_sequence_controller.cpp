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

    green_object_radius_ = this->create_subscription<example_interfaces::msg::Float64>(
        "/object_radius", 10, std::bind(&SteerRelbot::update_green_object_radius, this, std::placeholders::_1));

}

void SteerRelbot::follow_green_object(const geometry_msgs::msg::PointStamped::SharedPtr green) {
    //get coordinates of green object
    double x_green = green->point.x; 

    //set setpoints to get error
    double setpoint_x = 0;
    double setpoint_size = 0.8;

    //set proportional gains for velocity and steering
    double gain_velocity = 10.0; 
    double gain_steering = 3.0;  

    //Determine steering proportional to horizontal error. Subtract from 1 side, add to other side, so /2.
    double steering = gain_steering * (x_green - setpoint_x)/2;
    //Determine velocity proportional to vertical error. Subtract from setpoint, so that it goes faster when object is smaller (further away).
    
    double velocity = gain_velocity * (setpoint_size - object_radius);

    //Compute motor commands. Left is negative because of the motors.
    left_velocity = -1 * (velocity + steering);
    right_velocity = velocity - steering; 


    if (object_radius <= 0.1) {
        left_velocity = 0; // If no object detected, don't move forward
        right_velocity = 0;
        RCLCPP_INFO(this->get_logger(), "No object detected, stopping.");
    }

    
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