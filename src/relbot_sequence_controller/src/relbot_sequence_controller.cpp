#include "steering.hpp"


SteerRelbot::SteerRelbot() : Node("steer_relbot") {
    RCLCPP_INFO(this->get_logger(), "Init");

    // initialize topics
    create_topics();
    RCLCPP_INFO(this->get_logger(), "Created Topics");

    // RCLCPP_INFO(this->get_logger(), "Change the integer value in the code to change the shape:");
    // RCLCPP_INFO(this->get_logger(), "1. Straight Line");
    // RCLCPP_INFO(this->get_logger(), "2. Circle");
    // RCLCPP_INFO(this->get_logger(), "3. Straight then 90 degree left");
    // RCLCPP_INFO(this->get_logger(), "4. Square");

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

void SteerRelbot::draw_L(double time) {

    const float LINE_TIME = 3; // Change this to change the length of the line segments
    const float CURVE_TIME = 6; //this timer was adjusted to make the curves around 90 degrees
    const float EDGE_TIME = LINE_TIME + CURVE_TIME; //this was created to make the code more readable
    
    if(time < LINE_TIME){
        //draw a straight line
        left_velocity = 5;
        right_velocity = -5;
    }
    else if(time < EDGE_TIME && time >= LINE_TIME){
        //draw a 90 degree left turn
        left_velocity = 2.02;
        right_velocity = -1;
    }
    else if(time < LINE_TIME + EDGE_TIME && time >= EDGE_TIME){
        //draw a straight line
        left_velocity = 5;
        right_velocity = -5;
    }
    else if(time >= LINE_TIME + EDGE_TIME){
        //stop moving and reset the flag so that the shape can be drawn again
        left_velocity = 0;
        right_velocity = 0;
        L_finished = true;
    }
     
}

void SteerRelbot::calculate_velocity() {
        
    /* Change the code here: */

    // Change this integer to change the shape the robot will drive in
    // 1. Straight Line
    // 2. Circle
    // 3. Straight then 90 degree left
    // 4. Square
    
    static int shape_choice = 4; 

    //find the current time
    rclcpp::Time current_time = this->get_clock()->now();
    
    //caculate the elapsed time since the start of the program
    double elapsed_time = (current_time - initial_time).seconds();
    //RCLCPP_INFO(this->get_logger(), "Elapsed time: %f seconds", elapsed_time);
    
    
    switch (shape_choice) {
        case 1:
            //This is a straight line
            //The velocities are equal in opposite directions to make the robot go straight       
            left_velocity = 5;
            right_velocity = -5;
            break;
        case 2:
            //This is a circle
            //The velocities are different to make the robot turn in a circle
            left_velocity = 5;
            right_velocity = -4;
            break;
        case 3:
            //This is a straight line followed by a 90 degree left turn
            //This method was created to make the code more readable and to avoid having a lot of if statements in this switch statement
            SteerRelbot::draw_L(elapsed_time); 
            break;
        case 4:
            //This is a square
            //This is used to draw a corner of the square
            SteerRelbot::draw_L(elapsed_time);

            //this is used to restart the timer to draw the next corner of the square after the first corner is finished
            if (L_finished){
                L_finished = false;
                initial_time = current_time;
            }
            break;
        default:
            RCLCPP_INFO(this->get_logger(), "Invalid choice");
            left_velocity = 0;
            right_velocity = 0;
    }
    
    /* End of your algorithm */
}

void SteerRelbot::follow_green_object(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    double x = msg->point.x;
    double y = msg->point.y;
    double gain = 3.0; // Proportional gain for steering

    left_velocity = gain * y + gain * x;
    right_velocity = -1 * gain * y + gain * x; 
}

void SteerRelbot::timer_callback() {
    // calculate velocity
    // calculate_velocity();
    //follow_green_object();

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