#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlebot_interfaces/msg/turtle_info.hpp"
#include "turtlebot_interfaces/msg/turtle_array.hpp"
#include "turtlebot_interfaces/srv/catch_turtle.hpp"
#include <vector>
#include <limits>

using namespace std::placeholders;
using namespace std::chrono_literals;

const double PI = 3.14159;

class TurtleControllerNode : public rclcpp::Node {
    public:
        TurtleControllerNode() : Node("turtle_controller"), kp_linear_(2.0), kp_angular_(2.0), has_target_(false)
        {
            turtle_remove_client_ = create_client<turtlebot_interfaces::srv::CatchTurtle>("catch_turtle");
            while(!turtle_remove_client_->wait_for_service(1s)) {
                RCLCPP_WARN(get_logger(), "Waiting for server...");
            }


            pose_sub_ = create_subscription<turtlesim::msg::Pose>("turtle1/pose", 10, std::bind(&TurtleControllerNode::callback_pose_sub, this, _1));
            vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
            timer_pub_ = create_wall_timer(0.1s, std::bind(&TurtleControllerNode::pub_vel, this));
            turtle_list_sub_ = create_subscription<turtlebot_interfaces::msg::TurtleArray>(
                "alive_turtles",
                10,
                std::bind(&TurtleControllerNode::callback_turtle_list_sub, this, _1));
        }
    private:
        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
        rclcpp::TimerBase::SharedPtr timer_pub_;
        rclcpp::Subscription<turtlebot_interfaces::msg::TurtleArray>::SharedPtr turtle_list_sub_;
        rclcpp::Client<turtlebot_interfaces::srv::CatchTurtle>::SharedPtr turtle_remove_client_;
    
        float x_current_;
        float y_current_;
        float theta_current_;
        float kp_linear_;
        float kp_angular_;
        float x_target_;
        float y_target_;
        std::string target_name_;
        bool has_target_;

        void callback_pose_sub(turtlesim::msg::Pose::SharedPtr msg_pose) {
            x_current_ = msg_pose->x;
            y_current_ = msg_pose->y;
            theta_current_ = msg_pose->theta;  

        }

        void callback_turtle_list_sub(const turtlebot_interfaces::msg::TurtleArray::SharedPtr msg_turtle_list) {
            int min_idx = 0;
            float min_distance = std::numeric_limits<float>::max();
            if (!msg_turtle_list->turtles.empty()) {
                for (size_t i = 0; i < msg_turtle_list->turtles.size(); ++i) {
                    float distance = sqrt((msg_turtle_list->turtles[i].x - x_current_) * (msg_turtle_list->turtles[i].x - x_current_)
                                            + (msg_turtle_list->turtles[i].y - y_current_) * (msg_turtle_list->turtles[i].y - y_current_));
                    if (distance < min_distance) {
                        min_distance = distance;
                        min_idx = i;
                    }

                }


                x_target_ = msg_turtle_list->turtles[min_idx].x;
                y_target_ = msg_turtle_list->turtles[min_idx].y;
                target_name_ = msg_turtle_list->turtles[min_idx].name;
                has_target_ = true;
            }
            else has_target_ = false;
        }

        void pub_vel() {
            auto msg_vel = geometry_msgs::msg::Twist();

            if (!has_target_){
                vel_pub_->publish(msg_vel);
                return;
            }

            float error_distance = sqrt((x_target_ - x_current_) * (x_target_ - x_current_) + (y_target_ - y_current_) * (y_target_ - y_current_));
            msg_vel.linear.x = kp_linear_ * error_distance;


            float error_angle = atan2(y_target_ - y_current_, x_target_ - x_current_) - theta_current_;
            while (error_angle > PI) error_angle -= 2*PI;
            while (error_angle < -PI) error_angle += 2*PI;
            msg_vel.angular.z = kp_angular_ * error_angle;

            if (error_distance  < 0.3f) {
                call_remove_turtle(target_name_);
            }

            vel_pub_->publish(msg_vel);
        }

        void call_remove_turtle(std::string name) {
            auto request = std::make_shared<turtlebot_interfaces::srv::CatchTurtle::Request>();
            request->name = name;

            turtle_remove_client_->async_send_request(request, std::bind(&TurtleControllerNode::callback_call_remove_turtle, this, _1));
        }

        void callback_call_remove_turtle(rclcpp::Client<turtlebot_interfaces::srv::CatchTurtle>::SharedFuture future) {
            auto response = future.get();
            
            if (response->success) {
                RCLCPP_INFO(get_logger(), "Remove successfully!");
            } else {
                RCLCPP_WARN(get_logger(), "Cannot find any turtle!");
            }
        }
 };

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}