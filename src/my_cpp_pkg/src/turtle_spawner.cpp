#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "turtlesim/srv/kill.hpp"
#include "turtlebot_interfaces/msg/turtle_info.hpp"
#include "turtlebot_interfaces/msg/turtle_array.hpp"
#include "turtlebot_interfaces/srv/catch_turtle.hpp"
#include <vector>
#include <cstdlib>
#include <ctime>


using namespace std::chrono_literals;
using namespace std::placeholders;

class TurtleSpawnerNode : public rclcpp::Node {
    public:
        TurtleSpawnerNode() : Node("turtle_spawner") 
        {
            spawn_client_ = create_client<turtlesim::srv::Spawn>("spawn");

            while(!spawn_client_->wait_for_service(1s)) {
                RCLCPP_WARN(get_logger(), "Waiting for server...");
            }

            spawn_timer_ = create_wall_timer(1.5s, std::bind(&TurtleSpawnerNode::callback_spawn_timer, this));
            turtle_list_pub_ = create_publisher<turtlebot_interfaces::msg::TurtleArray>("alive_turtles", 10);
            pub_timer_ = create_wall_timer(1s, std::bind(&TurtleSpawnerNode::pub_turtle_array, this));
            turtle_remove_srv_ = create_service<turtlebot_interfaces::srv::CatchTurtle>("catch_turtle",std::bind(&TurtleSpawnerNode::callback_remove_turtle, this, _1, _2));
            turtle_kill_client_ = create_client<turtlesim::srv::Kill>("kill");

        }

        void call_spawn(float x, float y) {
            auto request_spawn = std::make_shared<turtlesim::srv::Spawn::Request>();
            request_spawn->x = x;
            request_spawn->y = y;

            spawn_client_->async_send_request(
                request_spawn,
                [this, x, y](rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture future) {
                    auto response_spawn = future.get();
                    turtlebot_interfaces::msg::TurtleInfo new_turtle;

                    new_turtle.name = response_spawn->name;
                    new_turtle.x = x;
                    new_turtle.y = y;

                    turtles_.push_back(new_turtle); 
                }
            );
        }
    private:
        rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawn_client_;
        rclcpp::TimerBase::SharedPtr spawn_timer_;
        rclcpp::Publisher<turtlebot_interfaces::msg::TurtleArray>::SharedPtr turtle_list_pub_;
        rclcpp::TimerBase::SharedPtr pub_timer_;
        rclcpp::Service<turtlebot_interfaces::srv::CatchTurtle>::SharedPtr turtle_remove_srv_;
        rclcpp::Client<turtlesim::srv::Kill>::SharedPtr turtle_kill_client_;

        std::vector<turtlebot_interfaces::msg::TurtleInfo> turtles_;

        std::string name_;
        float coordinate_x_;
        float coordinate_y_;

        void callback_spawn_timer() {
            float x = static_cast<float>(std::rand() % 12);
            float y = static_cast<float>(std::rand() % 12);
            call_spawn(x,y);
        }

        void pub_turtle_array() {
            auto msg_turtle_array = turtlebot_interfaces::msg::TurtleArray();
            msg_turtle_array.turtles = turtles_;
            turtle_list_pub_->publish(msg_turtle_array);
        }

        void callback_remove_turtle(const turtlebot_interfaces::srv::CatchTurtle::Request::SharedPtr request,
                                          turtlebot_interfaces::srv::CatchTurtle::Response::SharedPtr response) {
            for (auto it = turtles_.begin(); it != turtles_.end(); ++it) {
                if (it->name == request->name) {
                    turtles_.erase(it);
                    response->success = true;

                    call_kill(request->name);
                    break;
                }
            }
            pub_turtle_array();
        }

        void call_kill(std::string name) {
            auto request = std::make_shared<turtlesim::srv::Kill::Request>();
            request->name = name;
            turtle_kill_client_->async_send_request(request);
        }

};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleSpawnerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}