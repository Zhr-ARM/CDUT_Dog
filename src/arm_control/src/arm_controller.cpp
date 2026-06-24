#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"

#include <algorithm>
#include <cctype>
#include <memory>
#include <chrono>
#include <string>

#include "arm_control/srv/plan_move.hpp"

using PlanMove = arm_control::srv::PlanMove;
using std::placeholders::_1;

class ArmController : public rclcpp::Node
{
    public:
        ArmController(const rclcpp::NodeOptions & options) : Node("target_to_moveit",options)
        {
            plan_client_ = this->create_client<PlanMove>("plan_moveit");
            vision_start_publisher_ =
                this->create_publisher<std_msgs::msg::Empty>("/arm_vision_motion/start", 10);
            box_start_publisher_ =
                this->create_publisher<std_msgs::msg::Empty>("/arm_box_motion/start", 10);
            suction_start_publisher_ =
                this->create_publisher<std_msgs::msg::Empty>("/arm_box_motion/suction", 10);
            lift_start_publisher_ =
                this->create_publisher<std_msgs::msg::Empty>("/arm_box_motion/lift", 10);
            place_ground_start_publisher_ =
                this->create_publisher<std_msgs::msg::Empty>("/arm_box_motion/place_ground", 10);
            place_stack_start_publisher_ =
                this->create_publisher<std_msgs::msg::Empty>("/arm_box_motion/place_stack", 10);
            home_start_publisher_ =
                this->create_publisher<std_msgs::msg::Empty>("/arm_home_motion/start", 10);
            RCLCPP_INFO(this->get_logger(), "Arm Controller Node has been started.");
            RCLCPP_INFO(
                this->get_logger(),
                "controller_command supports either 'x y z' or named commands: "
                "vision, box, suction, lift, place_ground, place_stack, home.");
            subscription_ = this->create_subscription<std_msgs::msg::String>(
                "controller_command",
                10,
                std::bind(&ArmController::command_callback, this, _1)
            );
        }
    private:
        static std::string normalize_command(const std::string & value)
        {
            std::string normalized;
            normalized.reserve(value.size());

            bool previous_was_separator = false;
            for (const unsigned char ch : value)
            {
                if (std::isspace(ch) || ch == '-' || ch == '/')
                {
                    if (!normalized.empty() && !previous_was_separator)
                    {
                        normalized.push_back('_');
                    }
                    previous_was_separator = true;
                    continue;
                }

                normalized.push_back(static_cast<char>(std::tolower(ch)));
                previous_was_separator = false;
            }

            while (!normalized.empty() && normalized.front() == '_')
            {
                normalized.erase(normalized.begin());
            }
            while (!normalized.empty() && normalized.back() == '_')
            {
                normalized.pop_back();
            }
            return normalized;
        }

        void publish_empty_command(
            const rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr & publisher,
            const char * label)
        {
            std_msgs::msg::Empty msg;
            publisher->publish(msg);
            RCLCPP_INFO(this->get_logger(), "Forwarded named command -> %s", label);
        }

        bool handle_named_command(const std::string & raw_command)
        {
            const std::string command = normalize_command(raw_command);
            if (command.empty())
            {
                return false;
            }

            if (command == "vision" || command == "look" || command == "observe") {
                publish_empty_command(vision_start_publisher_, "vision");
                return true;
            }
            if (command == "box" || command == "box_motion" || command == "pickup_sequence") {
                publish_empty_command(box_start_publisher_, "box");
                return true;
            }
            if (command == "suction" || command == "pick" || command == "grab") {
                publish_empty_command(suction_start_publisher_, "suction");
                return true;
            }
            if (command == "lift" || command == "lift_box") {
                publish_empty_command(lift_start_publisher_, "lift");
                return true;
            }
            if (command == "place_ground" || command == "ground") {
                publish_empty_command(place_ground_start_publisher_, "place_ground");
                return true;
            }
            if (command == "place_stack" || command == "stack") {
                publish_empty_command(place_stack_start_publisher_, "place_stack");
                return true;
            }
            if (command == "home" || command == "reset") {
                publish_empty_command(home_start_publisher_, "home");
                return true;
            }

            return false;
        }

        void command_callback(const std_msgs::msg::String::SharedPtr msg)
        {
            if (handle_named_command(msg->data))
            {
                return;
            }

            int number = sscanf(msg->data.c_str(), "%lf %lf %lf", &target_x, &target_y, &target_z);
            if(number == 3)
            {
                RCLCPP_INFO(this->get_logger(), "Received command: x=%.2f, y=%.2f, z=%.2f", target_x, target_y, target_z);
            
                call_plan_service();
            }
            else
            {
                RCLCPP_WARN(
                    this->get_logger(),
                    "Received invalid command: '%s'. Expected either 'x y z' or one of "
                    "vision/box/suction/lift/place_ground/place_stack/home.",
                    msg->data.c_str());
            }
            
        }
        void call_plan_service()
        {
            if (!plan_client_->wait_for_service(std::chrono::seconds(1)))
            {
                RCLCPP_WARN(this->get_logger(), "plan_moveit service is not available.");
                return;
            }
        
            auto request = std::make_shared<PlanMove::Request>();
            request->x = target_x;
            request->y = target_y;
            request->z = target_z;
            request->execute = true;
            request->position_tolerance = 0.0;
            request->use_target_orientation = false;
            request->use_level_constraint = false;
        
            auto response_callback =
                [this](rclcpp::Client<PlanMove>::SharedFuture future)
                {
                    auto response = future.get();
                
                    if (response->success)
                    {
                        RCLCPP_INFO(this->get_logger(), "MoveIt service success: %s", response->message.c_str());
                    }
                    else
                    {
                        RCLCPP_WARN(this->get_logger(), "MoveIt service failed: %s", response->message.c_str());
                    }
                };
            
            plan_client_->async_send_request(request, response_callback);
        }
        double target_x,target_y,target_z;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
        rclcpp::Client<PlanMove>::SharedPtr plan_client_;
        rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr vision_start_publisher_;
        rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr box_start_publisher_;
        rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr suction_start_publisher_;
        rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr lift_start_publisher_;
        rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr place_ground_start_publisher_;
        rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr place_stack_start_publisher_;
        rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr home_start_publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);

    auto node = std::make_shared<ArmController>(options);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
