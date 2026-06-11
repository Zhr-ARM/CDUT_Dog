#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <memory>
#include <chrono>

#include "arm_control/srv/plan_move.hpp"

using PlanMove = arm_control::srv::PlanMove;
using std::placeholders::_1;

class ArmController : public rclcpp::Node
{
    public:
        ArmController(const rclcpp::NodeOptions & options) : Node("target_to_moveit",options)
        {
            plan_client_ = this->create_client<PlanMove>("plan_moveit");
            RCLCPP_INFO(this->get_logger(), "Arm Controller Node has been started.");
            subscription_ = this->create_subscription<std_msgs::msg::String>(
                "controller_command",
                10,
                std::bind(&ArmController::command_callback, this, _1)
            );
        }
    private:
        void command_callback(const std_msgs::msg::String::SharedPtr msg)
        {
            int number = sscanf(msg->data.c_str(), "%lf %lf %lf", &target_x, &target_y, &target_z);
            if(number == 3)
            {
                RCLCPP_INFO(this->get_logger(), "Received command: x=%.2f, y=%.2f, z=%.2f", target_x, target_y, target_z);
            
                call_plan_service();
            }
            else
            {
                RCLCPP_WARN(this->get_logger(), "Received invalid command: '%s'", msg->data.c_str());
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
