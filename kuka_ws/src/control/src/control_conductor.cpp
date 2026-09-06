#include <memory>
#include <functional>
#include <thread>
#include <chrono>
#include <array>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "interfaces/srv/trajectory_request.hpp"
#include "interfaces/action/move_to.hpp"


class ControlConductor : public rclcpp::Node
{
    public:

        using TrajectoryRequest = interfaces::srv::TrajectoryRequest;
        using MoveTo = interfaces::action::MoveTo;
        using GoalHandleMoveTo = rclcpp_action::ServerGoalHandle<MoveTo>;

        std::shared_ptr<TrajectoryRequest::Request> traj_request = std::make_shared<TrajectoryRequest::Request>();

        ControlConductor() : Node("control_conductor")
        {
            RCLCPP_INFO(this->get_logger(), "control_conductor node started");

            this -> declare_parameter("MJCF_path", "");
            
            traj_srv_client_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
            move_to_server_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

            gen_traj_client_ =  this -> create_client<TrajectoryRequest>("generate_trajectory",
                                                     rclcpp::ServicesQoS(),
                                                     traj_srv_client_callback_group_);


            auto move_to_handle_goal = [this](const rclcpp_action::GoalUUID & uuid,
                                              std::shared_ptr<const MoveTo::Goal> goal)
            {
                RCLCPP_INFO(this->get_logger(), "Received move_to goal request to [coordinates]");
                (void)uuid;
                (void)goal;
                return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            };

            auto move_to_handle_cancel = [this](const std::shared_ptr<GoalHandleMoveTo> goal_handle)
            {
                RCLCPP_INFO(this->get_logger(), "move_to goal rejected");
                (void)goal_handle;
                return rclcpp_action::CancelResponse::ACCEPT;
            };

            auto move_to_handle_accepted = [this](const std::shared_ptr<GoalHandleMoveTo> goal_handle)
            {
                auto execute_in_thread = [this, goal_handle](){return this->move_to_execute(goal_handle);};
                std::thread{execute_in_thread}.detach();
            };

            this -> move_to_action_server_ = rclcpp_action::create_server<MoveTo>(
                this,
                "move_to",
                move_to_handle_goal,
                move_to_handle_cancel,
                move_to_handle_accepted,
                rcl_action_server_get_default_options(),
                move_to_server_callback_group_
                );
        }

    private:

        rclcpp_action::Server<MoveTo>::SharedPtr move_to_action_server_;
        rclcpp::Client<TrajectoryRequest>::SharedPtr gen_traj_client_;

        rclcpp::CallbackGroup::SharedPtr traj_srv_client_callback_group_;
        rclcpp::CallbackGroup::SharedPtr move_to_server_callback_group_;

        //3 Cartesian coordinates and Roll-Pitch-Yall angles
        std::array<double, 6> current_pos_ = {0.0, 0.0, 1.306, 0.0, 0.0, 0.0}; 

        void move_to_execute(const std::shared_ptr<GoalHandleMoveTo> goal_handle)
        {
            const auto goal = goal_handle->get_goal();
            //auto feedback = std::make_shared<MoveTo::Feedback>();
            auto result = std::make_shared<MoveTo::Result>();

            RCLCPP_INFO(this->get_logger(), "Executing goal");

            this->traj_request -> initial_cartesian_coord = current_pos_
            this->traj_request -> final_cartesian_coord = goal -> endeffector_desired_coords;
            this->traj_request -> time = 1.0;

            auto gen_traj_future = this->gen_traj_client_->async_send_request(traj_request);

            std::future_status gen_traj_status = gen_traj_future.wait_for(std::chrono::seconds(3));

            if (gen_traj_status == std::future_status::ready){
                RCLCPP_INFO(this->get_logger(), "Received generated trajectory");
                auto response = gen_traj_future.get();
                result->final_end_effector_pos = response -> joint_pos;
            }
            else {
            RCLCPP_INFO(this->get_logger(), "Error when executing goal");
            }

            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");

        }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    auto node = std::make_shared<ControlConductor>();
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}