#include <memory>
#include <functional>
#include <thread>
#include <chrono>
#include <vector>
#include <deque>
#include <string>

#include "Eigen/Dense"

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/parsers/mjcf.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "interfaces/srv/trajectory_request.hpp"
#include "interfaces/action/move_to.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "tf2_ros/transform_broadcaster.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "tf2_ros/buffer.hpp"

using namespace std::chrono_literals;

class ControlConductor : public rclcpp::Node
{
    public:

        using TrajectoryRequest = interfaces::srv::TrajectoryRequest;
        using MoveTo = interfaces::action::MoveTo;
        using GoalHandleMoveTo = rclcpp_action::ServerGoalHandle<MoveTo>;


        ControlConductor() : Node("control_conductor")
        {
            RCLCPP_INFO(this->get_logger(), "control_conductor node started");

            this -> declare_parameter("mjcf_model_path", "");

            std::vector<double> std_vec_Kp =  this -> declare_parameter("Kp", std::vector<double>{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0});
            std::vector<double> std_vec_Kd = this -> declare_parameter("Kd", std::vector<double>{6.324556, 6.324556, 6.324556, 6.324556, 6.324556, 6.324556, 6.324556});

            auto std_vector_to_eigen_matrix = [](std::vector<double> std_vec) -> Eigen::MatrixXd {
                Eigen::VectorXd eigen_vec = Eigen::VectorXd::Map(std_vec.data(), std_vec.size());
                return eigen_vec.asDiagonal();
            };

            this->Kp = std_vector_to_eigen_matrix(std_vec_Kp);
            this->Kd = std_vector_to_eigen_matrix(std_vec_Kd);

            traj_srv_client_cbg_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            move_to_server_cbg_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
            joint_state_subscriber_cbg_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);


            endeffector_tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            endeffector_tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*endeffector_tf_buffer_);

            gen_traj_client_ =  this -> create_client<TrajectoryRequest>("generate_trajectory",
                                                     rclcpp::ServicesQoS(),
                                                     traj_srv_client_cbg_);


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
                move_to_server_cbg_
                );
            
            this -> torque_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("joint_commands", 10);

            rclcpp::SubscriptionOptions options;
            options.callback_group = this-> joint_state_subscriber_cbg_;
            this -> joint_state_subscriber_ = this->create_subscription<sensor_msgs::msg::JointState>(
                "joint_state",
                10,
                std::bind(&ControlConductor::joint_state_callback, this, std::placeholders::_1),
                options);

            RCLCPP_INFO(this->get_logger(), "%s", this->get_parameter("mjcf_model_path").as_string().c_str());

            try{
                pinocchio::mjcf::buildModel(this->get_parameter("mjcf_model_path").as_string().c_str(), this->model_);
                pinocchio::Data data(this->model_);
                this->data_ = data;

                this->n_joints = (unsigned short)this->model_.nv;

                this->desired_joint_pos_ = pinocchio::neutral(this->model_);
                this->desired_joint_vel_ = Eigen::VectorXd::Zero(this->n_joints);
                this->desired_joint_acc_ = Eigen::VectorXd::Zero(this->n_joints);

                this->joint_traj_pos_.push_back(this->desired_joint_pos_);
                this->joint_traj_vel_.push_back(this->desired_joint_vel_);
                this->joint_traj_acc_.push_back(this->desired_joint_acc_);
                
                
            }catch (const std::exception& e) {
                RCLCPP_INFO(this -> get_logger(), "Error loading MJCF model: %s", e.what());
    
            }
            
            this->update_desired_state_timer_ = this->create_wall_timer(10ms, std::bind(&ControlConductor::update_desired_state, this));
            this->update_current_endeffector_tf_timer_ ;
        }

    private:

        rclcpp::CallbackGroup::SharedPtr traj_srv_client_cbg_{nullptr};
        rclcpp::CallbackGroup::SharedPtr move_to_server_cbg_{nullptr};
        rclcpp::CallbackGroup::SharedPtr joint_state_subscriber_cbg_{nullptr};

        rclcpp_action::Server<MoveTo>::SharedPtr move_to_action_server_{nullptr};
        rclcpp::Client<TrajectoryRequest>::SharedPtr gen_traj_client_{nullptr};
        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_{nullptr};
        rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr torque_publisher_{nullptr};

        std::shared_ptr<TrajectoryRequest::Request> traj_request = std::make_shared<TrajectoryRequest::Request>();
    
        std::unique_ptr<tf2_ros::Buffer> endeffector_tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> endeffector_tf_listener_{nullptr};
        
        geometry_msgs::msg::Transform current_endeffector_tf_;

        pinocchio::Model model_;
        pinocchio::Data data_;  
        unsigned short n_joints = 6;

        Eigen::MatrixXd Kp;
        Eigen::MatrixXd Kd;

        std::deque<Eigen::VectorXd> joint_traj_pos_ = {};
        std::deque<Eigen::VectorXd> joint_traj_vel_ = {};
        std::deque<Eigen::VectorXd> joint_traj_acc_ = {};

        Eigen::VectorXd desired_joint_pos_ = {};
        Eigen::VectorXd desired_joint_vel_ = {};
        Eigen::VectorXd desired_joint_acc_ = {};

        rclcpp::TimerBase::SharedPtr update_desired_state_timer_{nullptr};
        rclcpp::TimerBase::SharedPtr update_current_endeffector_tf_timer_{nullptr};
        
        void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
        {
            Eigen::VectorXd qpos = Eigen::VectorXd::Map(msg->position.data(), msg->position.size());
            Eigen::VectorXd qvel = Eigen::VectorXd::Map(msg->velocity.data(), msg->velocity.size());

            Eigen::VectorXd pos_error = this->desired_joint_pos_ - qpos;
            Eigen::VectorXd vel_error = this->desired_joint_vel_ - qvel;

            Eigen::VectorXd acc_signal = desired_joint_acc_ + this->Kd*vel_error + this->Kp*pos_error;

            pinocchio::rnea(this->model_, this->data_, qpos, qvel, acc_signal);

            Eigen::VectorXd torques = data_.tau;

            auto torque_message = std_msgs::msg::Float64MultiArray();
            torque_message.data = std::vector<double>(torques.data(), torques.data() + torques.size());

            this->torque_publisher_->publish(torque_message);
        }

        void move_to_execute(const std::shared_ptr<GoalHandleMoveTo> goal_handle)
        {
            const auto goal = goal_handle->get_goal();
            auto feedback = std::make_shared<MoveTo::Feedback>();
            auto result = std::make_shared<MoveTo::Result>();

            RCLCPP_INFO(this->get_logger(), "Executing goal");

            auto current_tf = this->endeffector_tf_buffer_->lookupTransform("world", "endeffector");

            this->traj_request -> initial_tf = current_tf;
            this->traj_request -> final_tf =  goal->desired_tf;
            this->traj_request -> time = goal->time;

            auto gen_traj_future = this->gen_traj_client_->async_send_request(traj_request);

            std::future_status gen_traj_status = gen_traj_future.wait_for(std::chrono::seconds(3));

            if (gen_traj_status != std::future_status::ready){

                RCLCPP_INFO(this->get_logger(), "Failed to receive trajectory");
                result -> final_tf = current_tf
                goal_handle->abort();
                return;
            }

            RCLCPP_INFO(this->get_logger(), "Received generated trajectory");
            auto response = gen_traj_future.get();

            this->joint_traj_pos_ = this -> flatten_vector_to_matrix(response->joint_pos, this->n_joints);
            this->joint_traj_vel_ = this -> flatten_vector_to_matrix(response->joint_vel, this->n_joints);
            this->joint_traj_acc_ = this -> flatten_vector_to_matrix(response->joint_acc, this->n_joints);
            
            rclcpp::Rate loop_rate(1000);
            bool goal_completed = false;
            while(rclcpp::ok() && !goal_completed) {

                if (goal_handle->is_canceling()) {
                    result -> final_tf = current_tf
                    goal_handle->canceled(result);
                    RCLCPP_INFO(this->get_logger(), "Goal canceled successfully");
                    return;
                }
                if (this->joint_traj_pos_.size() == 1) {
                    goal_completed = true;
                    break;
                }
                feedback -> current_tf = this->endeffector_tf_buffer_->lookupTransform("world", "endeffector");
                goal_handle->publish_feedback(feedback);
                loop_rate.sleep();
            }

            result->final_end_effector_tf = this->endeffector_tf_buffer_->lookupTransform("world", "endeffector");
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");

        }

        void update_desired_state()
        {
            if (this->desired_joint_pos_ != this->joint_traj_pos_.front()){
                this->desired_joint_pos_ = this->joint_traj_pos_.front();
            }
            if (this->joint_traj_pos_.size() > 1) {
                this->joint_traj_pos_.pop_front();
            }

            if (this->desired_joint_vel_ != this->joint_traj_vel_.front()){
                this->desired_joint_vel_ = this->joint_traj_vel_.front();
            }
            if (this->joint_traj_vel_.size() > 1) {
                this->joint_traj_vel_.pop_front();
            }

            if (this->desired_joint_acc_ != this->joint_traj_acc_.front()){
                this->desired_joint_acc_ = this->joint_traj_acc_.front();
            }
            if (this->joint_traj_acc_.size() > 1) {
                this->joint_traj_acc_.pop_front();
            }
        }

        std::deque<Eigen::VectorXd> flatten_vector_to_matrix(const std::vector<double> & vector, int row_size)
        {
            std::deque<Eigen::VectorXd> matrix;
            Eigen::VectorXd row;
            row.resize(row_size);
            std::vector<double>::const_iterator it;
            for (it = vector.begin(); it != vector.end(); it += row_size) {
                for (int i = 0; i<row_size; ++i){
                    row[i] = *(it + i);
                }
                matrix.push_back(row);
            }
            return matrix;
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