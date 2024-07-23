#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

std::vector<std::vector<std::vector<double>>> goal_list = {
  {{-2, -5}, {-4, -5}, {-4, -7.5}, {-6, -7.5}},
  {{-4, -7.5}, {-4, -5}, {-2, -5}, {1, -5}, {1, -6}, {3.5, -6}, {6, -6}, {6, -5}},
  {{6, -6}, {3.5, -6}, {1, -6}, {1, -5}, {-2, -5}}
};

class SendingGoalsClient : public rclcpp::Node
{
public:
    using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
    using GoalHandleSendingGoals = rclcpp_action::ClientGoalHandle<NavigateThroughPoses>;

    explicit SendingGoalsClient(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("sending_goals_client", options)
    {
        this->client_ptr_ = rclcpp_action::create_client<NavigateThroughPoses>(
            this, "sending_goals");

        this->timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&SendingGoalsClient::send_goals, this));
    }

private:
    rclcpp_action::Client<NavigateThroughPoses>::SharedPtr client_ptr_;
    rclcpp::TimerBase::SharedPtr timer_;
    int i = 0;

    void send_goals()
    {
        this->timer_->cancel();

        if (!this->client_ptr_->wait_for_action_server())
        {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            rclcpp::shutdown();
            return;
        }

        std::vector<NavigateThroughPoses::Goal> goal_msgs;

        for(int m = 0; m < goal_list.size(); m++) {
            auto goal_msg1 = NavigateThroughPoses::Goal();
            goal_msg1.poses.resize(goal_list[m].size());
            for(int p = 0; p < goal_list[m].size(); p++) {
                goal_msg1.poses[p].header.frame_id = "map";
                goal_msg1.poses[p].header.stamp = this->now();
                goal_msg1.poses[p].pose.position.x = goal_list[m][p][0];
                goal_msg1.poses[p].pose.position.y = goal_list[m][p][1];
                goal_msg1.poses[p].pose.orientation.z = 0;
                goal_msg1.poses[p].poses.orientation.w = 1;
            }
            goal_msgs.push_back(goal_msg1);
        }

        RCLCPP_INFO(this->get_logger(), "Sending goal to robot");

        auto send_goal_options = rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&SendingGoalsClient::goal_response_callback, this, std::placeholders::_1);
        send_goal_options.feedback_callback =
            std::bind(&SendingGoalsClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback =
            std::bind(&SendingGoalsClient::result_callback, this, std::placeholders::_1);

        this->client_ptr_->async_send_goal(goal_msgs.at(i % goal_msgs.size()), send_goal_options);
    }

    void goal_response_callback(GoalHandleSendingGoals::SharedPtr goal_handle)
    {
        if (!goal_handle)
        {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
        }
    }

    void feedback_callback(
        GoalHandleSendingGoals::SharedPtr,
        const std::shared_ptr<const NavigateThroughPoses::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), "Remain Distance: [%.2f]",
                    feedback->distance_remaining);
    }

    void result_callback(const GoalHandleSendingGoals::WrappedResult &result)
    {
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
        {
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");
            i++;
            this->timer_ = this->create_wall_timer(
                std::chrono::milliseconds(500),
                std::bind(&SendingGoalsClient::send_goals, this));
            break;
        }
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
            rclcpp::shutdown();
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
            rclcpp::shutdown();
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
            rclcpp::shutdown();
            break;
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SendingGoalsClient>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
