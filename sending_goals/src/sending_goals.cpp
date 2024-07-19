#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

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

        auto goal_msg1 = NavigateThroughPoses::Goal();
        goal_msg1.poses.resize(2);

        goal_msg1.poses[0].header.frame_id = "map";
        goal_msg1.poses[0].header.stamp = this->now();
        goal_msg1.poses[0].pose.position.x = -1;
        goal_msg1.poses[0].pose.position.y = 1.75;
        goal_msg1.poses[0].pose.orientation.z = 0;
        goal_msg1.poses[0].pose.orientation.w = 1;

        goal_msg1.poses[1].header.frame_id = "map";
        goal_msg1.poses[1].header.stamp = this->now();
        goal_msg1.poses[1].pose.position.x = 1.25;
        goal_msg1.poses[1].pose.position.y = 1.75;
        goal_msg1.poses[1].pose.orientation.z = 0;
        goal_msg1.poses[1].pose.orientation.w = 1;

        goal_msgs.push_back(goal_msg1);

        auto goal_msg2 = NavigateThroughPoses::Goal();
        goal_msg2.poses.resize(2);

        goal_msg2.poses[0].header.frame_id = "map";
        goal_msg2.poses[0].header.stamp = this->now();
        goal_msg2.poses[0].pose.position.x = 1.25;
        goal_msg2.poses[0].pose.position.y = 1.75;
        goal_msg2.poses[0].pose.orientation.z = 0;
        goal_msg2.poses[0].pose.orientation.w = 1;

        goal_msg2.poses[1].header.frame_id = "map";
        goal_msg2.poses[1].header.stamp = this->now();
        goal_msg2.poses[1].pose.position.x = 1.75;
        goal_msg2.poses[1].pose.position.y = 0;
        goal_msg2.poses[1].pose.orientation.z = -0.5;
        goal_msg2.poses[1].pose.orientation.w = -0.5;

        goal_msgs.push_back(goal_msg2);

        auto goal_msg3 = NavigateThroughPoses::Goal();
        goal_msg3.poses.resize(2);

        goal_msg3.poses[0].header.frame_id = "map";
        goal_msg3.poses[0].header.stamp = this->now();
        goal_msg3.poses[0].pose.position.x = 1.75;
        goal_msg3.poses[0].pose.position.y = 0;
        goal_msg3.poses[0].pose.orientation.z = -0.7;
        goal_msg3.poses[0].pose.orientation.w = -0.7;

        goal_msg3.poses[1].header.frame_id = "map";
        goal_msg3.poses[1].header.stamp = this->now();
        goal_msg3.poses[1].pose.position.x = 1;
        goal_msg3.poses[1].pose.position.y = -1.75;
        goal_msg3.poses[1].pose.orientation.z = 1;
        goal_msg3.poses[1].pose.orientation.w = 0;

        goal_msgs.push_back(goal_msg3);

        auto goal_msg4 = NavigateThroughPoses::Goal();
        goal_msg4.poses.resize(2);

        goal_msg4.poses[0].header.frame_id = "map";
        goal_msg4.poses[0].header.stamp = this->now();
        goal_msg4.poses[0].pose.position.x = 1;
        goal_msg4.poses[0].pose.position.y = -1.75;
        goal_msg4.poses[0].pose.orientation.z = 1;
        goal_msg4.poses[0].pose.orientation.w = 0;

        goal_msg4.poses[1].header.frame_id = "map";
        goal_msg4.poses[1].header.stamp = this->now();
        goal_msg4.poses[1].pose.position.x = -1;
        goal_msg4.poses[1].pose.position.y = -1.75;
        goal_msg4.poses[1].pose.orientation.z = 1;
        goal_msg4.poses[1].pose.orientation.w = 0;

        goal_msgs.push_back(goal_msg4);

        auto goal_msg5 = NavigateThroughPoses::Goal();
        goal_msg5.poses.resize(2);

        goal_msg5.poses[0].header.frame_id = "map";
        goal_msg5.poses[0].header.stamp = this->now();
        goal_msg5.poses[0].pose.position.x = -1;
        goal_msg5.poses[0].pose.position.y = -1.75;
        goal_msg5.poses[0].pose.orientation.z = 1;
        goal_msg5.poses[0].pose.orientation.w = 0;

        goal_msg5.poses[1].header.frame_id = "map";
        goal_msg5.poses[1].header.stamp = this->now();
        goal_msg5.poses[1].pose.position.x = -1.75;
        goal_msg5.poses[1].pose.position.y = 0;
        goal_msg5.poses[1].pose.orientation.z = 0.5;
        goal_msg5.poses[1].pose.orientation.w = 0.5;

        goal_msgs.push_back(goal_msg5);

        auto goal_msg6 = NavigateThroughPoses::Goal();
        goal_msg6.poses.resize(2);

        goal_msg6.poses[0].header.frame_id = "map";
        goal_msg6.poses[0].header.stamp = this->now();
        goal_msg6.poses[0].pose.position.x = -1.75;
        goal_msg6.poses[0].pose.position.y = 0;
        goal_msg6.poses[0].pose.orientation.z = 0.5;
        goal_msg6.poses[0].pose.orientation.w = 0.5;

        goal_msg6.poses[1].header.frame_id = "map";
        goal_msg6.poses[1].header.stamp = this->now();
        goal_msg6.poses[1].pose.position.x = -1;
        goal_msg6.poses[1].pose.position.y = 1.75;
        goal_msg6.poses[1].pose.orientation.z = 0;
        goal_msg6.poses[1].pose.orientation.w = 1;

        goal_msgs.push_back(goal_msg6);

        RCLCPP_INFO(this->get_logger(), "Sending goal to robot");

        auto send_goal_options = rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&SendingGoalsClient::goal_response_callback, this, std::placeholders::_1);
        send_goal_options.feedback_callback =
            std::bind(&SendingGoalsClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback =
            std::bind(&SendingGoalsClient::result_callback, this, std::placeholders::_1);

        this->client_ptr_->async_send_goal(goal_msgs.at(i % 6), send_goal_options);
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
            break;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
            break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown result code");
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
