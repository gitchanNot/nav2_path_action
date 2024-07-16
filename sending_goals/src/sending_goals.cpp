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
            this,
            "sending_goals");

        this->timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&SendingGoalsClient::send_goals, this)
        );
    }

private:
    rclcpp_action::Client<NavigateThroughPoses>::SharedPtr client_ptr_;
    rclcpp::TimerBase::SharedPtr timer_;

    void send_goals()
    {
        this->timer_->cancel();

        if (!this->client_ptr_->wait_for_action_server())
        {
            RCLCPP_ERROR(this->get_logger(), "Action server not avilable after waiting");
            rclcpp::shutdown();
            return;
        }

        auto goal_msg = NavigateThroughPoses::Goal();
        goal_msg.poses.resize(6);

        goal_msg.poses[0].header.frame_id = "map";
        goal_msg.poses[0].header.stamp = this->now();
        goal_msg.poses[0].pose.position.x = 1.76;
        goal_msg.poses[0].pose.position.y = -0.26;
        goal_msg.poses[0].pose.orientation.z = 0.71;
        goal_msg.poses[0].pose.orientation.w = 0.71;

        goal_msg.poses[1].header.frame_id = "map";
        goal_msg.poses[1].header.stamp = this->now();
        goal_msg.poses[1].pose.position.x = 1.26;
        goal_msg.poses[1].pose.position.y = 1.87;
        goal_msg.poses[1].pose.orientation.z = 0.98;
        goal_msg.poses[1].pose.orientation.w = 0.21;

        goal_msg.poses[2].header.frame_id = "map";
        goal_msg.poses[2].header.stamp = this->now();
        goal_msg.poses[2].pose.position.x = -0.69;
        goal_msg.poses[2].pose.position.y = 1.82;
        goal_msg.poses[2].pose.orientation.z = -0.94;
        goal_msg.poses[2].pose.orientation.w = 0.4;

        goal_msg.poses[3].header.frame_id = "map";
        goal_msg.poses[3].header.stamp = this->now();
        goal_msg.poses[3].pose.position.x = -1.90;
        goal_msg.poses[3].pose.position.y = 0.00;
        goal_msg.poses[3].pose.orientation.z = -0.71;
        goal_msg.poses[3].pose.orientation.w = 0.71;

        goal_msg.poses[4].header.frame_id = "map";
        goal_msg.poses[4].header.stamp = this->now();
        goal_msg.poses[4].pose.position.x = -1.26;
        goal_msg.poses[4].pose.position.y = -1.59;
        goal_msg.poses[4].pose.orientation.z = -0.27;
        goal_msg.poses[4].pose.orientation.w = 0.95;

        goal_msg.poses[5].header.frame_id = "map";
        goal_msg.poses[5].header.stamp = this->now();
        goal_msg.poses[5].pose.position.x = 1.02;
        goal_msg.poses[5].pose.position.y = -1.82;
        goal_msg.poses[5].pose.orientation.z = 0.34;
        goal_msg.poses[5].pose.orientation.w = 0.94;

        RCLCPP_INFO(this->get_logger(), "sending goals");

        auto send_goal_options = rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&SendingGoalsClient::goal_response_callback, this, std::placeholders::_1);
        send_goal_options.feedback_callback =
            std::bind(&SendingGoalsClient::feedback_callback, this, std::placeholders::_1);
        send_goal_options.result_callback =
        std::bind(&SendingGoalsClient::result_callback, this, std::placeholders::_1);

        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
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
        GoalHandleSendingGoals::SharedPtr)
    {
        RCLCPP_INFO(this->get_logger(), "Sending...");
    }

    void result_callback(const GoalHandleSendingGoals::WrappedResult &result)
    {
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(this->get_logger(), "Goal succeeded");
            break;
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
        rclcpp::shutdown();
    }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SendingGoalsClient>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}