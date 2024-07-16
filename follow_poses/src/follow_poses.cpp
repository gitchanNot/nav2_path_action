#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class NavigateThroughPosesServerClient : public rclcpp::Node
{
public:
  using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
  using GoalHandleNavigateThroughPosesClient = rclcpp_action::ClientGoalHandle<NavigateThroughPoses>;
  using GoalHandleNavigateThroughPosesServer = rclcpp_action::ServerGoalHandle<NavigateThroughPoses>;

  explicit NavigateThroughPosesServerClient(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("navigate_through_poses_server_client", options)
  {
    this->client_ptr_ = rclcpp_action::create_client<NavigateThroughPoses>(
      this,
      "navigate_through_poses");

    this->server_ptr_ = rclcpp_action::create_server<NavigateThroughPoses>(
      this,
      "sending_goals",
      std::bind(&NavigateThroughPosesServerClient::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&NavigateThroughPosesServerClient::handle_cancel, this, std::placeholders::_1),
      std::bind(&NavigateThroughPosesServerClient::handle_accepted, this, std::placeholders::_1)
    );
  }

private:
  rclcpp_action::Client<NavigateThroughPoses>::SharedPtr client_ptr_;
  rclcpp_action::Server<NavigateThroughPoses>::SharedPtr server_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const NavigateThroughPoses::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with %zu poses", goal->poses.size());
    (void)uuid;
    // Accept all goals
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleNavigateThroughPosesServer> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    // Accept all cancel requests
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleNavigateThroughPosesServer> goal_handle)
  {
    // This needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&NavigateThroughPosesServerClient::execute_goals, this, goal_handle)}.detach();
  }

  void execute_goals(const std::shared_ptr<GoalHandleNavigateThroughPosesServer> goal_handle)
  {
    if (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
      return;
    }

    auto goal = goal_handle->get_goal();
    auto goal_msg = NavigateThroughPoses::Goal();
    goal_msg.poses = goal->poses;

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&NavigateThroughPosesServerClient::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback =
      std::bind(&NavigateThroughPosesServerClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback =
      std::bind(&NavigateThroughPosesServerClient::result_callback, this, std::placeholders::_1);

    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

  void goal_response_callback(GoalHandleNavigateThroughPosesClient::SharedPtr goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
    GoalHandleNavigateThroughPosesClient::SharedPtr,
    const std::shared_ptr<const NavigateThroughPoses::Feedback> feedback)
  {
    // RCLCPP_INFO(this->get_logger(), "Current position: [%.2f, %.2f]",
    //   feedback->current_pose.pose.position.x,
    //   feedback->current_pose.pose.position.y);
  }

  void result_callback(const GoalHandleNavigateThroughPosesClient::WrappedResult & result)
  {
    switch (result.code) {
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
  auto node = std::make_shared<NavigateThroughPosesServerClient>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
