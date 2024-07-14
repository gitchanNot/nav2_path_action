#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class NavigateThroughPosesClient : public rclcpp::Node
{
public:
  using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
  using GoalHandleNavigateThroughPoses = rclcpp_action::ClientGoalHandle<NavigateThroughPoses>;

  explicit NavigateThroughPosesClient(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("navigate_through_poses_client", options)
  {
    this->client_ptr_ = rclcpp_action::create_client<NavigateThroughPoses>(
      this,
      "navigate_through_poses");

    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&NavigateThroughPosesClient::send_goal, this));
  }

private:
  rclcpp_action::Client<NavigateThroughPoses>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;

  void send_goal()
  {
    this->timer_->cancel();

    if (!this->client_ptr_->wait_for_action_server()) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
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


    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<NavigateThroughPoses>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&NavigateThroughPosesClient::goal_response_callback, this, std::placeholders::_1);
    send_goal_options.feedback_callback =
      std::bind(&NavigateThroughPosesClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback =
      std::bind(&NavigateThroughPosesClient::result_callback, this, std::placeholders::_1);
    
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

  void goal_response_callback(GoalHandleNavigateThroughPoses::SharedPtr goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
    GoalHandleNavigateThroughPoses::SharedPtr,
    const std::shared_ptr<const NavigateThroughPoses::Feedback> feedback)
  {
    RCLCPP_INFO(this->get_logger(), "Current position: [%.2f, %.2f]",
      feedback->current_pose.pose.position.x,
      feedback->current_pose.pose.position.y);
  }

  void result_callback(const GoalHandleNavigateThroughPoses::WrappedResult & result)
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

void execute(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NavigateThroughPosesClient>();
  rclcpp::spin(node);
  rclcpp::shutdown();
}
int main(int argc, char ** argv)
{
  execute(argc, argv);
  return 0;
}
