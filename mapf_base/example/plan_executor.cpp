/*********************************************************************
 *
 * MIT License
 *
 * Copyright (c) 2023 junyi zhou
 * Copyright (c) 2025 junyi zhou
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 *all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 *********************************************************************/
#include <cmath>
#include <fstream>
#include <string>
#include <vector>

#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "geometry_msgs/msg/pose_stamped.h"
#include "geometry_msgs/msg/twist.h"
#include "std_msgs/msg/bool.h"

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_util/lifecycle_node.hpp"

#include "mapf_msgs/msg/global_plan.hpp"
#include "mapf_msgs/msg/single_plan.hpp"

#include "mapf_ros/utils/utility.hpp"

using namespace std::placeholders;

class ParamServer : public rclcpp::Node {
public:
  int agent_num_;
  std::vector<std::string> agent_name_;

  std::string global_frame_id_;
  std::vector<std::string> base_frame_id_;
  std::vector<std::string> plan_topic_;

  double xy_goal_tolerance_;
  double yaw_goal_tolerance_;

  ParamServer(const std::string &node_name) : Node(node_name) {
    this->declare_parameter<double>("xy_goal_tolerance", 0.2);
    this->declare_parameter<double>("yaw_goal_tolerance", 0.2);
    this->declare_parameter<int>("agent_num", 1);
    this->declare_parameter<std::string>("global_frame_id", "map");

    this->get_parameter("xy_goal_tolerance", xy_goal_tolerance_);
    this->get_parameter("yaw_goal_tolerance", yaw_goal_tolerance_);
    this->get_parameter("agent_num", agent_num_);
    this->get_parameter("global_frame_id", global_frame_id_);

    agent_name_.resize(agent_num_);
    base_frame_id_.resize(agent_num_);
    plan_topic_.resize(agent_num_);

    for (int i = 0; i < agent_num_; ++i) {
      this->declare_parameter<std::string>("base_frame_id.agent_" + std::to_string(i), "base_link");
      this->declare_parameter<std::string>("plan_topic.agent_" + std::to_string(i), "plan");
      this->declare_parameter<std::string>("agent_name.agent_" + std::to_string(i), "agent_name_0");

      this->get_parameter("base_frame_id.agent_" + std::to_string(i), base_frame_id_[i]);
      this->get_parameter("plan_topic.agent_" + std::to_string(i), plan_topic_[i]);
      this->get_parameter("agent_name.agent_" + std::to_string(i), agent_name_[i]);
    }
  }
};

class PlanExecutor : public ParamServer {
private:
  std::mutex plan_mtx_;

  rclcpp::Subscription<mapf_msgs::msg::GlobalPlan>::SharedPtr sub_mapf_plan_;

  int make_span_;
  std::vector<mapf_msgs::msg::SinglePlan> plan_arr_;

  bool get_plan_;
  bool pose_initalize_;

  std::vector<geometry_msgs::msg::PoseStamped> cur_poses_;
  std::vector<geometry_msgs::msg::PoseStamped> last_goals_;

  std::unique_ptr<std::thread> planner_thread_;
  std::unique_ptr<std::thread> get_pose_thread_;

  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;
  using GoalHandleNavigateToPoseFuture =
      std::shared_future<std::shared_ptr<GoalHandleNavigateToPose>>;

  using Nav2ActionClient = rclcpp_action::Client<NavigateToPose>;
  std::vector<Nav2ActionClient::SharedPtr> ac_ptr_arr_;

  std::shared_ptr<tf2_ros::TransformListener> tf_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

public:
  PlanExecutor()
      : ParamServer("plan_executor_node"), make_span_(0), get_plan_(false),
        ac_ptr_arr_(agent_num_, nullptr) {

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    plan_arr_.resize(agent_num_);
    ac_ptr_arr_.resize(agent_num_);

    cur_poses_.resize(agent_num_);

    for (int i = 0; i < agent_num_; ++i) {
      ac_ptr_arr_[i] = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
          this, "/" + agent_name_[i] + "/navigate_to_pose");
    }

    sub_mapf_plan_ = this->create_subscription<mapf_msgs::msg::GlobalPlan>(
        "global_plan", 1, std::bind(&PlanExecutor::planCallback, this, std::placeholders::_1));

    planner_thread_ = std::make_unique<std::thread>(std::bind(&PlanExecutor::mbStateThread, this));
    get_pose_thread_ = std::make_unique<std::thread>(std::bind(&PlanExecutor::getPoseThread, this));

    rclcpp::Rate loop_rate(10);
    while (rclcpp::ok() && !pose_initalize_) {
      loop_rate.sleep();
    }

    last_goals_ = cur_poses_;
  }

  ~PlanExecutor() {}

  void getPoseThread() {
    RCLCPP_INFO(this->get_logger(), "get_pose_thread: Get current pose...");

    rclcpp::Rate loop_rate(10);
    while (rclcpp::ok()) {
      loop_rate.sleep();

      // get current pose
      for (int i = 0; i < agent_num_; ++i) {
        try {
          geometry_msgs::msg::PoseStamped robot_pose;
          tf2::toMsg(tf2::Transform::getIdentity(), robot_pose.pose);
          tf2::toMsg(tf2::Transform::getIdentity(), cur_poses_[i].pose);
          robot_pose.header.frame_id = base_frame_id_[i];
          robot_pose.header.stamp = rclcpp::Time(0);

          tf_buffer_->transform(robot_pose, cur_poses_[i], global_frame_id_);
        } catch (const tf2::TransformException &ex) {
          RCLCPP_ERROR(get_logger(), "Failed to transform pose for agent %d: %s", i, ex.what());
        }
      }

      if (!pose_initalize_) {
        RCLCPP_INFO(this->get_logger(), GREEN "INITIALIZE POSE DONE." NONE);
        pose_initalize_ = true;
      }
    }
  }

  void mbStateThread() {
    RCLCPP_INFO(this->get_logger(), "mapf_plan_thread: Plan and Read move base state...");
    rclcpp::Rate loop_rate(10);

    std::vector<GoalHandleNavigateToPoseFuture> send_goal_future_(agent_num_);

    while (rclcpp::ok()) {
      loop_rate.sleep();

      if (get_plan_) {
        std::unique_lock<std::mutex> lock(plan_mtx_);
        get_plan_ = false;

        std::vector<geometry_msgs::msg::PoseStamped> cur_final_goal(agent_num_);
        std::vector<geometry_msgs::msg::PoseStamped> cur_goal_(agent_num_);

        for (int n = 0; n < agent_num_; ++n) {
          last_goals_[n] = cur_final_goal[n] = plan_arr_[n].plan.poses.back();
        }

        // loop time step
        for (int i = 0; i < make_span_; ++i) {
          // loop each agent,
          for (int j = 0; j < plan_arr_.size(); ++j) {
            // For each agent, if the current time step is less than the total
            // time step, execute move_base
            if (i < plan_arr_[j].time_step.size()) {
              if (!ac_ptr_arr_[j]->wait_for_action_server()) {
                RCLCPP_ERROR(this->get_logger(),
                             "Agent %u action server not available after waiting", j);
                rclcpp::shutdown();
              }
              RCLCPP_INFO(this->get_logger(), "Agent %u send %u step goal(x, y) = (%f, %f)", j, i,
                          plan_arr_[j].plan.poses[i].pose.position.x,
                          plan_arr_[j].plan.poses[i].pose.position.y);

              auto send_goal_options = Nav2ActionClient::SendGoalOptions();

              send_goal_options.goal_response_callback =
                  [this, j](const GoalHandleNavigateToPose::SharedPtr &goal_handle) {
                    if (!goal_handle) {
                      RCLCPP_ERROR(this->get_logger(), "Agent %d goal was rejected", j);
                    } else {
                      RCLCPP_INFO(this->get_logger(), "Agent %d goal accepted, waiting for result",
                                  j);
                    }
                  };

              cur_goal_[j] = plan_arr_[j].plan.poses[i];
              auto goal_msg = getMBGoalFromGeoPose(plan_arr_[j].plan.poses[i]);
              send_goal_future_[j] = ac_ptr_arr_[j]->async_send_goal(goal_msg, send_goal_options);
            }
          } // end for

          // check if get new plan
          lock.unlock();
          loop_rate.sleep();
          lock.lock();
          // exit this cycle, ececute new plan
          if (get_plan_) {
            RCLCPP_INFO(this->get_logger(), "Plan changed, execute new plan..");
            break;
          }

          // wait for reach step goal
          RCLCPP_INFO(this->get_logger(), "Wait for reach step goal...");
          for (int j = 0; j < plan_arr_.size(); ++j) {
            if (i < plan_arr_[j].time_step.size()) {

              while (rclcpp::ok()) {
                loop_rate.sleep();

                if (i == plan_arr_[j].time_step.size() - 1) {
                  if (nearToCurGoal(cur_poses_[j], cur_goal_[j], xy_goal_tolerance_,
                                    yaw_goal_tolerance_)) {
                    RCLCPP_INFO(this->get_logger(),
                                "Agent %d reached %dth step goal(" GREEN "END" NONE ")!", j, i);
                    break;
                  }

                } else {
                  if (nearToCurGoal(cur_poses_[j], cur_goal_[j], 0.4)) {
                    RCLCPP_INFO(this->get_logger(), "Agent %d reached %dth step goal!", j, i);
                    break;
                  }
                }

                // check if get new plan
                lock.unlock();
                loop_rate.sleep();
                lock.lock();
                if (get_plan_) {
                  break;
                }
              } // end while
            }
          } // end for
        } // end loop time step
      }
    } // end while
  }

  bool nearToCurGoal(const geometry_msgs::msg::PoseStamped &cur_pose,
                     const geometry_msgs::msg::PoseStamped &cur_goal, double xy_tolerance,
                     double yaw_tolerance = 2 * M_PI) {
    double diff_x = cur_pose.pose.position.x - cur_goal.pose.position.x;
    double diff_y = cur_pose.pose.position.y - cur_goal.pose.position.y;
    double diff_yaw =
        tf2::getYaw(cur_pose.pose.orientation) - tf2::getYaw(cur_goal.pose.orientation);
    return abs(diff_yaw) < yaw_tolerance and
           (diff_x * diff_x + diff_y * diff_y) < xy_tolerance * xy_tolerance and
           (diff_x * diff_x + diff_y * diff_y) > 1e-6;
  }

  void planCallback(const mapf_msgs::msg::GlobalPlan::SharedPtr mapf_global_plan) {
    if (!equal(plan_arr_, mapf_global_plan->global_plan)) {
      std::lock_guard<std::mutex> lock(plan_mtx_);

      get_plan_ = true;
      make_span_ = mapf_global_plan->makespan;
      for (int i = 0; i < agent_num_; ++i) {
        plan_arr_[i] = mapf_global_plan->global_plan[i];
      }
      RCLCPP_INFO(this->get_logger(), GREEN "Get New plan.." NONE);
      RCLCPP_INFO(this->get_logger(), GREEN "MakeSpan: %d" NONE, make_span_);
      RCLCPP_INFO(this->get_logger(), GREEN "AgentNum: %d" NONE, agent_num_);
    }
  }

  bool equal(const mapf_msgs::msg::SinglePlan &a, const mapf_msgs::msg::SinglePlan &b) {
    if (a.time_step.size() != b.time_step.size() || a.plan.poses.size() != b.plan.poses.size()) {
      return false;
    }
    bool res = true;
    for (int i = 0; i < a.plan.poses.size(); ++i) {
      res &= (a.plan.poses[i].pose.position.x == b.plan.poses[i].pose.position.x &&
              a.plan.poses[i].pose.position.y == b.plan.poses[i].pose.position.y &&
              a.plan.poses[i].pose.orientation.w == b.plan.poses[i].pose.orientation.w);
    }
    return res;
  }

  bool equal(const std::vector<mapf_msgs::msg::SinglePlan> &a,
             const std::vector<mapf_msgs::msg::SinglePlan> &b) {
    if (a.size() != b.size()) {
      return false;
    }
    bool res = true;
    for (int i = 0; i < a.size(); ++i) {
      res &= equal(a[i], b[i]);
    }
    return res;
  }

  nav2_msgs::action::NavigateToPose::Goal
  getMBGoalFromGeoPose(const geometry_msgs::msg::PoseStamped &curr_location) {
    nav2_msgs::action::NavigateToPose::Goal tmp_goal;
    tmp_goal.pose.header.frame_id = "map";
    tmp_goal.pose.header.stamp = this->get_clock()->now();
    tmp_goal.pose.pose = curr_location.pose;
    return tmp_goal;
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PlanExecutor>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}