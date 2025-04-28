/*********************************************************************
 *
 * MIT License
 *
 * Copyright (c) 2023 Junyi zhou
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

#include "geometry_msgs/msg/pose_stamped.h"
#include "geometry_msgs/msg/twist.h"
#include "std_msgs/msg/bool.h"

#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "mapf_msgs/msg/global_plan.hpp"
#include "mapf_msgs/msg/single_plan.hpp"

#include "mapf_ros/utils/utility.hpp"

class ParamServer : public rclcpp::Node {
public:
  int agent_num_;
  std::vector<std::string> agent_name_;

  std::string global_frame_id_;
  std::vector<std::string> base_frame_id_;
  std::vector<std::string> plan_topic_;

  ParamServer(const std::string &node_name) : Node(node_name) {
    this->declare_parameter<int>("agent_num", 1);
    this->declare_parameter<std::string>("global_frame_id", "map");

    this->get_parameter("agent_num", agent_num_);
    this->get_parameter("global_frame_id", global_frame_id_);

    agent_name_.resize(agent_num_);
    base_frame_id_.resize(agent_num_);
    plan_topic_.resize(agent_num_);

    for (int i = 0; i < agent_num_; ++i) {
      this->declare_parameter<std::string>(
          "base_frame_id/agent_" + std::to_string(i), "base_link");
      this->declare_parameter<std::string>(
          "plan_topic/agent_" + std::to_string(i), "plan");
      this->declare_parameter<std::string>(
          "agent_name/agent_" + std::to_string(i), "agent_name_0");

      this->get_parameter("base_frame_id/agent_" + std::to_string(i),
                          base_frame_id_[i]);
      this->get_parameter("plan_topic/agent_" + std::to_string(i),
                          plan_topic_[i]);
      this->get_parameter("agent_name/agent_" + std::to_string(i),
                          agent_name_[i]);
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

  std::unique_ptr<std::thread> planner_thread_;

  using Nav2ActionClient =
      rclcpp_action::Client<nav2_msgs::action::NavigateToPose>;
  std::vector<Nav2ActionClient::SharedPtr> ac_ptr_arr_;

public:
  PlanExecutor()
      : ParamServer("plan_executor_node"), make_span_(0), get_plan_(false),
        ac_ptr_arr_(agent_num_, nullptr) {

    plan_arr_.resize(agent_num_);
    ac_ptr_arr_.resize(agent_num_);

    for (int i = 0; i < agent_num_; ++i) {
      ac_ptr_arr_[i] =
          rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(
              this->shared_from_this(),
              "/" + agent_name_[i] + "/navigate_to_pose");
    }

    sub_mapf_plan_ = this->create_subscription<mapf_msgs::msg::GlobalPlan>(
        "global_plan", 1,
        std::bind(&PlanExecutor::planCallback, this, std::placeholders::_1));

    planner_thread_ = std::make_unique<std::thread>(
        std::bind(&PlanExecutor::mbStateThread, this));
  }

  ~PlanExecutor() {}

  void mbStateThread() {
    RCLCPP_INFO(this->get_logger(),
                "mapf_plan_thread: Plan and Read move base state...");
    rclcpp::Rate loop_rate(10);

    using GoalHandleFuture = std::shared_future<std::shared_ptr<
        rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>>>;
    std::vector<GoalHandleFuture> send_goal_future(agent_num_);

    while (rclcpp::ok()) {
      loop_rate.sleep();
      if (get_plan_) {
        std::unique_lock<std::mutex> lock(plan_mtx_);
        get_plan_ = false;

        // loop time step
        for (int i = 0; i < make_span_; ++i) {

          // loop each agent,
          for (int j = 0; j < plan_arr_.size(); ++j) {
            // For each agent, if the current time step is less than the total
            // time step, execute move_base
            if (i < plan_arr_[j].time_step.size()) {
              auto goal_msg = getMBGoalFromGeoPose(plan_arr_[j].plan.poses[i]);
              send_goal_future[j] = ac_ptr_arr_[j]->async_send_goal(goal_msg);
            }
          } // end for

          // check if get new plan
          lock.unlock();
          loop_rate.sleep();
          lock.lock();
          // exit this cycle, ececute new plan
          if (get_plan_) {
            RCLCPP_INFO(this->get_logger(), "Plan changed, execute new plan..");
            for (int j = 0; j < plan_arr_.size(); ++j) {
              if (i < plan_arr_[j].time_step.size()) {
                ac_ptr_arr_[j]->async_cancel_goal(send_goal_future[j].get());
              }
            } // end for
            break;
          }

          // wait for reach step goal
          for (int j = 0; j < plan_arr_.size(); ++j) {
            if (i < plan_arr_[j].time_step.size()) {
              auto result_future =
                  ac_ptr_arr_[j]->async_get_result(send_goal_future[j].get());

              while (rclcpp::ok()) {
                auto status = result_future.wait_for(std::chrono::seconds(1));
                if (status == std::future_status::ready) {
                  auto result = result_future.get();
                  if (result.result &&
                      result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                    break;
                  }
                }
                // check if get new plan
                lock.unlock();
                loop_rate.sleep();
                lock.lock();
                if (get_plan_) {
                  ac_ptr_arr_[j]->async_cancel_goal(send_goal_future[j].get());
                  break;
                }
              }

              if (i == plan_arr_[j].time_step.size() - 1) {
                RCLCPP_INFO(this->get_logger(),
                            "Agent %d reached %dth step goal(" GREEN "END" NONE
                            ")!",
                            j, i);
              } else {
                RCLCPP_INFO(this->get_logger(),
                            "Agent %d reached %dth step goal!", j, i);
              }
            }
          } // end for
        } // end loop time step
      }
    } // end while
  }

  void
  planCallback(const mapf_msgs::msg::GlobalPlan::SharedPtr mapf_global_plan) {
    if (!equal(plan_arr_, mapf_global_plan->global_plan)) {
      std::lock_guard<std::mutex> lock(plan_mtx_);

      get_plan_ = true;
      make_span_ = mapf_global_plan->makespan;
      for (int i = 0; i < agent_num_; ++i) {
        // if
        plan_arr_[i] = mapf_global_plan->global_plan[i];
      }
      RCLCPP_INFO(this->get_logger(), GREEN "Get New plan.." NONE);
      RCLCPP_INFO(this->get_logger(), GREEN "MakeSpan: %d" NONE, make_span_);
      RCLCPP_INFO(this->get_logger(), GREEN "AgentNum: %d" NONE, agent_num_);
    }
  }

  bool equal(const mapf_msgs::msg::SinglePlan &a,
             const mapf_msgs::msg::SinglePlan &b) {
    if (a.time_step.size() != b.time_step.size() ||
        a.plan.poses.size() != b.plan.poses.size()) {
      return false;
    }
    bool res = true;
    for (int i = 0; i < a.plan.poses.size(); ++i) {
      res &=
          (a.plan.poses[i].pose.position.x == b.plan.poses[i].pose.position.x &&
           a.plan.poses[i].pose.position.y == b.plan.poses[i].pose.position.y &&
           a.plan.poses[i].pose.orientation.w ==
               b.plan.poses[i].pose.orientation.w);
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

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}