/*********************************************************************
 *
 * MIT License
 *
 * Copyright (c) 2023 Junyi zhou
 * Copyright (c) 2025 Junyi zhou
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
#include <algorithm>
#include <iostream>
#include <mutex>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/bool.hpp"

#include "mapf_msgs/msg/goal.hpp"

class GoalTransformer : public rclcpp::Node {
private:
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_mapf_goal_init_;
  rclcpp::Publisher<mapf_msgs::msg::Goal>::SharedPtr pub_mapf_goal_;
  std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> goal_sub_arr_;

  // mapf params
  int agent_num_;
  std::vector<std::string> goal_topic_;

  mapf_msgs::msg::Goal goal_arr_;

  std::mutex goal_mtx;

public:
  GoalTransformer();
  ~GoalTransformer();

  void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr goal,
                    geometry_msgs::msg::PoseStamped &goal_pose);

  void goalInitCallback(const std_msgs::msg::Bool::SharedPtr init);
}; // class GoalTransformer

GoalTransformer::GoalTransformer() : Node("goal_transformer_node") {
  this->declare_parameter<int>("agent_num", 1);
  this->get_parameter("agent_num", agent_num_);

  goal_sub_arr_.resize(agent_num_);
  goal_topic_.resize(agent_num_);
  goal_arr_.goal.poses.resize(agent_num_);

  // subscribe goal topic
  for (int i = 0; i < agent_num_; ++i) {
    std::string goal_topic_param = "goal_topic.agent_" + std::to_string(i);
    this->declare_parameter<std::string>(goal_topic_param, "goal");
    this->get_parameter(goal_topic_param, goal_topic_[i]);

    geometry_msgs::msg::PoseStamped &pose_i = goal_arr_.goal.poses[i];
    goal_sub_arr_[i] = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        goal_topic_[i], 5, [this, &pose_i](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
          goalCallback(msg, pose_i);
        });
  }

  // subscribe goal init flag
  sub_mapf_goal_init_ = this->create_subscription<std_msgs::msg::Bool>(
      "goal_init_flag", 1,
      std::bind(&GoalTransformer::goalInitCallback, this, std::placeholders::_1));
  // pub goal in mapf form
  pub_mapf_goal_ = this->create_publisher<mapf_msgs::msg::Goal>("mapf_goal", 1);
}

GoalTransformer::~GoalTransformer() {}

void GoalTransformer::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr goal,
                                   geometry_msgs::msg::PoseStamped &goal_pose) {
  std::lock_guard<std::mutex> lock(goal_mtx);
  goal_pose = *goal;
}

void GoalTransformer::goalInitCallback(const std_msgs::msg::Bool::SharedPtr init) {
  if (init->data) {
    goal_arr_.header.stamp = this->get_clock()->now();
    goal_arr_.initial = true;
    pub_mapf_goal_->publish(goal_arr_);
  }
}

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GoalTransformer>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}