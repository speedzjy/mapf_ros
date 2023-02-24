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
#include <algorithm>
#include <iostream>
#include <mutex>
#include <vector>

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>

#include <mapf_msgs/Goal.h>

class GoalTransformer {
private:
  ros::NodeHandle nh_;
  ros::Subscriber sub_mapf_goal_init_;
  ros::V_Subscriber goal_sub_arr_;
  ros::Publisher pub_mapf_goal_;

  // mapf params
  int agent_num_;
  std::vector<std::string> goal_topic_;

  mapf_msgs::Goal goal_arr_;

  std::mutex goal_mtx;

public:
  GoalTransformer();
  ~GoalTransformer();

  void goalCallback(const geometry_msgs::PoseStamped::ConstPtr &goal,
                    geometry_msgs::PoseStamped &goal_pose);

  void goalInitCallback(const std_msgs::Bool::ConstPtr &init);
}; // class GoalTransformer

GoalTransformer::GoalTransformer() {
  nh_.param<int>("agent_num", agent_num_, 1);
  goal_sub_arr_.resize(agent_num_);
  goal_topic_.resize(agent_num_);
  goal_arr_.goal.poses.resize(agent_num_);

  // subscribe goal topic
  for (int i = 0; i < agent_num_; ++i) {
    nh_.param<std::string>("goal_topic/agent_" + std::to_string(i),
                           goal_topic_[i], "goal");
    geometry_msgs::PoseStamped &pose_i = goal_arr_.goal.poses[i];
    goal_sub_arr_[i] = nh_.subscribe<geometry_msgs::PoseStamped>(
        goal_topic_[i], 5,
        [this, &pose_i](const geometry_msgs::PoseStamped::ConstPtr &msg) {
          goalCallback(msg, pose_i);
        });
  }

  // subscribe goal init flag
  sub_mapf_goal_init_ = nh_.subscribe<std_msgs::Bool>(
      "goal_init_flag", 1, &GoalTransformer::goalInitCallback, this);
  // pub goal in mapf form
  pub_mapf_goal_ = nh_.advertise<mapf_msgs::Goal>("mapf_goal", 1);
}

GoalTransformer::~GoalTransformer() {}

void GoalTransformer::goalCallback(
    const geometry_msgs::PoseStamped::ConstPtr &goal,
    geometry_msgs::PoseStamped &goal_pose) {
  std::lock_guard<std::mutex> lock(goal_mtx);
  goal_pose = *goal;
}

void GoalTransformer::goalInitCallback(const std_msgs::Bool::ConstPtr &init) {
  if (init->data) {
    goal_arr_.header.stamp = ros::Time::now();
    goal_arr_.initial = true;
    pub_mapf_goal_.publish(goal_arr_);
  }
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "goal_transformer_node");

  GoalTransformer gt;

  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();

  return 0;
}