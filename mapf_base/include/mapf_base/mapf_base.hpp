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
#pragma once

#ifndef MAPF_BASE_H
#define MAPF_BASE_H

#include <ros/ros.h>

#include <costmap_2d/costmap_2d.h>
#include <pluginlib/class_loader.h>

#include <std_srvs/Empty.h>
#include <std_srvs/SetBool.h>

#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>

#include "mapf_msgs/GlobalPlan.h"
#include "mapf_msgs/Goal.h"
#include "mapf_msgs/SinglePlan.h"
#include "mapf_ros/cbs/cbs_ros.hpp"

namespace mapf {
class MAPFBase {
public:
  MAPFBase(tf2_ros::Buffer &tf);
  ~MAPFBase();

  void getParam();

  void goalCallback(const mapf_msgs::Goal::ConstPtr &goal);

  nav_msgs::Path getRobotPose();

  // check if reach goal
  bool reachGoal();

  void doMAPFThread();

  void stateMachine();

  void publishPlan(const mapf_msgs::GlobalPlan &plan);

private:
  std::mutex mtx_mapf_goal_;
  std::mutex mtx_planner_;

  ros::NodeHandle nh_;
  ros::Subscriber sub_mapf_goal_;
  ros::V_Publisher pub_gui_plan_;
  ros::Publisher pub_mapf_global_plan_;

  // mapf params
  std::string planner_name_;
  int agent_num_;
  std::string global_frame_id_;
  double planner_time_tolerance_;
  double goal_tolerance_;
  std::vector<std::string> base_frame_id_;
  std::vector<std::string> plan_topic_;

  nav_msgs::Path goal_ros_;
  bool receive_mapf_goal_;
  bool run_mapf_;

  boost::thread *do_mapf_thread_;
  boost::thread *state_machine_thread_;

  tf2_ros::Buffer &tf_;
  costmap_2d::Costmap2DROS *costmap_ros_;

  pluginlib::ClassLoader<mapf::MAPFROS> mapf_loader_;
  boost::shared_ptr<mapf::MAPFROS> mapf_planner_;
};
}; // namespace mapf

#endif