/*********************************************************************
 *
 * MIT License
 *
 * Copyright (c) 2023 Junyi zhou
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
#pragma once

#ifndef CBS_ROS_H
#define CBS_ROS_H

#include <boost/thread.hpp>

#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/path.hpp"

#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

#include "../utils/timer.hpp"
#include "cbs.hpp"
#include "cbs_env.hpp"
#include "mapf_ros/mapf_ros.hpp"

namespace mapf {

class CBSROS : public mapf::MAPFROS {
public:
  CBSROS();

  CBSROS(std::string name, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros,
         nav2_util::LifecycleNode::SharedPtr node);

  void initialize(std::string name, std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros,
                  nav2_util::LifecycleNode::SharedPtr node) override;

  bool makePlan(const nav_msgs::msg::Path &start, const nav_msgs::msg::Path &goal,
                mapf_msgs::msg::GlobalPlan &plan, double &cost,
                const double &time_tolerance) override;

  // Update global obstacle thread
  void updateObstacleThread();

  void worldToMap(const double &wx, const double &wy, unsigned int &mx, unsigned int &my);
  void mapToWorld(const unsigned int &mx, const unsigned int &my, double &wx, double &wy);

  void generatePlan(const std::vector<PlanResult<State, Action, int>> &solution,
                    const nav_msgs::msg::Path &goal, mapf_msgs::msg::GlobalPlan &plan,
                    double &cost);

  // set the associated location in obstacles to be free
  void clearCell(const unsigned int &mx, const unsigned int &my);

  bool checkIsObstacle(const unsigned int &mx, const unsigned int &my);

  // check if point(mx, my) is surrounded by obstacle
  bool checkSurroundObstacle(const unsigned int &mx, const unsigned int &my);

  ~CBSROS();

protected:
  std::mutex mtx_obs_update_;

  nav2_costmap_2d::Costmap2D *costmap_;
  std::string global_frame_;

  // Update global obstacle thread
  boost::thread *update_obstacle_thread_;

  // mapf env
  std::unordered_set<Location> obstacles_;

  bool initialized_;

  // clock
  rclcpp::Clock::SharedPtr clock_;

  // Logger
  rclcpp::Logger logger_{rclcpp::get_logger("CBSPlanner")};

  nav2_util::LifecycleNode::SharedPtr node_;
};
}; // namespace mapf

#endif