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

#ifndef MAPF_ROS_H
#define MAPF_ROS_H

#include "rclcpp/rclcpp.hpp"

#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/costmap.hpp"
#include "nav_msgs/msg/path.hpp"

#include "mapf_msgs/msg/global_plan.hpp"

namespace mapf {

class MAPFROS {
public:
  virtual void initialize(std::string name,
                          std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros,
                          nav2_util::LifecycleNode::SharedPtr node) = 0;

  virtual bool makePlan(const nav_msgs::msg::Path &start, const nav_msgs::msg::Path &goal,
                        mapf_msgs::msg::GlobalPlan &plan, double &cost,
                        const double &time_tolerance) = 0;

  virtual ~MAPFROS() {}

protected:
  MAPFROS() {}
};
}; // namespace mapf

#endif