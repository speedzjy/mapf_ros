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
#include "rclcpp/rclcpp.hpp"

#include "pluginlib/class_list_macros.hpp"

#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>

#include "mapf_msgs/msg/global_plan.hpp"
#include "mapf_msgs/msg/goal.h"
#include "mapf_msgs/msg/single_plan.h"

// ROS Wrapper for ECBS
#include "mapf_ros/ecbs/ecbs_ros.hpp"

// Register plugin
PLUGINLIB_EXPORT_CLASS(mapf::ECBSROS, mapf::MAPFROS)

namespace mapf {

ECBSROS::ECBSROS() : costmap_(nullptr), initialized_(false) {}

ECBSROS::ECBSROS(std::string name,
                 std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros,
                 nav2_util::LifecycleNode::SharedPtr node)
    : costmap_(nullptr), initialized_(false) {
  initialize(name, costmap_ros, node);
}

void ECBSROS::initialize(
    std::string name,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros,
    nav2_util::LifecycleNode::SharedPtr node) {
  if (!initialized_) {
    // ROS_INFO("New ECBS planner.");
    node_ = node;
    clock_ = node_->get_clock();
    logger_ = node_->get_logger();

    node_->declare_parameter("ecbs.suboptimality", 1.0);
    node_->get_parameter("ecbs.suboptimality", suboptimality_);

    costmap_ = costmap_ros->getCostmap();
    global_frame_ = costmap_ros->getGlobalFrameID();

    update_obstacle_thread_ =
        new boost::thread(boost::bind(&ECBSROS::updateObstacleThread, this));

    initialized_ = true;
  }
}

void ECBSROS::updateObstacleThread() {
  RCLCPP_INFO(logger_, "update_obstacle_thread: Updating obstacle state...");
  rclcpp::Rate loop_rate(0.5); // update obstacle every 2s

  try {
    while (rclcpp::ok()) {
      int dimx = costmap_->getSizeInCellsX(),
          dimy = costmap_->getSizeInCellsY();
      const unsigned char *costarr = costmap_->getCharMap();

      {
        std::unique_lock<std::mutex> ulock(mtx_obs_update_, std::try_to_lock);
        if (ulock.owns_lock()) {
          obstacles_.clear();

          Timer timer;
          int offset = 0, num_obs = 0;
          for (int i = 0; i < dimy; ++i) {
            for (int j = 0; j < dimx; ++j) {
              if (costarr[offset] >=
                  nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
                obstacles_.insert(Location(j, i));
                num_obs++;
              }
              offset++;
            }
          }
          timer.stop();
        }
      }

      loop_rate.sleep();
      boost::this_thread::interruption_point();
    }
  } catch (boost::thread_interrupted const &) {
    RCLCPP_INFO(logger_, "ecbs_planner: Boost interrupt Exit Obstacle.");
  }
}

bool ECBSROS::makePlan(const nav_msgs::msg::Path &start,
                       const nav_msgs::msg::Path &goal,
                       mapf_msgs::msg::GlobalPlan &plan, double &cost,
                       const double &time_tolerance) {
  // until tf can handle transforming things that are way in the past... we'll
  // require the goal to be in our global frame
  if (goal.header.frame_id != global_frame_) {
    RCLCPP_ERROR(
        logger_,
        "The goal pose passed to this planner must be in the %s frame.  "
        "It is instead in the %s frame.",
        global_frame_.c_str(), goal.header.frame_id.c_str());
    return false;
  }

  if (start.header.frame_id != global_frame_) {
    RCLCPP_ERROR(
        logger_,
        "The start pose passed to this planner must be in the %s frame.  "
        "It is instead in the %s frame.",
        global_frame_.c_str(), start.header.frame_id.c_str());
    return false;
  }

  if (start.poses.empty() || goal.poses.empty()) {
    RCLCPP_ERROR(logger_, "Start and goal vectors are empty!");
    return false;
  }
  if (start.poses.size() != goal.poses.size()) {
    RCLCPP_ERROR(logger_, "Start and goal vectors are not the same length!");
    return false;
  }

  std::lock_guard<std::mutex> lock(mtx_obs_update_);

  int agent_num = start.poses.size();
  // mapf env
  std::vector<State> startStates;
  std::vector<Location> goals;

  // get agents start pose in world frame
  for (int i = 0; i < agent_num; ++i) {
    // transform to map form
    unsigned int start_x_i, start_y_i;
    worldToMap(start.poses[i].pose.position.x, start.poses[i].pose.position.y,
               start_x_i, start_y_i);
    startStates.emplace_back(State(0, start_x_i, start_y_i));

    unsigned int goal_x_i, goal_y_i;
    worldToMap(goal.poses[i].pose.position.x, goal.poses[i].pose.position.y,
               goal_x_i, goal_y_i);
    goals.emplace_back(Location(goal_x_i, goal_y_i));

    // because mapf run in low-resolution map goal points may beFreespace on
    // high-resolution maps but Obstacles on low precision maps so set start and
    // goal to free space

    // If the goal is an obstacle grid with at least one freespace around it, it
    // is reasonable to assume that it is the difference between high and low
    // resolution maps
    // If there's an obstacle at the goal, move_base will do the rest for it
    // (QAQ)

    if (checkSurroundObstacle(goal_x_i, goal_y_i)) {
      RCLCPP_ERROR(logger_, "Goal is surrounded by Obstacles");
      return false;
    }

    // clearCell(start_x_i, start_y_i);
    clearCell(goal_x_i, goal_y_i);
  } // end for

  // mapf search
  int dimx = costmap_->getSizeInCellsX(), dimy = costmap_->getSizeInCellsY();

  std::vector<PlanResult<State, Action, int>> solution;
  Environment mapf(dimx, dimy, obstacles_, goals, false);
  ECBS<State, Action, int, Conflict, Constraints, Environment> ecbs(
      mapf, suboptimality_);

  Timer timer;
  bool success = ecbs.search(startStates, solution, time_tolerance);
  // check time tolerance
  timer.stop();
  if (timer.elapsedSeconds() > time_tolerance) {
    RCLCPP_ERROR(logger_, "Planning time out! Cur time tolerance is %lf",
                 time_tolerance);
    return false;
  }

  if (success) {
    cost = 0;
    generatePlan(solution, goal, plan, cost);

    RCLCPP_DEBUG_STREAM(logger_, "Planning successful!");
    RCLCPP_DEBUG_STREAM(logger_, "runtime: " << timer.elapsedSeconds());
    RCLCPP_DEBUG_STREAM(logger_, "cost: " << cost);
    RCLCPP_DEBUG_STREAM(logger_,
                        "makespan(involve start & end): " << plan.makespan);
    RCLCPP_DEBUG_STREAM(logger_,
                        "highLevelExpanded: " << mapf.highLevelExpanded());
    RCLCPP_DEBUG_STREAM(logger_,
                        "lowLevelExpanded: " << mapf.lowLevelExpanded());
  } else {
    RCLCPP_ERROR(logger_, "Planning NOT successful!");
  }

  return success;
}

void ECBSROS::generatePlan(
    const std::vector<PlanResult<State, Action, int>> &solution,
    const nav_msgs::msg::Path &goal, mapf_msgs::msg::GlobalPlan &plan,
    double &cost) {
  int &makespan = plan.makespan;
  for (const auto &s : solution) {
    cost += s.cost;
  }

  plan.global_plan.resize(solution.size());

  for (size_t i = 0; i < solution.size(); ++i) {
    // create a message for the plan
    mapf_msgs::msg::SinglePlan &single_plan = plan.global_plan[i];
    nav_msgs::msg::Path &single_path = single_plan.plan;
    single_path.header.frame_id = global_frame_;
    single_path.header.stamp = clock_->now();

    for (const auto &state : solution[i].states) {
      geometry_msgs::msg::PoseStamped cur_pose;
      cur_pose.header.frame_id = single_path.header.frame_id;
      cur_pose.pose.orientation.w = 1;
      mapToWorld(state.first.x, state.first.y, cur_pose.pose.position.x,
                 cur_pose.pose.position.y);
      single_path.poses.push_back(cur_pose);
      single_plan.time_step.push_back(state.second);
    }

    // replace end point with goal point
    single_path.poses.back() = goal.poses[i];

    // pop start point if it is not a inplace plan
    if (single_path.poses.size() > 1) {
      single_path.poses.erase(single_path.poses.begin());
      single_plan.time_step.erase(single_plan.time_step.begin());
    }
  } // end solution for

  // compute makespan
  makespan = 0;
  for (const auto &single_plan : plan.global_plan) {
    plan.makespan = std::max<int>(plan.makespan, single_plan.plan.poses.size());
  }
}

void ECBSROS::worldToMap(const double &wx, const double &wy, unsigned int &mx,
                         unsigned int &my) {
  if (!costmap_->worldToMap(wx, wy, mx, my)) {
    RCLCPP_WARN(logger_,
                "The robot's start position is off the global costmap. "
                "Planning will "
                "always fail, are you sure the robot has been properly "
                "localized?");
  }
}

void ECBSROS::mapToWorld(const unsigned int &mx, const unsigned int &my,
                         double &wx, double &wy) {
  costmap_->mapToWorld(mx, my, wx, wy);
}

void ECBSROS::clearCell(const unsigned int &mx, const unsigned int &my) {
  // costmap_->setCost(mx, my, costmap_2d::FREE_SPACE);
  if (obstacles_.find(Location(mx, my)) != obstacles_.end()) {
    obstacles_.erase(Location(mx, my));
  }
}

bool ECBSROS::checkIsObstacle(const unsigned int &mx, const unsigned int &my) {
  return (costmap_->getCost(mx, my) >=
          nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
}

bool ECBSROS::checkSurroundObstacle(const unsigned int &mx,
                                    const unsigned int &my) {
  int dimx = costmap_->getSizeInCellsX(), dimy = costmap_->getSizeInCellsY();
  bool check_surround = true;
  std::vector<std::pair<int, int>> step{{0, 1}, {0, -1}, {-1, 0}, {1, 0}};
  for (const auto &s : step) {
    const int &x = s.first, &y = s.second;
    if (mx + x >= 0 && mx + x < dimx && my + y >= 0 && my + y < dimy) {
      check_surround &= checkIsObstacle(mx + x, my + y);
    }
  }
  return check_surround;
}

ECBSROS::~ECBSROS() {
  update_obstacle_thread_->interrupt();
  update_obstacle_thread_->join();
  delete update_obstacle_thread_;

  costmap_ = nullptr;

  RCLCPP_INFO(logger_, "Exit ECBS planner.");
}

} // namespace mapf