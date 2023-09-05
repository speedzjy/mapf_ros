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
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>

#include "mapf_msgs/GlobalPlan.h"
#include "mapf_msgs/Goal.h"
#include "mapf_msgs/SinglePlan.h"

// ROS Wrapper for SIPP
#include "mapf_ros/sipp/prioritized_sipp_ros.hpp"

// Register plugin
PLUGINLIB_EXPORT_CLASS(mapf::SIPPROS, mapf::MAPFROS)

namespace mapf {

SIPPROS::SIPPROS() : costmap_(nullptr), initialized_(false) {}

SIPPROS::SIPPROS(std::string name, costmap_2d::Costmap2DROS *costmap_ros)
    : costmap_(nullptr), initialized_(false) {
  initialize(name, costmap_ros);
}

void SIPPROS::initialize(std::string name,
                         costmap_2d::Costmap2DROS *costmap_ros) {
  if (!initialized_) {
    // ROS_INFO("New CBS planner.");

    costmap_ = costmap_ros->getCostmap();
    global_frame_ = costmap_ros->getGlobalFrameID();

    update_obstacle_thread_ =
        new boost::thread(boost::bind(&SIPPROS::updateObstacleThread, this));

    initialized_ = true;
  }
}

void SIPPROS::updateObstacleThread() {
  ROS_INFO_NAMED("update_obstacle_thread", "Updating obstacle state...");
  ros::NodeHandle thread_nh;
  ros::Rate loop_rate(0.5); // update obstacle every 2s

  try {
    while (thread_nh.ok()) {
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
              if (costarr[offset] >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
                obstacles_.insert(State(j, i));
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
    ROS_INFO_NAMED("sipp_planner", "Boost interrupt Exit Obstacle.");
  }
}

bool SIPPROS::makePlan(const nav_msgs::Path &start, const nav_msgs::Path &goal,
                       mapf_msgs::GlobalPlan &plan, double &cost,
                       const double &time_tolerance) {
  // until tf can handle transforming things that are way in the past... we'll
  // require the goal to be in our global frame
  if (goal.header.frame_id != global_frame_) {
    ROS_ERROR("The goal pose passed to this planner must be in the %s frame.  "
              "It is instead in the %s frame.",
              global_frame_.c_str(), goal.header.frame_id.c_str());
    return false;
  }

  if (start.header.frame_id != global_frame_) {
    ROS_ERROR("The start pose passed to this planner must be in the %s frame.  "
              "It is instead in the %s frame.",
              global_frame_.c_str(), start.header.frame_id.c_str());
    return false;
  }

  if (start.poses.empty() || goal.poses.empty()) {
    ROS_ERROR("Start and goal vectors are empty!");
    return false;
  }
  if (start.poses.size() != goal.poses.size()) {
    ROS_ERROR("Start and goal vectors are not the same length!");
    return false;
  }

  std::lock_guard<std::mutex> lock(mtx_obs_update_);

  int agent_num = start.poses.size();
  // mapf env
  std::vector<State> startStates;
  std::vector<State> goals;

  // get agents start pose in world frame
  for (int i = 0; i < agent_num; ++i) {
    // transform to map form
    unsigned int start_x_i, start_y_i;
    worldToMap(start.poses[i].pose.position.x, start.poses[i].pose.position.y,
               start_x_i, start_y_i);
    startStates.emplace_back(State(start_x_i, start_y_i));

    unsigned int goal_x_i, goal_y_i;
    worldToMap(goal.poses[i].pose.position.x, goal.poses[i].pose.position.y,
               goal_x_i, goal_y_i);
    goals.emplace_back(State(goal_x_i, goal_y_i));

    // because mapf run in low-resolution map goal points may beFreespace on
    // high-resolution maps but Obstacles on low precision maps so set start and
    // goal to free space

    // If the goal is an obstacle grid with at least one freespace around it, it
    // is reasonable to assume that it is the difference between high and low
    // resolution maps
    // If there's an obstacle at the goal, move_base will do the rest for it
    // (QAQ)

    if (checkSurroundObstacle(goal_x_i, goal_y_i)) {
      ROS_ERROR("Goal is surrounded by Obstacles");
      return false;
    }

    clearCell(start_x_i, start_y_i);
    clearCell(goal_x_i, goal_y_i);
  } // end for

  // mapf search
  int dimx = costmap_->getSizeInCellsX(), dimy = costmap_->getSizeInCellsY();

  typedef SIPP<State, State, Action, int, Environment> sipp_t;
  // Plan (sequentially)
  std::map<State, std::vector<sipp_t::interval>> allCollisionIntervals;
  std::vector<PlanResult<State, Action, int>> solutions(goals.size());

  Timer timer;

  bool success = false;
  for (size_t i = 0; i < goals.size(); ++i) {
    // ROS_INFO_STREAM("Planning for agent " << i << " ...");

    Environment env(dimx, dimy, obstacles_, goals[i]);
    sipp_t sipp(env);

    for (const auto &collisionIntervals : allCollisionIntervals) {
      sipp.setCollisionIntervals(collisionIntervals.first,
                                 collisionIntervals.second);
    }
    // Plan
    PlanResult<State, Action, int> &solution = solutions[i];
    success = sipp.search(startStates[i], Action::Wait, solution, DBL_MAX);

    if (success) {
      // update collision intervals
      auto lastState = solution.states[0];
      for (size_t i = 1; i < solution.states.size(); ++i) {
        if (solution.states[i].first != lastState.first) {
          allCollisionIntervals[lastState.first].push_back(sipp_t::interval(
              lastState.second, solution.states[i].second - 1));
          lastState = solution.states[i];
        }
      }
      allCollisionIntervals[solution.states.back().first].push_back(
          sipp_t::interval(solution.states.back().second,
                           std::numeric_limits<int>::max()));
    } else {
      ROS_ERROR("Planning NOT successful!");
    }
  }

  timer.stop();
  // if (timer.elapsedSeconds() > time_tolerance) {
  //   ROS_ERROR("Planning time out! Cur time tolerance is %lf",
  //   time_tolerance); return false;
  // }

  if (success) {
    cost = 0;
    generatePlan(solutions, goal, plan, cost);

    ROS_DEBUG_STREAM("Planning successful!");
    ROS_DEBUG_STREAM("runtime: " << timer.elapsedSeconds());
    ROS_DEBUG_STREAM("cost: " << cost);
    ROS_DEBUG_STREAM("makespan(involve start & end): " << plan.makespan);
  }

  return success;
}

void SIPPROS::generatePlan(
    const std::vector<PlanResult<State, Action, int>> &solution,
    const nav_msgs::Path &goal, mapf_msgs::GlobalPlan &plan, double &cost) {
  int &makespan = plan.makespan;
  for (const auto &s : solution) {
    cost += s.cost;
    makespan = std::max<int>(makespan, s.cost);
  }
  // add start point (the fisrt step is to get the center of the first grid)
  makespan += 1;

  plan.global_plan.resize(solution.size());

  for (size_t i = 0; i < solution.size(); ++i) {
    // create a message for the plan
    mapf_msgs::SinglePlan &single_plan = plan.global_plan[i];
    nav_msgs::Path &single_path = single_plan.plan;
    single_path.header.frame_id = global_frame_;
    single_path.header.stamp = ros::Time::now();

    for (const auto &state : solution[i].states) {
      geometry_msgs::PoseStamped cur_pose;
      cur_pose.header.frame_id = single_path.header.frame_id;
      cur_pose.pose.orientation.w = 1;
      mapToWorld(state.first.x, state.first.y, cur_pose.pose.position.x,
                 cur_pose.pose.position.y);
      single_path.poses.push_back(cur_pose);
      single_plan.time_step.push_back(state.second);
    }

    // replace end point with goal point
    single_path.poses.back() = goal.poses[i];

  } // end solution for
}

void SIPPROS::worldToMap(const double &wx, const double &wy, unsigned int &mx,
                         unsigned int &my) {
  if (!costmap_->worldToMap(wx, wy, mx, my)) {
    ROS_WARN("The robot's start position is off the global costmap. "
             "Planning will "
             "always fail, are you sure the robot has been properly "
             "localized?");
  }
}

void SIPPROS::mapToWorld(const unsigned int &mx, const unsigned int &my,
                         double &wx, double &wy) {
  costmap_->mapToWorld(mx, my, wx, wy);
}

void SIPPROS::clearCell(const unsigned int &mx, const unsigned int &my) {
  // costmap_->setCost(mx, my, costmap_2d::FREE_SPACE);
  if (obstacles_.find(State(mx, my)) != obstacles_.end()) {
    obstacles_.erase(State(mx, my));
  }
}

bool SIPPROS::checkIsObstacle(const unsigned int &mx, const unsigned int &my) {
  return (costmap_->getCost(mx, my) >= costmap_2d::INSCRIBED_INFLATED_OBSTACLE);
}

bool SIPPROS::checkSurroundObstacle(const unsigned int &mx,
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

SIPPROS::~SIPPROS() {
  update_obstacle_thread_->interrupt();
  update_obstacle_thread_->join();
  delete update_obstacle_thread_;

  costmap_ = nullptr;

  ROS_INFO("Exit SIPP planner.");
}

} // namespace mapf