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

#include "ros/package.h"
#include "ros/ros.h"

#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"

#include "std_msgs/Bool.h"
#include "tf/tf.h"

#include "actionlib/client/simple_action_client.h"
#include "move_base_msgs/MoveBaseAction.h"

#include "mapf_msgs/GlobalPlan.h"
#include "mapf_msgs/SinglePlan.h"

#include "mapf_ros/utils/utility.hpp"

class ParamServer {
public:
  ros::NodeHandle ps_nh_;

  int agent_num_;
  std::vector<std::string> agent_name_;

  std::string global_frame_id_;
  std::vector<std::string> base_frame_id_;
  std::vector<std::string> plan_topic_;

  ParamServer() {
    ps_nh_.param<int>("agent_num", agent_num_, 1);
    ps_nh_.param<std::string>("global_frame_id", global_frame_id_, "map");

    agent_name_.resize(agent_num_);
    base_frame_id_.resize(agent_num_);
    plan_topic_.resize(agent_num_);

    for (int i = 0; i < agent_num_; ++i) {
      ps_nh_.param<std::string>("base_frame_id/agent_" + std::to_string(i),
                                base_frame_id_[i], "base_link");
      ps_nh_.param<std::string>("plan_topic/agent_" + std::to_string(i),
                                plan_topic_[i], "plan");
      ps_nh_.param<std::string>("agent_name/agent_" + std::to_string(i),
                                agent_name_[i], "agent_name_0");
    }
  }
};

class PlanExecutor : public ParamServer {
private:
  std::mutex plan_mtx_;

  ros::NodeHandle nh_;
  ros::Subscriber sub_mapf_plan_;

  int make_span_;
  std::vector<mapf_msgs::SinglePlan> plan_arr_;

  bool get_plan_;

  boost::thread *planner_thread_;

  typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>
      MoveBaseActionClient;
  typedef std::shared_ptr<MoveBaseActionClient> MoveBaseActionClientPtr;
  std::vector<MoveBaseActionClientPtr> ac_ptr_arr_;

public:
  PlanExecutor()
      : ParamServer(), make_span_(0), get_plan_(false),
        ac_ptr_arr_(agent_num_, nullptr) {

    plan_arr_.resize(agent_num_);
    ac_ptr_arr_.resize(agent_num_);

    ros::NodeHandle pub_nh("/");
    for (int i = 0; i < agent_num_; ++i) {
      ac_ptr_arr_[i] = std::make_shared<MoveBaseActionClient>(
          "/" + agent_name_[i] + "/move_base", true);
    }

    sub_mapf_plan_ = nh_.subscribe<mapf_msgs::GlobalPlan>(
        "global_plan", 1, &PlanExecutor::planCallback, this);

    planner_thread_ =
        new boost::thread(boost::bind(&PlanExecutor::mbStateThread, this));
  }

  ~PlanExecutor() {}

  void mbStateThread() {
    ROS_INFO_NAMED("mapf_plan_thread", "Plan and Read move base state...");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);
    while (n.ok()) {
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
              ac_ptr_arr_[j]->sendGoal(
                  getMBGoalFromGeoPose(plan_arr_[j].plan.poses[i]));
            }
          } // end for

          // check if get new plan
          lock.unlock();
          loop_rate.sleep();
          lock.lock();
          // exit this cycle, ececute new plan
          if (get_plan_) {
            ROS_INFO(YELLOW "Plan changed, execute new plan.." NONE);
            for (int j = 0; j < plan_arr_.size(); ++j) {
              if (i < plan_arr_[j].time_step.size()) {
                ac_ptr_arr_[j]->cancelGoal();
              }
            } // end for
            break;
          }

          // wait for reach step goal
          for (int j = 0; j < plan_arr_.size(); ++j) {
            if (i < plan_arr_[j].time_step.size()) {
              while (ac_ptr_arr_[j]->getState() !=
                     actionlib::SimpleClientGoalState::SUCCEEDED) {
                // check if get new plan
                lock.unlock();
                loop_rate.sleep();
                lock.lock();
                if (get_plan_) {
                  // ROS_INFO(YELLOW "Plan changed, execute new plan.." NONE);
                  ac_ptr_arr_[j]->cancelGoal();
                  break;
                }
                ac_ptr_arr_[j]->waitForResult(ros::Duration(1, 0));
              }
              if (i == plan_arr_[j].time_step.size() - 1) {
                ROS_INFO("Agent %d reached %dth step goal(" GREEN "END" NONE
                         ")!",
                         j, i);
              } else {
                ROS_INFO("Agent %d reached %dth step goal!", j, i);
              }
            }
          } // end for
        }   // end loop time step
      }
    } // end while
  }

  void planCallback(const mapf_msgs::GlobalPlan::ConstPtr &mapf_global_plan) {
    if (!equal(plan_arr_, mapf_global_plan->global_plan)) {
      std::lock_guard<std::mutex> lock(plan_mtx_);

      get_plan_ = true;
      make_span_ = mapf_global_plan->makespan;
      for (int i = 0; i < agent_num_; ++i) {
        // if
        plan_arr_[i] = mapf_global_plan->global_plan[i];
      }
      ROS_INFO(GREEN "Get New plan.." NONE);
      ROS_INFO(GREEN "MakeSpan: %d" NONE, make_span_);
      ROS_INFO(GREEN "AgentNum: %d" NONE, agent_num_);
    }
  }

  bool equal(const mapf_msgs::SinglePlan &a, const mapf_msgs::SinglePlan &b) {
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

  bool equal(const std::vector<mapf_msgs::SinglePlan> &a,
             const std::vector<mapf_msgs::SinglePlan> &b) {
    if (a.size() != b.size()) {
      return false;
    }
    bool res = true;
    for (int i = 0; i < a.size(); ++i) {
      res &= equal(a[i], b[i]);
    }
    return res;
  }

  move_base_msgs::MoveBaseGoal
  getMBGoalFromGeoPose(const geometry_msgs::PoseStamped &curr_location) {
    move_base_msgs::MoveBaseGoal tmp_goal;
    tmp_goal.target_pose.header.frame_id = "map";
    tmp_goal.target_pose.header.stamp = ros::Time::now();
    tmp_goal.target_pose.pose = curr_location.pose;
    return tmp_goal;
  }
};

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "plan_executor_node");

  PlanExecutor PE;

  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();

  return 0;
}