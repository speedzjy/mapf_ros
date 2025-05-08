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
#include "nav2_util/lifecycle_utils.hpp"

#include "mapf_base/mapf_base.hpp"

namespace mapf {
MAPFBase::MAPFBase(const rclcpp::NodeOptions &options)
    : nav2_util::LifecycleNode("mapf_base_node", "", options),
      mapf_loader_("mapf_ros", "mapf::MAPFROS"), receive_mapf_goal_(false), run_mapf_(false) {
  RCLCPP_INFO(get_logger(), "Creating");

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  getParam();

  costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
      "mapf_costmap", std::string{get_namespace()}, "mapf_costmap");
}

MAPFBase::~MAPFBase() {
  costmap_thread_.reset();

  // delete the thread
  do_mapf_thread_->interrupt();
  do_mapf_thread_->join();
  delete do_mapf_thread_;
  state_machine_thread_->interrupt();
  state_machine_thread_->join();
  delete state_machine_thread_;
}

nav2_util::CallbackReturn MAPFBase::on_configure(const rclcpp_lifecycle::State & /*state*/) {
  RCLCPP_INFO(get_logger(), "Configuring");

  costmap_ros_->configure();

  // Launch a thread to run the costmap node
  costmap_thread_ = std::make_unique<nav2_util::NodeThread>(costmap_ros_);

  pub_gui_plan_.resize(agent_num_);
  for (int i = 0; i < agent_num_; ++i) {
    pub_gui_plan_[i] = create_publisher<nav_msgs::msg::Path>(plan_topic_[i], 1);
  }

  // goal in mapf form
  sub_mapf_goal_ = create_subscription<mapf_msgs::msg::Goal>(
      "mapf_goal", 1, std::bind(&MAPFBase::goalCallback, this, std::placeholders::_1));
  pub_mapf_global_plan_ = create_publisher<mapf_msgs::msg::GlobalPlan>("global_plan", 1);

  // 打印所有已加载的参数名称和值
  auto param_names = this->list_parameters({}, 10).names;

  if (param_names.empty()) {
    RCLCPP_WARN(this->get_logger(), "No parameters found.");
  } else {
    for (const auto &param_name : param_names) {
      rclcpp::Parameter param;
      this->get_parameter(param_name, param);
      RCLCPP_INFO(this->get_logger(), "Parameter: %s = %s", param_name.c_str(),
                  param.value_to_string().c_str());
    }
  }

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn MAPFBase::on_activate(const rclcpp_lifecycle::State & /*state*/) {
  RCLCPP_INFO(get_logger(), "Activating");

  for (int i = 0; i < agent_num_; ++i) {
    pub_gui_plan_[i]->on_activate();
  }
  pub_mapf_global_plan_->on_activate();

  costmap_ros_->activate();

  do_mapf_thread_ = new boost::thread(boost::bind(&MAPFBase::doMAPFThread, this));
  state_machine_thread_ = new boost::thread(boost::bind(&MAPFBase::stateMachine, this));

  // create a local planner
  try {
    mapf_planner_ = mapf_loader_.createUniqueInstance(planner_name_);
    RCLCPP_INFO(this->get_logger(), "Created local_planner %s", planner_name_.c_str());
    mapf_planner_->initialize(mapf_loader_.getName(planner_name_), costmap_ros_,
                              shared_from_this());
  } catch (const pluginlib::PluginlibException &ex) {
    RCLCPP_FATAL(this->get_logger(),
                 "Failed to create the %s planner, are you sure it is properly "
                 "registered and that the containing library is built? Exception: %s",
                 planner_name_.c_str(), ex.what());
    return nav2_util::CallbackReturn::FAILURE;
  }

  createBond();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn MAPFBase::on_deactivate(const rclcpp_lifecycle::State & /*state*/) {
  RCLCPP_INFO(get_logger(), "Deactivating");

  for (int i = 0; i < agent_num_; ++i) {
    pub_gui_plan_[i]->on_deactivate();
  }
  pub_mapf_global_plan_->on_deactivate();

  costmap_ros_->deactivate();

  // destroy bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn MAPFBase::on_cleanup(const rclcpp_lifecycle::State & /*state*/) {
  RCLCPP_INFO(get_logger(), "Cleaning up");

  // delete the planner
  mapf_planner_.reset();

  for (int i = 0; i < agent_num_; ++i) {
    pub_gui_plan_[i].reset();
  }
  pub_mapf_global_plan_.reset();

  tf_.reset();

  costmap_ros_->cleanup();
  costmap_thread_.reset();
  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn MAPFBase::on_shutdown(const rclcpp_lifecycle::State &) {
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

void MAPFBase::getParam() {
  this->declare_parameter<int>("agent_num", 1);
  this->get_parameter("agent_num", agent_num_);

  this->declare_parameter<std::string>("mapf_planner", "mapf_planner/CBSROS");
  this->get_parameter("mapf_planner", planner_name_);

  this->declare_parameter<double>("planner_time_tolerance", DBL_MAX);
  this->get_parameter("planner_time_tolerance", planner_time_tolerance_);

  this->declare_parameter<double>("goal_tolerance", 1.0);
  this->get_parameter("goal_tolerance", goal_tolerance_);

  this->declare_parameter<std::string>("global_frame_id", "map");
  this->get_parameter("global_frame_id", global_frame_id_);

  base_frame_id_.resize(agent_num_);
  plan_topic_.resize(agent_num_);

  for (int i = 0; i < agent_num_; ++i) {
    std::string base_frame_param = "base_frame_id.agent_" + std::to_string(i);
    std::string plan_topic_param = "plan_topic.agent_" + std::to_string(i);

    this->declare_parameter<std::string>(base_frame_param, "base_link");
    this->get_parameter(base_frame_param, base_frame_id_[i]);

    this->declare_parameter<std::string>(plan_topic_param, "plan");
    this->get_parameter(plan_topic_param, plan_topic_[i]);
  }
}

void MAPFBase::goalCallback(const mapf_msgs::msg::Goal::SharedPtr goal) {
  std::lock_guard<std::mutex> lock(mtx_mapf_goal_);
  goal_ros_ = goal->goal;
  goal_ros_.header.frame_id = global_frame_id_;
  goal_ros_.header.stamp = this->get_clock()->now();
  receive_mapf_goal_ = true;
  RCLCPP_INFO(this->get_logger(), "Receive mapf goal...");
}

nav_msgs::msg::Path MAPFBase::getRobotPose() {
  nav_msgs::msg::Path start;
  start.header.frame_id = global_frame_id_;
  start.header.stamp = this->get_clock()->now();
  start.poses.clear();
  start.poses.resize(agent_num_);

  for (int i = 0; i < agent_num_; ++i) {
    // get tf
    try {
      // 准备输入 pose
      geometry_msgs::msg::PoseStamped robot_pose;
      tf2::toMsg(tf2::Transform::getIdentity(), robot_pose.pose);
      robot_pose.header.frame_id = base_frame_id_[i];
      robot_pose.header.stamp = rclcpp::Time(0);

      // 初始化输出 pose
      tf2::toMsg(tf2::Transform::getIdentity(), start.poses[i].pose);
      tf_buffer_->transform(robot_pose, start.poses[i], global_frame_id_);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_ERROR(get_logger(), "Failed to transform pose for agent %d: %s", i, ex.what());
    }
  }
  return start;
}

bool MAPFBase::reachGoal() {
  bool reach = true;
  nav_msgs::msg::Path start = getRobotPose();
  for (int i = 0; i < start.poses.size(); ++i) {
    double diff_x = start.poses[i].pose.position.x - goal_ros_.poses[i].pose.position.x,
           diff_y = start.poses[i].pose.position.y - goal_ros_.poses[i].pose.position.y;
    reach &= (abs(diff_x) < goal_tolerance_ && abs(diff_y) < goal_tolerance_);
  }
  return reach;
}

void MAPFBase::stateMachine() {
  RCLCPP_INFO(this->get_logger(), "MAPF state machine Thread...");
  rclcpp::Rate loop_rate(5);

  try {
    while (rclcpp::ok()) {
      std::unique_lock<std::mutex> lock(mtx_mapf_goal_);
      std::unique_lock<std::mutex> lock_planner(mtx_planner_);

      if (receive_mapf_goal_) {
        receive_mapf_goal_ = false;
        run_mapf_ = true;
      }

      if (run_mapf_) {
        if (reachGoal()) {
          run_mapf_ = false;
        }
      }

      lock.unlock();
      lock_planner.unlock();
      loop_rate.sleep();
      boost::this_thread::interruption_point();
    } // end while
  } catch (...) {
    RCLCPP_INFO(this->get_logger(), "Exit State Machine thread.");
  }
}

void MAPFBase::doMAPFThread() {
  RCLCPP_INFO(this->get_logger(), "MAPF thread: Start active mapf algorithm...");
  rclcpp::Rate loop_rate(10);

  try {
    while (rclcpp::ok()) {
      std::unique_lock<std::mutex> lock_planner(mtx_planner_);
      bool run_mapf = run_mapf_;
      lock_planner.unlock();
      if (run_mapf) {
        nav_msgs::msg::Path start_ros = getRobotPose();
        double cost = 0;
        mapf_msgs::msg::GlobalPlan plan;
        if (mapf_planner_->makePlan(start_ros, goal_ros_, plan, cost, planner_time_tolerance_)) {
          publishPlan(plan);
        }
      }
      loop_rate.sleep();
      boost::this_thread::interruption_point();
    } // end while
  } catch (...) {
    RCLCPP_INFO(this->get_logger(), "Exit Do mapf thread.");
  }
}

void MAPFBase::publishPlan(const mapf_msgs::msg::GlobalPlan &plan) {
  for (size_t i = 0; i < plan.global_plan.size(); ++i) {
    pub_gui_plan_[i]->publish(plan.global_plan[i].plan);
  }
  pub_mapf_global_plan_->publish(plan);
}

} // namespace mapf
