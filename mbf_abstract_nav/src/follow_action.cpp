/*
 *  Copyright 2018, Magazino GmbH, Sebastian Pütz, Jorge Santos Simón
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *  1. Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *
 *  3. Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  follow_action.cpp
 *
 *  authors:
 *    Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *    Jorge Santos Simón <santos@magazino.eu>
 *
 */

#include "mbf_abstract_nav/follow_action.h"

namespace mbf_abstract_nav
{

FollowAction::FollowAction(
    const std::string &action_name,
    const mbf_utility::RobotInformation &robot_info)
    : AbstractActionBase(action_name, robot_info)
{
  ROS_INFO_STREAM_NAMED("follow_action", "Run constructor");
}

void FollowAction::start(
    GoalHandle &goal_handle,
    typename AbstractFollowExecution::Ptr execution_ptr
)
{
  ROS_INFO_NAMED("follow_path", "FollowAction start");
  if(goal_handle.getGoalStatus().status == actionlib_msgs::GoalStatus::RECALLING)
  {
    ROS_INFO_NAMED("follow_path", "Goal is being recalled");
    goal_handle.setCanceled();
    return;
  }

  uint8_t slot = goal_handle.getGoal()->concurrency_slot;

  bool update_plan = false;
  ROS_INFO_STREAM_NAMED("follow_path", "update_plan: " << update_plan);
  slot_map_mtx_.lock();
  std::map<uint8_t, ConcurrencySlot>::iterator slot_it = concurrency_slots_.find(slot);
  if(slot_it != concurrency_slots_.end() && slot_it->second.in_use)
  {
    ROS_INFO_NAMED("follow_path", "slot_it != concurrency_slots_.end() && slot_it->second.in_use");
    boost::lock_guard<boost::mutex> goal_guard(goal_mtx_);
    if ((slot_it->second.execution->getName() == goal_handle.getGoal()->controller ||
         goal_handle.getGoal()->controller.empty()) &&
        slot_it->second.goal_handle.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)
    {
      ROS_INFO_STREAM_NAMED(name_, "Updating running controller goal of slot " << static_cast<int>(slot));
      update_plan = true;
      // Goal requests to run the same controller on the same concurrency slot already in use:
      // we update the goal handle and pass the new plan and tolerances from the action to the
      // execution without stopping it
      execution_ptr = slot_it->second.execution;
      ROS_INFO_NAMED("follow_path", "execution_ptr->setNewPlan");
      execution_ptr->setNewPlan(goal_handle.getGoal()->tolerance_from_action,
                                goal_handle.getGoal()->dist_tolerance,
                                goal_handle.getGoal()->angle_tolerance);
      // Update also goal pose, so the feedback remains consistent
      // TODO: do this somewhere else?
      //goal_pose_ = goal_handle.getGoal()->path.poses.back();
      mbf_msgs::FollowPathResult result;
      fillFollowPathResult(mbf_msgs::FollowPathResult::CANCELED, "Goal preempted by a new plan", result);
      concurrency_slots_[slot].goal_handle.setCanceled(result, result.message);
      ROS_INFO_NAMED("follow_path", "Old goal is cancelled");
      concurrency_slots_[slot].goal_handle = goal_handle;
      concurrency_slots_[slot].goal_handle.setAccepted();
      ROS_INFO_NAMED("follow_path", "New goal is accepted");
    }
  }
  slot_map_mtx_.unlock();
  if(!update_plan)
  {
    // Otherwise run parent version of this method
    AbstractActionBase::start(goal_handle, execution_ptr);
  }
}

void FollowAction::runImpl(GoalHandle &goal_handle, AbstractFollowExecution &execution)
{
  ROS_INFO_NAMED("follow_path", "runImpl");
  goal_mtx_.lock();
  // Note that we always use the goal handle stored on the concurrency slots map, as it can change when replanning
  uint8_t slot = goal_handle.getGoal()->concurrency_slot;
  goal_mtx_.unlock();

  ROS_DEBUG_STREAM_NAMED(name_, "Start action "  << name_);

  // ensure we don't provide values from previous execution on case of error before filling both poses
  goal_pose_ = geometry_msgs::PoseStamped();
  robot_pose_ = geometry_msgs::PoseStamped();

  ros::NodeHandle private_nh("~");

  double oscillation_timeout_tmp;
  private_nh.param("oscillation_timeout", oscillation_timeout_tmp, 0.0);
  ros::Duration oscillation_timeout(oscillation_timeout_tmp);

  double oscillation_distance;
  private_nh.param("oscillation_distance", oscillation_distance, 0.03);

  mbf_msgs::FollowPathResult result;
  mbf_msgs::FollowPathFeedback feedback;

  typename AbstractFollowExecution::ControllerState state_moving_input;
  bool controller_active = true;

  goal_mtx_.lock();
  const mbf_msgs::FollowPathGoal &goal = *(goal_handle.getGoal().get());

  // const std::vector<geometry_msgs::PoseStamped> &plan = goal.path.poses;
  // if (plan.empty())
  // {
  //   fillFollowPathResult(mbf_msgs::FollowPathResult::INVALID_PATH, "Controller started with an empty plan!", result);
  //   goal_handle.setAborted(result, result.message);
  //   ROS_ERROR_STREAM_NAMED(name_, result.message << " Canceling the action call.");
  //   controller_active = false;
  //   goal_mtx_.unlock();
  //   return;
  // }

  // goal_pose_ = plan.back();
  // ROS_DEBUG_STREAM_NAMED(name_, "Called action \""
  //     << name_ << "\" with plan:" << std::endl
  //     << "frame: \"" << goal.path.header.frame_id << "\" " << std::endl
  //     << "stamp: " << goal.path.header.stamp << std::endl
  //     << "poses: " << goal.path.poses.size() << std::endl
  //     << "goal: (" << goal_pose_.pose.position.x << ", "
  //     << goal_pose_.pose.position.y << ", "
  //     << goal_pose_.pose.position.z << ")");

  goal_mtx_.unlock();


  geometry_msgs::PoseStamped oscillation_pose;
  ros::Time last_oscillation_reset = ros::Time::now();

  bool first_cycle = true;

  while (controller_active && ros::ok())
  {
    // goal_handle could change between the loop cycles due to adapting the plan
    // with a new goal received for the same concurrency slot
    if (!robot_info_.getRobotPose(robot_pose_))
    {
      controller_active = false;
      fillFollowPathResult(mbf_msgs::FollowPathResult::TF_ERROR, "Could not get the robot pose!", result);
      goal_mtx_.lock();
      goal_handle.setAborted(result, result.message);
      goal_mtx_.unlock();
      ROS_ERROR_STREAM_NAMED(name_, result.message << " Canceling the action call.");
      break;
    }

    if (first_cycle)
    {
      // init oscillation pose
      oscillation_pose = robot_pose_;
    }

    goal_mtx_.lock();
    state_moving_input = execution.getState();

  //   switch (state_moving_input)
  //   {
  //     case AbstractFollowExecution::INITIALIZED:
  //       execution.setNewPlan(plan, goal.tolerance_from_action, goal.dist_tolerance, goal.angle_tolerance);
  //       execution.start();
  //       break;

  //     case AbstractFollowExecution::STOPPED:
  //       ROS_WARN_STREAM_NAMED(name_, "The controller has been stopped rigorously!");
  //       controller_active = false;
  //       result.outcome = mbf_msgs::FollowPathResult::STOPPED;
  //       result.message = "Controller has been stopped!";
  //       goal_handle.setAborted(result, result.message);
  //       break;

  //     case AbstractFollowExecution::CANCELED:
  //       ROS_INFO_STREAM("Action \"exe_path\" canceled");
  //       fillFollowPathResult(mbf_msgs::FollowPathResult::CANCELED, "Controller canceled", result);
  //       goal_handle.setCanceled(result, result.message);
  //       controller_active = false;
  //       break;

  //     case AbstractFollowExecution::STARTED:
  //       ROS_DEBUG_STREAM_NAMED(name_, "The moving has been started!");
  //       break;

  //     case AbstractFollowExecution::PLANNING:
  //       if (execution.isPatienceExceeded())
  //       {
  //         ROS_INFO_STREAM("Try to cancel the plugin \"" << name_ << "\" after the patience time has been exceeded!");
  //         if (execution.cancel())
  //         {
  //           ROS_INFO_STREAM("Successfully canceled the plugin \"" << name_ << "\" after the patience time has been exceeded!");
  //         }
  //       }
  //       break;

  //     case AbstractFollowExecution::MAX_RETRIES:
  //       ROS_WARN_STREAM_NAMED(name_, "The controller has been aborted after it exceeded the maximum number of retries!");
  //       controller_active = false;
  //       fillFollowPathResult(execution.getOutcome(), execution.getMessage(), result);
  //       goal_handle.setAborted(result, result.message);
  //       break;

  //     case AbstractFollowExecution::PAT_EXCEEDED:
  //       ROS_WARN_STREAM_NAMED(name_, "The controller has been aborted after it exceeded the patience time");
  //       controller_active = false;
  //       fillFollowPathResult(mbf_msgs::FollowPathResult::PAT_EXCEEDED, execution.getMessage(), result);
  //       goal_handle.setAborted(result, result.message);
  //       break;

  //     case AbstractFollowExecution::NO_PLAN:
  //       ROS_WARN_STREAM_NAMED(name_, "The controller has been started without a plan!");
  //       controller_active = false;
  //       fillFollowPathResult(mbf_msgs::FollowPathResult::INVALID_PATH, "Controller started without a path", result);
  //       goal_handle.setAborted(result, result.message);
  //       break;

  //     case AbstractFollowExecution::EMPTY_PLAN:
  //       ROS_WARN_STREAM_NAMED(name_, "The controller has received an empty plan");
  //       controller_active = false;
  //       fillFollowPathResult(mbf_msgs::FollowPathResult::INVALID_PATH, "Controller started with an empty plan", result);
  //       goal_handle.setAborted(result, result.message);
  //       break;

  //     case AbstractFollowExecution::INVALID_PLAN:
  //       ROS_WARN_STREAM_NAMED(name_, "The controller has received an invalid plan");
  //       controller_active = false;
  //       fillFollowPathResult(mbf_msgs::FollowPathResult::INVALID_PATH, "Controller started with an invalid plan", result);
  //       goal_handle.setAborted(result, result.message);
  //       break;

  //     case AbstractFollowExecution::NO_LOCAL_CMD:
  //       ROS_WARN_STREAM_THROTTLE_NAMED(3, name_, "No velocity command received from controller! "
  //           << execution.getMessage());
  //       controller_active = execution.isMoving();
  //       if (!controller_active)
  //       {
  //         fillFollowPathResult(execution.getOutcome(), execution.getMessage(), result);
  //         goal_handle.setAborted(result, result.message);
  //       }
  //       else
  //       {
  //         publishFollowPathFeedback(goal_handle, execution.getOutcome(), execution.getMessage(),
  //                                execution.getVelocityCmd());
  //       }
  //       break;

  //     case AbstractFollowExecution::GOT_LOCAL_CMD:
  //       if (!oscillation_timeout.isZero())
  //       {
  //         // check if oscillating
  //         if (mbf_utility::distance(robot_pose_, oscillation_pose) >= oscillation_distance)
  //         {
  //           last_oscillation_reset = ros::Time::now();
  //           oscillation_pose = robot_pose_;
  //         }
  //         else if (last_oscillation_reset + oscillation_timeout < ros::Time::now())
  //         {
  //           ROS_WARN_STREAM_NAMED(name_, "The controller is oscillating for "
  //               << (ros::Time::now() - last_oscillation_reset).toSec() << "s");

  //           execution.cancel();
  //           controller_active = false;
  //           fillFollowPathResult(mbf_msgs::FollowPathResult::OSCILLATION, "Oscillation detected!", result);
  //           goal_handle.setAborted(result, result.message);
  //           break;
  //         }
  //       }
  //       publishFollowPathFeedback(goal_handle, execution.getOutcome(), execution.getMessage(), execution.getVelocityCmd());
  //       break;

  //     case AbstractFollowExecution::ARRIVED_GOAL:
  //       ROS_DEBUG_STREAM_NAMED(name_, "Controller succeeded; arrived at goal");
  //       controller_active = false;
  //       fillFollowPathResult(mbf_msgs::FollowPathResult::SUCCESS, "Controller succeeded; arrived at goal!", result);
  //       goal_handle.setSucceeded(result, result.message);
  //       break;

  //     case AbstractFollowExecution::INTERNAL_ERROR:
  //       ROS_FATAL_STREAM_NAMED(name_, "Internal error: Unknown error thrown by the plugin: " << execution.getMessage());
  //       controller_active = false;
  //       fillFollowPathResult(mbf_msgs::FollowPathResult::INTERNAL_ERROR, "Internal error: Unknown error thrown by the plugin!", result);
  //       goal_handle.setAborted(result, result.message);
  //       break;

  //     default:
  //       std::stringstream ss;
  //       ss << "Internal error: Unknown state in a move base flex controller execution with the number: "
  //          << static_cast<int>(state_moving_input);
  //       fillFollowPathResult(mbf_msgs::FollowPathResult::INTERNAL_ERROR, ss.str(), result);
  //       ROS_FATAL_STREAM_NAMED(name_, result.message);
  //       goal_handle.setAborted(result, result.message);
  //       controller_active = false;
  //   }
    goal_mtx_.unlock();

    if (controller_active)
    {
      // try to sleep a bit
      // normally this thread should be woken up from the controller execution thread
      // in order to transfer the results to the controller
      execution.waitForStateUpdate(boost::chrono::milliseconds(500));
    }

    first_cycle = false;
  }  // while (controller_active && ros::ok())

  if (!controller_active)
  {
    ROS_DEBUG_STREAM_NAMED(name_, "\"" << name_ << "\" action ended properly.");
  }
  else
  {
    // normal on continuous replanning
    ROS_DEBUG_STREAM_NAMED(name_, "\"" << name_ << "\" action has been stopped!");
  }
}

void FollowAction::publishFollowPathFeedback(
        GoalHandle &goal_handle,
        uint32_t outcome, const std::string &message,
        const geometry_msgs::TwistStamped &current_twist)
{
  mbf_msgs::FollowPathFeedback feedback;
  feedback.outcome = outcome;
  feedback.message = message;

  feedback.last_cmd_vel = current_twist;
  if (feedback.last_cmd_vel.header.stamp.isZero())
    feedback.last_cmd_vel.header.stamp = ros::Time::now();

  feedback.current_pose = robot_pose_;
  feedback.dist_to_goal = static_cast<float>(mbf_utility::distance(robot_pose_, goal_pose_));
  feedback.angle_to_goal = static_cast<float>(mbf_utility::angle(robot_pose_, goal_pose_));
  goal_handle.publishFeedback(feedback);
}

void FollowAction::fillFollowPathResult(
        uint32_t outcome, const std::string &message,
        mbf_msgs::FollowPathResult &result)
{
  result.outcome = outcome;
  result.message = message;
  result.final_pose = robot_pose_;
  result.dist_to_goal = static_cast<float>(mbf_utility::distance(robot_pose_, goal_pose_));
  result.angle_to_goal = static_cast<float>(mbf_utility::angle(robot_pose_, goal_pose_));
}

} /* mbf_abstract_nav */
