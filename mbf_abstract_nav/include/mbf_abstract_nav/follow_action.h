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
 *  follow_action.h
 *
 *  authors:
 *    Sebastian Pütz <spuetz@uni-osnabrueck.de>
 *    Jorge Santos Simón <santos@magazino.eu>
 *
 */

#ifndef MBF_ABSTRACT_NAV__FOLLOW_ACTION_H_
#define MBF_ABSTRACT_NAV__FOLLOW_ACTION_H_

#include <actionlib/server/action_server.h>

#include <mbf_msgs/FollowPathAction.h>
#include <mbf_utility/robot_information.h>

#include "mbf_abstract_nav/abstract_action_base.hpp"
#include "mbf_abstract_nav/abstract_follow_execution.h"

namespace mbf_abstract_nav
{

class FollowAction :
    public AbstractActionBase<mbf_msgs::FollowPathAction, AbstractFollowExecution>
{
 public:

  typedef boost::shared_ptr<FollowAction> Ptr;

  FollowAction(const std::string &name,
                   const mbf_utility::RobotInformation &robot_info);

  /**
   * @brief Start controller action.
   * Override abstract action version to allow updating current plan without stopping execution.
   * @param goal_handle Reference to the goal handle received on action execution callback.
   * @param execution_ptr Pointer to the execution descriptor.
   */
  void start(
      GoalHandle &goal_handle,
      typename AbstractFollowExecution::Ptr execution_ptr
  );

  void runImpl(GoalHandle &goal_handle, AbstractFollowExecution& execution);

protected:
  void publishFollowPathFeedback(
          GoalHandle &goal_handle,
          uint32_t outcome, const std::string &message,
          const geometry_msgs::TwistStamped &current_twist);

  /**
   * @brief Utility method to fill the FollowPath action result in a single line
   * @param outcome FollowPath action outcome
   * @param message FollowPath action message
   * @param result The action result to fill
   */
  void fillFollowPathResult(
        uint32_t outcome, const std::string &message,
        mbf_msgs::FollowPathResult &result);

  boost::mutex goal_mtx_; ///< lock goal handle for updating it while running
  geometry_msgs::PoseStamped robot_pose_; ///< Current robot pose
  geometry_msgs::PoseStamped goal_pose_;  ///< Current goal pose

};
}



#endif //MBF_ABSTRACT_NAV__FOLLOW_ACTION_H_