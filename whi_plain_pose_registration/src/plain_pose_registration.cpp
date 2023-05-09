/******************************************************************
plain motion planning class for pose registration logic plugins

Features:
- plain motion planning for pose registration
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

******************************************************************/
#include "whi_plain_pose_registration/plain_pose_registration.h"

#include <pluginlib/class_list_macros.h>

namespace pose_registration_plugins
{
    PlainPoseRegistration::PlainPoseRegistration()
    {

    }

    bool PlainPoseRegistration::computeVelocityCommands(geometry_msgs::Twist& CmdVel)
    {

    }

    PLUGINLIB_EXPORT_CLASS(pose_registration_plugins::PlainPoseRegistration, whi_pose_registration::BasePoseRegistration)
} // namespace pose_registration_plugins
