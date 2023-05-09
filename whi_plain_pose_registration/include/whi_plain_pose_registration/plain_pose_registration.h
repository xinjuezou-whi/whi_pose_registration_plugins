/******************************************************************
plain motion planning class for pose registration logic plugins

Features:
- plain motion planning for pose registration
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2023-05-09: Initial version
2022-xx-xx: xxx
******************************************************************/
#pragma once
#include <whi_pose_registration/base_pose_registration.h>

namespace pose_registration_plugins
{
    class PlainPoseRegistration : public whi_pose_registration::BasePoseRegistration
    {
    public:
        PlainPoseRegistration();
        virtual ~PlainPoseRegistration() = default;

    public:
        bool computeVelocityCommands(geometry_msgs::Twist& CmdVel) override;
    };
} // namespace pose_registration_plugins
