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
#include <ros/ros.h> 
#include <whi_pose_registration/base_pose_registration.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>

#include <memory>

namespace pose_registration_plugins
{
    class PlainPoseRegistration : public whi_pose_registration::BasePoseRegistration
    {
    public:
        PlainPoseRegistration();
        virtual ~PlainPoseRegistration() = default;

    public:
        void initialize(const std::string& LaserTopic) override;
        bool computeVelocityCommands(geometry_msgs::Twist& CmdVel) override;

    private:
        void subCallbackLaserScan(const sensor_msgs::LaserScan::ConstPtr& Laser);

    private:
        std::unique_ptr<laser_geometry::LaserProjection> cloud_projector_{ nullptr };
#ifndef DEBUG
        std::unique_ptr<ros::Publisher> pub_point_cloud_{ nullptr };
        std::unique_ptr<ros::Publisher> pub_filtered_{ nullptr };
        std::map<std::string, std::unique_ptr<ros::Publisher>> pubs_map_seg_;
        std::map<std::string, std::unique_ptr<ros::Publisher>> pubs_map_inliers_;
#endif
    };
} // namespace pose_registration_plugins
