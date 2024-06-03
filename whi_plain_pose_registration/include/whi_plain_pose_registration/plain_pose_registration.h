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
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <memory>
#include <mutex>

namespace pose_registration_plugins
{
    class PlainPoseRegistration : public whi_pose_registration::BasePoseRegistration
    {
    public:
        PlainPoseRegistration();
        virtual ~PlainPoseRegistration() = default;

    public:
        void initialize() override;
        void computeVelocityCommands(geometry_msgs::Twist& CmdVel) override;
        void standby(const geometry_msgs::PoseStamped& PatternPose) override;
        int goalState() override;

    private:
        void update(const ros::TimerEvent& Event);
        void subCallbackLaserScan(const sensor_msgs::LaserScan::ConstPtr& Laser);

    private:
        double feature_arch_radius_{ 0.06 };
        double feature_arch_radius_tolerance_{ 0.01 };
        geometry_msgs::Pose pose_feature_;
        double distance_to_charge_{ 0.0 };
        bool downsampling_{ true };
        std::vector<double> downsampling_coeffs_;
        double line_distance_thresh_{ 0.001 };
        double circle_distance_thresh_{ 0.001 };
        double feature_segment_distance_thresh_{ 0.04 };
        int feature_min_size_{ 10 };
        int feature_max_size_{ 200 };
        double tf_listener_frequency_{ 20 };
        std::mutex mtx_cut_min_;
        geometry_msgs::TransformStamped tf_baselink_map_;
        geometry_msgs::TransformStamped tf_laser_map_;
        geometry_msgs::TransformStamped tf_map_laser_;
        geometry_msgs::TransformStamped tf_laser_baselink_;
        geometry_msgs::Pose pose_target_;
        double found_feature_angle_{ 0.0 };
        double xy_tolerance_{ 0.02 };
        double yaw_tolerance_{ 0.087 };
        enum State
        {
            STA_ALIGNED = 0,
            STA_TO_VERTICAL, STA_MOVE_VERTICAL,
            STA_TO_ALIGN, STA_MOVE_ALIGN,
            STA_DONE, STA_FAILED
        };
        int state_{ STA_DONE };
        double find_tried_time_{ 0.0 };
        /// segment related
        std::string segment_type_{ "region_growing" };
        int k_neighbour_{ 50 };
        double k_radius_{ 0.2 };
        // cut-min
        geometry_msgs::Point center_;
        double radius_{ 0.1 };
        double sigma_{ 0.25 };
        double weight_{ 0.8 };
        int cut_min_neighbour_{ 5 };
        // region growing
        int region_growing_neighbour_{ 30 };
        double angle_{ 3.0 }; // degrees
        double curvature_{ 1.0 };
        int min_cluster_size_{ 50 };
        int max_cluster_size_{ 1000 };
        // conditional Euclidean
        double cluster_radius_{ 0.2 };
        double intensity_tolerance_{ 5.0 };
        std::unique_ptr<ros::Timer> non_realtime_loop_{ nullptr };
        std::unique_ptr<ros::Publisher> pub_stage_target_{ nullptr };
#ifndef DEBUG
        std::unique_ptr<ros::Publisher> pub_projected_{ nullptr };
        std::unique_ptr<ros::Publisher> pub_converted_{ nullptr };
        std::unique_ptr<ros::Publisher> pub_downsampled_{ nullptr };
        std::map<std::string, std::unique_ptr<ros::Publisher>> pubs_map_seg_;
        std::map<std::string, std::unique_ptr<ros::Publisher>> pubs_map_inliers_;
#endif
    };
} // namespace pose_registration_plugins
