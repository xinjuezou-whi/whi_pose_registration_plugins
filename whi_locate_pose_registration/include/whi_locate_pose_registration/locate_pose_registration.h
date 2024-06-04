/******************************************************************
relocate motion planning class for pose registration logic plugins

Features:
- relocate motion planning for pose registration
- xxx

Written by Yue Zhou, sevendull@163.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2024-05-11: Initial version
2022-xx-xx: xxx
******************************************************************/
#pragma once
#include <ros/ros.h> 
#include <whi_pose_registration/base_pose_registration.h>
#include "std_srvs/SetBool.h"
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "whi_pose_registration_common/pcl_utilities.h"
#include <memory>
#include <mutex>
#include <sensor_msgs/Imu.h>

namespace pose_registration_plugins
{

    struct FeatureConfig
    {
        std::string name;
        int leftright = 1;
        std::vector<double> cur_pose;
        std::vector<double> feature_pose;
        std::vector<double> target_rela_pose;
    };

    class LocatePoseRegistration : public whi_pose_registration::BasePoseRegistration
    {
    public:
        LocatePoseRegistration();
        virtual ~LocatePoseRegistration() = default;

    public:
        void initialize() override;
        void computeVelocityCommands(geometry_msgs::Twist& CmdVel) override;
        void standby(const geometry_msgs::PoseStamped& PatternPose) override;
        int goalState() override;

    private:
        void subCallbackLaserScan(const sensor_msgs::LaserScan::ConstPtr& Laser);
        void subCallbackImu(const sensor_msgs::Imu::ConstPtr& Imudata);
        bool checkcurpose();

    private:
        geometry_msgs::Pose pose_feature_;
        geometry_msgs::Pose pose_arrive_;
        geometry_msgs::Pose pose_standby_;
        geometry_msgs::Pose feature_cur_pose_;
        double curpose_thresh_;
        double tf_listener_frequency_{ 20 };
        std::mutex mtx_cut_min_;
        geometry_msgs::TransformStamped tf_baselink_map_;
        geometry_msgs::Pose pose_target_;
        std::vector<double> target_rela_pose_ ;
        int leftorright_{ 1 };
        std::string model_cloud_path_;
        std::vector<FeatureConfig> features_config_;
        double xy_tolerance_{ 0.02 };
        double yaw_tolerance_{ 0.087 };
        double feature_angle_;
        double distance_horizon_;
        double distance_vertical_;
        double distance_drive_;
        double distthresh_horizon_;
        bool iszerorela_{ false };           // 相对位置配置为 0 
        double xyvel_{ 0.1 };
        double rotvel_{ 0.2 };
        std::vector<float> ndtsample_coeffs_;
        int ndtmaxiter_;
        std::string segment_type_{ "region_growing" };
        std::string mapframe_;
        // cut-min
        int mincut_size_{ 300 };
        geometry_msgs::Point center_;
        double radius_{ 0.1 };
        double sigma_{ 0.25 };
        double weight_{ 0.8 };
        int cut_min_neighbour_{ 5 };
        // segment don
        double seg_scale1_;
        double seg_scale2_;
        double seg_threshold_;
        double seg_radius_;    
        // Euclidean
        double feature_segment_distance_thresh_{ 0.04 };
        int feature_min_size_{ 10 };
        int feature_max_size_{ 200 };            
        enum State
        {
            STA_START = 0,
            STA_REGISTRATE,
            STA_REGISTRATE_FINE,
            STA_ALIGN,
            STA_PRE_HORIZON,
            STA_TO_HORIZON,
            STA_ROUTE_HORIZON,
            STA_PRE_VERTICAL,
            STA_TO_VERTICAL,
            STA_ROUTE_VERTICAL,
            STA_PRE_ORIENTATION,
            STA_TO_ORIENTATION,
            STA_DONE,
            STA_FAILED,
            STA_WAIT_IMU,
            STA_WAIT_SCAN,
            STA_DEBUG
        };
        int state_{ STA_DONE };
        int prestate_{ STA_DONE };
        double find_tried_time_{ 0.0 };

        pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_;
        bool issetimu_{ false };
        std::unique_ptr<ros::Subscriber> sub_imu_{ nullptr };
        double angleyaw_imu_;
        double angletar_imu_;
        std::mutex mtx_imu_;
        double imu_delta_;

        int debug_count_{ 0 };

    };
} // namespace pose_registration_plugins
