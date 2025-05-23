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
2024-06-19: update imu navigation while route vertical
2024-06-20: modify endpose relative location fixed ;add curpose orientation angle ;
            modify  STA_ADJUST_VERTICAL to STA_WAIT_SCAN
2024-06-21: featurepose getinitvertical and horizon Reverse order
******************************************************************/
#pragma once
#include <ros/ros.h> 
#include <std_srvs/SetBool.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/Imu.h>
#include <whi_pose_registration/base_pose_registration.h>
#include <whi_pose_registration_common/pcl_utilities.h>

#include <memory>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <deque>
#include <nav_msgs/Odometry.h>

namespace pose_registration_plugins
{

    struct TargetRelaPose
    {
        std::vector<double> target_rela_pose;
        std::string direction = "direct";
        bool using_inertial;
    };

    struct FeatureConfig
    {
        std::string name;
        std::vector<double> cur_pose;
        std::vector<double> feature_pose;
        std::deque<TargetRelaPose> target_rela_pose_vec;
    };

    enum State
    {
        STA_START = 0,
        STA_REGISTRATE,
        STA_REGISTRATE_FINE,
        STA_REGISTRATING,
        STA_ALIGN,
        STA_PRE_ROT_ANGLE,
        STA_ROT_ANGLE,
        STA_BACK,
        STA_PRE_ROT_VERTICAL,
        STA_ROT_VERTICAL,
        STA_ADJUST_REGIST,
        STA_ADJUST_AFTER_REGIST,
        STA_ADJUST_VERTICAL,
        STA_ADJUST_LAST_ROT,
        
        STA_PRE_HORIZON,
        STA_TO_HORIZON,
        STA_ROUTE_HORIZON,
        STA_PRE_VERTICAL,
        STA_TO_VERTICAL,
        STA_ROUTE_VERTICAL,
        STA_PRE_ORIENTATION,
        STA_TO_ORIENTATION,
        STA_PRE_NEXT,
        STA_DONE,
        STA_FAILED,
        STA_WAIT_SCAN,
        STA_DEBUG,

        STA_CHARGE_WALK,        // CHARGE_WALK
        STA_CHARGE_PRE_ROT,
        STA_CHARGE_ROT,
        STA_CHARGE_HORIZON,

        STA_OFFSET_YAW,         // OFFSET
        STA_OFFSET_ROTATING,
        STA_OFFSET_Y,
        STA_OFFSET_FORWARD,
        STA_OFFSET_MOVING,
        STA_OFFSET_X,               

        STA_START_BACK,         // PLAN_START  导航新增目标点后,判断是否在充电桩附近, 目前是写死旋转180度
        STA_START_PRE_ROT,
        STA_START_ROT,

        ADAPT_FIRST_ROT,          // ADAPT  通过对靶标物的特征调整对准
        ADAPT_PRE_ROT_ANGLE,
        ADAPT_ROT_ANGLE,
        ADAPT_BACK,
        ADAPT_PRE_ROT_VERTICAL,
        ADAPT_ROT_VERTICAL,
        ADAPT_ADJUST_VERTICAL,
        ADAPT_PRE_HORIZON,
        ADAPT_PRE_NEXT,
        ADAPT_ROUTE_VERTICAL


    };

    class LocatePoseRegistration : public whi_pose_registration::BasePoseRegistration
    {
    public:
        LocatePoseRegistration();
        virtual ~LocatePoseRegistration();

    public:
        void initialize() override;
        void computeVelocityCommands(geometry_msgs::Twist& CmdVel) override;
        void standby(const whi_interfaces::PoseRegistrationGoalConstPtr& Goal) override;
        int goalState() override;

    private:
        void stateRegistration(geometry_msgs::Twist& CmdVel, double YawImu);
        void stateOffset(geometry_msgs::Twist& CmdVel, double YawImu);
        void stateStart(geometry_msgs::Twist& CmdVel, double YawImu);
        void stateChargeWalk(geometry_msgs::Twist& CmdVel, double YawImu);
        void stateAdjust(geometry_msgs::Twist& CmdVel, double YawImu);
        void subCallbackLaserScan(const sensor_msgs::LaserScan::ConstPtr& Laser);
        void subCallbackImu(const sensor_msgs::Imu::ConstPtr& Imudata);
        void subCallbackOdom(const nav_msgs::Odometry::ConstPtr& msg);
        bool checkcurpose();
        void updateCurrentPose();
        double getrightImu(double angletar);
        std::shared_ptr<void> registration(const sensor_msgs::LaserScan::ConstPtr& Laser);
        void threadRegistration();
        void updatePre(double YawImu);
        bool checkCurpointNearCharge();

    private:
        bool is_fixed_location_;
        geometry_msgs::Pose pose_feature_;
        geometry_msgs::Pose pose_arrive_;
        geometry_msgs::Pose pose_standby_;
        geometry_msgs::Pose pose_standby_odom_;
        geometry_msgs::Pose pose_odom_;
        bool using_odom_pose_;
        geometry_msgs::Pose feature_cur_pose_;
        double pattern_met_location_thresh_{ 0.5 };
        double pattern_met_radius_thresh_{ 0.05 };
        double tf_listener_frequency_{ 20 };
        std::mutex mtx_cut_min_;
        geometry_msgs::TransformStamped tf_baselink_map_;
        geometry_msgs::Pose pose_target_;
        geometry_msgs::Pose pose_end_;
        geometry_msgs::Pose getfea_cur_pose_;
        geometry_msgs::Pose pose_pre_;
        double yaw_pre_{ 0.0 };
        double controller_frequency_;
        double predict_period_count_;
        double predict_dist_thresh_;
        double get_align_imu_{ 0.0 };
        double get_align_angle_{ 0.0 };
        double get_horizon_imu_{ 0.0 };
        double get_horizon_angle_{ 0.0 };   
        double get_horizon_direct_imu_{ 0.0 };     
        double get_vertical_direct_imu_{ 0.0 };
        double rot_offset_{ 0.0 };
        geometry_msgs::Pose vertical_start_pose_;
        double horizon_offset_vel_{ 0.1 };
        double vertical_to_rotvel_{ 0.1 };
        double imu_adjust_rot_vel_{ 0.01 };
        double imu_adjust_rot_thresh_{ 0.02 };
        bool using_inertial_{ true };
        bool horizon_test_{ true };
        double inertial_rotvel_{ 0.25 };
        double inertial_xyvel_{ 0.04 };
        std::vector<double> target_rela_pose_ ;
        std::deque<TargetRelaPose> target_rela_pose_vec_;
        std::string route_vertical_direct_;
        int leftorright_{ 1 };
        std::string model_cloud_path_;
        std::vector<FeatureConfig> features_config_;
        double xy_tolerance_{ 0.02 };
        double yaw_tolerance_{ 0.087 };
        double regist_linear_x_thresh_{ 0.03 };
        double regist_linear_y_thresh_{ 0.03 };
        double regist_yaw_thresh_{ 0.087 };
        double zig_angle_;
        double feature_angle_;
        double distance_horizon_;
        double distance_vertical_;
        double registAngle_;
        double distthresh_horizon_;
        double distance_todrive_{ 0.0 };
        double xyvel_{ 0.1 };
        double rotvel_{ 0.2 };
        double fine_tune_ratio_rot_{ 0.6 };
        double fine_tune_ratio_linear_{ 0.6 };
        bool is_fine_tune_{ false };
        double rot_back_ratio_{ 1.0 };
        double lazer_motor_diff_{ 0.412 };
        double line_reg_thresh_{ 0.05 };
        std::vector<float> ndtsample_coeffs_;
        int ndtmaxiter_;
        int operate_index_{ -1 };
        std::string segment_type_{ "region_growing" };
        std::string mapframe_;
        std::string laser_frame_;
        std::string imu_frame_;
        std::vector<double> laser_pose_;
        double wait_scan_time_;
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
        int state_{ STA_DONE };
        int prestate_{ STA_DONE };

        pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_;
        bool using_imu_{ false };
        std::unique_ptr<ros::Subscriber> sub_imu_{ nullptr };
        double angleyaw_imu_;
        double angle_target_imu_;
        std::mutex mtx_imu_;

        EventQueue<void>::UniquePtr queue_scan_{ nullptr };
        std::thread th_registration_;
        std::condition_variable cv_;
        std::mutex mtx_cv_;
        std::atomic<bool> terminated_{ false };

        std::string odom_topic_;
        std::unique_ptr<ros::Subscriber> sub_odom_;

        std::string packpath_;
        std::vector<double> charge_walk_pose_;
        std::vector<double> charge_point_;
        double distance_charge_limit_;
        bool need_charge_near_{ false };   
        double rot_min_vel_{ 0.02 };
        double rot_back_yaw_tolerance_{ 0.5 }; 
        double angleDiff_g_;    

        // offset
        std::array<double, 3> offsets_; // x, y, yaw

        // debug
        int debug_count_{ 0 };
        bool debug_visualize_{ false };
        bool using_regist_{ true };
        double fit_line_thresh_{ 0.01 };
        std::vector<double> target_pose_;
        std::string target_shape_{ "symmetry" };
        double cross_angle_;
        double horizon_angle_;
        double cross_angle_tolerance_;
        double horizon_angle_tolerance_;
        double max_horizon_tolerance_{ 0.08 };
        int iteration_max_count_{ 0 };
        int iteration_count_{ 0 };
    };
} // namespace pose_registration_plugins
