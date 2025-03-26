/******************************************************************
locate motion planning class for pose registration logic plugins

Features:
- locate motion planning for pose registration
- xxx

Written by Yue Zhou, sevendull@163.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

******************************************************************/
#include "whi_locate_pose_registration/locate_pose_registration.h"
#include "whi_pose_registration_common/pcl_visualize.h"
#include "whi_pose_registration_common/pose_utilities.h"

#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>
#include <thread>
#include <chrono>
#include <yaml-cpp/yaml.h>
#include <ros/package.h>

namespace pose_registration_plugins
{
    LocatePoseRegistration::LocatePoseRegistration()
        : BasePoseRegistration()
    {
        /// node version and copyright announcement
	    std::cout << "\nWHI loacate pose registration plugin VERSION 00.08.1" << std::endl;
	    std::cout << "Copyright © 2024-2025 Wheel Hub Intelligent Co.,Ltd. All rights reserved\n" << std::endl;
    }

    LocatePoseRegistration::~LocatePoseRegistration()
    {
        cv_.notify_all();

        terminated_.store(true);
        if (th_registration_.joinable())
        {
            th_registration_.join();
        }
    }

    void LocatePoseRegistration::initialize()
    {
        /// params
        // common
        std::string laserScanTopic, imuTopic;
        node_handle_->param("pose_registration/LocatePose/model_cloud_path", model_cloud_path_, std::string("modelcloudpath"));
        std::string config_file;
        node_handle_->param("pose_registration/LocatePose/features_file", config_file, std::string("modelcloudpath"));
        printf("configfile is %s\n", config_file.c_str());
        try
        {
            YAML::Node config_node = YAML::LoadFile(config_file.c_str());
            YAML::Node featurenode = config_node;
            for (const auto &it : featurenode) 
            {
                FeatureConfig onefeature;
                for (const auto &pair : it) 
                {
                    if (pair.first.as<std::string>() == "name")
                    {
                        onefeature.name = pair.second.as<std::string>();
                    }                         
                    else if (pair.first.as<std::string>() == "cur_pose")
                    {
                        for (const auto& item : pair.second)
                        {
                            onefeature.cur_pose.push_back(item.as<double>());
                        }
                    }
                    else if (pair.first.as<std::string>() == "feature_pose")
                    {
                        for (const auto& item : pair.second)
                        {
                            onefeature.feature_pose.push_back(item.as<double>());
                        }
                    }
                    else if (pair.first.as<std::string>() == "target_relative_pose")
                    {
                        for (const auto& onepose : pair.second)
                        {
                            TargetRelaPose onetarget_rela_pose;
                            //onefeature.target_rela_pose.push_back(item.as<double>());
                            onetarget_rela_pose.direction = "direct";
                            onetarget_rela_pose.using_inertial = false;
                            for(const auto& subpair : onepose)
                            {
                                if (subpair.first.as<std::string>() == "pose")
                                {
                                    for (const auto& item : subpair.second)
                                    {
                                        onetarget_rela_pose.target_rela_pose.push_back(item.as<double>());
                                    }
                                }
                                else if (subpair.first.as<std::string>() == "drive_direction")
                                {
                                    onetarget_rela_pose.direction = subpair.second.as<std::string>();
                                }
                                else if (subpair.first.as<std::string>() == "using_inertial")
                                {
                                    onetarget_rela_pose.using_inertial = subpair.second.as<bool>();
                                }                                   
                            }
                            onefeature.target_rela_pose_vec.push_back(onetarget_rela_pose);
                        }
                    }
                }
                features_config_.push_back(onefeature);
            }

            for (auto oneiter = features_config_.begin(); oneiter!= features_config_.end(); oneiter++)
            {
                printf("onefeature:\n");
                printf("name: %s, cur_pose: [%f, %f, %f ]", (*oneiter).name.c_str(), (*oneiter).cur_pose[0], (*oneiter).cur_pose[1], (*oneiter).cur_pose[2]);
                printf("feature_pose: [%f, %f, %f]\n", (*oneiter).feature_pose[0], (*oneiter).feature_pose[1], (*oneiter).feature_pose[2]);
                for (auto onepose : (*oneiter).target_rela_pose_vec)
                {
                    printf("target_rela_pose: [%f, %f, %f]", onepose.target_rela_pose[0], onepose.target_rela_pose[1], onepose.target_rela_pose[2]);
                    printf("using_inertial: %d", onepose.using_inertial);
                    printf("direction: %s\n",onepose.direction.c_str());
                }

            }
        }
        catch (const std::exception& e)
        {
            ROS_FATAL_STREAM("failed to load protocol config file " << config_file);
        }

        node_handle_->param("pose_registration/controller_frequency", controller_frequency_, 10.0);
        node_handle_->param("pose_registration/xy_tolerance", xy_tolerance_, 0.02);
        node_handle_->param("pose_registration/yaw_tolerance", yaw_tolerance_, 5.0);
        node_handle_->param("pose_registration/LocatePose/regist_linear_x_thresh", regist_linear_x_thresh_, 0.03);
        node_handle_->param("pose_registration/LocatePose/regist_linear_y_thresh", regist_linear_y_thresh_, 0.03);
        node_handle_->param("pose_registration/LocatePose/regist_yaw_thresh", regist_yaw_thresh_, 0.087);
        node_handle_->param("pose_registration/LocatePose/zig_angle", zig_angle_, 45.0);
        node_handle_->param("pose_registration/LocatePose/inertial_xyvel", inertial_xyvel_, 0.04);
        //node_handle_->param("pose_registration/LocatePose/using_inertial", using_inertial_, true);
        node_handle_->param("pose_registration/LocatePose/inertial_rotvel", inertial_rotvel_, 0.25);

        zig_angle_ = angles::from_degrees(zig_angle_);
        regist_yaw_thresh_ = angles::from_degrees(regist_yaw_thresh_);
        yaw_tolerance_ = angles::from_degrees(yaw_tolerance_);
        if (!node_handle_->getParam("pose_registration/LocatePose/laser_pose", laser_pose_))
        {
            laser_pose_.resize(6);
        }
        printf("laser_pose, x: %f, y: %f, z: %f, roll: %f, pitch: %f, yaw: %f\n",
            laser_pose_[0], laser_pose_[1], laser_pose_[2], laser_pose_[3], laser_pose_[4], laser_pose_[5]);
        node_handle_->param("pose_registration/LocatePose/imu_frame", imu_frame_, std::string("imu"));        
        node_handle_->param("pose_registration/LocatePose/laser_frame", laser_frame_, std::string("laser"));        
        node_handle_->param("pose_registration/LocatePose/tf_listener_frequency", tf_listener_frequency_, 20.0);
        node_handle_->param("pose_registration/LocatePose/laser_scan_topic", laserScanTopic, std::string("scan"));
        node_handle_->param("pose_registration/LocatePose/base_link_frame", base_link_frame_, std::string("base_link"));
        node_handle_->param("pose_registration/LocatePose/map_frame", mapframe_, std::string("map"));
        node_handle_->param("pose_registration/LocatePose/imu_topic", imuTopic, std::string("imu")); //issetimu
        node_handle_->param("pose_registration/LocatePose/using_imu", using_imu_, false); 
        node_handle_->param("pose_registration/LocatePose/horizon_aligned_thresh", distthresh_horizon_, 0.005);   
        node_handle_->param("pose_registration/LocatePose/pattern_met_location_thresh", pattern_met_location_thresh_, 0.5);  
        node_handle_->param("pose_registration/LocatePose/pattern_met_radius_thresh", pattern_met_radius_thresh_, 0.08); 
        node_handle_->param("pose_registration/LocatePose/predict_dist_thresh", predict_dist_thresh_, 0.05); 
        node_handle_->param("pose_registration/LocatePose/predict_period_count", predict_period_count_, 1.0); 
        node_handle_->param("pose_registration/LocatePose/horizon_offset_vel", horizon_offset_vel_, 0.1); 
        node_handle_->param("pose_registration/LocatePose/vertical_to_rotvel", vertical_to_rotvel_, 0.1); 
        node_handle_->param("pose_registration/LocatePose/wait_scan_time", wait_scan_time_, 0.8);  
        node_handle_->param("pose_registration/LocatePose/is_fixed_location", is_fixed_location_, false); //
        node_handle_->param("pose_registration/LocatePose/rot_offset", rot_offset_, 0.0);  
        node_handle_->param("pose_registration/LocatePose/fine_tune_ratio_rot", fine_tune_ratio_rot_, 0.6); 
        node_handle_->param("pose_registration/LocatePose/fine_tune_ratio_linear", fine_tune_ratio_linear_, 0.6); 
        node_handle_->param("pose_registration/LocatePose/rot_back_ratio", rot_back_ratio_, 1.0);   
        node_handle_->param("pose_registration/LocatePose/lazer_motor_diff", lazer_motor_diff_, 0.412);  // 
        node_handle_->param("pose_registration/LocatePose/line_reg_thresh", line_reg_thresh_, 0.05);
        if (!node_handle_->getParam("pose_registration/LocatePose/ndt_sample_coeffs", ndtsample_coeffs_))
        {
            for (int i = 0; i < 3; ++i)
            {
                ndtsample_coeffs_.push_back(0.005);
            }
        }        
        node_handle_->param("pose_registration/LocatePose/mincut_size", mincut_size_, 300);
        node_handle_->param("pose_registration/LocatePose/radius", radius_, 0.1);
        node_handle_->param("pose_registration/LocatePose/sigma", sigma_, 0.25);
        node_handle_->param("pose_registration/LocatePose/weight", weight_, 0.8);
        node_handle_->param("pose_registration/LocatePose/cut_min_neighbour", cut_min_neighbour_, 5);

        node_handle_->param("pose_registration/LocatePose/seg_normal_min_scale", seg_scale1_, 1.0);
        node_handle_->param("pose_registration/LocatePose/seg_normal_max_scale", seg_scale2_, 1.0);
        node_handle_->param("pose_registration/LocatePose/seg_threshold", seg_threshold_, 0.2);
        node_handle_->param("pose_registration/LocatePose/seg_radius", seg_radius_, 5.0);

        node_handle_->param("pose_registration/LocatePose/debug_count", debug_count_, 0);
        node_handle_->param("pose_registration/LocatePose/debug_visualize", debug_visualize_, false);
        
        node_handle_->param("pose_registration/LocatePose/feature_segment_distance_thresh",
            feature_segment_distance_thresh_, 0.04);
        node_handle_->param("pose_registration/LocatePose/feature_min_size", feature_min_size_, 10);
        node_handle_->param("pose_registration/LocatePose/feature_max_size", feature_max_size_, 200);
        node_handle_->param("pose_registration/LocatePose/ndt_maxiter", ndtmaxiter_, 5000);
        node_handle_->param("pose_registration/LocatePose/odom_topic", odom_topic_, std::string("odom"));
        node_handle_->param("pose_registration/LocatePose/using_odom_pose", using_odom_pose_, false);
        node_handle_->param("pose_registration/LocatePose/imu_adjust_rot_vel", imu_adjust_rot_vel_, 0.01);
        node_handle_->param("pose_registration/LocatePose/imu_adjust_rot_thresh", imu_adjust_rot_thresh_, 0.02);

        if (!node_handle_->getParam("pose_registration/charge_walk_pose", charge_walk_pose_))
        {
            for (int i = 0; i < 3; ++i)
            {
                charge_walk_pose_.push_back(0.0);
            }
        }  
        if (!node_handle_->getParam("pose_registration/charge_point", charge_point_))
        {
            for (int i = 0; i < 3; ++i)
            {
                charge_point_.push_back(0.0);
            }
        }          
        node_handle_->param("pose_registration/distance_charge_limit", distance_charge_limit_, 0.05);
        node_handle_->param("pose_registration/need_charge_near", need_charge_near_, false);      
        node_handle_->param("pose_registration/LocatePose/rot_back_yaw_tolerance", rot_back_yaw_tolerance_, 0.5);
        node_handle_->param("pose_registration/LocatePose/rot_min_vel", rot_min_vel_, 0.05); 
        rot_back_yaw_tolerance_ = angles::from_degrees(rot_back_yaw_tolerance_); 
        sub_laser_scan_ = std::make_unique<ros::Subscriber>(node_handle_->subscribe<sensor_msgs::LaserScan>(
		    laserScanTopic, 10, std::bind(&LocatePoseRegistration::subCallbackLaserScan, this, std::placeholders::_1)));
        sub_imu_ = std::make_unique<ros::Subscriber>(node_handle_->subscribe<sensor_msgs::Imu>(
		    imuTopic, 10, std::bind(&LocatePoseRegistration::subCallbackImu, this, std::placeholders::_1)));
        sub_odom_ = std::make_unique<ros::Subscriber>(
            node_handle_->subscribe<nav_msgs::Odometry>(odom_topic_, 1, boost::bind(&LocatePoseRegistration::subCallbackOdom, this, _1)) );
        // event queue for laser scan
        queue_scan_ = std::make_unique<EventQueue<void>>(1, false);
        threadRegistration();

        packpath_ = ros::package::getPath("whi_locate_pose_registration");
        horizon_test_ = false;
    }

    void LocatePoseRegistration::computeVelocityCommands(geometry_msgs::Twist& CmdVel)
    {
        double yawFromImu = 0.0;
        {
            const std::lock_guard<std::mutex> lock(mtx_imu_);
            yawFromImu = angleyaw_imu_;
        }
        
        stateRegistration(CmdVel, yawFromImu);
        stateOffset(CmdVel, yawFromImu);
        stateStart(CmdVel,yawFromImu);
    }

    void LocatePoseRegistration::standby(const whi_interfaces::PoseRegistrationGoalConstPtr& Goal)
    {
        node_handle_->param("pose_registration/LocatePose/xyvel", xyvel_, 0.05);
        node_handle_->param("pose_registration/LocatePose/rotvel", rotvel_, 0.1);
        if (Goal->velocity_scale > 1e-3)
        {
            xyvel_ *= Goal->velocity_scale;
            rotvel_ *= Goal->velocity_scale;
        }

        if (Goal->target_pose.header.frame_id == "charging_safe_pose") // flag of back to the safe pose to avoid collision
        {
            // charge to walk
            state_ = STA_CHARGE_WALK;

            printf("standby, start STA_CHARGE_WALK\n");
            updateCurrentPose();
        }
        else if (Goal->target_pose.header.frame_id == "motion_offsets") // flag of move offset
        {
            offsets_[0] = Goal->target_pose.pose.position.x;
            offsets_[1] = Goal->target_pose.pose.position.y;
            offsets_[2] = PoseUtilities::toEuler(Goal->target_pose.pose.orientation)[2];

            state_ = STA_OFFSET_YAW;
        }
        else if (Goal->target_pose.header.frame_id == "plan_start") // flag of move offset
        {
            ROS_INFO("start plan_start");
            bool needStartBack = false;
            needStartBack = checkCurpointNearCharge();
            if (needStartBack)
            {
                state_ = STA_START_BACK;
                updateCurrentPose();
            }
            else
            {
                state_ = STA_DONE;
            }
        }
        else
        {
            // registration logic
            bool curposeright = checkcurpose();
            if (curposeright)
            {
                prestate_ = STA_START;
                state_ = STA_WAIT_SCAN;
                printf("in standby\n");
                operate_index_ = 0;
            }
            else
            {
                state_ = STA_FAILED;
                ROS_WARN("in standby, but curpose is wrong, check config or not near the pattern");
            }
        }
    }

    int LocatePoseRegistration::goalState()
    {
        if (state_ == STA_DONE)
        {
            return GS_REACHED;
        }
        else if (state_ == STA_FAILED)
        {
            return GS_FAILED;
        }
        else
        {
            return GS_PROCEEDING;
        }
    }

    void LocatePoseRegistration::stateRegistration(geometry_msgs::Twist& CmdVel, double YawImu)
    {
        if (state_ == STA_ALIGN)
        {
            geometry_msgs::TransformStamped transBaselinkMap = listenTf(mapframe_, base_link_frame_, ros::Time(0));
            double angleDiff = angles::shortest_angular_distance(
                PoseUtilities::toEuler(transBaselinkMap.transform.rotation)[2],
                PoseUtilities::toEuler(pose_target_.orientation)[2]);
            if (using_imu_)
            {
                angleDiff = angles::shortest_angular_distance(YawImu, angle_target_imu_);
                if (debug_count_ == 5)
                {
                    printf("in state STA_ALIGN, angle_target_imu_ = %f, YawImu = %f, angleDiff = %f\n",
                        angle_target_imu_, YawImu, angleDiff );
                }
            }
            else
            {
                if (debug_count_ == 5)
                {
                    printf("in state STA_ALIGN, angle_target_imu_ = %f, angleDiff = %f\n",
                        angle_target_imu_, angleDiff );
                }
            }        
            if (fabs(angleDiff) < yaw_tolerance_)
            {
                CmdVel.linear.x = 0.0;
                CmdVel.angular.z = 0.0;
                if (prestate_ == STA_REGISTRATE_FINE)
                {
                    // new modify
                    // determine the process in registration
                    updateCurrentPose(); 
                    prestate_ = STA_ALIGN;
                    state_ = STA_WAIT_SCAN;
                    printf("STA_ALIGN finished, YawImu = %f\n", YawImu);
                }
                else if (prestate_ == STA_REGISTRATE)
                {
                    state_ = STA_WAIT_SCAN;
                    printf("STA_REGISTRATE finished, wait scan\n");
                }
            }
            else
            {
                CmdVel.linear.x = 0.0;
                //CmdVel.angular.z = is_fine_tune_ ? PoseUtilities::signOf(sin(angleDiff)) * rotvel_ * fine_tune_ratio_rot_ : PoseUtilities::signOf(sin(angleDiff)) * rotvel_ ;
                double vel_z = fabs(angleDiff) / fabs(angleDiff_g_) * rotvel_;
                vel_z = vel_z > rot_min_vel_ ? vel_z : rot_min_vel_;
                CmdVel.angular.z = PoseUtilities::signOf(sin(angleDiff)) * vel_z;
            }
        }
        else if (state_ == STA_PRE_ROT_ANGLE)
        {
            geometry_msgs::TransformStamped transBaselinkMap = listenTf(mapframe_.c_str(), base_link_frame_, ros::Time(0));
            double angleBaselink = PoseUtilities::toEuler(transBaselinkMap.transform.rotation)[2];

            pose_target_.orientation = PoseUtilities::fromEuler(0.0, 0.0,
                angles::shortest_angular_distance(PoseUtilities::signOf(distance_horizon_) * zig_angle_, angleBaselink));
            angle_target_imu_ = angles::shortest_angular_distance(
                PoseUtilities::signOf(distance_horizon_) * zig_angle_, YawImu);

            state_ = STA_ROT_ANGLE;
            printf("STA_PRE_ROT_ANGLE finished, start STA_ROT_ANGLE, curyaw is: %f ,angle_target_imu_ is: %f\n", YawImu, angle_target_imu_);
        }
        else if (state_ == STA_ROT_ANGLE)
        {
            geometry_msgs::TransformStamped transBaselinkMap = listenTf(mapframe_.c_str(), base_link_frame_, ros::Time(0));
            double angleDiff = angles::shortest_angular_distance(
                PoseUtilities::toEuler(transBaselinkMap.transform.rotation)[2], PoseUtilities::toEuler(pose_target_.orientation)[2]);
            if (using_imu_)
            {
                angleDiff = angles::shortest_angular_distance(YawImu, angle_target_imu_);
            }       
            if (fabs(angleDiff) < rot_back_yaw_tolerance_)
            {
                CmdVel.linear.x = 0.0;
                CmdVel.angular.z = 0.0;
                state_ = STA_BACK;
                distance_todrive_ = fabs(distance_horizon_) / sin(zig_angle_);
                updateCurrentPose();
                printf("STA_ROT_ANGLE finished, curyaw is: %f\n", YawImu);
                printf("STA_ROT_ANGLE finished, start STA_BACK, distance_todrive: %f\n", distance_todrive_);
            }
            else
            {
                CmdVel.linear.x = 0.0;
                CmdVel.angular.z = is_fine_tune_ ? PoseUtilities::signOf(sin(angleDiff)) * rotvel_ * fine_tune_ratio_rot_ : PoseUtilities::signOf(sin(angleDiff)) * rotvel_ ;
            }
        }
        else if (state_ == STA_BACK)
        {
            geometry_msgs::TransformStamped transBaselinkMap = listenTf(mapframe_.c_str(), base_link_frame_, ros::Time(0));
            geometry_msgs::Pose curpose;
            curpose.position.x = transBaselinkMap.transform.translation.x;
            curpose.position.y = transBaselinkMap.transform.translation.y;
            double routedis = 0.0;
            if (using_odom_pose_)
            {
                routedis = PoseUtilities::distance(pose_odom_, pose_standby_odom_);
            }
            else
            {
                routedis = PoseUtilities::distance(curpose,pose_standby_);
            }              
            double extradis = distance_todrive_ + 0.1;  //0.3
            double distDiff = distance_todrive_ * rot_back_ratio_ - routedis;   // add rot_back_ratio 旋转时候的误差，通过系数补偿
          
            //行驶距离超出计算值 30cm ；偏差过大，说明前面对齐失败了
            if (routedis > extradis)
            {
                CmdVel.linear.x = 0.0;
                CmdVel.linear.y = 0.0;
                state_ = STA_FAILED;
                printf("fail at STA_BACK, routedis too far, align or rotate wrong\n");
            }
            else if (fabs(distDiff) < xy_tolerance_)
            {
                CmdVel.linear.x = 0.0;
                CmdVel.angular.z = 0.0;
                state_ = STA_PRE_ROT_VERTICAL;
                printf("STA_BACK finished, start STA_PRE_ROT_VERTICAL\n"); 
            }
            else
            {
                CmdVel.linear.x = is_fine_tune_ ? -1 * PoseUtilities::signOf(distDiff) * xyvel_ * fine_tune_ratio_linear_: -1 * PoseUtilities::signOf(distDiff) * xyvel_;
                CmdVel.linear.y = 0.0;
            }
        }
        else if (state_ == STA_PRE_ROT_VERTICAL)
        {
            geometry_msgs::TransformStamped transBaselinkMap = listenTf(mapframe_.c_str(), base_link_frame_, ros::Time(0));
            double angleBaselink = PoseUtilities::toEuler(transBaselinkMap.transform.rotation)[2];

            pose_target_.orientation = PoseUtilities::fromEuler(0.0, 0.0, angleBaselink + PoseUtilities::signOf(distance_horizon_) * zig_angle_);
            angle_target_imu_ = YawImu + PoseUtilities::signOf(distance_horizon_) * zig_angle_;

            angle_target_imu_ = getrightImu(angle_target_imu_);
            updateCurrentPose(); 
            state_ = STA_ROT_VERTICAL;
            printf("STA_PRE_ROT_VERTICAL finished, start STA_ROT_VERTICAL, curyaw is: %f, angle_target_imu_ is: %f\n", YawImu, angle_target_imu_);   
        }
        else if (state_ == STA_ROT_VERTICAL)
        {
            geometry_msgs::TransformStamped transBaselinkMap = listenTf(mapframe_.c_str(), base_link_frame_, ros::Time(0));
            double angleDiff = angles::shortest_angular_distance(
                PoseUtilities::toEuler(transBaselinkMap.transform.rotation)[2], PoseUtilities::toEuler(pose_target_.orientation)[2]);
            if (using_imu_)
            {
                angleDiff = angles::shortest_angular_distance(YawImu, angle_target_imu_);
            }            
            if (fabs(angleDiff) < rot_back_yaw_tolerance_)
            {
                CmdVel.linear.x = 0.0;
                CmdVel.angular.z = 0.0;
                //modify here 修改这里,优化配准
                state_ = STA_ADJUST_VERTICAL;    //STA_ADJUST_REGIST
                //prestate_ = state_;
                //state_ = STA_WAIT_SCAN;
                updateCurrentPose(); 
                distance_todrive_ = fabs(distance_horizon_) / tan(zig_angle_) + distance_vertical_; //在这里计算
                printf("STA_ROT_VERTICAL finished, curyaw is: %f\n", YawImu);
                //printf("STA_ROT_VERTICAL finished, prepare for STA_ADJUST_REGIST, cal for distance_todrive_ = %f\n", distance_todrive_);
            }
            else
            {
                CmdVel.linear.x = 0.0;
                CmdVel.angular.z = is_fine_tune_ ? PoseUtilities::signOf(sin(angleDiff)) * rotvel_ * fine_tune_ratio_rot_ :PoseUtilities::signOf(sin(angleDiff)) * rotvel_ ;
            }
        }
        else if (state_ == STA_ADJUST_AFTER_REGIST)
        {
            geometry_msgs::TransformStamped transBaselinkMap = listenTf(mapframe_.c_str(), base_link_frame_, ros::Time(0));
            double angleDiff = angles::shortest_angular_distance(
                PoseUtilities::toEuler(transBaselinkMap.transform.rotation)[2], PoseUtilities::toEuler(pose_target_.orientation)[2]);
            if (using_imu_)
            {
                angleDiff = angles::shortest_angular_distance(YawImu, angle_target_imu_);
            }            
            if (fabs(angleDiff) < yaw_tolerance_)
            {
                CmdVel.linear.x = 0.0;
                CmdVel.angular.z = 0.0;
                state_ = STA_ADJUST_VERTICAL;
                updateCurrentPose(); 

                printf("STA_ADJUST_AFTER_REGIST finished, start STA_ADJUST_VERTICAL, distance_todrive_ = %f\n", distance_todrive_);
            }
            else
            {
                CmdVel.linear.x = 0.0;
                CmdVel.angular.z = is_fine_tune_ ? PoseUtilities::signOf(sin(angleDiff)) * rotvel_ * fine_tune_ratio_rot_ :PoseUtilities::signOf(sin(angleDiff)) * rotvel_ ;
            }
        }
        else if (state_ == STA_ADJUST_VERTICAL)
        {
            geometry_msgs::TransformStamped transBaselinkMap = listenTf(mapframe_.c_str(), base_link_frame_, ros::Time(0));
            geometry_msgs::Pose curpose;
            curpose.position.x = transBaselinkMap.transform.translation.x;
            curpose.position.y = transBaselinkMap.transform.translation.y;
            double routedis = 0.0;
            if (using_odom_pose_)
            {
                routedis = PoseUtilities::distance(pose_odom_, pose_standby_odom_);
            }
            else
            {
                routedis = PoseUtilities::distance(curpose,pose_standby_);
            }              
            double extradis = fabs(distance_todrive_) + 0.1;
            double distDiff = fabs(distance_todrive_) - routedis;
            //行驶距离超出计算值 30cm ；偏差过大，说明前面对齐失败了
            if (routedis > extradis)
            {
                CmdVel.linear.x = 0.0;
                CmdVel.linear.y = 0.0;
                state_ = STA_FAILED;
                printf("fail at STA_ADJUST_VERTICAL, routedis too far, align or rotate wrong\n");
            }
            else if (fabs(distDiff) < xy_tolerance_)
            {
                CmdVel.linear.x = 0.0;
                CmdVel.angular.z = 0.0;

                prestate_ = state_;
                state_ = STA_WAIT_SCAN;
            }
            else
            {
                CmdVel.linear.x = is_fine_tune_ ? PoseUtilities::signOf(distDiff) * PoseUtilities::signOf(distance_todrive_) * xyvel_ * fine_tune_ratio_linear_ :PoseUtilities::signOf(distDiff) * PoseUtilities::signOf(distance_todrive_) * xyvel_;
                CmdVel.linear.y = 0.0;
            }
        }
        else if (state_ == STA_ADJUST_LAST_ROT)
        {
            geometry_msgs::TransformStamped transBaselinkMap = listenTf(mapframe_.c_str(), base_link_frame_, ros::Time(0));
            double angleDiff = angles::shortest_angular_distance(
                PoseUtilities::toEuler(transBaselinkMap.transform.rotation)[2], PoseUtilities::toEuler(pose_target_.orientation)[2]);
            if (using_imu_)
            {
                angleDiff = angles::shortest_angular_distance(YawImu, angle_target_imu_);
            }           
            if (fabs(angleDiff) < yaw_tolerance_)
            {
                CmdVel.linear.x = 0.0;
                CmdVel.angular.z = 0.0;
                state_ = STA_PRE_HORIZON;
                updateCurrentPose();
                printf("STA_ADJUST_LAST_ROT finished, curyaw is: %f\n", YawImu);
                printf("STA_ADJUST_LAST_ROT finished, to STA_PRE_HORIZON \n");
            }
            else
            {
                CmdVel.linear.x = 0.0;
                CmdVel.angular.z =  PoseUtilities::signOf(sin(angleDiff)) * 0.1 ;   // rot vel = 0.1
            }
        }
        else if (state_ == STA_PRE_HORIZON)
        {  
            if (target_rela_pose_vec_.empty())
            {
                printf("in STA_PRE_HORIZON, target_rela_pose_vec_ is empty, done\n");
                CmdVel.linear.x = 0.0;
                CmdVel.angular.z = 0.0;
                state_ = STA_DONE;
                return ;                
            }

            auto front_target_rela = target_rela_pose_vec_.front();
            target_rela_pose_[0] = front_target_rela.target_rela_pose[0];
            target_rela_pose_[1] = front_target_rela.target_rela_pose[1];
            target_rela_pose_[2] = front_target_rela.target_rela_pose[2];  
            route_vertical_direct_ = front_target_rela.direction;
            using_inertial_ = front_target_rela.using_inertial;     
            if (PoseUtilities::signOf(target_rela_pose_[1]) == 0 )
            {
                leftorright_ = 1;
            }
            else
            {
                leftorright_ = PoseUtilities::signOf(target_rela_pose_[1]);
            }            
            target_rela_pose_vec_.pop_front();

            geometry_msgs::TransformStamped transBaselinkMap = listenTf(mapframe_.c_str(), base_link_frame_, ros::Time(0));
            double angleBaselink = PoseUtilities::toEuler(transBaselinkMap.transform.rotation)[2];
            pose_target_.orientation = PoseUtilities::fromEuler(0.0, 0.0, angleBaselink + leftorright_ * 0.5 * M_PI);
            angle_target_imu_ = YawImu + leftorright_ * 0.5 * M_PI;       
            angle_target_imu_ = getrightImu(angle_target_imu_);
            updateCurrentPose();

            distance_horizon_ = fabs(target_rela_pose_[1]) - leftorright_ * pose_feature_.position.y;    // mark here by zhouyue
            distance_vertical_ = target_rela_pose_[0] + pose_feature_.position.x;
            get_align_imu_ = YawImu;
            get_align_angle_ = angleBaselink;
            geometry_msgs::Pose relapos;
            relapos.position.x = distance_vertical_;
            relapos.position.y = target_rela_pose_[1] - pose_feature_.position.y;
            relapos.orientation = PoseUtilities::fromEuler(0.0, 0.0, 0.0);

            //pose_end_ = PoseUtilities::applyTransform(relapos, transBaselinkMap);

            geometry_msgs::Pose curpose;
            curpose.position.x = transBaselinkMap.transform.translation.x;
            curpose.position.y = transBaselinkMap.transform.translation.y;  

            geometry_msgs::TransformStamped trans = transBaselinkMap;
            trans.transform.translation.x = getfea_cur_pose_.position.x;
            trans.transform.translation.y = getfea_cur_pose_.position.y;
            trans.transform.rotation = getfea_cur_pose_.orientation;
            printf("STA_PRE_HORIZON curpose is: [%f, %f], angleBaselink: %f, get_align_imu_: %f\n",
                curpose.position.x, curpose.position.y, angleBaselink, get_align_imu_);   

            if (fabs(target_rela_pose_[0]) < 0.001 && fabs(target_rela_pose_[1]) < 0.001)
            {
                if (fabs(target_rela_pose_[2]) < 0.001)
                {
                    state_ = STA_PRE_NEXT;
                    printf("STA_PRE_HORIZON to STA_PRE_NEXT, sta_done\n");
                }
                else
                {
                    pose_end_.position.x = getfea_cur_pose_.position.x;
                    pose_end_.position.y = getfea_cur_pose_.position.y;
                    state_ = STA_PRE_ORIENTATION;
                    printf("pose_end is: [%f, %f]\n", pose_end_.position.x, pose_end_.position.y);
                    printf("target_rela_pose_[2] > 0, start STA_PRE_ORIENTATION\n");
                }
            }
            else if (fabs(target_rela_pose_[1]) < 0.001)
            {
                distance_vertical_ = target_rela_pose_[0];
                relapos.position.x = distance_vertical_;
                relapos.position.y = 0;
                relapos.orientation = PoseUtilities::fromEuler(0.0, 0.0, 0.0);
                pose_end_ = PoseUtilities::applyTransform(relapos, transBaselinkMap);
                vertical_start_pose_.position.x = transBaselinkMap.transform.translation.x;
                vertical_start_pose_.position.y = transBaselinkMap.transform.translation.y;
                if (!using_inertial_)
                {
                    state_ = STA_ROUTE_VERTICAL;
                    get_vertical_direct_imu_ = YawImu;
                    printf("at STA_PRE_HORIZON, !using_inertial, direct to STA_ROUTE_VERTICAL, distance_vertical_=%f \n",distance_vertical_);
                }
            }
            else
            {
                geometry_msgs::Pose relapos;
                relapos.position.x = fabs(target_rela_pose_[1]);
                relapos.position.y = 0;
                relapos.orientation = PoseUtilities::fromEuler(0.0, 0.0, 0.0);
                geometry_msgs::TransformStamped subtrans = transBaselinkMap;
                auto getfea_cur_pose_angle = PoseUtilities::toEuler(getfea_cur_pose_.orientation)[2];
                if (is_fixed_location_)
                {
                    subtrans.transform.translation.x = getfea_cur_pose_.position.x;
                    subtrans.transform.translation.y = getfea_cur_pose_.position.y;
                    subtrans.transform.rotation = PoseUtilities::fromEuler(0.0, 0.0, getfea_cur_pose_angle + leftorright_ * 0.5 * M_PI);
                }
                else
                {
                    subtrans.transform.rotation = PoseUtilities::fromEuler(0.0, 0.0, get_align_angle_ + leftorright_ * 0.5 * M_PI);
                }
                pose_end_ = PoseUtilities::applyTransform(relapos, subtrans);

                state_ = STA_TO_HORIZON;
                printf("STA_PRE_HORIZON finished, start STA_TO_HORIZON, pose_end_ is:[%f, %f]\n",
                    pose_end_.position.x, pose_end_.position.y);
            }
        }
        else if (state_ == STA_PRE_NEXT)
        {
            geometry_msgs::TransformStamped transBaselinkMap = listenTf(mapframe_.c_str(), base_link_frame_, ros::Time(0));

            if (target_rela_pose_vec_.empty())
            {
                ROS_INFO("---------- in STA_PRE_NEXT, pose registrated, curpose is:[%f, %f], YawImu is:%f",
                    transBaselinkMap.transform.translation.x, transBaselinkMap.transform.translation.y, YawImu);
                CmdVel.linear.x = 0.0;
                CmdVel.angular.z = 0.0;
                state_ = STA_DONE;
                return ;
            }
            
            auto front_target_rela = target_rela_pose_vec_.front();
            target_rela_pose_[0] = front_target_rela.target_rela_pose[0];
            target_rela_pose_[1] = front_target_rela.target_rela_pose[1];
            target_rela_pose_[2] = front_target_rela.target_rela_pose[2];  
            route_vertical_direct_ = front_target_rela.direction;     
            using_inertial_ = front_target_rela.using_inertial;      
            if (PoseUtilities::signOf(target_rela_pose_[1]) == 0 )
            {
                leftorright_ = 1;
            }
            else
            {
                leftorright_ = PoseUtilities::signOf(target_rela_pose_[1]);
            }            
            target_rela_pose_vec_.pop_front();

            double angleBaselink = PoseUtilities::toEuler(transBaselinkMap.transform.rotation)[2];
            pose_target_.orientation = PoseUtilities::fromEuler(0.0, 0.0, angleBaselink + leftorright_ * 0.5 * M_PI);
            angle_target_imu_ = YawImu + leftorright_ * 0.5 * M_PI;       
            angle_target_imu_ = getrightImu(angle_target_imu_);
            updateCurrentPose();

            distance_horizon_ = fabs(target_rela_pose_[1]);    // mark here by zhouyue
            distance_vertical_ = fabs(target_rela_pose_[0]);
            geometry_msgs::Pose relapos;
            relapos.position.x = target_rela_pose_[0];
            relapos.position.y = target_rela_pose_[1];
            relapos.orientation = PoseUtilities::fromEuler(0.0, 0.0, 0.0);

            geometry_msgs::Pose curpose;
            curpose.position.x = transBaselinkMap.transform.translation.x;
            curpose.position.y = transBaselinkMap.transform.translation.y;  
            geometry_msgs::Pose last_pose_end;
            last_pose_end = pose_end_;
            geometry_msgs::TransformStamped trans = transBaselinkMap;
            trans.transform.translation.x = pose_end_.position.x;
            trans.transform.translation.y = pose_end_.position.y;
            //trans.transform.rotation = getfea_cur_pose_.orientation;
            //getfea_cur_pose_.orientation.w = 1.0;
            pose_end_ = PoseUtilities::applyTransform(relapos, trans);

            printf("STA_PRE_NEXT curpose is: [%f, %f], angleBaselink:%f, get_align_imu_: %f\n",
                curpose.position.x, curpose.position.y, angleBaselink, get_align_imu_);   

            if (fabs(target_rela_pose_[0]) < 0.001 && fabs(target_rela_pose_[1]) < 0.001)
            {
                if (fabs(target_rela_pose_[2]) < 0.001)
                {
                    state_ = STA_PRE_NEXT;
                    printf("STA_PRE_NEXT, sta_done\n");
                }
                else
                {
                    state_ = STA_PRE_ORIENTATION;
                    printf("target_rela_pose_[2] > 0, start STA_PRE_ORIENTATION\n");
                }
            }
            else if (fabs(target_rela_pose_[1]) < 0.001)
            {
                relapos.position.x = (route_vertical_direct_ == "inverse")? (-1 * target_rela_pose_[0]) : target_rela_pose_[0];
                relapos.position.y = 0;
                relapos.orientation = PoseUtilities::fromEuler(0.0, 0.0, 0.0);
                geometry_msgs::TransformStamped subtrans = transBaselinkMap;
                subtrans.transform.translation.x = last_pose_end.position.x;
                subtrans.transform.translation.y = last_pose_end.position.y;
                if (is_fixed_location_)
                {
                    subtrans.transform.rotation = getfea_cur_pose_.orientation;
                }         
                else
                {
                    subtrans.transform.rotation = PoseUtilities::fromEuler(0.0, 0.0, get_align_angle_);
                }       
                pose_end_ = PoseUtilities::applyTransform(relapos, subtrans);
                vertical_start_pose_.position.x = transBaselinkMap.transform.translation.x;
                vertical_start_pose_.position.y = transBaselinkMap.transform.translation.y;
                prestate_ = state_;
                //state_ = STA_PRE_ROT_ROUTE_VERTICAL;
                if (!using_inertial_)
                {
                    state_ = STA_ROUTE_VERTICAL;
                    get_vertical_direct_imu_ = YawImu;
                    //updateCurrentPose(); 
                    printf("!using_inertial, direct to STA_ROUTE_VERTICAL, distance_vertical_ = %f\n",distance_vertical_);
                }           

            }
            else
            {
                state_ = STA_TO_HORIZON;
                printf("STA_PRE_NEXT finish, start STA_TO_HORIZON\n"); 
            }

        }        
        else if (state_ == STA_TO_HORIZON)
        {
            geometry_msgs::TransformStamped transBaselinkMap = listenTf(mapframe_.c_str(), base_link_frame_, ros::Time(0));
            double angleDiff = angles::shortest_angular_distance(
                PoseUtilities::toEuler(transBaselinkMap.transform.rotation)[2], PoseUtilities::toEuler(pose_target_.orientation)[2]);
            if (using_imu_)
            {
                angleDiff = angles::shortest_angular_distance(YawImu, angle_target_imu_);
            }
            if (fabs(angleDiff) < yaw_tolerance_)
            {
                CmdVel.linear.x = 0.0;
                CmdVel.angular.z = 0.0;

                prestate_ = state_;
                printf("sta_to_horizon finish, YawImu = %f\n", YawImu);
                if (!using_inertial_ || horizon_test_)
                {
                    state_ = STA_ROUTE_HORIZON;
                    printf("!using_inertial_, direct to STA_ROUTE_HORIZON\n");

                }
                updateCurrentPose(); 
                get_horizon_direct_imu_ = YawImu;
            }
            else
            {
                double near_tolerance_ = 0.0;
                near_tolerance_ = angles::from_degrees(15);
                if (fabs(angleDiff) < near_tolerance_)
                {
                    CmdVel.linear.x = 0.0;
                    CmdVel.angular.z = PoseUtilities::signOf(sin(angleDiff)) * 0.05;
                }
                else
                {
                    CmdVel.linear.x = 0.0;
                    CmdVel.angular.z = PoseUtilities::signOf(sin(angleDiff)) * rotvel_;
                }
                if (debug_count_ == 5)
                {
                    printf("angular.z: %f\n", CmdVel.angular.z);
                }
            }
        }
        else if (state_ == STA_ROUTE_HORIZON)
        {
            static std::string substate = "direct" ;

            if (!using_inertial_ || horizon_test_)
            {
                geometry_msgs::TransformStamped transBaselinkMap = listenTf(mapframe_.c_str(), base_link_frame_, ros::Time(0));
                geometry_msgs::Pose curpose;
                curpose.position.x = transBaselinkMap.transform.translation.x;
                curpose.position.y = transBaselinkMap.transform.translation.y;
                double routedis = 0.0;
                if (using_odom_pose_)
                {
                    routedis = PoseUtilities::distance(pose_odom_, pose_standby_odom_);
                }
                else
                {
                    routedis = PoseUtilities::distance(curpose,pose_standby_);
                }
                double extradis = fabs(distance_horizon_) + 0.1;
                double distDiff = fabs(distance_horizon_) - routedis;
                //行驶距离超出计算值 30cm ；偏差过大，说明前面对齐失败了
                if (routedis > extradis)
                {
                    CmdVel.linear.x = 0.0;
                    CmdVel.linear.y = 0.0;
                    state_ = STA_FAILED;
                    printf("fail at STA_ROUTE_HORIZON, routedis too far, align or rotate wrong\n");
                }
                else if (fabs(distDiff) < xy_tolerance_)
                {
                    CmdVel.linear.x = 0.0;
                    CmdVel.angular.z = 0.0;
                    state_ = STA_PRE_VERTICAL;
                    printf("STA_ROUTE_HORIZON finish, start STA_PRE_VERTICAL\n"); 
                }
                else
                {
                    CmdVel.linear.x = PoseUtilities::signOf(distDiff) * horizon_offset_vel_;
                    CmdVel.linear.y = 0.0;
                    auto imuangleDiff = angles::shortest_angular_distance(YawImu, get_horizon_direct_imu_);
                    //printf("imuangleDiff is: %f\n", imuangleDiff);
                    if ( fabs(imuangleDiff) > imu_adjust_rot_thresh_ )
                    {
                        CmdVel.angular.z = PoseUtilities::signOf(imuangleDiff) * imu_adjust_rot_vel_ ;
                    }
                    else
                    {
                        CmdVel.angular.z = 0 ;
                    }
                }
            }
        }
        else if (state_ == STA_PRE_VERTICAL)
        {
            geometry_msgs::TransformStamped transBaselinkMap = listenTf(mapframe_.c_str(), base_link_frame_, ros::Time(0));
            if (distance_vertical_ < 0.001 )        //   mark here by zhouyue
            {
                state_ = STA_PRE_NEXT;
                printf("after ROUTE_HORIZON, distance_vertical = 0, to STA_PRE_NEXT\n");
                return ;
            }

            double angleBaselink = PoseUtilities::toEuler(transBaselinkMap.transform.rotation)[2];
            pose_target_.position.x = 0.0;
            pose_target_.position.y = 0.0;
            pose_target_.orientation = PoseUtilities::fromEuler(0.0, 0.0, angleBaselink - leftorright_ * 0.5 * M_PI);
            //pose_target_.orientation = PoseUtilities::fromEuler(0.0, 0.0, get_align_angle_); 
            angle_target_imu_ = YawImu - leftorright_ * 0.5 * M_PI - angles::from_degrees(rot_offset_);
            angle_target_imu_ = getrightImu(angle_target_imu_);
            //angle_target_imu_ = getrightImu(get_align_imu_);
            printf("in state STA_PRE_VERTICAL finished, angle_target_imu_ = %f, YawImu = %f\n",
                angle_target_imu_, YawImu);
            //进入第二条路径，更改当前点为起始点
            updateCurrentPose();  

            printf("STA_PRE_VERTICAL finished, start STA_TO_VERTICAL, now curpose is: [%f, %f]\n",
                transBaselinkMap.transform.translation.x, transBaselinkMap.transform.translation.y); 
            state_ = STA_TO_VERTICAL;
        }
        else if (state_ == STA_TO_VERTICAL)
        {
            geometry_msgs::TransformStamped transBaselinkMap = listenTf(mapframe_.c_str(), base_link_frame_, ros::Time(0));
            double angleDiff = angles::shortest_angular_distance(
                PoseUtilities::toEuler(transBaselinkMap.transform.rotation)[2], PoseUtilities::toEuler(pose_target_.orientation)[2]);
            if (using_imu_)
            {
                angleDiff = angles::shortest_angular_distance(YawImu, angle_target_imu_);
            }

            if (fabs(angleDiff) < yaw_tolerance_)
            {
                CmdVel.linear.x = 0.0;
                CmdVel.angular.z = 0.0;
                prestate_ = state_;
                //state_ = STA_PRE_ROT_ROUTE_VERTICAL;
                if (!using_inertial_)
                {
                    state_ = STA_ROUTE_VERTICAL;
                    printf("STA_TO_VERTICAL finished, direct to STA_ROUTE_VERTICAL, now YawImu = %f\n", YawImu);
                    get_vertical_direct_imu_ = YawImu;
                }
                updateCurrentPose(); 
            }
            else
            {
                double near_tolerance_ = 0.0;
                near_tolerance_ = angles::from_degrees(15);
                if (fabs(angleDiff) < near_tolerance_)
                {
                    CmdVel.linear.x = 0.0;
                    CmdVel.angular.z = PoseUtilities::signOf(sin(angleDiff)) * 0.05;
                }
                else
                {
                    CmdVel.linear.x = 0.0;
                    CmdVel.angular.z = PoseUtilities::signOf(sin(angleDiff)) * vertical_to_rotvel_;
                }
                if (debug_count_ == 5)
                {
                    printf("angular.z: %f\n", CmdVel.angular.z);
                }                
            }
        }              
        else if (state_ == STA_ROUTE_VERTICAL)
        {
            static std::string substate = "direct" ;
            static int lastforward = 0; // 0: direct ,1:left , -1:right
            int direction;
            if (route_vertical_direct_ == "direct")
            {
                direction = 1;
            }
            else if (route_vertical_direct_ == "inverse")
            {
                direction = -1;
            }
            else
            {
                ROS_WARN_STREAM("config route_vertical_direct_ error, check config");
            }
            if (!using_inertial_)
            {
                // blind logic
                geometry_msgs::TransformStamped transBaselinkMap = listenTf(mapframe_.c_str(), base_link_frame_, ros::Time(0));
                geometry_msgs::Pose curpose;
                curpose.position.x = transBaselinkMap.transform.translation.x;
                curpose.position.y = transBaselinkMap.transform.translation.y;
                double routedis = 0.0;
                if (using_odom_pose_)
                {
                    routedis = PoseUtilities::distance(pose_odom_, pose_standby_odom_);
                }
                else
                {
                    routedis = PoseUtilities::distance(curpose,pose_standby_);
                }                
                double extradis = fabs(distance_vertical_) + 0.1;
                double distDiff = fabs(distance_vertical_) - routedis;
                //第二条路径行驶距离超出计算值 30cm ；偏差过大，说明前面有问题
                if (routedis > extradis)
                {
                    CmdVel.linear.x = 0.0;
                    CmdVel.linear.y = 0.0;
                    state_ = STA_FAILED;
                    printf("fail at STA_ROUTE_VERTICAL, routedis too far, route1 rotate wrong\n");
                }
                else if (fabs(distDiff) < xy_tolerance_)
                {
                    CmdVel.linear.x = 0.0;
                    CmdVel.angular.z = 0.0;
                    if(fabs(target_rela_pose_[2]) < 0.01)
                    {
                        state_ = STA_PRE_NEXT;
                        printf("STA_ROUTE_VERTICAL finished, to STA_PRE_NEXT\n");
                        printf("arrive curpose is: [%f, %f]\n", curpose.position.x, curpose.position.y);
                        if(debug_count_ > 0)
                        {
                            //state_ = STA_DEBUG;
                        }

                    }
                    else
                    {
                        state_ = STA_PRE_ORIENTATION;
                        printf("STA_ROUTE_VERTICAL finished, to sta STA_PRE_ORIENTATION\n");
                    }                
                }
                else
                {
                    CmdVel.linear.x = direction * PoseUtilities::signOf(distDiff) * PoseUtilities::signOf(distance_vertical_) * xyvel_;
                    CmdVel.linear.y = 0.0;

                    auto imuangleDiff = angles::shortest_angular_distance(YawImu, get_vertical_direct_imu_ + angles::from_degrees(rot_offset_) );
                    //printf("imuangleDiff is: %f\n", imuangleDiff);
                    if ( fabs(imuangleDiff) > imu_adjust_rot_thresh_ )
                    {
                        CmdVel.angular.z = PoseUtilities::signOf(imuangleDiff) * imu_adjust_rot_vel_ ;
                    }
                    else
                    {
                        CmdVel.angular.z = 0 ;
                    }                    
                }
            }
        }
        else if (state_ == STA_PRE_ORIENTATION)
        {
            auto rotangle = angles::from_degrees(fabs(target_rela_pose_[2]));
            geometry_msgs::TransformStamped transBaselinkMap = listenTf(mapframe_.c_str(), base_link_frame_, ros::Time(0));
            double angleBaselink = PoseUtilities::toEuler(transBaselinkMap.transform.rotation)[2];
            pose_target_.position.x = 0.0;
            pose_target_.position.y = 0.0;
            pose_target_.orientation = PoseUtilities::fromEuler(0.0, 0.0, angleBaselink + PoseUtilities::signOf(target_rela_pose_[2]) * rotangle); 

            angle_target_imu_ = YawImu + PoseUtilities::signOf(target_rela_pose_[2]) * rotangle;   
            angle_target_imu_ = getrightImu(angle_target_imu_);
            state_ = STA_TO_ORIENTATION;    
            printf("STA_PRE_ORIENTATION finished, to sta STA_TO_ORIENTATION, YawImu = %f, angle_target_imu_ = %f\n",
                YawImu, angle_target_imu_);       
        }
        else if (state_ == STA_TO_ORIENTATION)
        {
            geometry_msgs::TransformStamped transBaselinkMap = listenTf(mapframe_.c_str(), base_link_frame_, ros::Time(0));
            double angleDiff = angles::shortest_angular_distance(
                PoseUtilities::toEuler(transBaselinkMap.transform.rotation)[2], PoseUtilities::toEuler(pose_target_.orientation)[2]);
            if (using_imu_)
            {
                angleDiff = angles::shortest_angular_distance(YawImu, angle_target_imu_);
            }            
            if (fabs(angleDiff) < yaw_tolerance_)
            {
                CmdVel.linear.x = 0.0;
                CmdVel.angular.z = 0.0;
                state_ = STA_PRE_NEXT;
                printf("STA_TO_ORIENTATION finished, YawImu = %f, to STA_PRE_NEXT\n", YawImu);
            }
            else
            {
                double near_tolerance_ = 0.0;
                near_tolerance_ = angles::from_degrees(15);
                if (fabs(angleDiff) < near_tolerance_)
                {
                    CmdVel.linear.x = 0.0;
                    CmdVel.angular.z = PoseUtilities::signOf(sin(angleDiff)) * 0.05;
                }
                else
                {
                    CmdVel.linear.x = 0.0;
                    CmdVel.angular.z = PoseUtilities::signOf(sin(angleDiff)) * vertical_to_rotvel_;
                    prestate_ = state_;
                }
            }
        }
        else if (state_ == STA_WAIT_SCAN)
        {
            static auto preTime = ros::Time::now();
            auto timeNow = ros::Time::now();
            static int index = 0;

            if (index == 0)
            {
                preTime = timeNow;
                index = 1;
            }

            CmdVel.linear.x = 0.0;
            CmdVel.angular.z = 0.0;

            if ((timeNow - preTime).toSec() >= wait_scan_time_)
            {
                index = 0;
                if (prestate_ == STA_REGISTRATE || prestate_ == STA_ADJUST_VERTICAL || prestate_ == STA_ALIGN )
                {
                    state_ = STA_REGISTRATE_FINE;
                    printf("start STA_REGISTRATE_FINE\n");
                }
                else if (prestate_ == STA_START)
                {
                    state_ = STA_REGISTRATE;
                    printf("start STA_REGISTRATE\n");
                }
                else if (prestate_ == STA_ROT_VERTICAL)
                {
                    state_ = STA_ADJUST_REGIST;
                    printf("start STA_ADJUST_REGIST\n");
                }
            }
        }
        else if (state_ == STA_CHARGE_WALK)
        {
            geometry_msgs::TransformStamped transBaselinkMap = listenTf(mapframe_.c_str(), base_link_frame_, ros::Time(0));
            geometry_msgs::Pose curpose;
            curpose.position.x = transBaselinkMap.transform.translation.x;
            curpose.position.y = transBaselinkMap.transform.translation.y;
            double routedis = PoseUtilities::distance(curpose,pose_standby_);
            double distance_to = fabs(charge_walk_pose_[0]);
            double extradis = distance_to + 0.1;
            double distDiff = distance_to - routedis;
            //行驶距离超出计算值 10cm ；偏差过大，说明前面对齐失败了
            if (routedis > extradis)
            {
                CmdVel.linear.x = 0.0;
                CmdVel.linear.y = 0.0;
                state_ = STA_FAILED;
                printf("fail at STA_CHARGE_WALK, routedis too far, something wrong\n");
            }
            else if (fabs(distDiff) < xy_tolerance_)
            {
                CmdVel.linear.x = 0.0;
                CmdVel.angular.z = 0.0;
                state_ = STA_CHARGE_PRE_ROT;
                printf("STA_CHARGE_WALK finished, start STA_CHARGE_PRE_ROT\n"); 
            }
            else
            {
                CmdVel.linear.x = PoseUtilities::signOf(charge_walk_pose_[0]) * PoseUtilities::signOf(distDiff) * xyvel_;
                CmdVel.linear.y = 0.0;
            }
        }
        else if (state_ == STA_CHARGE_PRE_ROT)
        {
            if (fabs(charge_walk_pose_[1]) < 0.00001 )
            {
                state_ = STA_DONE;
                printf("arrive done, sta_done\n");
                return ;
            }

            geometry_msgs::TransformStamped transBaselinkMap = listenTf(mapframe_.c_str(), base_link_frame_, ros::Time(0));
            double angleBaselink = PoseUtilities::toEuler(transBaselinkMap.transform.rotation)[2];
            pose_target_.position.x = 0.0;
            pose_target_.position.y = 0.0;
            pose_target_.orientation = PoseUtilities::fromEuler(0.0, 0.0, angleBaselink + PoseUtilities::signOf(charge_walk_pose_[0]) * PoseUtilities::signOf(charge_walk_pose_[1]) * 0.5 * M_PI); 

            angle_target_imu_ = YawImu + PoseUtilities::signOf(charge_walk_pose_[0]) * PoseUtilities::signOf(charge_walk_pose_[1]) * 0.5 * M_PI;   
            angle_target_imu_ = getrightImu(angle_target_imu_);
            state_ = STA_CHARGE_ROT;    
            printf("STA_CHARGE_PRE_ROT finished, to sta STA_CHARGE_ROT, YawImu = %f, angle_target_imu_ = %f\n",
                YawImu, angle_target_imu_);       

        }
        else if (state_ == STA_CHARGE_ROT)
        {
            geometry_msgs::TransformStamped transBaselinkMap = listenTf(mapframe_.c_str(), base_link_frame_, ros::Time(0));
            double angleDiff = angles::shortest_angular_distance(
                PoseUtilities::toEuler(transBaselinkMap.transform.rotation)[2], PoseUtilities::toEuler(pose_target_.orientation)[2]);
            if (using_imu_)
            {
                angleDiff = angles::shortest_angular_distance(YawImu, angle_target_imu_);
            }
      
            if (fabs(angleDiff) < yaw_tolerance_)
            {
                CmdVel.linear.x = 0.0;
                CmdVel.angular.z = 0.0;
                state_ = STA_CHARGE_HORIZON;
                updateCurrentPose();
                printf("STA_CHARGE_ROT finished, YawImu = %f, to STA_CHARGE_HORIZON\n", YawImu);
            }
            else
            {
                double near_tolerance_ = 0.0;
                near_tolerance_ = angles::from_degrees(15);
                if (fabs(angleDiff) < near_tolerance_)
                {
                    CmdVel.linear.x = 0.0;
                    CmdVel.angular.z = PoseUtilities::signOf(sin(angleDiff)) * 0.05;
                }
                else
                {
                    CmdVel.linear.x = 0.0;
                    CmdVel.angular.z = PoseUtilities::signOf(sin(angleDiff)) * vertical_to_rotvel_;
                    prestate_ = state_;
                }
            }
        }
        else if (state_ == STA_CHARGE_HORIZON)
        {
            geometry_msgs::TransformStamped transBaselinkMap = listenTf(mapframe_.c_str(), base_link_frame_, ros::Time(0));
            geometry_msgs::Pose curpose;
            curpose.position.x = transBaselinkMap.transform.translation.x;
            curpose.position.y = transBaselinkMap.transform.translation.y;
            double routedis = PoseUtilities::distance(curpose,pose_standby_);
            double distance_to = fabs(charge_walk_pose_[1]);
            double extradis = distance_to + 0.1;
            double distDiff = distance_to - routedis;
            //行驶距离超出计算值 10cm ；偏差过大，说明前面对齐失败了
            if (routedis > extradis)
            {
                CmdVel.linear.x = 0.0;
                CmdVel.linear.y = 0.0;
                state_ = STA_FAILED;
                printf("fail at STA_CHARGE_HORIZON, routedis too far, something wrong\n");
            }
            else if (fabs(distDiff) < xy_tolerance_)
            {
                CmdVel.linear.x = 0.0;
                CmdVel.angular.z = 0.0;
                state_ = STA_DONE;
                printf("STA_CHARGE_HORIZON finished, start STA_DONE\n"); 
            }
            else
            {
                CmdVel.linear.x = PoseUtilities::signOf(distDiff) * xyvel_;
                CmdVel.linear.y = 0.0;
            }
        }
        // else
        // {
        //     CmdVel.linear.x = 0.0;
        //     CmdVel.linear.y = 0.0;
        //     CmdVel.angular.z = 0.0;
        // }

        if (debug_count_ == 2)
        {
            CmdVel.linear.x = 0.0;
            CmdVel.linear.y = 0.0;
            CmdVel.angular.z = 0.0;
            return ;
        }
    }

    void LocatePoseRegistration::stateOffset(geometry_msgs::Twist& CmdVel, double YawImu)
    {
        if (state_ == STA_OFFSET_YAW)
        {
            updatePre(YawImu);

            prestate_ = state_;
            if (fabs(offsets_[2]) > 1e-3)
            {
                CmdVel.linear.x = 0.0;
                CmdVel.angular.z = PoseUtilities::signOf(offsets_[2]) * rotvel_;

                state_ = STA_OFFSET_ROTATING;
            }
            else
            {
                state_ = STA_OFFSET_Y;
            }
        }
        else if (state_ == STA_OFFSET_ROTATING)
        {
            double diff2Pre = 0.0;
            if (using_imu_)
            {
                diff2Pre = angles::shortest_angular_distance(YawImu, yaw_pre_);
            }
            else
            {
                if (using_odom_pose_)
                {
                    diff2Pre = angles::shortest_angular_distance(
                        PoseUtilities::toEuler(pose_odom_.orientation)[2], yaw_pre_);
                }
                else
                {
                    geometry_msgs::TransformStamped baseInMap = listenTf(mapframe_, base_link_frame_, ros::Time(0));
                    diff2Pre = angles::shortest_angular_distance(
                        PoseUtilities::toEuler(baseInMap.transform.rotation)[2], yaw_pre_);
                }
            }

            double angleDiff = prestate_ == STA_OFFSET_YAW ? fabs(offsets_[2]) : zig_angle_;

            if (fabs(angleDiff - fabs(diff2Pre)) < yaw_tolerance_)
            {
                CmdVel.linear.x = 0.0;
                CmdVel.angular.z = 0.0;

                if (prestate_ == STA_OFFSET_YAW)
                {
                    prestate_ = state_;
                    state_ = STA_OFFSET_Y;
                }
                else if (prestate_ == STA_OFFSET_Y)
                {
                    prestate_ = state_;
                    state_ = STA_OFFSET_FORWARD;
                }
                else if (prestate_ == STA_OFFSET_MOVING)
                {
                    prestate_ = state_;
                    state_ = STA_OFFSET_X;
                }
                else
                {
                    // undefined
                    state_ = STA_FAILED;
                }
            }
        }
        else if (state_ == STA_OFFSET_Y)
        {
            prestate_ = state_;

            if (fabs(offsets_[1]) > 1e-3)
            {
                updatePre(YawImu);

                CmdVel.linear.x = 0.0;
                CmdVel.angular.z = PoseUtilities::signOf(offsets_[1]) * rotvel_;

                state_ = STA_OFFSET_ROTATING;
            }
            else
            {
                state_ = STA_OFFSET_X;
            }
        }
        else if (state_ == STA_OFFSET_FORWARD)
        {
            updatePre(YawImu);

            distance_todrive_ = fabs(offsets_[1] / sin(zig_angle_));
            CmdVel.linear.x = xyvel_;
            CmdVel.angular.z = 0.0;

            prestate_ = state_;
            state_ = STA_OFFSET_MOVING;
        }
        else if (state_ == STA_OFFSET_MOVING)
        {
            double diff2Pre = 0.0;
            if (using_odom_pose_)
            {
                diff2Pre = PoseUtilities::distance(pose_odom_, pose_pre_);
            }
            else
            {
                geometry_msgs::TransformStamped baseInMap = listenTf(mapframe_, base_link_frame_, ros::Time(0));
                diff2Pre = PoseUtilities::distance(PoseUtilities::convert(baseInMap), pose_pre_);
            }

            if (fabs(diff2Pre - distance_todrive_) < xy_tolerance_)
            {
                CmdVel.linear.x = 0.0;
                CmdVel.angular.z = 0.0;

                if (prestate_ == STA_OFFSET_FORWARD)
                {
                    updatePre(YawImu);
                    prestate_ = state_;

                    CmdVel.linear.x = 0.0;
                    CmdVel.angular.z = -PoseUtilities::signOf(offsets_[1]) * rotvel_;

                    state_ = STA_OFFSET_ROTATING;
                }
                else if (prestate_ == STA_OFFSET_X)
                {
                    state_ = STA_DONE;
                }
                else
                {
                    // undefined
                    state_ = STA_FAILED;
                }
            }
        }
        else if (state_ == STA_OFFSET_X)
        {
            auto moveOffsetX = [&]()
            {
                if (fabs(offsets_[0]) > 1e-3)
                {
                    updatePre(YawImu);

                    distance_todrive_ = fabs(offsets_[0]);
                    CmdVel.linear.x = PoseUtilities::signOf(offsets_[0]) * xyvel_;
                    CmdVel.angular.z = 0.0;

                    prestate_ = state_;
                    state_ = STA_OFFSET_MOVING;
                }
                else
                {
                    state_ = STA_DONE;
                }
            };

            if (prestate_ == STA_OFFSET_ROTATING)
            {
                if (fabs(offsets_[1]) > 1e-3)
                {
                    updatePre(YawImu);

                    double dist = offsets_[0] - fabs(offsets_[1] / tan(zig_angle_));
                    distance_todrive_ = fabs(dist);
                    CmdVel.linear.x = PoseUtilities::signOf(dist) * xyvel_;
                    CmdVel.angular.z = 0.0;

                    prestate_ = state_;
                    state_ = STA_OFFSET_MOVING;
                }
                else
                {
                    moveOffsetX();
                }
            }
            else
            {
                moveOffsetX();
            }
        }
        // else
        // {
        //     // undefined
        //     CmdVel.linear.x = 0.0;
        //     CmdVel.angular.z = 0.0;

        //     state_ = STA_FAILED;
        // }
    }

    void LocatePoseRegistration::stateStart(geometry_msgs::Twist& CmdVel, double YawImu)
    {
        if (state_ == STA_START_BACK)
        {
            geometry_msgs::TransformStamped transBaselinkMap = listenTf(mapframe_.c_str(), base_link_frame_, ros::Time(0));
            geometry_msgs::Pose curpose;
            curpose.position.x = transBaselinkMap.transform.translation.x;
            curpose.position.y = transBaselinkMap.transform.translation.y;
            double routedis = PoseUtilities::distance(curpose,pose_standby_);
            double distance_to = fabs(charge_walk_pose_[0]);
            double extradis = distance_to + 0.1;
            double distDiff = distance_to - routedis;
            //行驶距离超出计算值 10cm ；偏差过大，说明前面对齐失败了
            if (routedis > extradis)
            {
                CmdVel.linear.x = 0.0;
                CmdVel.linear.y = 0.0;
                state_ = STA_FAILED;
                printf("fail at STA_START_BACK, routedis too far, something wrong\n");
            }
            else if (fabs(distDiff) < xy_tolerance_)
            {
                CmdVel.linear.x = 0.0;
                CmdVel.angular.z = 0.0;
                state_ = STA_START_PRE_ROT;
                printf("STA_START_BACK finished, start STA_START_PRE_ROT\n"); 
            }
            else
            {
                CmdVel.linear.x = PoseUtilities::signOf(charge_walk_pose_[0]) * PoseUtilities::signOf(distDiff) * xyvel_;
                CmdVel.linear.y = 0.0;
            }
        }
        else if (state_ == STA_START_PRE_ROT)
        {
            geometry_msgs::TransformStamped transBaselinkMap = listenTf(mapframe_.c_str(), base_link_frame_, ros::Time(0));
            double angleBaselink = PoseUtilities::toEuler(transBaselinkMap.transform.rotation)[2];
            pose_target_.position.x = 0.0;
            pose_target_.position.y = 0.0;
            pose_target_.orientation = PoseUtilities::fromEuler(0.0, 0.0, angleBaselink + M_PI); 

            angle_target_imu_ = YawImu + M_PI;   
            angle_target_imu_ = getrightImu(angle_target_imu_);
            state_ = STA_START_ROT;    
            printf("STA_START_PRE_ROT finished, to sta STA_START_ROT, YawImu = %f, angle_target_imu_ = %f\n",
                YawImu, angle_target_imu_);       

        }
        else if (state_ == STA_START_ROT)
        {
            geometry_msgs::TransformStamped transBaselinkMap = listenTf(mapframe_.c_str(), base_link_frame_, ros::Time(0));
            double angleDiff = angles::shortest_angular_distance(
                PoseUtilities::toEuler(transBaselinkMap.transform.rotation)[2], PoseUtilities::toEuler(pose_target_.orientation)[2]);
            if (using_imu_)
            {
                angleDiff = angles::shortest_angular_distance(YawImu, angle_target_imu_);
            }
      
            if (fabs(angleDiff) < yaw_tolerance_)
            {
                CmdVel.linear.x = 0.0;
                CmdVel.angular.z = 0.0;
                state_ = STA_DONE;
                updateCurrentPose();
                printf("STA_START_ROT finished, YawImu = %f, have done STA_DONE\n", YawImu);
            }
            else
            {
                double near_tolerance_ = 0.0;
                near_tolerance_ = angles::from_degrees(5);
                if (fabs(angleDiff) < near_tolerance_)
                {
                    CmdVel.linear.x = 0.0;
                    CmdVel.angular.z = PoseUtilities::signOf(sin(angleDiff)) * 0.05;
                }
                else
                {
                    CmdVel.linear.x = 0.0;
                    CmdVel.angular.z = PoseUtilities::signOf(sin(angleDiff)) * vertical_to_rotvel_;
                    prestate_ = state_;
                }
            }
        }


    }

    bool LocatePoseRegistration::checkCurpointNearCharge()
    {
        bool isNearCharge = false;
        geometry_msgs::TransformStamped transBaselinkMap = listenTf(mapframe_.c_str(), base_link_frame_, ros::Time(0));
        geometry_msgs::Pose curpose;
        curpose.position.x = transBaselinkMap.transform.translation.x;
        curpose.position.y = transBaselinkMap.transform.translation.y;

        geometry_msgs::Pose charge_point_pose;
        charge_point_pose.position.x = charge_point_[0];
        charge_point_pose.position.y = charge_point_[1];
        double posedis = PoseUtilities::distance(curpose,charge_point_pose);
        ROS_INFO("curpose is : [%f,%f] , dis to chargepoint is:%f",curpose.position.x,curpose.position.y,posedis);
        if (posedis < distance_charge_limit_)
        {
            ROS_INFO("posedis < distance_charge_limit_ ,need back");
            isNearCharge = true;
        }
        else
        {
            ROS_INFO("posedis > distance_charge_limit_ ,don't need back");
        }
        if (!need_charge_near_)
        {
            ROS_INFO("!need_charge_near ,set false");
            isNearCharge = false;
        }
        return isNearCharge;

    }


    bool LocatePoseRegistration::checkcurpose()
    {
        geometry_msgs::TransformStamped transBaselinkMap = listenTf(mapframe_.c_str(), base_link_frame_, ros::Time(0));
        geometry_msgs::Pose curpose;
        curpose.position.x = transBaselinkMap.transform.translation.x;
        curpose.position.y = transBaselinkMap.transform.translation.y;

        double mindist = std::numeric_limits<double>::max();
        FeatureConfig getfeature;
        for (auto oneiter = features_config_.begin();oneiter != features_config_.end(); oneiter++)
        {   
            geometry_msgs::Pose fea_cur_pose;
            fea_cur_pose.position.x = (*oneiter).cur_pose[0];
            fea_cur_pose.position.y = (*oneiter).cur_pose[1];
            double posedis = PoseUtilities::distance(curpose,fea_cur_pose);
            if (posedis < mindist)
            {
                mindist = posedis;
                getfeature = *oneiter;
            }
        }

        printf("getfeature, name is %s\n" , getfeature.name.c_str());
        geometry_msgs::Pose getfea_cur_pose;
        getfea_cur_pose.position.x = getfeature.cur_pose[0];
        getfea_cur_pose.position.y = getfeature.cur_pose[1];
        double posedis = PoseUtilities::distance(curpose,getfea_cur_pose);
        printf("curpose: [%f, %f], pose dis is %f\n", curpose.position.x, curpose.position.y, posedis);
        if (posedis > pattern_met_location_thresh_)
        {
            return false;
        }
        else
        {
            pose_feature_.position.x = getfeature.feature_pose[0];
            pose_feature_.position.y = getfeature.feature_pose[1];
            target_rela_pose_.resize(3);
            target_rela_pose_vec_.assign(getfeature.target_rela_pose_vec.begin(), getfeature.target_rela_pose_vec.end());
            getfea_cur_pose_.position.x = getfeature.cur_pose[0];
            getfea_cur_pose_.position.y = getfeature.cur_pose[1];
            getfea_cur_pose_.orientation = PoseUtilities::fromEuler(0.0, 0.0, getfeature.cur_pose[2]);
            printf("getfea_cur_pose_.orientation yaw: %f\n", getfeature.cur_pose[2]);

            std::string model_cloud_file;
            model_cloud_file = model_cloud_path_ + "/" + getfeature.name + ".pcd";
            target_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
            if (pcl::io::loadPCDFile<pcl::PointXYZ>(model_cloud_file.c_str(), *target_cloud_) == -1)
            {
                PCL_ERROR("Can not find the file model_cloud ");
                return false;
            }
            ROS_INFO("loaded: %d from the model_cloud file", target_cloud_->size());

            return true;
        }
    }

    void LocatePoseRegistration::updateCurrentPose()
    {
        geometry_msgs::TransformStamped transBaselinkMap = listenTf(mapframe_.c_str(), base_link_frame_, ros::Time(0));
        pose_standby_.position.x = transBaselinkMap.transform.translation.x;    
        pose_standby_.position.y = transBaselinkMap.transform.translation.y; 

        pose_standby_odom_.position.x = pose_odom_.position.x;
        pose_standby_odom_.position.y = pose_odom_.position.y;
    }

    void LocatePoseRegistration::subCallbackLaserScan(const sensor_msgs::LaserScan::ConstPtr& Laser)
    {
        if (queue_scan_)
        {
            queue_scan_->produce(std::bind(&LocatePoseRegistration::registration, this,
                Laser), "registration");
        }

        if (state_ == STA_REGISTRATE || state_ == STA_REGISTRATE_FINE)
        {
            std::lock_guard<std::mutex> lk(mtx_cv_);
            cv_.notify_all();

            prestate_ = state_;
            state_ = STA_REGISTRATING;
        }
    }

    void LocatePoseRegistration::subCallbackImu(const sensor_msgs::Imu::ConstPtr& Imudata)
    {
        const std::lock_guard<std::mutex> lock(mtx_imu_);
        tf2::Quaternion quaternion(Imudata->orientation.x, Imudata->orientation.y, Imudata->orientation.z,
            Imudata->orientation.w);
        angleyaw_imu_ = PoseUtilities::toEuler(quaternion)[2];
    }

    void LocatePoseRegistration::subCallbackOdom(const nav_msgs::Odometry::ConstPtr& Odom)
    {
        pose_odom_.position.x = Odom->pose.pose.position.x;
        pose_odom_.position.y = Odom->pose.pose.position.y;
    }

    double LocatePoseRegistration::getrightImu(double angletar)
    {
        double rightangle = 0.0;
        if (angletar < -M_PI)
        {
            rightangle = angletar + 2 * M_PI;
        }
        else if (angletar > M_PI)
        {
            rightangle = angletar - 2 * M_PI;
        }
        else 
        {
            rightangle = angletar;
        }
        return rightangle;
    }

    using PclPoints = std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>>;
    // Function to calculate the RMSE (Root Mean Squared Error)
    static double calculateRMSE(const PclPoints& Points, double Slope, double Intercept)
    {
        double errorSum = 0;
        for (const auto& it : Points)
        {
            double predictedY = Slope * it.x + Intercept;
            double error = it.y - predictedY;
            errorSum += error * error;
        }

        return std::sqrt(errorSum / Points.size());
    }

    // Function to calculate the slope (m) and intercept (b) of the best-fit line
    static double linearRegression(const PclPoints& Points, double& Slope, double& Intercept)
    {
        if (Points.empty())
        {
            std::cerr << "Error: Data sets must have the same number of points and cannot be empty." << std::endl;
            return 1.0;
        }

        double sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
        for (const auto& it : Points)
        {
            sumX += it.x;
            sumY += it.y;
            sumXY += it.x * it.y;
            sumX2 += it.x * it.x;
        }

        double denominator = Points.size() * sumX2 - sumX * sumX;
        if (denominator == 0)
        {
            std::cerr << "Error: Denominator is zero, cannot compute linear regression." << std::endl;
            return 1.0;
        }

        Slope = (Points.size() * sumXY - sumX * sumY) / denominator;
        Intercept = (sumY * sumX2 - sumX * sumXY) / denominator;

        return calculateRMSE(Points, Slope, Intercept);
    }

    std::shared_ptr<void> LocatePoseRegistration::registration(const sensor_msgs::LaserScan::ConstPtr& Laser)
    {
        ros::Time begin = ros::Time::now();

        /// convert to pcl cloud
        auto pclCloud = PclUtilities<pcl::PointXYZ>::fromMsgLaserScan(*Laser);
        operate_index_++;
        ROS_INFO("---------- registrating operate_index = %d", operate_index_);
        printf("read %d from the laserscan\n", pclCloud->size());
        
        /// transform the laser according to user specified frame
        // get user define frame
        geometry_msgs::TransformStamped lasertransform;
        lasertransform.header.frame_id = laser_frame_;
        lasertransform.child_frame_id = laser_frame_;
        lasertransform.header.stamp = ros::Time::now();
        lasertransform.transform.translation.x = laser_pose_[0];
        lasertransform.transform.translation.y = laser_pose_[1];
        lasertransform.transform.translation.z = laser_pose_[2];
        tf2::Quaternion q;
        q.setRPY(angles::from_degrees(laser_pose_[3]),
            angles::from_degrees(laser_pose_[4]), angles::from_degrees(laser_pose_[5]));  
        lasertransform.transform.rotation.x = q.x();
        lasertransform.transform.rotation.y = q.y();
        lasertransform.transform.rotation.z = q.z();
        lasertransform.transform.rotation.w = q.w(); 
        // transform
        for (int i= 0; i < pclCloud->points.size(); i++)
        {
            geometry_msgs::Pose pointpose,aftertrans_pose;
            pointpose.position.x = pclCloud->points[i].x;
            pointpose.position.y = pclCloud->points[i].y;
            pointpose.position.z = pclCloud->points[i].z;
            pointpose.orientation = PoseUtilities::fromEuler(0.0, 0.0, 0.0);
            aftertrans_pose = PoseUtilities::applyTransform(pointpose, lasertransform);
            pclCloud->points[i].x = aftertrans_pose.position.x;
            pclCloud->points[i].y = aftertrans_pose.position.y;
            pclCloud->points[i].z = aftertrans_pose.position.z;
        }

        /// segment don
        pcl::PointCloud<pcl::PointXYZ>::Ptr outcloud(new pcl::PointCloud<pcl::PointXYZ>) ;
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> outcloudvec;
        printf("start segment_don\n");
        PclUtilities<pcl::PointXYZ>::segment_don(pclCloud, target_cloud_, 
            outcloudvec, seg_scale1_, seg_scale2_, seg_threshold_, seg_radius_, ndtsample_coeffs_, ndtmaxiter_);
        printf("finished segment_don\n");
        if (outcloudvec.size() == 0)
        {
            state_ = STA_FAILED;
            std::string outfile;
            outfile = packpath_ + "/pcd/testgetall.pcd";
            pcl::io::savePCDFileASCII (outfile, *pclCloud);                 
            ROS_WARN("segment_don, get none features, please check config or condition");
            return nullptr;
        }

        //--------------------从几个可能的特征找出最近的-----------
        // filter the target out within multiple patterns
        // comparation in laser frame
        geometry_msgs::Pose originpose;
        originpose.position.x = 0;
        originpose.position.y = 0;
        double center_dist_min = std::numeric_limits<double>::max();
        for (auto& oneiter : outcloudvec)
        {          
            double k, b, lineReg;
            lineReg = 1.0;
            lineReg = linearRegression(oneiter->points, k, b);
            printf("lineReg = %f\n", lineReg);
            if (lineReg < line_reg_thresh_ )
            {
                printf("lineReg < lineRegThresh, lineReg = %f\n", lineReg);
                continue;
            }
            std::vector<mypoint> pvec;
            for (int i = 0; i < oneiter->points.size(); ++i)
            {
                mypoint onepoint;
                onepoint.x = oneiter->points[i].x;
                onepoint.y = oneiter->points[i].y;
                pvec.push_back(onepoint);
            }
            double minradius;
            mypoint center;
            min_circle_cover(pvec, minradius, center);
            geometry_msgs::Pose centerpoint;
            centerpoint.position.x = center.x;
            centerpoint.position.y = center.y;
            double center_dist = PoseUtilities::distance(originpose,centerpoint);
            double dis_from_feature = fabs(center_dist - pose_feature_.position.x);  // 应该是最接近标志物的距离,从配置中读取的标志物距离
            if(dis_from_feature < center_dist_min)
            {
                center_dist_min = dis_from_feature;
                *outcloud = *oneiter ;
            }
        }
        
        if (outcloud->points.empty())
        {
            state_ = STA_FAILED;               
            printf("outcloud->points.size() <= 0\n");
            return nullptr;
        }

        //---------------------- 求最小包围圆 ----------------
        // flitered target
        std::vector<mypoint> pvec;
        for (int i = 0; i < outcloud->points.size(); ++i)
        {
            mypoint onepoint;
            onepoint.x = outcloud->points[i].x;
            onepoint.y = outcloud->points[i].y;
            pvec.push_back(onepoint);
        }
        double minradius;
        mypoint center;
        min_circle_cover(pvec, minradius, center);
        printf("outcloud, radius: %.10lf\n center.x: %.10lf center.y: %.10lf\n", minradius, center.x, center.y);
        minradius = minradius + 0.030 ; 

        // 求模板点云的最小包围圆半径，作比较
        std::vector<mypoint> tarpvec;
        for (int i = 0; i < target_cloud_->points.size(); ++i)
        {
            mypoint onepoint;
            onepoint.x = target_cloud_->points[i].x;
            onepoint.y = target_cloud_->points[i].y;
            tarpvec.push_back(onepoint);
        }
        double tarminradius;
        mypoint tarcenter;
        min_circle_cover(tarpvec, tarminradius, tarcenter);
        printf("targetcloud, radius: %.10lf\n  center.x: %.10lf center.y: %.10lf\n", tarminradius, tarcenter.x, tarcenter.y);
        
        // check if the filtered target meet the template pattern
        double delta_radius = minradius - tarminradius;
        printf("delta_radius is: %f\n", fabs(delta_radius));
        if (fabs(delta_radius) > pattern_met_radius_thresh_)
        {
            printf("outcloud radius is far from target radius, maybe wrong\n");
            std::string outfile;
            outfile = packpath_ + "/pcd/testgetall.pcd";
            pcl::io::savePCDFileASCII (outfile, *pclCloud);                     
            state_ = STA_FAILED;
            return nullptr;
        }

        //--------------------- 在原点云上 mincut ------
        printf("start segmentMinCut");
        std::vector<pcl::PointIndices> minclusterIndices;
        pcl::PointXYZ minpointCenter;
        minpointCenter.x = center.x;
        minpointCenter.y = center.y;
        minpointCenter.z = 0;

        minclusterIndices = PclUtilities<pcl::PointXYZ>::segmentMinCut(pclCloud, minpointCenter,
            minradius, cut_min_neighbour_, sigma_, weight_);
        printf("total minclusters number from epic: %d\n", minclusterIndices.size()) ;
        for (int i = 0; i < minclusterIndices.size(); ++i)
        {
            printf("mincluster %d has points %d\n" , i, minclusterIndices[i].indices.size());
        }
        int ci = 0;
        pcl::PointCloud<pcl::PointXYZ>::Ptr minoutcloud(new pcl::PointCloud<pcl::PointXYZ>) ;
        for (const auto& cluster : minclusterIndices)
        {
            ci++;
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFeature(new pcl::PointCloud<pcl::PointXYZ>());
            PclUtilities<pcl::PointXYZ>::extractTo(pclCloud, cluster, cloudFeature);
            printf("extractTo cluster indices = %d, cloudFeature->size() = %d\n", ci,cloudFeature->size());

            if (!cloudFeature->empty() && cloudFeature->size() < mincut_size_)
            {
                *minoutcloud = *cloudFeature;
            }
        }
        if (minoutcloud->points.empty())
        {
            state_ = STA_FAILED;
            ROS_WARN("segmentMinCut fail");
            return nullptr;
        }            
        printf("finish segmentMinCut, get outcloud\n");
        static PclVisualize<pcl::PointXYZ> viewer;
        if (state_ == STA_DEBUG)
        {
            printf("add cur cloud while registration done\n");
            
            viewer.addviewCloud(minoutcloud, "after_cur_features");
            state_ = STA_DONE;
            return nullptr;
        }

        printf("after segment, start registration\n");
        /// registration
        // 对点云进行平移, 激光中心和车体旋转中心
        for (int i = 0; i < minoutcloud->points.size(); ++i)
        {
            minoutcloud->points[i].x = minoutcloud->points[i].x - lazer_motor_diff_;
        }
        Eigen::Vector3f registAngles;
        double score;
        std::vector<double> transxy;
        Eigen::Matrix4f outmatrix;
        if (!PclUtilities<pcl::PointXYZ>::regist_sacia_ndt(target_cloud_, minoutcloud,
            registAngles, transxy, score, ndtsample_coeffs_, ndtmaxiter_, outmatrix, true))
        {
            state_ = STA_FAILED;
            ROS_WARN("registration failed");
            return nullptr;
        }
        
        if (debug_visualize_)
        {
            for (int i = 0; i < minoutcloud->points.size(); ++i)
            {
                minoutcloud->points[i].x = minoutcloud->points[i].x + lazer_motor_diff_;
            }
            pcl::PointCloud<pcl::PointXYZ>::Ptr transcloud(new pcl::PointCloud<pcl::PointXYZ>) ;
            pcl::transformPointCloud(*minoutcloud, *transcloud, outmatrix);
            pcl::PointCloud<pcl::PointXYZ>::Ptr targetoffset(new pcl::PointCloud<pcl::PointXYZ>);
            *targetoffset = *target_cloud_;
            for (int i = 0; i < targetoffset->points.size(); ++i)
            {
                targetoffset->points[i].x = targetoffset->points[i].x + lazer_motor_diff_;
            }            
            viewer.viewCloud02(minoutcloud, "cur_features",
                transcloud, "debug_features", target_cloud_, "target_features", targetoffset, "target_offset");
        }
        if (debug_count_ == 1 || debug_count_ == 3)
        {
            printf("debug_count_ == 1 or 3, outfile\n");
            std::string outfile;
            outfile = packpath_ + "/pcd/testgetseg.pcd";
            pcl::io::savePCDFileASCII (outfile, *minoutcloud);
            outfile = packpath_ + "/pcd/testgetall.pcd";
            pcl::io::savePCDFileASCII (outfile, *pclCloud); 

            if (debug_count_ == 1)
            {
                state_ = STA_DONE;
                return nullptr;
            }
        }

        if (registAngles[2] > 0.5 * M_PI)
        {
            state_ = STA_FAILED;
            printf("registangle > 0.5 pi, it is wrong, registration fail\n");
            return nullptr;
        }
        
        // 转换矩阵正常的情况
        geometry_msgs::TransformStamped transBaselinkMap = listenTf(mapframe_.c_str(), base_link_frame_, ros::Time(0));
        if (fabs(registAngles[0]) < 0.02 && fabs(registAngles[1]) < 0.02)
        {
            // yaw aligned, move to done or linear alignment if there is
            double transxyreal = transxy[0] - lazer_motor_diff_;
            printf("transxyreal is: %f\n", transxyreal);
            if ( (fabs(transxyreal) < regist_linear_x_thresh_ && fabs(transxy[1]) < regist_linear_y_thresh_)  &&
                fabs(registAngles[2]) < regist_yaw_thresh_)
            {
                printf("align right, next state\n");
                printf("to STA_PRE_HORIZON\n");
                state_ = STA_PRE_HORIZON;

                // add here by zhouyue for adjust last rot
                /*
                printf("first to STA_ADJUST_LAST_ROT\n");
                double angleBaselink = PoseUtilities::toEuler(transBaselinkMap.transform.rotation)[2];
                pose_target_.orientation = PoseUtilities::fromEuler(0.0, 0.0,
                angles::shortest_angular_distance(registAngles[2], angleBaselink));
                double yawFromImu = 0.0;
                {
                    const std::lock_guard<std::mutex> lock(mtx_imu_);
                    yawFromImu = angleyaw_imu_;
                }
                angle_target_imu_ = angles::shortest_angular_distance(registAngles[2], yawFromImu);
                state_ = STA_ADJUST_LAST_ROT;
                */
                return nullptr;
            }

            if (fabs(transxyreal) < 1.8 * regist_linear_x_thresh_ || fabs(transxy[1]) < 1.8 * regist_linear_y_thresh_ || fabs(registAngles[2]) < 1.8 * regist_yaw_thresh_ )
            {
                is_fine_tune_ = true;
            }
            else
            {
                is_fine_tune_ = false;
            }
            registAngle_ = registAngles[2];
            // otherwise, rotate to align
            printf("%s\n", fabs(registAngles[2]) < regist_yaw_thresh_ ? 
                std::string("algin angle met").c_str() : std::string("algin angle unmet").c_str());

            double angleBaselink = PoseUtilities::toEuler(transBaselinkMap.transform.rotation)[2];
            // pose_target_.orientation = PoseUtilities::fromEuler(0.0, 0.0, angleBaselink - registAngles[2]);
            pose_target_.orientation = PoseUtilities::fromEuler(0.0, 0.0,
                angles::shortest_angular_distance(registAngles[2], angleBaselink));
            double yawFromImu = 0.0;
            {
                const std::lock_guard<std::mutex> lock(mtx_imu_);
                yawFromImu = angleyaw_imu_;
            }
            angle_target_imu_ = angles::shortest_angular_distance(registAngles[2], yawFromImu);
            printf("registration , angle_target_imu_ = %f, now yawFromImu = %f\n",
                angle_target_imu_, yawFromImu);

            angleDiff_g_ = angles::shortest_angular_distance(yawFromImu, angle_target_imu_);
            distance_vertical_ = transxy[0] - lazer_motor_diff_;
            distance_horizon_ = transxy[1];
            printf("distance_vertical = %f, distance_horizon_ = %f\n", distance_vertical_, distance_horizon_);

            angleBaselink = PoseUtilities::toEuler(transBaselinkMap.transform.rotation)[2];
            updateCurrentPose(); 
            pose_standby_.orientation = PoseUtilities::fromEuler(0.0, 0.0, angleBaselink);

            // new add , determine the process here 
            if (fabs(registAngles[2]) < regist_yaw_thresh_)
            {
                if (fabs(distance_horizon_) < distthresh_horizon_)
                {
                    distance_todrive_ = distance_vertical_;
                    state_ = STA_ADJUST_VERTICAL;

                    printf("after registration, STA_ALIGN met, direct to STA_ADJUST_VERTICAL, distance_todrive_=%f\n", distance_todrive_);
                }                    
                else
                {
                    state_ = STA_PRE_ROT_ANGLE;
                    printf("after registration, STA_ALIGN met, start STA_PRE_ROT_ANGLE\n");
                }

                return nullptr;
            }

            state_ = STA_ALIGN;
        }
        else
        {
            // 转换矩阵奇异的情况：
            // 暂时设为失败，后面再改

            state_ = STA_FAILED;
            ROS_ERROR("process failed, registration failed");
        }

        printf("processing time: %f\n", (ros::Time::now() - begin).toSec());

        return nullptr;
    }

    void LocatePoseRegistration::threadRegistration()
    {
        th_registration_ = std::thread
        {
            [this]() -> void
            {
                while (!terminated_.load())
                {
                    {
                        std::unique_lock<std::mutex> lk(mtx_cv_);
                        cv_.wait(lk);
                    }

                    EventQueue<void>::EventFunc eventRegistration(queue_scan_->consume("registration", true));
                    if (eventRegistration)
                    {
                        // invoke the event means executing the action binded with it
                        eventRegistration();
                    }
                }
            }
        };
    }

    void LocatePoseRegistration::updatePre(double YawImu)
    {
        // pose
        if (using_odom_pose_)
        {
            pose_pre_ = pose_odom_;
        }
        else
        {
            geometry_msgs::TransformStamped baseInMap = listenTf(mapframe_, base_link_frame_, ros::Time(0));
            pose_pre_ = PoseUtilities::convert(baseInMap);
        }
        
        // yaw
        if (using_imu_)
        {
            yaw_pre_ = YawImu;
        }
        else
        {
            if (using_odom_pose_)
            {
                yaw_pre_ = PoseUtilities::toEuler(pose_odom_.orientation)[2];
            }
            else
            {
                geometry_msgs::TransformStamped baseInMap = listenTf(mapframe_, base_link_frame_, ros::Time(0));
                yaw_pre_ = PoseUtilities::toEuler(baseInMap.transform.rotation)[2];
            }
        }
    }

    PLUGINLIB_EXPORT_CLASS(pose_registration_plugins::LocatePoseRegistration, whi_pose_registration::BasePoseRegistration)
} // namespace pose_registration_plugins
