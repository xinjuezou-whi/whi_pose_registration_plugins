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
	    std::cout << "\nWHI loacate pose registration plugin VERSION 00.06.6" << std::endl;
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
        printf("configfile is %s \n", config_file.c_str());
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
                    if (pair.first.as<std::string>() == "cur_pose")
                    {
                        for (const auto& item : pair.second)
                        {
                            onefeature.cur_pose.push_back(item.as<double>());
                        }
                    }
                    if (pair.first.as<std::string>() == "feature_pose")
                    {
                        for (const auto& item : pair.second)
                        {
                            onefeature.feature_pose.push_back(item.as<double>());
                        }
                    }
                    
                    if (pair.first.as<std::string>() == "target_relative_pose")
                    {
                        for (const auto& onepose : pair.second)
                        {
                            TargetRelaPose onetarget_rela_pose;
                            //onefeature.target_rela_pose.push_back(item.as<double>());
                            for(const auto& subpair : onepose)
                            {
                                if (subpair.first.as<std::string>() == "pose")
                                {
                                    for (const auto& item : subpair.second)
                                    {
                                        onetarget_rela_pose.target_rela_pose.push_back(item.as<double>());
                                    }
                                }
                                onetarget_rela_pose.direction = "direct";
                                if (subpair.first.as<std::string>() == "drive_direction")
                                {
                                    onetarget_rela_pose.direction = subpair.second.as<std::string>();
                                }
                                onetarget_rela_pose.using_inertial = 0;
                                if (subpair.first.as<std::string>() == "using_inertial")
                                {
                                    onetarget_rela_pose.using_inertial = subpair.second.as<int>();
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
                printf("onefeature: \n");
                printf("name: %s ,cur_pose: [ %f, %f, %f  ]" , (*oneiter).name.c_str() , (*oneiter).cur_pose[0],(*oneiter).cur_pose[1],(*oneiter).cur_pose[2] );
                printf("feature_pose: [%f, %f, %f] \n",(*oneiter).feature_pose[0],(*oneiter).feature_pose[1],(*oneiter).feature_pose[2]);
                for (auto onepose : (*oneiter).target_rela_pose_vec )
                {
                    printf("target_rela_pose: [%f, %f, %f]",onepose.target_rela_pose[0],onepose.target_rela_pose[1],onepose.target_rela_pose[2]);
                    printf("using_inertial: %d", onepose.using_inertial);
                    printf("direction: %s \n",onepose.direction.c_str());
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
        node_handle_->param("pose_registration/LocatePose/regist_linear_thresh", regist_linear_thresh_, 0.03);
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
        ROS_INFO("laser_pose , x: %f,y: %f,z: %f,roll: %f,pitch: %f,yaw: %f",laser_pose_[0],laser_pose_[1],laser_pose_[2],laser_pose_[3],laser_pose_[4],laser_pose_[5]);
        node_handle_->param("pose_registration/LocatePose/imu_frame", imu_frame_, std::string("imu"));        
        node_handle_->param("pose_registration/LocatePose/laser_frame", laser_frame_, std::string("laser"));        
        node_handle_->param("pose_registration/LocatePose/tf_listener_frequency", tf_listener_frequency_, 20.0);
        node_handle_->param("pose_registration/LocatePose/laser_scan_topic", laserScanTopic, std::string("scan"));
        node_handle_->param("pose_registration/LocatePose/base_link_frame", base_link_frame_, std::string("base_link"));
        node_handle_->param("pose_registration/LocatePose/map_frame", mapframe_, std::string("map"));
        node_handle_->param("pose_registration/LocatePose/imu_topic", imuTopic, std::string("imu")); //issetimu
        node_handle_->param("pose_registration/LocatePose/using_imu", issetimu_, false); 
        node_handle_->param("pose_registration/LocatePose/xyvel", xyvel_, 0.05);
        node_handle_->param("pose_registration/LocatePose/rotvel", rotvel_, 0.1);    
        node_handle_->param("pose_registration/LocatePose/horizon_aligned_thresh", distthresh_horizon_, 0.005);   
        node_handle_->param("pose_registration/LocatePose/pattern_met_location_thresh", pattern_met_location_thresh_, 0.5);  
        node_handle_->param("pose_registration/LocatePose/pattern_met_radius_thresh", pattern_met_radius_thresh_, 0.08); 
        node_handle_->param("pose_registration/LocatePose/predict_dist_thresh", predict_dist_thresh_, 0.05); 
        node_handle_->param("pose_registration/LocatePose/predict_period_count", predict_period_count_, 1.0); 
        node_handle_->param("pose_registration/LocatePose/horizon_offset_vel", horizon_offset_vel_, 0.1); 
        node_handle_->param("pose_registration/LocatePose/vertical_to_rotvel", vertical_to_rotvel_, 0.1); 
        node_handle_->param("pose_registration/LocatePose/wait_scan_time", wait_scan_time_, 0.8);  
        node_handle_->param("pose_registration/LocatePose/is_fixed_location", is_fixed_location_, false);

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
        
        node_handle_->param("pose_registration/LocatePose/feature_segment_distance_thresh",
            feature_segment_distance_thresh_, 0.04);
        node_handle_->param("pose_registration/LocatePose/feature_min_size", feature_min_size_, 10);
        node_handle_->param("pose_registration/LocatePose/feature_max_size", feature_max_size_, 200);
        node_handle_->param("pose_registration/LocatePose/ndt_maxiter", ndtmaxiter_, 5000);
        node_handle_->param("pose_registration/LocatePose/odom_topic", odom_topic_, std::string("odom"));
        node_handle_->param("pose_registration/LocatePose/using_odom_pose", using_odom_pose_, false);  
        node_handle_->param("pose_registration/LocatePose/imu_adjust_rot_vel", imu_adjust_rot_vel_, 0.01);   //
        node_handle_->param("pose_registration/LocatePose/imu_adjust_rot_thresh", imu_adjust_rot_thresh_, 0.02); 
        sub_laser_scan_ = std::make_unique<ros::Subscriber>(node_handle_->subscribe<sensor_msgs::LaserScan>(
		    laserScanTopic, 10, std::bind(&LocatePoseRegistration::subCallbackLaserScan, this, std::placeholders::_1)));
        sub_imu_ = std::make_unique<ros::Subscriber>(node_handle_->subscribe<sensor_msgs::Imu>(
		    imuTopic, 10, std::bind(&LocatePoseRegistration::subCallbackImu, this, std::placeholders::_1)));

        odom_sub_ = std::make_unique<ros::Subscriber>(
          node_handle_->subscribe<nav_msgs::Odometry>(odom_topic_, 1, boost::bind(&LocatePoseRegistration::odomCallback, this, _1)) );
        // event queue for laser scan
        queue_scan_ = std::make_unique<EventQueue<void>>(10, false);
        threadRegistration();

        packpath_ = ros::package::getPath("whi_locate_pose_registration");
        ROS_INFO("packpath is :%s",packpath_.c_str());
        horizon_test_ = true;
    }

    void LocatePoseRegistration::computeVelocityCommands(geometry_msgs::Twist& CmdVel)
    {
        double yawFromImu = 0.0;
        {
            const std::lock_guard<std::mutex> lock(mtx_imu_);
            yawFromImu = angleyaw_imu_;
        }
        
        if (state_ == STA_ALIGN)
        {
            geometry_msgs::TransformStamped transBaselinkMap = listenTf(mapframe_, base_link_frame_, ros::Time(0));
            // double angleDiff = PoseUtilities::toEuler(pose_target_.orientation)[2] - 
            //     PoseUtilities::toEuler(transBaselinkMap.transform.rotation)[2];
            double angleDiff = angles::shortest_angular_distance(
                PoseUtilities::toEuler(transBaselinkMap.transform.rotation)[2],
                PoseUtilities::toEuler(pose_target_.orientation)[2]);
            if (issetimu_)
            {
                // angleDiff = angle_target_imu_ - yawFromImu;
                angleDiff = angles::shortest_angular_distance(yawFromImu, angle_target_imu_);
                if (debug_count_ == 5)
                {
                    ROS_INFO("in state STA_ALIGN, angle_target_imu_ = %f, yawFromImu = %f, angleDiff = %f",
                        angle_target_imu_, yawFromImu, angleDiff );
                }
            }
            else
            {
                if (debug_count_ == 5)
                {
                    ROS_INFO("in state STA_ALIGN, angle_target_imu_ = %f, angleDiff=%f",
                        angle_target_imu_, angleDiff );
                }
            }        
            if (fabs(angleDiff) < yaw_tolerance_)
            {
                CmdVel.linear.x = 0.0;
                CmdVel.angular.z = 0.0;
                if (prestate_ == STA_REGISTRATE_FINE)
                {
                    if (fabs(distance_horizon_) < distthresh_horizon_)
                    {
                        distance_todrive_ = distance_vertical_;
                        state_ = STA_ADJUST_VERTICAL;

                        ROS_INFO(" STA_REGISTRATE_FINE finish ,direct to STA_ADJUST_VERTICAL ,distance_todrive_=%f ",distance_todrive_);
                    }
                    else
                    {
                        state_ = STA_PRE_ROT_ANGLE;
                        ROS_INFO("STA_REGISTRATE_FINE finish, start STA_PRE_ROT_ANGLE");
                    }
                    updateCurrentPose(); 

                    ROS_INFO("STA_REGISTRATE_FINE finish, yawFromImu = %f", yawFromImu);
                }
                else if (prestate_ == STA_REGISTRATE)
                {
                    state_ = STA_WAIT_SCAN;
                    ROS_INFO("STA_REGISTRATE finish, wait scan");
                }
            }
            else
            {
                CmdVel.linear.x = 0.0;
                CmdVel.angular.z = PoseUtilities::signOf(sin(angleDiff)) * rotvel_;
            }
        }
        else if (state_ == STA_PRE_ROT_ANGLE)
        {
            geometry_msgs::TransformStamped transBaselinkMap = listenTf(mapframe_.c_str(), base_link_frame_, ros::Time(0));
            double angleBaselink = PoseUtilities::toEuler(transBaselinkMap.transform.rotation)[2];

            // pose_target_.orientation = PoseUtilities::fromEuler(0.0, 0.0, angleBaselink - PoseUtilities::signOf(distance_horizon_) * zig_angle_);
            // angle_target_imu_ = yawFromImu - PoseUtilities::signOf(distance_horizon_) * zig_angle_;
            pose_target_.orientation = PoseUtilities::fromEuler(0.0, 0.0,
                angles::shortest_angular_distance(PoseUtilities::signOf(distance_horizon_) * zig_angle_, angleBaselink));
            angle_target_imu_ = angles::shortest_angular_distance(
                PoseUtilities::signOf(distance_horizon_) * zig_angle_, yawFromImu);

            state_ = STA_ROT_ANGLE;
            ROS_INFO("STA_PRE_ROT_ANGLE finish,  start STA_ROT_ANGLE ");
        }
        else if (state_ == STA_ROT_ANGLE)
        {
            geometry_msgs::TransformStamped transBaselinkMap = listenTf(mapframe_.c_str(), base_link_frame_, ros::Time(0));
            // double angleDiff = PoseUtilities::toEuler(pose_target_.orientation)[2] - 
            //     PoseUtilities::toEuler(transBaselinkMap.transform.rotation)[2];
            double angleDiff = angles::shortest_angular_distance(
                PoseUtilities::toEuler(transBaselinkMap.transform.rotation)[2], PoseUtilities::toEuler(pose_target_.orientation)[2]);
            if (issetimu_)
            {
                // angleDiff = angle_target_imu_ - yawFromImu;
                angleDiff = angles::shortest_angular_distance(yawFromImu, angle_target_imu_);
                if (debug_count_ == 5)
                {
                    ROS_INFO("in state STA_ROT_ANGLE, angle_target_imu_ = %f, yawFromImu = %f, angleDiff = %f",
                        angle_target_imu_, yawFromImu, angleDiff);
                }
            }
            else
            {
                if (debug_count_ == 5)
                {
                    ROS_INFO("in state STA_ROT_ANGLE, angle_target_imu_ = %f, angleDiff = %f",
                        angle_target_imu_, angleDiff);
                }
            }              
            if (fabs(angleDiff) < yaw_tolerance_)
            {
                CmdVel.linear.x = 0.0;
                CmdVel.angular.z = 0.0;
                state_ = STA_BACK;
                distance_todrive_ = fabs(distance_horizon_) / sin(zig_angle_);
                updateCurrentPose();
                ROS_INFO("STA_ROT_ANGLE finish, start STA_BACK , distance_todrive: %f",distance_todrive_);
            }
            else
            {
                CmdVel.linear.x = 0.0;
                CmdVel.angular.z = PoseUtilities::signOf(sin(angleDiff)) * rotvel_;
            }
        }
        else if(state_ == STA_BACK)
        {
            geometry_msgs::TransformStamped transBaselinkMap = listenTf(mapframe_.c_str(), base_link_frame_, ros::Time(0));
            geometry_msgs::Pose curpose;
            curpose.position.x = transBaselinkMap.transform.translation.x;
            curpose.position.y = transBaselinkMap.transform.translation.y;
            double routedis = PoseUtilities::distance(curpose,pose_standby_);
            double extradis = distance_todrive_ + 0.3;
            double distDiff = distance_todrive_ - routedis;
            //行驶距离超出计算值 30cm ；偏差过大，说明前面对齐失败了
            if (routedis > extradis)
            {
                CmdVel.linear.x = 0.0;
                CmdVel.linear.y = 0.0;
                state_ = STA_FAILED;
                ROS_INFO("fail at STA_BACK ,routedis too far , align or rotate wrong ");
            }
            else if (fabs(distDiff) < xy_tolerance_)
            {
                CmdVel.linear.x = 0.0;
                CmdVel.angular.z = 0.0;
                state_ = STA_PRE_ROT_VERTICAL;
                ROS_INFO("STA_BACK finish, start STA_PRE_ROT_VERTICAL"); 
            }
            else
            {
                CmdVel.linear.x = -1 * PoseUtilities::signOf(distDiff) * xyvel_;
                CmdVel.linear.y = 0.0;
            }
        }
        else if (state_ == STA_PRE_ROT_VERTICAL)
        {
            geometry_msgs::TransformStamped transBaselinkMap = listenTf(mapframe_.c_str(), base_link_frame_, ros::Time(0));
            double angleBaselink = PoseUtilities::toEuler(transBaselinkMap.transform.rotation)[2];

            pose_target_.orientation = PoseUtilities::fromEuler(0.0, 0.0, angleBaselink + PoseUtilities::signOf(distance_horizon_) * zig_angle_);
            angle_target_imu_ = yawFromImu + PoseUtilities::signOf(distance_horizon_) * zig_angle_;

            angle_target_imu_ = getrightImu(angle_target_imu_);
            updateCurrentPose(); 
            state_ = STA_ROT_VERTICAL;
            ROS_INFO("STA_PRE_ROT_VERTICAL finish,  start STA_ROT_VERTICAL ");   
        }
        else if (state_ == STA_ROT_VERTICAL)
        {
            geometry_msgs::TransformStamped transBaselinkMap = listenTf(mapframe_.c_str(), base_link_frame_, ros::Time(0));
            // double angleDiff = PoseUtilities::toEuler(pose_target_.orientation)[2] - 
            //     PoseUtilities::toEuler(transBaselinkMap.transform.rotation)[2];
            double angleDiff = angles::shortest_angular_distance(
                PoseUtilities::toEuler(transBaselinkMap.transform.rotation)[2], PoseUtilities::toEuler(pose_target_.orientation)[2]);
            if (issetimu_)
            {
                // angleDiff = angle_target_imu_ - yawFromImu;
                angleDiff = angles::shortest_angular_distance(yawFromImu, angle_target_imu_);
            }            
            if (fabs(angleDiff) < yaw_tolerance_)
            {
                CmdVel.linear.x = 0.0;
                CmdVel.angular.z = 0.0;
                state_ = STA_ADJUST_VERTICAL;
                updateCurrentPose(); 
                distance_todrive_ = fabs(distance_horizon_) * cos(zig_angle_) + distance_vertical_;
                ROS_INFO("STA_ROT_VERTICAL finish, start STA_ADJUST_VERTICAL, distance_todrive_ =%f", distance_todrive_);
            }
            else
            {
                CmdVel.linear.x = 0.0;
                CmdVel.angular.z = PoseUtilities::signOf(sin(angleDiff)) * rotvel_;
            }
        }
        else if (state_ == STA_ADJUST_VERTICAL)
        {
            geometry_msgs::TransformStamped transBaselinkMap = listenTf(mapframe_.c_str(), base_link_frame_, ros::Time(0));
            geometry_msgs::Pose curpose;
            curpose.position.x = transBaselinkMap.transform.translation.x;
            curpose.position.y = transBaselinkMap.transform.translation.y;
            double routedis = PoseUtilities::distance(curpose, pose_standby_);
            double extradis = fabs(distance_todrive_) + 0.3;
            double distDiff = fabs(distance_todrive_) - routedis;
            //行驶距离超出计算值 30cm ；偏差过大，说明前面对齐失败了
            if (routedis > extradis)
            {
                CmdVel.linear.x = 0.0;
                CmdVel.linear.y = 0.0;
                state_ = STA_FAILED;
                ROS_INFO("fail at STA_ADJUST_VERTICAL ,routedis too far , align or rotate wrong ");
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
                CmdVel.linear.x = PoseUtilities::signOf(distDiff) * PoseUtilities::signOf(distance_todrive_) * xyvel_;
                CmdVel.linear.y = 0.0;
            }
        }
        else if (state_ == STA_PRE_HORIZON)
        {  
            if (target_rela_pose_vec_.empty())
            {
                ROS_INFO(" in STA_PRE_HORIZON, target_rela_pose_vec_ is empty , done ");
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
            angle_target_imu_ = yawFromImu + leftorright_ * 0.5 * M_PI;       
            angle_target_imu_ = getrightImu(angle_target_imu_);
            updateCurrentPose();

            distance_horizon_ = fabs(target_rela_pose_[1]) - leftorright_ * pose_feature_.position.y;    // mark here by zhouyue
            distance_vertical_ = target_rela_pose_[0] + pose_feature_.position.x;
            get_align_imu_ = yawFromImu;
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
            //getfea_cur_pose_.orientation.w = 1.0;
            //pose_end_ = PoseUtilities::applyTransform(relapos, trans);

       
            ROS_INFO("STA_PRE_HORIZON curpose is :[%f, %f ] ,angleBaselink:%f , get_align_imu_: %f",curpose.position.x, curpose.position.y, angleBaselink, get_align_imu_);   
            //ROS_INFO("pose_end is :[%f, %f ]",pose_end_.position.x, pose_end_.position.y);

            if (fabs(target_rela_pose_[0]) < 0.001 && fabs(target_rela_pose_[1]) < 0.001)
            {
                if (fabs(target_rela_pose_[2]) < 0.001)
                {
                    state_ = STA_PRE_NEXT;
                    ROS_INFO(" STA_PRE_HORIZON to STA_PRE_NEXT , sta_done");
                }
                else
                {
                    {
                        pose_end_.position.x = getfea_cur_pose_.position.x;
                        pose_end_.position.y = getfea_cur_pose_.position.y;
                        state_ = STA_PRE_ORIENTATION;
                        ROS_INFO("pose_end is :[%f, %f ]",pose_end_.position.x, pose_end_.position.y);
                        ROS_INFO(" target_rela_pose_[2] > 0, start STA_PRE_ORIENTATION ");

                    }

                }
            }
            else if (fabs(target_rela_pose_[1]) < 0.001)
            {
                relapos.position.x = distance_vertical_;
                relapos.position.y = 0;
                relapos.orientation = PoseUtilities::fromEuler(0.0, 0.0, 0.0);
                pose_end_ = PoseUtilities::applyTransform(relapos, transBaselinkMap);
                vertical_start_pose_.position.x = transBaselinkMap.transform.translation.x;
                vertical_start_pose_.position.y = transBaselinkMap.transform.translation.y;
                state_ = STA_PRE_ROT_ROUTE_VERTICAL;
                ROS_INFO("STA_PRE_HORIZON finish, target_rela_pose_[1] =0 ,  start STA_PRE_ROT_ROUTE_VERTICAL "); 
                ROS_INFO("pose_end is :[%f, %f ]",pose_end_.position.x, pose_end_.position.y);

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
                ROS_INFO("STA_PRE_HORIZON finish,  start STA_TO_HORIZON, pose_end_ is :[%f,%f]",pose_end_.position.x,pose_end_.position.y); 

            }

        }
        else if (state_ == STA_PRE_NEXT)
        {
            geometry_msgs::TransformStamped transBaselinkMap = listenTf(mapframe_.c_str(), base_link_frame_, ros::Time(0));

            if (target_rela_pose_vec_.empty())
            {
                ROS_INFO(" in STA_PRE_NEXT ,  arrive done ,now curpose is: [ %f, %f ] , yawFromImu is:%f ",transBaselinkMap.transform.translation.x,transBaselinkMap.transform.translation.y,yawFromImu);
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
            angle_target_imu_ = yawFromImu + leftorright_ * 0.5 * M_PI;       
            angle_target_imu_ = getrightImu(angle_target_imu_);
            updateCurrentPose();

            distance_horizon_ = fabs(target_rela_pose_[1]);    // mark here by zhouyue
            distance_vertical_ = fabs(target_rela_pose_[0]);
            //get_align_imu_ = yawFromImu;
            //get_align_angle_ = angleBaselink;
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

       
            ROS_INFO("STA_PRE_NEXT curpose is :[%f, %f ] ,angleBaselink:%f , get_align_imu_: %f",curpose.position.x, curpose.position.y, angleBaselink, get_align_imu_);   
            //ROS_INFO("STA_PRE_NEXT pose_end is :[%f, %f ]",pose_end_.position.x, pose_end_.position.y);

            if (fabs(target_rela_pose_[0]) < 0.001 && fabs(target_rela_pose_[1]) < 0.001)
            {
                if (fabs(target_rela_pose_[2]) < 0.001)
                {
                    state_ = STA_PRE_NEXT;
                    ROS_INFO(" STA_PRE_NEXT  , sta_done");
                }
                else
                {
                    state_ = STA_PRE_ORIENTATION;
                    ROS_INFO(" target_rela_pose_[2] > 0, start STA_PRE_ORIENTATION ");
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
                    ROS_INFO("!using_inertial, direct to STA_ROUTE_VERTICAL ");
                }
                else
                {
                    state_ = STA_PRE_ROT_ROUTE_VERTICAL;
                    ROS_INFO("using_inertial,  STA_PRE_ROT_ROUTE_VERTICAL ");
                    ROS_INFO("STA_PRE_NEXT finish, target_rela_pose_[1] =0 ,  start STA_PRE_ROT_ROUTE_VERTICAL, target pose_end_: [%f,%f] ",pose_end_.position.x, pose_end_.position.y); 
                }                

            }
            else
            {
                state_ = STA_TO_HORIZON;
                ROS_INFO("STA_PRE_NEXT finish,  start STA_TO_HORIZON "); 
            }

        }        
        else if (state_ == STA_TO_HORIZON)
        {
            geometry_msgs::TransformStamped transBaselinkMap = listenTf(mapframe_.c_str(), base_link_frame_, ros::Time(0));
            // double angleDiff = PoseUtilities::toEuler(pose_target_.orientation)[2] - 
            //     PoseUtilities::toEuler(transBaselinkMap.transform.rotation)[2];
            double angleDiff = angles::shortest_angular_distance(
                PoseUtilities::toEuler(transBaselinkMap.transform.rotation)[2], PoseUtilities::toEuler(pose_target_.orientation)[2]);
            if (issetimu_)
            {
                // angleDiff = angle_target_imu_ - yawFromImu;
                angleDiff = angles::shortest_angular_distance(yawFromImu, angle_target_imu_);
                if (debug_count_ == 5)
                {
                    ROS_INFO("in state STA_TO_HORIZON, angle_target_imu_ = %f, yawFromImu = %f, angleDiff = %f",
                        angle_target_imu_, yawFromImu, angleDiff );
                }
            }
            else
            {
                if (debug_count_ == 5)
                {
                    ROS_INFO("in state STA_TO_HORIZON, angle_target_imu_ = %f, angleDiff = %f",
                        angle_target_imu_, angleDiff);
                }
            }
            if (fabs(angleDiff) < yaw_tolerance_)
            {
                CmdVel.linear.x = 0.0;
                CmdVel.angular.z = 0.0;

                prestate_ = state_;
                ROS_INFO("sta_to_horizon finish, yawFromImu = %f", yawFromImu);
                if (!using_inertial_ || horizon_test_)
                {
                    state_ = STA_ROUTE_HORIZON;
                    ROS_INFO("!using_inertial_ , direct to  STA_ROUTE_HORIZON");

                }
                else
                {
                    state_ = STA_PRE_ROT_ROUTE_HORIZON;
                    ROS_INFO("using_inertial_ , to  STA_PRE_ROT_ROUTE_HORIZON");
                }
                updateCurrentPose(); 
                get_horizon_direct_imu_ = yawFromImu;
            }
            else
            {
                CmdVel.linear.x = 0.0;
                CmdVel.angular.z = PoseUtilities::signOf(sin(angleDiff)) * rotvel_;
                if (debug_count_ == 5)
                {
                    ROS_INFO("angular.z : %f ",CmdVel.angular.z);
                }
            }
        }
        else if (state_ == STA_PRE_ROT_ROUTE_HORIZON)
        {
            geometry_msgs::TransformStamped transBaselinkMap = listenTf(mapframe_.c_str(), base_link_frame_, ros::Time(0));
            updateCurrentPose();
            vertical_start_pose_.position.x = transBaselinkMap.transform.translation.x;
            vertical_start_pose_.position.y = transBaselinkMap.transform.translation.y;
            geometry_msgs::Point pntEnd,pntCur,pntForward;
            pntEnd.x = pose_end_.position.x;
            pntEnd.y = pose_end_.position.y;
            pntCur.x = vertical_start_pose_.position.x;
            pntCur.y = vertical_start_pose_.position.y;    
            pntForward.x = pntCur.x + 0.2;
            pntForward.y = pntCur.y;
            auto vecCurEnd = PoseUtilities::createVector2D(pntCur,pntEnd);
            auto vecForward = PoseUtilities::createVector2D(pntCur,pntForward);
            double targetYaw = PoseUtilities::angleBetweenVectors2D(vecForward, vecCurEnd);
            ROS_INFO("targetYaw = %f, curYaw = %f", targetYaw, PoseUtilities::toEuler(transBaselinkMap.transform.rotation)[2]);
            pose_target_.orientation = PoseUtilities::fromEuler(0.0, 0.0, targetYaw);
            double angleDiff = angles::shortest_angular_distance(
                PoseUtilities::toEuler(transBaselinkMap.transform.rotation)[2], PoseUtilities::toEuler(pose_target_.orientation)[2]);
            ROS_INFO("angleDiff = %f",angleDiff);
            state_ = STA_ROT_ROUTE_HORIZON;
            ROS_INFO("STA_PRE_ROT_ROUTE_HORIZON finish ,start STA_ROT_ROUTE_HORIZON,vertical_start_pose_ :[%f ,%f] ",vertical_start_pose_.position.x ,vertical_start_pose_.position.y );

        }
        else if (state_ == STA_ROT_ROUTE_HORIZON)
        {
            geometry_msgs::TransformStamped transBaselinkMap = listenTf(mapframe_.c_str(), base_link_frame_, ros::Time(0));
            double angleDiff = angles::shortest_angular_distance(
                PoseUtilities::toEuler(transBaselinkMap.transform.rotation)[2], PoseUtilities::toEuler(pose_target_.orientation)[2]);

            if (fabs(angleDiff) < yaw_tolerance_)
            {
                CmdVel.linear.x = 0.0;
                CmdVel.angular.z = 0.0;

                state_ = STA_PRE_ROUTE_HORIZON;             
                ROS_INFO("STA_ROT_ROUTE_HORIZON finish ,start STA_PRE_ROUTE_HORIZON");

            }
            else
            {
                CmdVel.linear.x = 0.0;
                CmdVel.angular.z = PoseUtilities::signOf(sin(angleDiff)) * rotvel_;          
            }
        }
        else if (state_ == STA_PRE_ROUTE_HORIZON)
        {
            geometry_msgs::TransformStamped transBaselinkMap = listenTf(mapframe_.c_str(), base_link_frame_, ros::Time(0));
            updateCurrentPose();
            vertical_start_pose_.position.x = transBaselinkMap.transform.translation.x;
            vertical_start_pose_.position.y = transBaselinkMap.transform.translation.y;

            get_horizon_imu_ =  get_align_imu_ + leftorright_ * 0.5 * M_PI;
            get_horizon_angle_ = get_align_angle_ + leftorright_ * 0.5 * M_PI;
            state_ = STA_ROUTE_HORIZON;
            ROS_INFO("curpose is : [%f,%f]",vertical_start_pose_.position.x,vertical_start_pose_.position.y);
            ROS_INFO("STA_PRE_ROUTE_HORIZON finish ,start STA_ROUTE_HORIZON");
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
                    routedis = PoseUtilities::distance(get_pose_odom_,pose_standby_odom_);
                }
                else
                {
                    routedis = PoseUtilities::distance(curpose,pose_standby_);
                }
                double extradis = fabs(distance_horizon_) + 0.3;
                double distDiff = fabs(distance_horizon_) - routedis;
                //行驶距离超出计算值 30cm ；偏差过大，说明前面对齐失败了
                if (routedis > extradis)
                {
                    CmdVel.linear.x = 0.0;
                    CmdVel.linear.y = 0.0;
                    state_ = STA_FAILED;
                    ROS_INFO("fail at STA_ROUTE_HORIZON ,routedis too far , align or rotate wrong ");
                }
                else if (fabs(distDiff) < xy_tolerance_)
                {
                    CmdVel.linear.x = 0.0;
                    CmdVel.angular.z = 0.0;
                    state_ = STA_PRE_VERTICAL;
                    ROS_INFO("STA_ROUTE_HORIZON finish,  start STA_PRE_VERTICAL "); 
                }
                else
                {
                    CmdVel.linear.x = PoseUtilities::signOf(distDiff) * horizon_offset_vel_;
                    CmdVel.linear.y = 0.0;
                    auto imuangleDiff = angles::shortest_angular_distance(yawFromImu, get_horizon_direct_imu_);
                    ROS_INFO("imuangleDiff is:%f ",imuangleDiff);
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
            else
            {
                // inertial navigation logic
                geometry_msgs::TransformStamped transBaselinkMap = listenTf(mapframe_.c_str(), base_link_frame_, ros::Time(0));
                double angleBaselink = PoseUtilities::toEuler(transBaselinkMap.transform.rotation)[2];
                geometry_msgs::Pose curpose;
                curpose.position.x = transBaselinkMap.transform.translation.x;
                curpose.position.y = transBaselinkMap.transform.translation.y;
                double diffX = pose_end_.position.x - curpose.position.x;
                double diffY = pose_end_.position.y - curpose.position.y;
                double diffAngle = get_horizon_angle_ - PoseUtilities::toEuler(transBaselinkMap.transform.rotation)[2];
                if (issetimu_)
                {
                    diffAngle = angles::shortest_angular_distance(yawFromImu, get_horizon_imu_);
                }
                double curdis = PoseUtilities::distance(curpose,pose_end_);
                //ROS_INFO("diffx: %f, diffy: %f,diffAngle: %f ,curdis:%f ",diffX,diffY,diffAngle,curdis);
                if (fabs(diffX) < xy_tolerance_ && fabs(diffY) < xy_tolerance_ && substate != "endrot"  )
                {
                    CmdVel.linear.x = 0.0;
                    CmdVel.angular.z = 0.0;
                    if (fabs(diffAngle) <=  yaw_tolerance_)
                    {
                        state_ = STA_PRE_VERTICAL;
                        ROS_INFO("diffX diffY min, STA_ROUTE_HORIZON finish, to STA_PRE_VERTICAL ");
                        substate = "direct" ;
                        return ;
                    }
                    else 
                    {
                        substate = "pre_endrot";
                        ROS_INFO(" diffX diffY min, start pre_endrot ");
                        //angle_target_imu_ = getrightImu(angle_target_imu_);                              
                    }
                }

                geometry_msgs::Point pntEnd,pntCur,pntStart,pntNext,pntCross;
                pntEnd.x = pose_end_.position.x;
                pntEnd.y = pose_end_.position.y;
                pntCur.x = curpose.position.x;
                pntCur.y = curpose.position.y;       
                pntStart.x = vertical_start_pose_.position.x;
                pntStart.y = vertical_start_pose_.position.y;    
                geometry_msgs::Pose nextPose,poseDelta;
                poseDelta.position.x = inertial_xyvel_ * predict_period_count_ / controller_frequency_;
                poseDelta.orientation = PoseUtilities::fromEuler(0.0, 0.0, 0.0);
                nextPose = PoseUtilities::applyTransform(poseDelta, transBaselinkMap);
                pntNext.x = nextPose.position.x;
                pntNext.y = nextPose.position.y;
                //ROS_INFO("curpose:[%f,%f] , nextpose:[%f,%f]",pntCur.x,pntCur.y,pntNext.x,pntNext.y);

                //求当前点到目标路径垂直交点             
                double k1 = (pntEnd.y - pntStart.y)/(pntEnd.x - pntStart.x);
                double k2 = (pntStart.x - pntEnd.x)/(pntEnd.y - pntStart.y);
                pntCross.x = (pntCur.y - pntStart.y + k1 * pntStart.x - k2 * pntCur.x)/(k1 - k2);
                pntCross.y = k1*(pntCross.x - pntStart.x) + pntStart.y;

                //ROS_INFO("pntCross:[%f,%f]",pntCross.x,pntCross.y);

                auto vecCrossNext = PoseUtilities::createVector2D(pntCross, pntNext);
                auto vecCurEnd = PoseUtilities::createVector2D(pntCur,pntEnd);
                auto vecStartEnd = PoseUtilities::createVector2D(pntStart,pntEnd);
                auto vecStartNext = PoseUtilities::createVector2D(pntStart,pntNext);
                auto vecStartCur = PoseUtilities::createVector2D(pntStart,pntCur);
                auto vecCurNext = PoseUtilities::createVector2D(pntCur,pntNext);

                double nextAngle = PoseUtilities::angleBetweenVectors2D(vecStartEnd, vecCrossNext);
                double startNextAngle = PoseUtilities::angleBetweenVectors2D(vecStartEnd, vecCurNext); 
                double disCrossNext = PoseUtilities::distance(pntNext,pntCross);
                double nextDelta = disCrossNext * sin(fabs(nextAngle));
                double curAngle = PoseUtilities::angleBetweenVectors2D(vecStartEnd,vecStartCur);
                //ROS_INFO("nextAngle: %f, nextDelta: %f, curAngle: %f ,startNextAngle:%f", nextAngle, nextDelta, curAngle,startNextAngle);
                if(PoseUtilities::signOf(nextAngle) != PoseUtilities::signOf(curAngle))
                {
                    //ROS_INFO("------------ +++++++++++++signOf(nextAngle) != signOf(curAngle)  --------");
                }

                double routedis = PoseUtilities::distance(pntStart,pntEnd);
                double curStartdis = PoseUtilities::distance(pntCur,pntStart);

                //double forwardAngle = PoseUtilities::angleBetweenVectors2D(vecForward, vecCurEnd);
                double curendAngle = PoseUtilities::angleBetweenVectors2D(vecCurEnd, vecStartEnd);
                //ROS_INFO("curendAngle: %f", curendAngle);
                if (substate == "direct" )
                {
                    if (fabs(curendAngle) >= M_PI / 2 || curStartdis >= routedis)
                    {
                        substate = "pre_endrot";
                        ROS_INFO("fabs(delta) >= M_PI / 2, start pre_endrot , curendAngle: %f",curendAngle);   
                        ROS_INFO("curdis : %f, routedis : %f",curStartdis, routedis);  
                    }
                    else
                    {
                        CmdVel.linear.x = inertial_xyvel_;
                        if (nextDelta > predict_dist_thresh_ || diffAngle > yaw_tolerance_ * 2)
                        {

                            if(PoseUtilities::signOf(nextAngle) == PoseUtilities::signOf(startNextAngle))
                            {
                                CmdVel.angular.z = -1 * PoseUtilities::signOf(nextAngle) * inertial_rotvel_ *
                                    fabs(nextAngle) / 5.0;
                            }
                            else
                            {
                                // invert direction 
                                CmdVel.angular.z = PoseUtilities::signOf(nextAngle) * inertial_rotvel_ *
                                    fabs(nextAngle) / 5.0 *1.5 ;
                            }
                        }
                        else    
                        {
                            CmdVel.angular.z = 0.0;
                        }
                        CmdVel.angular.z = fabs(CmdVel.angular.z) > inertial_rotvel_ ?
                            PoseUtilities::signOf(CmdVel.angular.z) * inertial_rotvel_ : CmdVel.angular.z;
                        //ROS_INFO("CmdVel.angular.z: %f", CmdVel.angular.z);
                    }
                }
                else if (substate == "pre_endrot")
                {
                    //pose_target_.orientation = PoseUtilities::fromEuler(0.0, 0.0, get_align_angle_); 
                    //angle_target_imu_ = get_align_imu_; 
                    state_ = STA_PRE_VERTICAL;
                    substate = "direct";
                }
                else if (substate == "endrot" )
                {
                    geometry_msgs::TransformStamped transBaselinkMap = listenTf(mapframe_.c_str(), base_link_frame_, ros::Time(0));
                    // double angleDiff = PoseUtilities::toEuler(pose_target_.orientation)[2] - 
                    //     PoseUtilities::toEuler(transBaselinkMap.transform.rotation)[2];
                    double angleDiff = angles::shortest_angular_distance(
                        PoseUtilities::toEuler(transBaselinkMap.transform.rotation)[2], PoseUtilities::toEuler(pose_target_.orientation)[2]);
                    if (issetimu_)
                    {
                        // angleDiff = angle_target_imu_ - yawFromImu;
                        angleDiff = angles::shortest_angular_distance(yawFromImu, angle_target_imu_);
                    }
                    if (fabs(angleDiff) < yaw_tolerance_)
                    {
                        CmdVel.linear.x = 0.0;
                        CmdVel.angular.z = 0.0;

                        if (fabs(target_rela_pose_[2]) < 0.01)
                        {
                            state_ = STA_PRE_NEXT;
                            ROS_INFO("endrot STA_ROUTE_VERTICAL finish ,to STA_PRE_NEXT  ");
                        }
                        else
                        {
                            state_ = STA_PRE_ORIENTATION;
                            ROS_INFO("endrot STA_ROUTE_VERTICAL finish ,to sta STA_PRE_ORIENTATION ");

                        }
                        substate = "direct" ;
                    }
                    else
                    {
                        CmdVel.linear.x = 0.0;
                        CmdVel.angular.z = PoseUtilities::signOf(sin(angleDiff)) * rotvel_;
                        ROS_INFO("in endrot , rot");                
                    }
                }
            }

        }
        else if (state_ == STA_PRE_VERTICAL)
        {
            geometry_msgs::TransformStamped transBaselinkMap = listenTf(mapframe_.c_str(), base_link_frame_, ros::Time(0));
            if (distance_vertical_ < 0.001 )
            {
                state_ = STA_PRE_NEXT;
                ROS_INFO("after ROUTE_HORIZON,distance_vertical = 0 , to STA_PRE_NEXT");
                return ;
            }

            double angleBaselink = PoseUtilities::toEuler(transBaselinkMap.transform.rotation)[2];
            pose_target_.position.x = 0.0;
            pose_target_.position.y = 0.0;
            pose_target_.orientation = PoseUtilities::fromEuler(0.0, 0.0, angleBaselink - leftorright_ * 0.5 * M_PI);
            //pose_target_.orientation = PoseUtilities::fromEuler(0.0, 0.0, get_align_angle_); 
            angle_target_imu_ = yawFromImu - leftorright_ * 0.5 * M_PI;
            angle_target_imu_ = getrightImu(angle_target_imu_);
            //angle_target_imu_ = getrightImu(get_align_imu_);
            ROS_INFO("in state STA_PRE_VERTICAL finish, angle_target_imu_ = %f, yawFromImu = %f",
                angle_target_imu_, yawFromImu);
            //进入第二条路径，更改当前点为起始点
            updateCurrentPose();  

            ROS_INFO("STA_PRE_VERTICAL finish, start STA_TO_VERTICAL, now,curpose is: [%f,%f]",transBaselinkMap.transform.translation.x, transBaselinkMap.transform.translation.y); 
            state_ = STA_TO_VERTICAL;
        }
        else if (state_ == STA_TO_VERTICAL)
        {
            geometry_msgs::TransformStamped transBaselinkMap = listenTf(mapframe_.c_str(), base_link_frame_, ros::Time(0));
            // double angleDiff = PoseUtilities::toEuler(pose_target_.orientation)[2] - 
            //     PoseUtilities::toEuler(transBaselinkMap.transform.rotation)[2];
            double angleDiff = angles::shortest_angular_distance(
                PoseUtilities::toEuler(transBaselinkMap.transform.rotation)[2], PoseUtilities::toEuler(pose_target_.orientation)[2]);
            if (issetimu_)
            {
                // angleDiff = angle_target_imu_ - yawFromImu;
                angleDiff = angles::shortest_angular_distance(yawFromImu, angle_target_imu_);
                if (debug_count_ == 5)
                {
                    ROS_INFO("in state STA_TO_VERTICAL, angle_target_imu_ = %f, yawFromImu = %f, angleDiff=%f",
                        angle_target_imu_, yawFromImu, angleDiff);
                }
            }
            else
            {
                if (debug_count_ == 5)
                {
                    ROS_INFO("in state STA_TO_VERTICAL, angle_target_imu_ = %f, angleDiff=%f",
                        angle_target_imu_, angleDiff);
                }
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
                    ROS_INFO("STA_TO_VERTICAL finish ,direct to STA_ROUTE_VERTICAL ");
                }
                else
                {
                    state_ = STA_PRE_ROT_ROUTE_VERTICAL;
                    ROS_INFO("STA_TO_VERTICAL finish ,start STA_PRE_ROT_ROUTE_VERTICAL ");
                }
                updateCurrentPose(); 

            }
            else
            {
                CmdVel.linear.x = 0.0;
                CmdVel.angular.z = PoseUtilities::signOf(sin(angleDiff)) * vertical_to_rotvel_;
                if (debug_count_ == 5)
                {
                    ROS_INFO("angular.z : %f ",CmdVel.angular.z);
                }                
                //prestate_ = state_;
            }
        }      
        else if (state_ == STA_PRE_ROT_ROUTE_VERTICAL)
        {
            geometry_msgs::TransformStamped transBaselinkMap = listenTf(mapframe_.c_str(), base_link_frame_, ros::Time(0));
            if (prestate_ == STA_TO_VERTICAL)
            {
                geometry_msgs::Pose relapos;
                relapos.position.x = distance_vertical_;
                relapos.position.y = 0;
                relapos.orientation = PoseUtilities::fromEuler(0.0, 0.0, 0.0);
                geometry_msgs::TransformStamped subtrans = transBaselinkMap;
                if (is_fixed_location_)
                {
                    subtrans.transform.translation.x = pose_end_.position.x;
                    subtrans.transform.translation.y = pose_end_.position.y;                
                    subtrans.transform.rotation = getfea_cur_pose_.orientation;
                }
                else
                {                   
                    subtrans.transform.rotation = PoseUtilities::fromEuler(0.0, 0.0, get_align_angle_);
                }
                pose_end_ = PoseUtilities::applyTransform(relapos, subtrans);
            }    

            // align the heading direction of target 
            updateCurrentPose();
            vertical_start_pose_.position.x = transBaselinkMap.transform.translation.x;
            vertical_start_pose_.position.y = transBaselinkMap.transform.translation.y;
            ROS_INFO("STA_PRE_ROT_ROUTE_VERTICAL finish ,start STA_ROT_ROUTE_VERTICAL,vertical_start_pose_ :[%f ,%f] ",vertical_start_pose_.position.x ,vertical_start_pose_.position.y );
            ROS_INFO("pose_end_ is:[%f,%f]",pose_end_.position.x,pose_end_.position.y);
            geometry_msgs::Point pntEnd,pntCur,pntForward;
            pntEnd.x = pose_end_.position.x;
            pntEnd.y = pose_end_.position.y;
            pntCur.x = vertical_start_pose_.position.x;
            pntCur.y = vertical_start_pose_.position.y;    
            pntForward.x = pntCur.x + 0.2;
            pntForward.y = pntCur.y;
            auto vecCurEnd = PoseUtilities::createVector2D(pntCur,pntEnd);
            auto vecForward = PoseUtilities::createVector2D(pntCur,pntForward);
            double targetYaw = PoseUtilities::angleBetweenVectors2D(vecForward, vecCurEnd);
            ROS_INFO("in cal targetYaw is : %f",targetYaw);
            if (route_vertical_direct_ == "inverse")
            {
                targetYaw += M_PI;
                ROS_INFO("in cal,inverse targetYaw is : %f",targetYaw);
                get_align_imu_ = get_align_imu_ + M_PI;
                get_align_angle_ = get_align_angle_ + M_PI  ; 
                get_align_imu_ = getrightImu(get_align_imu_);               
            }
            ROS_INFO("targetYaw = %f, curYaw = %f", targetYaw, PoseUtilities::toEuler(transBaselinkMap.transform.rotation)[2]);
            pose_target_.orientation = PoseUtilities::fromEuler(0.0, 0.0, targetYaw);
            double angleDiff = angles::shortest_angular_distance(
                PoseUtilities::toEuler(transBaselinkMap.transform.rotation)[2], PoseUtilities::toEuler(pose_target_.orientation)[2]);
            ROS_INFO("angleDiff = %f",angleDiff);
            state_ = STA_ROT_ROUTE_VERTICAL;
        }  
        else if (state_ == STA_ROT_ROUTE_VERTICAL)
        {
            geometry_msgs::TransformStamped transBaselinkMap = listenTf(mapframe_.c_str(), base_link_frame_, ros::Time(0));
            // double angleDiff = PoseUtilities::toEuler(pose_target_.orientation)[2] - 
            //     PoseUtilities::toEuler(transBaselinkMap.transform.rotation)[2];
            double angleDiff = angles::shortest_angular_distance(
                PoseUtilities::toEuler(transBaselinkMap.transform.rotation)[2], PoseUtilities::toEuler(pose_target_.orientation)[2]);

            if (fabs(angleDiff) < yaw_tolerance_)
            {
                CmdVel.linear.x = 0.0;
                CmdVel.angular.z = 0.0;

                state_ = STA_ROUTE_VERTICAL;
                updateCurrentPose(); 
                vertical_start_pose_.position.x = transBaselinkMap.transform.translation.x;
                vertical_start_pose_.position.y = transBaselinkMap.transform.translation.y;                
                ROS_INFO("STA_ROT_ROUTE_VERTICAL finish ,start STA_ROUTE_VERTICAL,vertical_start_pose_ :[%f ,%f] ",vertical_start_pose_.position.x ,vertical_start_pose_.position.y );

            }
            else
            {
                CmdVel.linear.x = 0.0;
                CmdVel.angular.z = PoseUtilities::signOf(sin(angleDiff)) * rotvel_;          
            }
        }        
        else if (state_ == STA_ROUTE_VERTICAL)
        {
            static std::string substate = "direct" ;
            static int lastforward = 0; // 0: direct ,1:left , -1:right
            int direction;
            if (route_vertical_direct_ == "direct")
                direction = 1;
            else if (route_vertical_direct_ == "inverse"  )
                direction = -1;
            else
            {
                ROS_INFO("config route_vertical_direct_ error ,check config ");
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
                    routedis = PoseUtilities::distance(get_pose_odom_,pose_standby_odom_);
                }
                else
                {
                    routedis = PoseUtilities::distance(curpose,pose_standby_);
                }                
                double extradis = fabs(distance_vertical_) + 0.3;
                double distDiff = fabs(distance_vertical_) - routedis;
                //第二条路径行驶距离超出计算值 30cm ；偏差过大，说明前面有问题
                if (routedis > extradis)
                {
                    CmdVel.linear.x = 0.0;
                    CmdVel.linear.y = 0.0;
                    state_ = STA_FAILED;
                    ROS_INFO("fail at STA_ROUTE_VERTICAL ,routedis too far , route1 rotate wrong ");
                }
                else if (fabs(distDiff) < xy_tolerance_)
                {
                    CmdVel.linear.x = 0.0;
                    CmdVel.angular.z = 0.0;
                    if(fabs(target_rela_pose_[2]) < 0.01)
                    {
                        state_ = STA_PRE_NEXT;
                        ROS_INFO(" STA_ROUTE_VERTICAL finish ,to STA_PRE_NEXT  ");
                        ROS_INFO(" arrive curpose is :[%f, %f]",curpose.position.x , curpose.position.y);
                        if(debug_count_ > 0)
                        {
                            //state_ = STA_DEBUG;
                        }

                    }
                    else
                    {
                        state_ = STA_PRE_ORIENTATION;
                        ROS_INFO(" STA_ROUTE_VERTICAL finish ,to sta STA_PRE_ORIENTATION ");
                    }                
                }
                else
                {
                    CmdVel.linear.x = direction * PoseUtilities::signOf(distDiff) * PoseUtilities::signOf(distance_vertical_) * xyvel_;
                    CmdVel.linear.y = 0.0;
                }
            }
            else
            {
                // inertial navigation logic
                geometry_msgs::TransformStamped transBaselinkMap = listenTf(mapframe_.c_str(), base_link_frame_, ros::Time(0));
                double angleBaselink = PoseUtilities::toEuler(transBaselinkMap.transform.rotation)[2];
                geometry_msgs::Pose curpose;
                curpose.position.x = transBaselinkMap.transform.translation.x;
                curpose.position.y = transBaselinkMap.transform.translation.y;
                double diffX = pose_end_.position.x - curpose.position.x;
                double diffY = pose_end_.position.y - curpose.position.y;
                double diffAngle = get_align_angle_ - PoseUtilities::toEuler(transBaselinkMap.transform.rotation)[2];
                if (issetimu_)
                {
                    diffAngle = angles::shortest_angular_distance(yawFromImu, get_align_imu_);
                }
                double curdis = PoseUtilities::distance(curpose,pose_end_);
                //ROS_INFO("diffx: %f, diffy: %f,diffAngle: %f ,curdis:%f ",diffX,diffY,diffAngle,curdis);
                if (fabs(diffX) < xy_tolerance_ && fabs(diffY) < xy_tolerance_ && substate != "endrot"  )
                {
                    CmdVel.linear.x = 0.0;
                    CmdVel.angular.z = 0.0;
                    if (fabs(target_rela_pose_[2]) < 0.01)
                    {
                        if (fabs(diffAngle) <=  yaw_tolerance_)
                        {
                            state_ = STA_PRE_NEXT;
                            ROS_INFO("diffX diffY min, STA_ROUTE_VERTICAL finish, to STA_PRE_NEXT ");
                            substate = "direct" ;
                            lastforward = 0;
                            return ;
                        }
                        else 
                        {
                            substate = "pre_endrot";
                            ROS_INFO(" diffX diffY min, start pre_endrot ");
                            //angle_target_imu_ = getrightImu(angle_target_imu_);                              
                        }
                    }
                    else
                    {
                        if (fabs(diffAngle) <=  yaw_tolerance_)
                        {
                            state_ = STA_PRE_ORIENTATION;
                            ROS_INFO("diffX diffY min, STA_ROUTE_VERTICAL finish, to sta STA_PRE_ORIENTATION");
                            substate = "direct" ;
                            lastforward = 0;
                            return ;
                        }
                        else 
                        {
                            substate = "pre_endrot"; 
                        }                        
                    }
                }

                geometry_msgs::Point pntEnd,pntCur,pntStart,pntNext,pntCross;
                pntEnd.x = pose_end_.position.x;
                pntEnd.y = pose_end_.position.y;
                pntCur.x = curpose.position.x;
                pntCur.y = curpose.position.y;       
                pntStart.x = vertical_start_pose_.position.x;
                pntStart.y = vertical_start_pose_.position.y;    
                geometry_msgs::Pose nextPose,poseDelta;
                poseDelta.position.x = direction * inertial_xyvel_ * predict_period_count_ / controller_frequency_;
                poseDelta.orientation = PoseUtilities::fromEuler(0.0, 0.0, 0.0);
                nextPose = PoseUtilities::applyTransform(poseDelta, transBaselinkMap);
                pntNext.x = nextPose.position.x;
                pntNext.y = nextPose.position.y;
                //ROS_INFO("curpose:[%f,%f] , nextpose:[%f,%f]",pntCur.x,pntCur.y,pntNext.x,pntNext.y);

                //求当前点到目标路径垂直交点             
                double k1 = (pntEnd.y - pntStart.y)/(pntEnd.x - pntStart.x);
                double k2 = (pntStart.x - pntEnd.x)/(pntEnd.y - pntStart.y);
                pntCross.x = (pntCur.y - pntStart.y + k1 * pntStart.x - k2 * pntCur.x)/(k1 - k2);
                pntCross.y = k1*(pntCross.x - pntStart.x) + pntStart.y;

                //ROS_INFO("pntCross:[%f,%f]",pntCross.x,pntCross.y);

                auto vecCrossNext = PoseUtilities::createVector2D(pntCross, pntNext);
                auto vecCurEnd = PoseUtilities::createVector2D(pntCur,pntEnd);
                auto vecStartEnd = PoseUtilities::createVector2D(pntStart,pntEnd);
                auto vecStartNext = PoseUtilities::createVector2D(pntStart,pntNext);
                auto vecStartCur = PoseUtilities::createVector2D(pntStart,pntCur);
                auto vecCurNext = PoseUtilities::createVector2D(pntCur,pntNext);

                double nextAngle = PoseUtilities::angleBetweenVectors2D(vecStartEnd, vecCrossNext);
                double startNextAngle = PoseUtilities::angleBetweenVectors2D(vecStartEnd, vecCurNext); 
                double disCrossNext = PoseUtilities::distance(pntNext,pntCross);
                double nextDelta = disCrossNext * sin(fabs(nextAngle));
                double curAngle = PoseUtilities::angleBetweenVectors2D(vecStartEnd,vecStartCur);
                //ROS_INFO("nextAngle: %f, nextDelta: %f, curAngle: %f ,startNextAngle:%f", nextAngle, nextDelta, curAngle,startNextAngle);
                if(PoseUtilities::signOf(nextAngle) != PoseUtilities::signOf(curAngle))
                {
                    //ROS_INFO("------------ +++++++++++++signOf(nextAngle) != signOf(curAngle)  --------");
                }

                double routedis = PoseUtilities::distance(pntStart,pntEnd);
                double curStartdis = PoseUtilities::distance(pntCur,pntStart);

                //double forwardAngle = PoseUtilities::angleBetweenVectors2D(vecForward, vecCurEnd);
                double curendAngle = PoseUtilities::angleBetweenVectors2D(vecCurEnd, vecStartEnd);
                //ROS_INFO("curendAngle: %f", curendAngle);
                if (substate == "direct" )
                {
                    if (fabs(curendAngle) >= M_PI / 2 || curStartdis >= routedis)
                    {
                        substate = "pre_endrot";
                        ROS_INFO("fabs(delta) >= M_PI / 2, start pre_endrot , curendAngle: %f",curendAngle);   
                        ROS_INFO("curdis : %f, routedis : %f",curStartdis, routedis);  
                    }
                    else
                    {
                        CmdVel.linear.x = direction * inertial_xyvel_;
                        if (nextDelta > predict_dist_thresh_ || diffAngle > yaw_tolerance_ * 2)
                        {

                            if(PoseUtilities::signOf(nextAngle) == PoseUtilities::signOf(startNextAngle))
                            {
                                CmdVel.angular.z = -1 * PoseUtilities::signOf(nextAngle) * inertial_rotvel_ *
                                    fabs(nextAngle) / 5.0;
                            }
                            else
                            {
                                // invert direction 
                                CmdVel.angular.z = PoseUtilities::signOf(nextAngle) * inertial_rotvel_ *
                                    fabs(nextAngle) / 5.0 *1.5 ;
                            }
                        }
                        else    
                        {
                            CmdVel.angular.z = 0.0;
                        }
                        CmdVel.angular.z = fabs(CmdVel.angular.z) > inertial_rotvel_ ?
                            PoseUtilities::signOf(CmdVel.angular.z) * inertial_rotvel_ : CmdVel.angular.z;
                        //ROS_INFO("CmdVel.angular.z: %f", CmdVel.angular.z);
                    }
                }
                else if (substate == "pre_endrot")
                {
                    pose_target_.orientation = PoseUtilities::fromEuler(0.0, 0.0, get_align_angle_); 
                    angle_target_imu_ = get_align_imu_; 
                    substate = "endrot";
                }
                else if (substate == "endrot" )
                {
                    geometry_msgs::TransformStamped transBaselinkMap = listenTf(mapframe_.c_str(), base_link_frame_, ros::Time(0));
                    // double angleDiff = PoseUtilities::toEuler(pose_target_.orientation)[2] - 
                    //     PoseUtilities::toEuler(transBaselinkMap.transform.rotation)[2];
                    double angleDiff = angles::shortest_angular_distance(
                        PoseUtilities::toEuler(transBaselinkMap.transform.rotation)[2], PoseUtilities::toEuler(pose_target_.orientation)[2]);
                    if (issetimu_)
                    {
                        // angleDiff = angle_target_imu_ - yawFromImu;
                        angleDiff = angles::shortest_angular_distance(yawFromImu, angle_target_imu_);
                    }
                    if (fabs(angleDiff) < yaw_tolerance_)
                    {
                        CmdVel.linear.x = 0.0;
                        CmdVel.angular.z = 0.0;

                        if (fabs(target_rela_pose_[2]) < 0.01)
                        {
                            state_ = STA_PRE_NEXT;
                            ROS_INFO("endrot STA_ROUTE_VERTICAL finish ,to STA_PRE_NEXT  ");
                        }
                        else
                        {
                            state_ = STA_PRE_ORIENTATION;
                            ROS_INFO("endrot STA_ROUTE_VERTICAL finish ,to sta STA_PRE_ORIENTATION ");

                        }
                        substate = "direct" ;
                        lastforward = 0;
                    }
                    else
                    {
                        CmdVel.linear.x = 0.0;
                        CmdVel.angular.z = PoseUtilities::signOf(sin(angleDiff)) * rotvel_;
                        //ROS_INFO("in endrot , rot");                
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

            angle_target_imu_ = yawFromImu + PoseUtilities::signOf(target_rela_pose_[2]) * rotangle;   
            angle_target_imu_ = getrightImu(angle_target_imu_);
            state_ = STA_TO_ORIENTATION;    
            ROS_INFO(" STA_PRE_ORIENTATION finish ,to sta STA_TO_ORIENTATION ,yawFromImu = %f ,angle_target_imu_ = %f ",yawFromImu,angle_target_imu_);       
        }
        else if (state_ == STA_TO_ORIENTATION)
        {
            geometry_msgs::TransformStamped transBaselinkMap = listenTf(mapframe_.c_str(), base_link_frame_, ros::Time(0));
            // double angleDiff = PoseUtilities::toEuler(pose_target_.orientation)[2] - 
            //     PoseUtilities::toEuler(transBaselinkMap.transform.rotation)[2];
            double angleDiff = angles::shortest_angular_distance(
                PoseUtilities::toEuler(transBaselinkMap.transform.rotation)[2], PoseUtilities::toEuler(pose_target_.orientation)[2]);
            if (issetimu_)
            {
                // angleDiff = angle_target_imu_ - yawFromImu;
                angleDiff = angles::shortest_angular_distance(yawFromImu, angle_target_imu_);
                if (debug_count_ == 7)
                {
                    ROS_INFO("in state STA_TO_ORIENTATION, angle_target_imu_ = %f, yawFromImu = %f, angleDiff = %f",
                        angle_target_imu_, yawFromImu, angleDiff);
                }
            }
            else
            {
                if (debug_count_ == 7)
                {
                    ROS_INFO("in state STA_TO_ORIENTATION, pose_target_angle = %f, curbaselinkangle = %f,angleDiff = %f",
                        PoseUtilities::toEuler(pose_target_.orientation)[2],PoseUtilities::toEuler(transBaselinkMap.transform.rotation)[2], angleDiff);
                }
            }              
            if (fabs(angleDiff) < yaw_tolerance_)
            {
                CmdVel.linear.x = 0.0;
                CmdVel.angular.z = 0.0;
                state_ = STA_PRE_NEXT;
                ROS_INFO(" STA_TO_ORIENTATION finish , to STA_PRE_NEXT  ");
            }
            else
            {
                CmdVel.linear.x = 0.0;
                CmdVel.angular.z = PoseUtilities::signOf(sin(angleDiff)) * rotvel_;
                prestate_ = state_;
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
                if (prestate_ == STA_REGISTRATE || prestate_ == STA_ADJUST_VERTICAL )
                {
                    state_ = STA_REGISTRATE_FINE;
                    ROS_INFO("start STA_REGISTRATE_FINE ");
                }
                else if (prestate_ == STA_START)
                {
                    state_ = STA_REGISTRATE;
                    ROS_INFO("start STA_REGISTRATE ");
                }
            }
        }
        else
        {
            CmdVel.linear.x = 0.0;
            CmdVel.linear.y = 0.0;
            CmdVel.angular.z = 0.0;
        }

        if (debug_count_ == 2)
        {
            CmdVel.linear.x = 0.0;
            CmdVel.linear.y = 0.0;
            CmdVel.angular.z = 0.0;
            return ;
        }
    }

    void LocatePoseRegistration::standby(const geometry_msgs::PoseStamped& PatternPose)
    {
        bool curposeright = checkcurpose();
        if (curposeright)
        {
            prestate_ = STA_START;
            state_ = STA_WAIT_SCAN;
            ROS_INFO("in standby");
            operate_index_ = 0;
        }
        else
        {
            state_ = STA_FAILED;
            ROS_INFO("in standby , but curpose is wrong ,check config or not near the pattern");
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

        ROS_INFO("getfeature, name is %s" , getfeature.name.c_str());
        geometry_msgs::Pose getfea_cur_pose;
        getfea_cur_pose.position.x = getfeature.cur_pose[0];
        getfea_cur_pose.position.y = getfeature.cur_pose[1];
        double posedis = PoseUtilities::distance(curpose,getfea_cur_pose);
        ROS_INFO("curpose :[%f, %f], pose dis is %f" ,curpose.position.x,curpose.position.y, posedis);
        if (posedis > pattern_met_location_thresh_)
        {
            return false;
        }
        else
        {
            pose_feature_.position.x = getfeature.feature_pose[0];
            pose_feature_.position.y = getfeature.feature_pose[1];
            target_rela_pose_.resize(3);
            target_rela_pose_vec_.assign(getfeature.target_rela_pose_vec.begin(),getfeature.target_rela_pose_vec.end());
            getfea_cur_pose_.position.x = getfeature.cur_pose[0];
            getfea_cur_pose_.position.y = getfeature.cur_pose[1];
            getfea_cur_pose_.orientation = PoseUtilities::fromEuler(0.0, 0.0, getfeature.cur_pose[2]);
            ROS_INFO("getfea_cur_pose_.orientation yaw : %f",getfeature.cur_pose[2]);
            /*
            if (PoseUtilities::signOf(target_rela_pose_[1]) == 0 )
            {
                leftorright_ = 1;
            }
            else
            {
                leftorright_ = PoseUtilities::signOf(target_rela_pose_[1]);
            }*/

            std::string model_cloud_file;
            model_cloud_file = model_cloud_path_ + "/" + getfeature.name + ".pcd";
            target_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
            if (pcl::io::loadPCDFile<pcl::PointXYZ>(model_cloud_file.c_str(), *target_cloud_) == -1)
            {
                PCL_ERROR("Can not find the file model_cloud ");
                return false;
            }
            ROS_INFO("Loaded : %d from the model_cloud file", target_cloud_->size());

            return true;
        }
    }

    void LocatePoseRegistration::updateCurrentPose()
    {
        geometry_msgs::TransformStamped transBaselinkMap = listenTf(mapframe_.c_str(), base_link_frame_, ros::Time(0));
        pose_standby_.position.x = transBaselinkMap.transform.translation.x;    
        pose_standby_.position.y = transBaselinkMap.transform.translation.y; 

        pose_standby_odom_.position.x = get_pose_odom_.position.x;
        pose_standby_odom_.position.y = get_pose_odom_.position.y;
    }

    void LocatePoseRegistration::subCallbackLaserScan(const sensor_msgs::LaserScan::ConstPtr& Laser)
    {
        if (queue_scan_)
        {
            queue_scan_->produce(std::bind(&LocatePoseRegistration::registration, this,
                Laser), "registration");
        }

        if (state_ == STA_REGISTRATE || state_ == STA_REGISTRATE_FINE || state_ == STA_DEBUG)
        {
            std::lock_guard<std::mutex> lk(mtx_cv_);
            cv_.notify_all();
        }
    }

    void LocatePoseRegistration::subCallbackImu(const sensor_msgs::Imu::ConstPtr& Imudata)
    {
        const std::lock_guard<std::mutex> lock(mtx_imu_);
        tf2::Quaternion quaternion(Imudata->orientation.x, Imudata->orientation.y, Imudata->orientation.z,
            Imudata->orientation.w);
        angleyaw_imu_ = PoseUtilities::toEuler(quaternion)[2];
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

    std::shared_ptr<void> LocatePoseRegistration::registration(const sensor_msgs::LaserScan::ConstPtr& Laser)
    {
        ros::Time begin = ros::Time::now();

        /// convert to pcl cloud
        auto pclCloud = PclUtilities<pcl::PointXYZ>::fromMsgLaserScan(*Laser);
        operate_index_++;
        ROS_INFO("Loaded : %d from the laserscan" , pclCloud->size());
        ROS_INFO("*************************************-----------------registration operate_index=%d",operate_index_);
        
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
        ROS_INFO("start segment_don");
        PclUtilities<pcl::PointXYZ>::segment_don(pclCloud, target_cloud_, 
            outcloudvec, seg_scale1_, seg_scale2_, seg_threshold_, seg_radius_, ndtsample_coeffs_, ndtmaxiter_);
        ROS_INFO("finishi segment_don");
        if (outcloudvec.size() == 0)
        {
            state_ = STA_FAILED;
            std::string outfile;
            outfile = packpath_ + "/pcd/testgetall.pcd";
            pcl::io::savePCDFileASCII (outfile, *pclCloud);                 
            ROS_INFO(" segment_don , get none features , check config or condition");
            return nullptr;
        }

        //--------------------从几个可能的特征找出最近的-----------
        // filter the target out within multiple patterns
        // comparation in laser frame
        geometry_msgs::Pose originpose;
        originpose.position.x = 0;
        originpose.position.y = 0;
        double center_dist_min = std::numeric_limits<double>::max();
        for (auto oneiter = outcloudvec.begin(); oneiter != outcloudvec.end(); oneiter++)
        {
            std::vector<mypoint> pvec;
            for (int i = 0; i < (*oneiter)->points.size(); ++i)
            {
                mypoint onepoint;
                onepoint.x = (*oneiter)->points[i].x;
                onepoint.y = (*oneiter)->points[i].y;
                pvec.push_back(onepoint);
            }
            double minradius;
            mypoint center;
            min_circle_cover(pvec, minradius, center);
            geometry_msgs::Pose centerpoint;
            centerpoint.position.x = center.x;
            centerpoint.position.y = center.y;
            double center_dist = PoseUtilities::distance(originpose,centerpoint);
            if(center_dist < center_dist_min)
            {
                center_dist_min = center_dist;
                *outcloud = **oneiter ;
            }
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
        ROS_INFO("outcloud ,radius : %.10lf \n  center.x: %.10lf center.y: %.10lf ",minradius,center.x, center.y);
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
        ROS_INFO("targetcloud ,radius : %.10lf \n  center.x: %.10lf center.y: %.10lf ",tarminradius,tarcenter.x, tarcenter.y);
        
        // check if the filtered target meet the template pattern
        double delta_radius = minradius - tarminradius;
        ROS_INFO("delta_radius is: %f ",fabs(delta_radius));
        if(fabs(delta_radius) > pattern_met_radius_thresh_)
        {
            ROS_INFO("outcloud radius is far from target radius, maybe wrong");
            state_ = STA_FAILED;
            return nullptr;
        }

        //--------------------- 在原点云上 mincut ------
        ROS_INFO("start segmentMinCut");
        std::vector<pcl::PointIndices> minclusterIndices;
        pcl::PointXYZ minpointCenter;
        minpointCenter.x = center.x;
        minpointCenter.y = center.y;
        minpointCenter.z = 0;

        minclusterIndices = PclUtilities<pcl::PointXYZ>::segmentMinCut(pclCloud, minpointCenter,
            minradius, cut_min_neighbour_, sigma_, weight_);
        ROS_INFO("total minclusters number from epic: %d", minclusterIndices.size()) ;
        for (int i = 0; i < minclusterIndices.size(); ++i)
        {
            ROS_INFO("mincluster %d has points %d" , i, minclusterIndices[i].indices.size());
        }
        int ci = 0;
        pcl::PointCloud<pcl::PointXYZ>::Ptr minoutcloud(new pcl::PointCloud<pcl::PointXYZ>) ;
        for (const auto& cluster : minclusterIndices)
        {
            ci++;
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFeature(new pcl::PointCloud<pcl::PointXYZ>());
            PclUtilities<pcl::PointXYZ>::extractTo(pclCloud, cluster, cloudFeature);
            ROS_INFO("extractTo cluster indices = %d , cloudFeature->size() =%d " , ci,cloudFeature->size());

            if (!cloudFeature->empty() && cloudFeature->size() < mincut_size_)
            {
                *minoutcloud = *cloudFeature;
            }
        }
        if (minoutcloud->points.empty())
        {
            state_ = STA_FAILED;
            ROS_INFO("segmentMinCut fail");
            return nullptr;
        }            
        ROS_INFO("finish segmentMinCut, get outcloud");
        static PclVisualize<pcl::PointXYZ> viewer;
        if (state_ == STA_DEBUG)
        {
            ROS_INFO("add cur cloud while registration done");
            
            viewer.addviewCloud(minoutcloud,"after_cur_features");
            state_ = STA_DONE;
            return nullptr;
        }

        ROS_INFO("after segment, start registration ......");
        /// registration
        Eigen::Vector3f registAngles;
        double score;
        std::vector<double> transxy;
        Eigen::Matrix4f outmatrix;
        if (!PclUtilities<pcl::PointXYZ>::regist_sacia_ndt(target_cloud_, minoutcloud,
            registAngles, transxy, score, ndtsample_coeffs_, ndtmaxiter_, outmatrix, true))
        {
            state_ = STA_FAILED;
            ROS_INFO("registration failed");
            return nullptr;
        }
        ROS_INFO("out regist, roll:%f, pitch:%f, yaw: %f, x trans: %f, ytrans: %f",
            registAngles[0], registAngles[1],  registAngles[2], transxy[0], transxy[1]);
        
        if (debug_count_ > 0)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr transcloud(new pcl::PointCloud<pcl::PointXYZ>) ;
            pcl::transformPointCloud(*minoutcloud, *transcloud, outmatrix);
            //static PclVisualize<pcl::PointXYZ> viewer;
            viewer.viewCloud02(minoutcloud,"cur_features" , transcloud, "debug_features", target_cloud_,"target_features");
        }
        if (debug_count_ == 1 || debug_count_ == 3)
        {
            ROS_INFO("debug_count_ == 1 or 3 ,outfile");
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
            ROS_INFO("registangle > 0.5 pi, it is wrong, registration fail");
            return nullptr;
        }
        
        // 转换矩阵正常的情况
        geometry_msgs::TransformStamped transBaselinkMap = listenTf(mapframe_.c_str(), base_link_frame_, ros::Time(0));
        if (fabs(registAngles[0]) < 0.02 && fabs(registAngles[1]) < 0.02)
        {
            // yaw aligned, move to done or linear alignment if there is
            if (fabs(transxy[0]) < regist_linear_thresh_ && fabs(transxy[1]) < regist_linear_thresh_ &&
                fabs(registAngles[2]) < regist_yaw_thresh_)
            {
                ROS_INFO("align right , next state");
                ROS_INFO("to STA_PRE_HORIZON ");
                state_ = STA_PRE_HORIZON;
                return nullptr;
            }

            // otherwise, rotate to align
            ROS_INFO("%s", fabs(registAngles[2]) < yaw_tolerance_ ? 
                std::string(" algin angle met ").c_str() : std::string("algin angle unmet ").c_str());

            double angleBaselink = PoseUtilities::toEuler(transBaselinkMap.transform.rotation)[2];
            // pose_target_.orientation = PoseUtilities::fromEuler(0.0, 0.0, angleBaselink - registAngles[2]);
            pose_target_.orientation = PoseUtilities::fromEuler(0.0, 0.0,
                angles::shortest_angular_distance(registAngles[2], angleBaselink));
            double yawFromImu = 0.0;
            {
                const std::lock_guard<std::mutex> lock(mtx_imu_);
                yawFromImu = angleyaw_imu_;
            }
            // angle_target_imu_ = yawFromImu - registAngles[2];
            angle_target_imu_ = angles::shortest_angular_distance(registAngles[2], yawFromImu);
            ROS_INFO("start sta_to_horizon, angle_target_imu_ = %f , now yawFromImu = %f",
                angle_target_imu_, yawFromImu);

            distance_vertical_ = transxy[0];
            distance_horizon_ = transxy[1];
            ROS_INFO("distance_vertical = %f, distance_horizon_ = %f", distance_vertical_, distance_horizon_);

            angleBaselink = PoseUtilities::toEuler(transBaselinkMap.transform.rotation)[2];
            updateCurrentPose(); 
            pose_standby_.orientation = PoseUtilities::fromEuler(0.0, 0.0, angleBaselink);

            prestate_ = state_;
            state_ = STA_ALIGN;
        }
        else
        {
            // 转换矩阵奇异的情况：
            // 暂时设为失败，后面再改

            state_ = STA_FAILED;
            ROS_INFO("process fail, registration failed");
        }

        ROS_INFO("processing time: %f",(ros::Time::now() - begin).toSec());

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

    void LocatePoseRegistration::odomCallback(const nav_msgs::Odometry::ConstPtr& Odom)
    {
        ROS_INFO_STREAM_ONCE("Odometry received on topic " << odom_topic_);
        // we assume that the odometry is published in the frame of the base
        //boost::mutex::scoped_lock lock(odom_mutex_);
        base_odom_ = *Odom;
        if (base_odom_.header.stamp.isZero())
            base_odom_.header.stamp = ros::Time::now();
        //ROS_INFO("base_odom. pose,position is:[%f,%f]",Odom->pose.pose.position.x,Odom->pose.pose.position.y);
        geometry_msgs::TransformStamped transBaselinkMap = listenTf(mapframe_, base_link_frame_, ros::Time(0));
        geometry_msgs::Pose pointpose;
        pointpose.position.x = Odom->pose.pose.position.x;
        pointpose.position.y = Odom->pose.pose.position.y;
        pointpose.position.z = Odom->pose.pose.position.z;
        pointpose.orientation = Odom->pose.pose.orientation;
        get_pose_odom_.position.x = Odom->pose.pose.position.x;
        get_pose_odom_.position.y = Odom->pose.pose.position.y;
        auto aftertrans_pose = PoseUtilities::applyTransform(pointpose, transBaselinkMap);
        //ROS_INFO("getpose is :[%f,%f]",aftertrans_pose.position.x,aftertrans_pose.position.y);
        //ROS_INFO("cur tfpose is:[%f,%f]",transBaselinkMap.transform.translation.x,transBaselinkMap.transform.translation.y);

    }

    PLUGINLIB_EXPORT_CLASS(pose_registration_plugins::LocatePoseRegistration, whi_pose_registration::BasePoseRegistration)
} // namespace pose_registration_plugins
