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
	    std::cout << "\nWHI loacate pose registration plugin VERSION 00.04.1" << std::endl;
	    std::cout << "Copyright © 2024-2025 Wheel Hub Intelligent Co.,Ltd. All rights reserved\n" << std::endl;
    }

    LocatePoseRegistration::~LocatePoseRegistration()
    {
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
        printf("configfile is %s \n",config_file.c_str());
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
                    for (const auto& item : pair.second)
                    {
                        onefeature.target_rela_pose.push_back(item.as<double>());
                    }
                }                    

            }
            features_config_.push_back(onefeature);

        }
        

        for (auto oneiter = features_config_.begin(); oneiter!= features_config_.end(); oneiter++)
        {
            printf("onefeature: \n");
            printf("name: %s ,cur_pose: [ %f, %f, %f  ]" , (*oneiter).name.c_str() , (*oneiter).cur_pose[0],(*oneiter).cur_pose[1],(*oneiter).cur_pose[2] );
            printf("feature_pose: [%f, %f, %f]",(*oneiter).feature_pose[0],(*oneiter).feature_pose[1],(*oneiter).feature_pose[2]);
            printf("target_rela_pose: [%f, %f, %f]",(*oneiter).target_rela_pose[0],(*oneiter).target_rela_pose[1],(*oneiter).target_rela_pose[2]);

        }
        
        node_handle_->param("pose_registration/xy_tolerance", xy_tolerance_, 0.02);
        node_handle_->param("pose_registration/yaw_tolerance", yaw_tolerance_, 5.0);
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
        node_handle_->param("pose_registration/LocatePose/pattern_met_distance", curpose_thresh_, 0.5);  
        node_handle_->param("pose_registration/LocatePose/delta_radius_thresh", delta_radius_thresh_, 0.5);    

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
        sub_laser_scan_ = std::make_unique<ros::Subscriber>(node_handle_->subscribe<sensor_msgs::LaserScan>(
		    laserScanTopic, 10, std::bind(&LocatePoseRegistration::subCallbackLaserScan, this, std::placeholders::_1)));
        sub_imu_ = std::make_unique<ros::Subscriber>(node_handle_->subscribe<sensor_msgs::Imu>(
		    imuTopic, 10, std::bind(&LocatePoseRegistration::subCallbackImu, this, std::placeholders::_1)));

        // event queue for laser scan
        queue_scan_ = std::make_unique<EventQueue<void>>(10, false);
        threadRegistration();
    }

    void LocatePoseRegistration::computeVelocityCommands(geometry_msgs::Twist& CmdVel)
    {
        const std::lock_guard<std::mutex> lock(mtx_imu_);
        if (state_ == STA_ALIGN)
        {
            geometry_msgs::TransformStamped transBaselinkMap = listenTf(mapframe_, base_link_frame_, ros::Time(0));
            double angleDiff = PoseUtilities::toEuler(pose_target_.orientation)[2] - 
                PoseUtilities::toEuler(transBaselinkMap.transform.rotation)[2];
            if (issetimu_)
            {
                angleDiff = angletar_imu_ - angleyaw_imu_;
            }
            if (debug_count_ == 5)
            {
                ROS_INFO("in state STA_ALIGN , angletar_imu_ = %f ,angleyaw_imu_=%f ,angleDiff=%f ",angletar_imu_,angleyaw_imu_,angleDiff );
            }                
            if (fabs(angleDiff) < yaw_tolerance_)
            {
                CmdVel.linear.x = 0.0;
                CmdVel.angular.z = 0.0;
                if (prestate_ == STA_REGISTRATE_FINE)
                {
                    state_ = STA_PRE_HORIZON;
                    
                    if (iszerorela_ && fabs(distance_horizon_) < distthresh_horizon_)
                    {
                        state_ = STA_ROUTE_VERTICAL;
                        ROS_INFO(" zero config, STA_REGISTRATE_FINE finish ,direct to STA_ROUTE_VERTICAL ");
                    }
                    ROS_INFO("STA_REGISTRATE_FINE finish, start STA_PRE_HORIZON");
                    ROS_INFO("STA_REGISTRATE_FINE finish , angleyaw_imu_ =%f ",angleyaw_imu_);
                }
                else if (prestate_ == STA_REGISTRATE)
                {
                    state_ = STA_WAIT_SCAN;
                    ROS_INFO("vertical finish, wait scan ");
                }
            }
            else
            {
                CmdVel.linear.x = 0.0;
                CmdVel.angular.z = PoseUtilities::signOf(sin(angleDiff)) * rotvel_;
            }
        }
        else if (state_ == STA_TO_VERTICAL)
        {
            geometry_msgs::TransformStamped transBaselinkMap = listenTf(mapframe_.c_str(), base_link_frame_, ros::Time(0));
            double angleDiff = PoseUtilities::toEuler(pose_target_.orientation)[2] - 
                PoseUtilities::toEuler(transBaselinkMap.transform.rotation)[2];
            if (issetimu_)
                angleDiff = angletar_imu_ - angleyaw_imu_ ;
            if (debug_count_ == 5)
            {
                ROS_INFO("in state STA_TO_VERTICAL , angletar_imu_ = %f ,angleyaw_imu_=%f ,angleDiff=%f ",angletar_imu_,angleyaw_imu_,angleDiff );
            }
            if (fabs(angleDiff) < yaw_tolerance_)
            {
                CmdVel.linear.x = 0.0;
                CmdVel.angular.z = 0.0;

                state_ = STA_ROUTE_VERTICAL;
                ROS_INFO("STA_TO_VERTICAL finish ,start STA_ROUTE_VERTICAL " );
            }
            else
            {
                CmdVel.linear.x = 0.0;
                CmdVel.angular.z = PoseUtilities::signOf(sin(angleDiff)) * rotvel_;
                prestate_ = state_;
            }
        }
        else if (state_ == STA_PRE_HORIZON)
        {
            geometry_msgs::TransformStamped transBaselinkMap = listenTf(mapframe_.c_str(), base_link_frame_, ros::Time(0));
            double angleBaselink = PoseUtilities::toEuler(transBaselinkMap.transform.rotation)[2];
            if (iszerorela_ )
            {
                pose_target_.orientation = PoseUtilities::fromEuler(0.0, 0.0, angleBaselink + PoseUtilities::signOf(distance_horizon_) * 0.5 * M_PI);
                angletar_imu_ = angleyaw_imu_ + PoseUtilities::signOf(distance_horizon_) * 0.5 * M_PI;
            }else
            {
                pose_target_.orientation = PoseUtilities::fromEuler(0.0, 0.0, angleBaselink + leftorright_ * 0.5 * M_PI);
                angletar_imu_ = angleyaw_imu_ + leftorright_ * 0.5 * M_PI;
            }            
            angletar_imu_ = getrightImu(angletar_imu_);
            state_ = STA_TO_HORIZON;
            ROS_INFO("STA_PRE_HORIZON finish,  start STA_TO_HORIZON "); 
        }
        else if (state_ == STA_TO_HORIZON)
        {
            geometry_msgs::TransformStamped transBaselinkMap = listenTf(mapframe_.c_str(), base_link_frame_, ros::Time(0));
            double angleDiff = PoseUtilities::toEuler(pose_target_.orientation)[2] - 
                PoseUtilities::toEuler(transBaselinkMap.transform.rotation)[2];
            if (issetimu_)
                angleDiff = angletar_imu_ - angleyaw_imu_ ;
            if (debug_count_ == 5)
            {
                ROS_INFO("in state STA_TO_HORIZON , angletar_imu_ = %f ,angleyaw_imu_=%f ,angleDiff=%f ",angletar_imu_,angleyaw_imu_,angleDiff );
            }
            if (fabs(angleDiff) < yaw_tolerance_)
            {
                CmdVel.linear.x = 0.0;
                CmdVel.angular.z = 0.0;

                state_ = STA_ROUTE_HORIZON;
                ROS_INFO("sta_to_horizon finish , angleyaw_imu_ =%f ",angleyaw_imu_);
                ROS_INFO("STA_TO_HORIZON finish ");

            }
            else
            {
                CmdVel.linear.x = 0.0;
                CmdVel.angular.z = PoseUtilities::signOf(sin(angleDiff)) * rotvel_;
                if (debug_count_ == 5)
                {
                    ROS_INFO("angular.z : %f ",CmdVel.angular.z);
                }
                prestate_ = state_;            
            }
        }
        else if (state_ == STA_ROUTE_HORIZON)
        {
            geometry_msgs::TransformStamped transBaselinkMap = listenTf(mapframe_.c_str(), base_link_frame_, ros::Time(0));
            geometry_msgs::Pose curpose;
            curpose.position.x = transBaselinkMap.transform.translation.x;
            curpose.position.y = transBaselinkMap.transform.translation.y;
            double routedis = PoseUtilities::distance(curpose,pose_standby_);
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
                CmdVel.linear.x = PoseUtilities::signOf(distDiff) * xyvel_;
                CmdVel.linear.y = 0.0;
            }
        }
        else if (state_ == STA_PRE_VERTICAL)
        {
            geometry_msgs::TransformStamped transBaselinkMap = listenTf(mapframe_.c_str(), base_link_frame_, ros::Time(0));
            double angleBaselink = PoseUtilities::toEuler(transBaselinkMap.transform.rotation)[2];
            pose_target_.position.x = 0.0;
            pose_target_.position.y = 0.0;
            if (iszerorela_)
            {
                pose_target_.orientation = PoseUtilities::fromEuler(0.0, 0.0, angleBaselink - PoseUtilities::signOf(distance_horizon_) * 0.5 * M_PI); 
                angletar_imu_ = angleyaw_imu_ - PoseUtilities::signOf(distance_horizon_) * 0.5 * M_PI;
            }
            else
            {
                pose_target_.orientation = PoseUtilities::fromEuler(0.0, 0.0, angleBaselink - leftorright_ * 0.5 * M_PI); 
                angletar_imu_ = angleyaw_imu_ - leftorright_ * 0.5 * M_PI;
            }
            angletar_imu_ = getrightImu(angletar_imu_);
            ROS_INFO("in state STA_PRE_VERTICAL finish , angletar_imu_ = %f ,angleyaw_imu_=%f",angletar_imu_,angleyaw_imu_ );
            //进入第二条路径，更改当前点为起始点
            pose_standby_.position.x = transBaselinkMap.transform.translation.x;    
            pose_standby_.position.y = transBaselinkMap.transform.translation.y;  
            ROS_INFO("STA_PRE_VERTICAL finish,  start STA_TO_VERTICAL "); 
            state_ = STA_TO_VERTICAL;
        }
        else if (state_ == STA_ROUTE_VERTICAL)
        {
            geometry_msgs::TransformStamped transBaselinkMap = listenTf(mapframe_.c_str(), base_link_frame_, ros::Time(0));
            geometry_msgs::Pose curpose;
            curpose.position.x = transBaselinkMap.transform.translation.x;
            curpose.position.y = transBaselinkMap.transform.translation.y;
            double routedis = PoseUtilities::distance(curpose,pose_standby_);
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
                if(fabs(target_rela_pose_[2]) < 1 || iszerorela_)
                {
                    state_ = STA_DONE;
                    ROS_INFO(" STA_ROUTE_VERTICAL finish ,arrive done ");

                }else
                {
                    state_ = STA_PRE_ORIENTATION;
                    ROS_INFO(" STA_ROUTE_VERTICAL finish ,to sta STA_PRE_ORIENTATION ");
                }                
            }
            else
            {
                CmdVel.linear.x = PoseUtilities::signOf(distDiff) * PoseUtilities::signOf(distance_vertical_) * xyvel_;
                CmdVel.linear.y = 0.0;
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
            angletar_imu_ = angleyaw_imu_ + PoseUtilities::signOf(target_rela_pose_[2]) * rotangle;   
            angletar_imu_ = getrightImu(angletar_imu_);
            state_ = STA_TO_ORIENTATION;    
            ROS_INFO(" STA_TO_ORIENTATION finish ,to sta STA_TO_ORIENTATION ");       
        }
        else if (state_ == STA_TO_ORIENTATION)
        {
            geometry_msgs::TransformStamped transBaselinkMap = listenTf(mapframe_.c_str(), base_link_frame_, ros::Time(0));
            double angleDiff = PoseUtilities::toEuler(pose_target_.orientation)[2] - 
                PoseUtilities::toEuler(transBaselinkMap.transform.rotation)[2];
            if (issetimu_)
                angleDiff = angletar_imu_ - angleyaw_imu_ ;
            if (debug_count_ == 5)
            {
                ROS_INFO("in state STA_TO_ORIENTATION , angletar_imu_ = %f ,angleyaw_imu_=%f ,angleDiff=%f ",angletar_imu_,angleyaw_imu_,angleDiff );
            }                
            if (fabs(angleDiff) < yaw_tolerance_)
            {
                CmdVel.linear.x = 0.0;
                CmdVel.angular.z = 0.0;
                state_ = STA_DONE;
                ROS_INFO(" STA_TO_ORIENTATION finish ,arrive done ");
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

            CmdVel.angular.z = 0;

            if ((timeNow - preTime).toSec() >= 0.2 )
            {
                index = 0;
                if (prestate_ == STA_REGISTRATE)
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
        else if (state_ == STA_WAIT_IMU)
        {
            static auto preTime = ros::Time::now();
            auto timeNow = ros::Time::now();
            static int index = 0;

            if (index == 0)
            {
                preTime = timeNow;
                index = 1;
            }

            CmdVel.angular.z = 0;

            if ((timeNow - preTime).toSec() >= 0.01 )
            {
                index = 0;
                if (prestate_ == STA_REGISTRATE || prestate_ == STA_REGISTRATE_FINE )
                {
                    state_ = STA_ALIGN;
                }
                else
                {
                    state_ = prestate_;
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
        if (posedis > curpose_thresh_)
        {
            return false;
        }
        else
        {
            pose_feature_.position.x = getfeature.feature_pose[0];
            pose_feature_.position.y = getfeature.feature_pose[1];
            target_rela_pose_.resize(3);
            target_rela_pose_[0] = getfeature.target_rela_pose[0];
            target_rela_pose_[1] = getfeature.target_rela_pose[1];
            target_rela_pose_[2] = getfeature.target_rela_pose[2];
            if (PoseUtilities::signOf(target_rela_pose_[1]) == 0 )
            {
                leftorright_ = 1;
            }
            else
            {
                leftorright_ = PoseUtilities::signOf(target_rela_pose_[1]);
            }

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

    void LocatePoseRegistration::subCallbackLaserScan(const sensor_msgs::LaserScan::ConstPtr& Laser)
    {
        if (state_ == STA_REGISTRATE || state_ == STA_REGISTRATE_FINE)
        {
            std::lock_guard<std::mutex> lk(mtx_cv_);
            cv_.notify_all();
        }
        else
        {
            if (queue_scan_)
            {
                queue_scan_->produce(std::bind(&LocatePoseRegistration::registration, this,
                    Laser), "registration");
            }
        }
    }

    void LocatePoseRegistration::subCallbackImu(const sensor_msgs::Imu::ConstPtr& Imudata)
    {
        const std::lock_guard<std::mutex> lock(mtx_imu_);
        tf2::Quaternion quaternion(Imudata->orientation.x, Imudata->orientation.y, Imudata->orientation.z,
            Imudata->orientation.w);
        angleyaw_imu_ = PoseUtilities::toEuler(quaternion)[2];
        //ROS_INFO("angleyaw_imu =%f ",angleyaw_imu_);
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
        ROS_INFO("Loaded : %d from the laserscan" , pclCloud->size());
        
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
            outfile = "/home/nvidia/catkin_workspace/src/whi_pose_registration_plugins/whi_locate_pose_registration/pcd/testgetall.pcd";
            pcl::io::savePCDFileASCII (outfile, *pclCloud);                 
            ROS_INFO(" segment_don , get none features , check config or condition");
            return nullptr;
        }

        //--------------------从几个可能的特征找出最近的-----------
        // Xinjue: compared with pose (0,0)?
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
        minradius = minradius + 0.050 ; 

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
        double delta_radius = minradius - tarminradius;
        ROS_INFO("delta_radius is: %f ",fabs(delta_radius));
        if(fabs(delta_radius) > delta_radius_thresh_)
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
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr transcloud(new pcl::PointCloud<pcl::PointXYZ>) ;
        pcl::transformPointCloud(*minoutcloud, *transcloud, outmatrix);
        if (debug_count_ > 0)
        {
            static PclVisualize<pcl::PointXYZ> viewer;
            viewer.viewCloud02(minoutcloud,"cur_features" , transcloud, "debug_features", target_cloud_,"target_features");
        }
        //ROS_INFO(" outmatrix: ");
        //std::cout << outmatrix << std::endl;
        ROS_INFO("after segment, start registration ......");
        
        if (debug_count_ == 1 || debug_count_ == 3)
        {
            ROS_INFO("debug_count_ == 1 or 3 ,outfile");
            std::string outfile;
            outfile = "/home/nvidia/catkin_workspace/src/whi_pose_registration_plugins/whi_locate_pose_registration/pcd/testgetseg.pcd";
            pcl::io::savePCDFileASCII (outfile, *minoutcloud);
            outfile = "/home/nvidia/catkin_workspace/src/whi_pose_registration_plugins/whi_locate_pose_registration/pcd/testgetall.pcd";
            pcl::io::savePCDFileASCII (outfile, *pclCloud); 

            if (debug_count_ == 1)
            {
                state_ = STA_DONE;
                return nullptr;
            }
        }

        geometry_msgs::TransformStamped transBaselinkMap = listenTf(mapframe_.c_str(), base_link_frame_, ros::Time(0));

        if (registAngles[2] > 0.5 * M_PI)
        {
            state_ = STA_FAILED;
            ROS_INFO("registangle > 0.5 pi, it is wrong, registration fail");
            return nullptr;
        }
        
        // 转换矩阵正常的情况
        if (fabs(registAngles[0]) < 0.02 && fabs(registAngles[1]) == 0)
        {
            if (fabs(registAngles[2]) < yaw_tolerance_)
            {
                // 正常到位,进入下一个状态，旋转至水平
                ROS_INFO("normal algin, sta to horizon");
            }
            else
            {
                // 需要精确对齐
                ROS_INFO("need algin, sta to vertical");
            }
            const std::lock_guard<std::mutex> lock(mtx_imu_);
            double angleBaselink = PoseUtilities::toEuler(transBaselinkMap.transform.rotation)[2];
            pose_target_.position.x = 0.0;
            pose_target_.position.y = 0.0;
            pose_target_.orientation = PoseUtilities::fromEuler(0.0, 0.0, angleBaselink - registAngles[2]);  
            angletar_imu_ = angleyaw_imu_ - registAngles[2];
            //state_ = STA_TO_HORIZON;       
            ROS_INFO("start sta_to_horizon, angletar_imu_ = %f , now angleyaw_imu_= %f",
                angletar_imu_, angleyaw_imu_);
            iszerorela_ = false;
            //根据当前位置和标靶特征位置，计算相对距离;distance_horizon_,distance_vertical_
            {
                distance_vertical_ = target_rela_pose_[0] + pose_feature_.position.y + transxy[0];      // this plus transxy[1] ,because forward
                distance_horizon_ = fabs(target_rela_pose_[1]) + leftorright_ * transxy[1] - leftorright_ * pose_feature_.position.x;      // this plus ,because to left

                if (fabs(target_rela_pose_[0]) < 0.001 && fabs(target_rela_pose_[1]) < 0.001)
                {
                    distance_vertical_ = transxy[0];
                    distance_horizon_ = transxy[1];
                    iszerorela_ = true;
                    double angleBaselink = PoseUtilities::toEuler(transBaselinkMap.transform.rotation)[2];
                    pose_target_.position.x = 0.0;
                    pose_target_.position.y = 0.0;
                    pose_target_.orientation = PoseUtilities::fromEuler(0.0, 0.0, angleBaselink - registAngles[2]);
                    angletar_imu_ = angleyaw_imu_ - registAngles[2];
                }
                ROS_INFO("distance_vertical = %f , distance_horizon_ = %f , iszerorela =%d",distance_vertical_, distance_horizon_, iszerorela_);

            }
            angletar_imu_ = getrightImu(angletar_imu_);
            angleBaselink = PoseUtilities::toEuler(transBaselinkMap.transform.rotation)[2];
            pose_standby_.position.x = transBaselinkMap.transform.translation.x;
            pose_standby_.position.y = transBaselinkMap.transform.translation.y;
            pose_standby_.orientation = PoseUtilities::fromEuler(0.0, 0.0, angleBaselink);
            distance_drive_ = 0.0;

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

                    EventQueue<void>::EventFunc eventRegistration(queue_scan_->consume("registration"));
                    if (eventRegistration)
                    {
                        // invoke the event means executing the action binded with it
                        eventRegistration();
                    }
                }
            }
        };
    }

    PLUGINLIB_EXPORT_CLASS(pose_registration_plugins::LocatePoseRegistration, whi_pose_registration::BasePoseRegistration)
} // namespace pose_registration_plugins
