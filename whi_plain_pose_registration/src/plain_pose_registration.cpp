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
#include "whi_pose_registration_common/pcl_utilities.h"
#include "whi_pose_registration_common/pcl_visualize.h"
#include "whi_pose_registration_common/pose_utilities.h"

#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>

namespace pose_registration_plugins
{
    PlainPoseRegistration::PlainPoseRegistration()
        : BasePoseRegistration()
    {
        /// node version and copyright announcement
	    std::cout << "\nWHI plain pose registration plugin VERSION 00.02.1" << std::endl;
	    std::cout << "Copyright © 2023-2025 Wheel Hub Intelligent Co.,Ltd. All rights reserved\n" << std::endl;
    }

    void PlainPoseRegistration::initialize()
    {
        /// params
        // common
        std::string laserScanTopic;
        std::vector<double> feature;
        bool pubStageTarget;
        if (!node_handle_->getParam("pose_registration/feature_pose", feature))
        {
            feature.resize(3);
        }
        else
        {
            pose_feature_.position.x = feature[0];
            pose_feature_.position.y = feature[1];
            pose_feature_.orientation = PoseUtilities::fromEuler(0.0, 0.0, angles::from_degrees(feature[2]));
        }
        node_handle_->param("pose_registration/distance_to_charge", distance_to_charge_, 0.0);
        node_handle_->param("pose_registration/xy_tolerance", xy_tolerance_, 0.02);
        node_handle_->param("pose_registration/yaw_tolerance", yaw_tolerance_, 5.0);
        yaw_tolerance_ = angles::from_degrees(yaw_tolerance_);
        node_handle_->param("pose_registration/PlainPose/tf_listener_frequency", tf_listener_frequency_, 20.0);
        node_handle_->param("pose_registration/PlainPose/laser_scan_topic", laserScanTopic, std::string("scan"));
        node_handle_->param("pose_registration/PlainPose/laser_frame", laser_frame_, std::string("laser"));
        node_handle_->param("pose_registration/PlainPose/base_link_frame", base_link_frame_, std::string("base_link"));
        node_handle_->param("pose_registration/PlainPose/feature_arch_radius", feature_arch_radius_, 0.06);
        node_handle_->param("pose_registration/PlainPose/feature_arch_radius_tolerance",
            feature_arch_radius_tolerance_, 0.015);
        node_handle_->param("pose_registration/PlainPose/downsampling", downsampling_, true);
        if (!node_handle_->getParam("pose_registration/PlainPose/downsampleing_coeffs", downsampling_coeffs_))
        {
            for (int i = 0; i < 3; ++i)
            {
                downsampling_coeffs_.push_back(0.01);
            }
        }
        node_handle_->param("pose_registration/PlainPose/line_distance_thresh", line_distance_thresh_, 0.001);
        node_handle_->param("pose_registration/PlainPose/circle_distance_thresh", circle_distance_thresh_, 0.001);
        node_handle_->param("pose_registration/PlainPose/feature_segment_distance_thresh",
            feature_segment_distance_thresh_, 0.04);
        node_handle_->param("pose_registration/PlainPose/feature_min_size", feature_min_size_, 10);
        node_handle_->param("pose_registration/PlainPose/feature_max_size", feature_max_size_, 200);
        node_handle_->param("pose_registration/PlainPose/segment_type", segment_type_, std::string("region_growing"));
        node_handle_->param("pose_registration/PlainPose/k_neighbour", k_neighbour_, 50);
        node_handle_->param("pose_registration/PlainPose/k_radius", k_radius_, 0.2);
        node_handle_->param("pose_registration/PlainPose/publish_stage_target", pubStageTarget, false);
        // specific
        node_handle_->param("pose_registration/PlainPose/min_cluster_size", min_cluster_size_, 50);
        node_handle_->param("pose_registration/PlainPose/max_cluster_size", max_cluster_size_, 1000);
        node_handle_->param("pose_registration/PlainPose/radius", radius_, 0.1);
        node_handle_->param("pose_registration/PlainPose/sigma", sigma_, 0.25);
        node_handle_->param("pose_registration/PlainPose/weight", weight_, 0.8);
        node_handle_->param("pose_registration/PlainPose/cut_min_neighbour", cut_min_neighbour_, 5);
        node_handle_->param("pose_registration/PlainPose/region_growing_neighbour", region_growing_neighbour_, 30);
        node_handle_->param("pose_registration/PlainPose/angle", angle_, 3.0);
        node_handle_->param("pose_registration/PlainPose/curvature", curvature_, 1.0);
        node_handle_->param("pose_registration/PlainPose/cluster_radius", cluster_radius_, 0.2);
        node_handle_->param("pose_registration/PlainPose/intensity_tolerance", intensity_tolerance_, 5.0);

        sub_laser_scan_ = std::make_unique<ros::Subscriber>(node_handle_->subscribe<sensor_msgs::LaserScan>(
		    laserScanTopic, 10, std::bind(&PlainPoseRegistration::subCallbackLaserScan, this, std::placeholders::_1)));
        if (pubStageTarget)
        {
            pub_stage_target_ = std::make_unique<ros::Publisher>(
                node_handle_->advertise<geometry_msgs::PoseStamped>("stage_target", 10, false));
        }

        ros::Duration updateFreq = ros::Duration(1.0 / tf_listener_frequency_);
        non_realtime_loop_ = std::make_unique<ros::Timer>(
            node_handle_->createTimer(updateFreq, std::bind(&PlainPoseRegistration::update, this, std::placeholders::_1)));
    }

    void PlainPoseRegistration::computeVelocityCommands(const geometry_msgs::PoseStamped& PatternPose,
        geometry_msgs::Twist& CmdVel)
    {
        geometry_msgs::TransformStamped transBaselinkMap;
        {
            const std::lock_guard<std::mutex> lock(mtx_cut_min_);
            transBaselinkMap = tf_baselink_map_;
        }

        if (state_ == STA_TO_VERTICAL || state_ == STA_TO_ALIGN)
        {
            double angleDiff = PoseUtilities::toEuler(pose_target_.orientation)[2] - 
                PoseUtilities::toEuler(transBaselinkMap.transform.rotation)[2];
            if (fabs(angleDiff) < yaw_tolerance_)
            {
                CmdVel.linear.x = 0.0;
                CmdVel.angular.z = 0.0;

                if (state_ == STA_TO_VERTICAL)
                {
                    state_ = STA_MOVE_VERTICAL;
                }
                else if (state_ == STA_TO_ALIGN)
                {
                    state_ = STA_ALIGNED;
                }
            }
            else
            {
                CmdVel.linear.x = 0.0;
                CmdVel.angular.z = PoseUtilities::signOf(sin(angleDiff)) * 0.1;
            }
        }
        else if (state_ == STA_MOVE_VERTICAL || state_ == STA_MOVE_ALIGN)
        {
            double xDiff = pose_target_.position.x - transBaselinkMap.transform.translation.x;
            double yDiff = pose_target_.position.y - transBaselinkMap.transform.translation.y;
            if (fabs(xDiff) < xy_tolerance_ && fabs(yDiff) < xy_tolerance_)
            {
                if (state_ == STA_MOVE_VERTICAL)
                {
                    double angleBaselink = PoseUtilities::toEuler(transBaselinkMap.transform.rotation)[2];
                    double sign = PoseUtilities::signOf(sin(found_feature_angle_ - angleBaselink));
                    CmdVel.linear.x = 0.0;
                    CmdVel.angular.z = sign * 0.1;

                    // target of rotating back to align with laser's x axis
                    pose_target_.position.x = 0.0;
                    pose_target_.position.y = 0.0;
                    pose_target_.orientation = PoseUtilities::fromEuler(0.0, 0.0, angleBaselink + sign * 0.5 * M_PI);

                    state_ = STA_TO_ALIGN;
                }
                else if (state_ == STA_MOVE_ALIGN)
                {
                    CmdVel.linear.x = 0.0;
                    CmdVel.angular.z = 0.0;

                    state_ = STA_DONE;
                }
            }
            else
            {
                geometry_msgs::Point pntBase;
                pntBase.x = transBaselinkMap.transform.translation.x;
                pntBase.y = transBaselinkMap.transform.translation.y;
                geometry_msgs::Point pntTarget;
                pntTarget.x = pose_target_.position.x;
                pntTarget.y = pose_target_.position.y;
                auto vecBase2Target = PoseUtilities::createVector2D(pntBase, pntTarget);
                auto vecXAxis = PoseUtilities::createVector2D(geometry_msgs::Point(), 1.0,
                    PoseUtilities::toEuler(transBaselinkMap.transform.rotation)[2]);
                auto angleFromX = PoseUtilities::angleBetweenVectors2D(vecXAxis, vecBase2Target);
                auto angleAbs = fabs(angleFromX);
                if (angleAbs < 0.5 * M_PI)
                {
                    // forward
                    CmdVel.linear.x = 0.06;
                    if (angleAbs > yaw_tolerance_)
                    {
                        CmdVel.angular.z = PoseUtilities::signOf(angleFromX) * 0.05;
                    }
                }
                else
                {
                    // backward
                    CmdVel.linear.x = -0.06;
                    if (0.5 * M_PI - angleAbs > yaw_tolerance_)
                    {
                        CmdVel.angular.z = PoseUtilities::signOf(-angleFromX) * 0.05;
                    }
                }
            }
        }
        else
        {
            CmdVel.linear.x = 0.0;
            CmdVel.angular.z = 0.0;
        }

        if (pub_stage_target_)
        {
            geometry_msgs::PoseStamped target;
            target.header.frame_id = "map";
            target.pose = pose_target_;
            pub_stage_target_->publish(target);
        }
    }

    void PlainPoseRegistration::standby()
    {
        state_ = STA_ALIGNED;
    }

    int PlainPoseRegistration::goalState()
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

    void PlainPoseRegistration::update(const ros::TimerEvent& Event)
    {
        {
            const std::lock_guard<std::mutex> lock(mtx_cut_min_);
            tf_baselink_map_ = listenTf("map", base_link_frame_, ros::Time(0));
            tf_laser_map_ = listenTf("map", laser_frame_, ros::Time(0));
            tf_map_laser_ = listenTf(laser_frame_, "map", ros::Time(0));
            tf_laser_baselink_ = listenTf(base_link_frame_, laser_frame_, ros::Time(0));
        }

        if (state_ == STA_ALIGNED)
        {
            // calculate the feature pose in laser frame
            auto featureInLaser = PoseUtilities::applyTransform(pose_feature_, tf_map_laser_);
            {
                const std::lock_guard<std::mutex> lock(mtx_cut_min_);
                center_.x = featureInLaser.position.x;
                center_.y = featureInLaser.position.y;
            }
#ifndef DEBUG
            std::cout << "featureInLaser x:" << featureInLaser.position.x << ",y:" << featureInLaser.position.y <<
                ",yaw:" << angles::to_degrees(PoseUtilities::toEuler(featureInLaser.orientation)[2]) << std::endl;
#endif
        }
    }

    void PlainPoseRegistration::subCallbackLaserScan(const sensor_msgs::LaserScan::ConstPtr& Laser)
    {
        if (state_ == STA_ALIGNED)
        {
            ros::Time begin = ros::Time::now();

#ifdef DEBUG
            // verify projected
            auto msgCloud2 = PclUtilities<>::msgLaserScanToMsgPointCloud2(*Laser);
            if (!pub_projected_)
            {
                pub_projected_ = std::make_unique<ros::Publisher>(
                    node_handle_->advertise<sensor_msgs::PointCloud2>("projected", 10, false));
            }
            pub_projected_->publish(msgCloud2);
            // verify converted cloud whether with RGB info
            auto pclCloudConverted = PclUtilities<pcl::PointXYZI>::fromMsgPointCloud2(msgCloud2);
            auto msgConverted = PclUtilities<pcl::PointXYZI>::toMsgPointCloud2(pclCloudConverted);
            if (!pub_converted_)
            {
                pub_converted_ = std::make_unique<ros::Publisher>(
                    node_handle_->advertise<sensor_msgs::PointCloud2>("converted", 10, false));
            }
            msgConverted.header.frame_id = "laser";
            pub_converted_->publish(msgConverted);
#endif
            /// convert to pcl cloud
            auto pclCloud = PclUtilities<pcl::PointXYZI>::fromMsgLaserScan(*Laser);
            /// downsampling
            auto pclOperating = pclCloud;
            if (downsampling_)
            {
                pclOperating = PclUtilities<pcl::PointXYZI>::downsampleVoxelGrid(pclCloud,
                    downsampling_coeffs_[0], downsampling_coeffs_[1], downsampling_coeffs_[2]);
#ifdef DEBUG
                auto msgDownsampled = PclUtilities<pcl::PointXYZI>::toMsgPointCloud2(pclOperating);
                if (!pub_downsampled_)
                {
                    pub_downsampled_ = std::make_unique<ros::Publisher>(
                        node_handle_->advertise<sensor_msgs::PointCloud2>("downsampled", 10, false));
                }
                msgDownsampled.header.frame_id = "laser";
                pub_downsampled_->publish(msgDownsampled);
#endif
            }
            /// segement features from epic
            std::vector<pcl::PointIndices> clusterIndices;
            if (segment_type_ == "cut_min")
            {
                pcl::PointXYZI pointCenter;
                {
                    const std::lock_guard<std::mutex> lock(mtx_cut_min_);
                    pointCenter.x = center_.x;
                    pointCenter.y = center_.y;
                    pointCenter.z = center_.z;
                }
                clusterIndices = PclUtilities<pcl::PointXYZI>::segmentMinCut(pclOperating, pointCenter,
                    radius_, cut_min_neighbour_, sigma_, weight_);
            }
            else if (segment_type_ == "region_growing")
            {
                clusterIndices = PclUtilities<pcl::PointXYZI>::segmentRegionGrowingKn(pclOperating,
                    k_neighbour_, region_growing_neighbour_, angles::from_degrees(angle_), curvature_,
                    min_cluster_size_, max_cluster_size_);
            }
            else if (segment_type_ == "cond_euclidean")
            {
                clusterIndices = PclUtilities<pcl::PointXYZI>::segmentConditionalEuclidean(pclOperating,
                    k_radius_, cluster_radius_, intensity_tolerance_, min_cluster_size_, max_cluster_size_);
            }
#ifndef DEBUG
            std::cout << "total clusters number from epic: " << clusterIndices.size() << std::endl;
            for (int i = 0; i < clusterIndices.size(); ++i)
            {
                std::cout << "cluster " << i << " has points " << clusterIndices[i].indices.size() << std::endl;
            }
#endif

            std::vector<std::vector<double>> foundCircles;
            for (const auto& cluster : clusterIndices)
            {
                pcl::PointCloud<pcl::PointXYZI>::Ptr cloudFeatures(new pcl::PointCloud<pcl::PointXYZI>());
                PclUtilities<pcl::PointXYZI>::extractTo(pclOperating, cluster, cloudFeatures);

                if (!cloudFeatures->empty() && cloudFeatures->size() < 500)
                {
#ifndef DEBUG
                    static PclVisualize<pcl::PointXYZI> viewer;
                    viewer.viewCloud(cloudFeatures, "debug_features");
#endif
                    /// segment each single feature from features
                    std::vector<pcl::PointIndices> featureIndices;
                    featureIndices = PclUtilities<pcl::PointXYZI>::segmentEuclidean(cloudFeatures,
                        feature_segment_distance_thresh_, feature_min_size_, feature_max_size_);
#ifdef DEBUG
                    std::cout << "total feature number from features: " << featureIndices.size() << std::endl;
                    for (int i = 0; i < featureIndices.size(); ++i)
                    {
                        std::cout << "feature " << i << " has points " << featureIndices[i].indices.size() << std::endl;
                    }
                    int i = 0;
#endif
                    for (const auto& feature : featureIndices)
                    {
                        pcl::PointCloud<pcl::PointXYZI>::Ptr cloudFeature(new pcl::PointCloud<pcl::PointXYZI>());
                        PclUtilities<pcl::PointXYZI>::extractTo(cloudFeatures, feature, cloudFeature);

#ifdef DEBUG
                        static PclVisualize<pcl::PointXYZI> viewer;
                        viewer.viewCloud(cloudFeature, "debug_feature" + std::to_string(i), 1 + i * 3);
                        ++i;
#endif
                        /// sample consensus
                        std::vector<int> inliersIndicesLine;
                        std::vector<double> coeffsLine;
                        std::vector<int> inliersIndicesCircle;
                        std::vector<double> coeffsCircle;
                        // give circle a higher priority since line tends to be less acurate
                        if (PclUtilities<pcl::PointXYZI>::sampleConsensusModelCircle2D(cloudFeature,
                            circle_distance_thresh_, inliersIndicesCircle, coeffsCircle))
                        {
#ifdef DEBUG
                            std::cout << "circle coeffs count: " << coeffsCircle.size() << std::endl;
                            for (const auto& it : coeffsCircle)
                            {
                                std::cout << it << std::endl;
                            }
#endif
                            if (fabs(coeffsCircle[2] - feature_arch_radius_) < feature_arch_radius_tolerance_)
                            {
                                foundCircles.push_back(coeffsCircle);
                            }
                        }
                    }
                }
            }

#ifndef DEBUG
            std::cout << "found circle number " << foundCircles.size() << std::endl;
            for (const auto& circle : foundCircles)
            {
                std::cout << "coeffs" << std::endl;
                for (const auto& coeff : circle)
                {
                    std::cout << coeff << std::endl;
                }
            }
#endif
            bool failedFind = false;
            if (!foundCircles.empty())
            {
                geometry_msgs::TransformStamped transLaserMap;
                geometry_msgs::TransformStamped transBaselinkMap;
                geometry_msgs::TransformStamped transLaserBaselink;
                {
                    const std::lock_guard<std::mutex> lock(mtx_cut_min_);
                    transLaserMap = tf_laser_map_;
                    transBaselinkMap = tf_baselink_map_;
                    transLaserBaselink = tf_laser_baselink_;
                }

                // calculate the difference in laser frame
                geometry_msgs::Pose poseMidLaser;
                geometry_msgs::Pose poseMidMap;
                if (foundCircles.size() == 2)
                {
                    poseMidLaser.position.x = foundCircles[0][0] + 0.5 * (foundCircles[1][0] - foundCircles[0][0]);
                    poseMidLaser.position.y = foundCircles[0][1] + 0.5 * (foundCircles[1][1] - foundCircles[0][1]);
                    poseMidLaser.orientation = PoseUtilities::fromEuler(0.0, 0.0, 0.0);
                    poseMidMap = PoseUtilities::applyTransform(poseMidLaser, transLaserMap);
                }
                else if (foundCircles.size() == 1)
                {
                    poseMidLaser.position.x = foundCircles[0][0];
                    poseMidLaser.position.y = foundCircles[0][1];
                    poseMidLaser.orientation = PoseUtilities::fromEuler(0.0, 0.0, 0.0);
                    poseMidMap = PoseUtilities::applyTransform(poseMidLaser, transLaserMap);
                    poseMidMap.orientation = pose_feature_.orientation;
                }
                else
                {
                    failedFind = true;
                }

                found_feature_angle_ = PoseUtilities::toEuler(poseMidMap.orientation)[2];
                double angleBaselink = PoseUtilities::toEuler(transBaselinkMap.transform.rotation)[2];
                geometry_msgs::Point pntBase;
                pntBase.x = transBaselinkMap.transform.translation.x;
                pntBase.y = transBaselinkMap.transform.translation.y;
                geometry_msgs::Point pntFeature;
                pntFeature.x = poseMidMap.position.x;
                pntFeature.y = poseMidMap.position.y;
                auto vecFeature = PoseUtilities::createVector2D(geometry_msgs::Point(), 1.0, found_feature_angle_);
                auto vecBaseFeature = PoseUtilities::createVector2D(pntBase, pntFeature);
                double delta = PoseUtilities::angleBetweenVectors2D(vecFeature, vecBaseFeature);
#ifndef DEBUG
                std::cout << "feature angle in map:" << angles::to_degrees(found_feature_angle_) <<
                    ", baselink angle in map:" << angles::to_degrees(angleBaselink) <<
                    ", align delta:" << angles::to_degrees(delta) << std::endl;
                std::cout << "feature x:" << pntFeature.x << ",y:" << pntFeature.y << std::endl;
#endif
                if (fabs(delta) < yaw_tolerance_)
                {
                    // target of charging position
                    geometry_msgs::Pose poseDelta;
                    poseDelta.position.x = distance_to_charge_;
                    poseDelta.orientation = PoseUtilities::fromEuler(0.0, 0.0, 0.0);
                    pose_target_ = PoseUtilities::applyTransform(poseDelta, transBaselinkMap);

                    state_ = STA_MOVE_ALIGN;
                }
                else
                {
                    // target of perpenticular to laser's x axis and base_link lay on x axis of laser
                    double distDelta = fabs(sin(delta)) * PoseUtilities::distance(pntBase, pntFeature);
                    geometry_msgs::Point deltaPoint;
                    deltaPoint.x = distDelta;
                    auto rotation = PoseUtilities::fromEuler(0.0, 0.0,
                        found_feature_angle_ + PoseUtilities::signOf(delta) * 0.5 * M_PI);
                    auto deltaInMap = PoseUtilities::applyVecRotation(deltaPoint, rotation);
                    pose_target_.position.x = transBaselinkMap.transform.translation.x + deltaInMap.x;
                    pose_target_.position.y = transBaselinkMap.transform.translation.y + deltaInMap.y;
                    pose_target_.orientation = rotation;
#ifdef DEBUG
                    std::cout << "rotated angle:" << angles::to_degrees(PoseUtilities::toEuler(rotation)[2]) << std::endl;
                    std::cout << "distDelta:" << distDelta << std::endl;
                    std::cout << "delta in map x:" << deltaInMap.x << ",y:" << deltaInMap.y << std::endl;
#endif
                    
                    state_ = STA_TO_VERTICAL;
                }

                find_tried_time_ = 0.0;
            }
            else
            {
                failedFind = true;
            }

            if (failedFind)
            {
                find_tried_time_ += (ros::Time::now() - begin).toSec();
                if (find_tried_time_ > 5.0)
                {
                    find_tried_time_ = 0.0;
                    state_ = STA_FAILED;
                }
            }

            std::cout << "processing time: " << (ros::Time::now() - begin).toSec() << std::endl;
        }
    }

    PLUGINLIB_EXPORT_CLASS(pose_registration_plugins::PlainPoseRegistration, whi_pose_registration::BasePoseRegistration)
} // namespace pose_registration_plugins
