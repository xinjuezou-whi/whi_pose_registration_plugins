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
#include "whi_plain_pose_registration/pcl_utilities.h"
#include "whi_plain_pose_registration/pcl_visualize.h"
#include "whi_plain_pose_registration/pose_utilities.h"

#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>

namespace pose_registration_plugins
{
    PlainPoseRegistration::PlainPoseRegistration()
        : BasePoseRegistration()
    {
        /// node version and copyright announcement
	    std::cout << "\nWHI plain pose registration plugin VERSION 00.01.11" << std::endl;
	    std::cout << "Copyright © 2023-2024 Wheel Hub Intelligent Co.,Ltd. All rights reserved\n" << std::endl;
    }

    void PlainPoseRegistration::initialize()
    {
        /// params
        // common
        std::string laserScanTopic;
        std::vector<double> center;
        node_handle_->param("pose_registration/PlainPose/laser_scan_topic", laserScanTopic, std::string("scan"));
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
        if (!node_handle_->getParam("pose_registration/PlainPose/center_point", center))
        {
            center.resize(3);
        }
        center_.position.x = center[0];
        center_.position.y = center[1];
        center_.position.z = center[2];
        node_handle_->param("pose_registration/PlainPose/k_neighbour", k_neighbour_, 50);
        node_handle_->param("pose_registration/PlainPose/k_radius", k_radius_, 0.2);
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

        ros::Duration updateFreq = ros::Duration(1.0 / 20.0);
        non_realtime_loop_ = std::make_unique<ros::Timer>(
            node_handle_->createTimer(updateFreq, std::bind(&PlainPoseRegistration::update, this, std::placeholders::_1)));
    }

    bool PlainPoseRegistration::computeVelocityCommands(geometry_msgs::Twist& CmdVel)
    {

    }

    void PlainPoseRegistration::update(const ros::TimerEvent& Event)
    {
        auto tfGeometry = listenTf("map", base_link_frame_, ros::Time(0));

        std::cout << "dddddddddddddddddddd x:" << tfGeometry.transform.translation.x <<
            ", y:" << tfGeometry.transform.translation.y <<
            ", yaw:" << angles::to_degrees(PoseUtilities::toEuler(tfGeometry.transform.rotation)[2]) << std::endl;
    }

    void PlainPoseRegistration::subCallbackLaserScan(const sensor_msgs::LaserScan::ConstPtr& Laser)
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
            pointCenter.x = center_.position.x;
            pointCenter.y = center_.position.y;
            pointCenter.z = center_.position.z;
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
#ifndef DEBUG
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
                        std::cout << "circle coeffs count: " << coeffsCircle.size() << std::endl;
                        for (const auto& it : coeffsCircle)
                        {
                            std::cout << it << std::endl;
                        }

                        if (fabs(coeffsCircle[2] - feature_arch_radius_) > feature_arch_radius_tolerance_)
                        {
                            if (PclUtilities<pcl::PointXYZI>::sampleConsensusModelLine(cloudFeature,
                                line_distance_thresh_, inliersIndicesLine, coeffsLine))
                            {
                                std::cout << "line coeffs count: " << coeffsLine.size() << std::endl;
                                for (const auto& it : coeffsLine)
                                {
                                    std::cout << it << std::endl;
                                }
                            }
                        }
                    }
                    else if (PclUtilities<pcl::PointXYZI>::sampleConsensusModelLine(cloudFeature,
                        line_distance_thresh_, inliersIndicesLine, coeffsLine))
                    {
#ifdef DEBUG
                        pcl::PointCloud<pcl::PointXYZI>::Ptr inliers(new pcl::PointCloud<pcl::PointXYZI>());
                        PclUtilities<pcl::PointXYZI>::extractTo(cloudFeature, inliersIndicesLine, inliers);
                        pcl::PointCloud<pcl::PointXYZI>::Ptr outliers(new pcl::PointCloud<pcl::PointXYZI>());
                        PclUtilities<pcl::PointXYZI>::extractTo(cloudFeature, inliersIndicesLine, outliers, true);
                        static PclVisualize<pcl::PointXYZI> viewer;
                        viewer.viewCloud(inliers, "debug_inliers", 3);
                        viewer.viewCloud(outliers, "debug_outliers", 1, std::array<double, 3>({ 1.0, 0.0, 0.0}));
#endif

                    }
                }
            }
        }

        std::cout << "processing time: " << (ros::Time::now() - begin).toSec() << std::endl;
    }

//     void PlainPoseRegistration::subCallbackEstimated(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& Estimated)
//     {
// #ifndef DEBUG
//         std::cout << "estimated x:" << Estimated->pose.pose.position.x << ", y:" << Estimated->pose.pose.position.y <<
//             ", yaw:" << angles::to_degrees(PoseUtilities::toEuler(Estimated->pose.pose.orientation)[2]) << std::endl;

//         geometry_msgs::Pose charger;
//         charger.position.x = 9.38556;
//         charger.position.y = 0.241915;
//         charger.orientation = PoseUtilities::fromEuler(0.0, 0.0, angles::from_degrees(139.384));

//         geometry_msgs::Pose feature;
//         feature.position.x = 8.43586;
//         feature.position.y = 1.0276;
//         feature.orientation = PoseUtilities::fromEuler(0.0, 0.0, angles::from_degrees(139.384));

//         geometry_msgs::Pose origin;
//         origin.orientation = PoseUtilities::fromEuler(0.0, 0.0, 0.0);

//         auto diffOrigin2Base = PoseUtilities::getTransform(origin, Estimated->pose.pose);
//         std::cout << "diff origin to base pose x:" << diffOrigin2Base.transform.translation.x <<
//             ", y:" << diffOrigin2Base.transform.translation.y <<
//             ", yaw:" << angles::to_degrees(PoseUtilities::toEuler(diffOrigin2Base.transform.rotation)[2]) << std::endl;

//         auto diffOrigin2Feature = PoseUtilities::getTransform(origin, feature);
//         std::cout << "diff origin to feature pose x:" << diffOrigin2Feature.transform.translation.x <<
//             ", y:" << diffOrigin2Feature.transform.translation.y <<
//             ", yaw:" << angles::to_degrees(PoseUtilities::toEuler(diffOrigin2Feature.transform.rotation)[2]) << std::endl;

//         auto diffFeature2Base = PoseUtilities::getTransform(feature, Estimated->pose.pose);
//         std::cout << "diff from feature to base pose x:" << diffFeature2Base.transform.translation.x <<
//             ", y:" << diffFeature2Base.transform.translation.y <<
//             ", yaw:" << angles::to_degrees(PoseUtilities::toEuler(diffFeature2Base.transform.rotation)[2]) << std::endl;

//         geometry_msgs::Pose diffPose;
//         diffPose.position.x = diffOrigin2Feature.transform.translation.x - diffOrigin2Base.transform.translation.x;
//         diffPose.position.y = diffOrigin2Feature.transform.translation.y - diffOrigin2Base.transform.translation.y;
//         diffPose.orientation = PoseUtilities::fromEuler(0, 0,
//             PoseUtilities::toEuler(diffOrigin2Feature.transform.rotation)[2] - PoseUtilities::toEuler(diffOrigin2Base.transform.rotation)[2]);
//         std::cout << "diff2 from feature to base pose x:" << diffPose.position.x <<
//             ", y:" << diffPose.position.y <<
//             ", yaw:" << angles::to_degrees(PoseUtilities::toEuler(diffPose.orientation)[2]) << std::endl;

//         geometry_msgs::TransformStamped trans2Base;
//         trans2Base.transform.rotation = PoseUtilities::fromEuler(0, 0,
//             -PoseUtilities::toEuler(diffPose.orientation)[2]);
//         geometry_msgs::Pose ddd = PoseUtilities::applyTransform(diffPose, trans2Base);
//         std::cout << "ddd pose x:" << ddd.position.x <<
//             ", y:" << ddd.position.y <<
//             ", yaw:" << angles::to_degrees(PoseUtilities::toEuler(ddd.orientation)[2]) << std::endl;

//         center_.position.x = -ddd.position.x;
//         center_.position.y = -ddd.position.y;
// #endif
//     }

    PLUGINLIB_EXPORT_CLASS(pose_registration_plugins::PlainPoseRegistration, whi_pose_registration::BasePoseRegistration)
} // namespace pose_registration_plugins
