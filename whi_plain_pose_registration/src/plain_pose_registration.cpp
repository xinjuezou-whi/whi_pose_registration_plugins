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

#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>

namespace pose_registration_plugins
{
    PlainPoseRegistration::PlainPoseRegistration()
        : BasePoseRegistration()
    {
        /// node version and copyright announcement
	    std::cout << "\nWHI plain pose registration plugin VERSION 00.01.6" << std::endl;
	    std::cout << "Copyright © 2023-2024 Wheel Hub Intelligent Co.,Ltd. All rights reserved\n" << std::endl;
    }

    void PlainPoseRegistration::initialize(const std::string& LaserTopic)
    {
        /// params
        // common
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
        if (!node_handle_->getParam("pose_registration/PlainPose/center_point", center_point_))
        {
            center_point_.resize(3);
        }
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

        topic_laser_scan_ = LaserTopic;
        sub_laser_scan_ = std::make_unique<ros::Subscriber>(node_handle_->subscribe<sensor_msgs::LaserScan>(
		    topic_laser_scan_, 10, std::bind(&PlainPoseRegistration::subCallbackLaserScan, this, std::placeholders::_1)));
    }

    bool PlainPoseRegistration::computeVelocityCommands(geometry_msgs::Twist& CmdVel)
    {

    }

    void PlainPoseRegistration::subCallbackLaserScan(const sensor_msgs::LaserScan::ConstPtr& Laser)
    {
        ros::Time begin = ros::Time::now();

#ifndef DEBUG
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
#ifndef DEBUG
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
            pointCenter.x = center_point_[0];
            pointCenter.y = center_point_[1];
            pointCenter.z = center_point_[2];
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
                /// segment single feature from epic
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
                    viewer.viewCloud(cloudFeature, "debug_features" + std::to_string(i), 1 + i * 3);
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
#ifndef DEBUG
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

    PLUGINLIB_EXPORT_CLASS(pose_registration_plugins::PlainPoseRegistration, whi_pose_registration::BasePoseRegistration)
} // namespace pose_registration_plugins
