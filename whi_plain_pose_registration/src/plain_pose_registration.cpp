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

#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>

namespace pose_registration_plugins
{
    PlainPoseRegistration::PlainPoseRegistration()
        : BasePoseRegistration()
    {
        /// node version and copyright announcement
	    std::cout << "\nWHI plain pose registration plugin VERSION 00.01.3" << std::endl;
	    std::cout << "Copyright © 2023-2024 Wheel Hub Intelligent Co.,Ltd. All rights reserved\n" << std::endl;
    }

    void PlainPoseRegistration::initialize(const std::string& LaserTopic)
    {
        // params
        node_handle_->param("pose_registration/PlainPose/segment_type", segment_type_, std::string("region_growing"));
        if (!node_handle_->getParam("pose_registration/PlainPose/center_point", center_point_))
        {
            center_point_.resize(3);
        }
        node_handle_->param("pose_registration/PlainPose/radius", radius_, 0.1);
        node_handle_->param("pose_registration/PlainPose/sigma", sigma_, 0.25);
        node_handle_->param("pose_registration/PlainPose/weight", weight_, 0.8);
        node_handle_->param("pose_registration/PlainPose/cut_min_neighbour", cut_min_neighbour_, 5);
        node_handle_->param("pose_registration/PlainPose/k_neighbour", k_neighbour_, 50);
        node_handle_->param("pose_registration/PlainPose/region_growing_neighbour", region_growing_neighbour_, 30);
        node_handle_->param("pose_registration/PlainPose/angle", angle_, 3.0);
        node_handle_->param("pose_registration/PlainPose/curvature", curvature_, 1.0);
        node_handle_->param("pose_registration/PlainPose/min_cluster_size", min_cluster_size_, 50);
        node_handle_->param("pose_registration/PlainPose/max_cluster_size", max_cluster_size_, 1000);

        topic_laser_scan_ = LaserTopic;
        sub_laser_scan_ = std::make_unique<ros::Subscriber>(node_handle_->subscribe<sensor_msgs::LaserScan>(
		    topic_laser_scan_, 10, std::bind(&PlainPoseRegistration::subCallbackLaserScan, this, std::placeholders::_1)));
    }

    bool PlainPoseRegistration::computeVelocityCommands(geometry_msgs::Twist& CmdVel)
    {

    }

    void PlainPoseRegistration::subCallbackLaserScan(const sensor_msgs::LaserScan::ConstPtr& Laser)
    {
#ifndef DEBUG
        auto msgCloud2 = PclUtilities::msgLaserScanToMsgPointCloud2(*Laser);
        if (!pub_projected_)
        {
            pub_projected_ = std::make_unique<ros::Publisher>(
                node_handle_->advertise<sensor_msgs::PointCloud2>("projected", 10, false));
        }
        pub_projected_->publish(msgCloud2);
#endif
        /// convert to pcl cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud = PclUtilities::fromMsgLaserScan(*Laser);
        /// downsampling
        pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloudfiltered = PclUtilities::downsampleVoxelGrid(pclCloud,
            0.01, 0.01, 0.01);
        /// segement
        std::vector<pcl::PointIndices> clusterIndices;
        if (segment_type_ == "cut_min")
        {
            pcl::PointXYZ pointCenter;
            pointCenter.x = center_point_[0];
            pointCenter.y = center_point_[1];
            pointCenter.z = center_point_[2];
            clusterIndices = PclUtilities::segmentMinCut(pclCloudfiltered, pointCenter,
                radius_, cut_min_neighbour_, sigma_, weight_);
        }
        else if (segment_type_ == "region_growing")
        {
            clusterIndices = PclUtilities::segmentRegionGrowingKn(pclCloudfiltered,
            k_neighbour_, region_growing_neighbour_, angles::from_degrees(angle_), curvature_,
            min_cluster_size_, max_cluster_size_);
        }
        else if (segment_type_ == "region_growing_rgb")
        {
            clusterIndices = PclUtilities::segmentRegionGrowingRGB(pclCloudfiltered,
                distance_, point_color_, region_color_, min_cluster_size_, max_cluster_size_);
        }
        int index = 0;
        for (const auto& cluster : clusterIndices)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloudCluster(new pcl::PointCloud<pcl::PointXYZ>);
            for (const auto& idx : cluster.indices)
            {
                cloudCluster->push_back((*pclCloudfiltered)[idx]);
            }
            cloudCluster->width = cloudCluster->size();
            cloudCluster->height = 1;
            cloudCluster->is_dense = true;

            /// sample consensus
            std::vector<double> coeffs;
            pcl::PointCloud<pcl::PointXYZ>::Ptr inliers;
            PclUtilities::sampleConsensusModelCircle2D(cloudCluster, coeffs, inliers);
#ifndef DEBUG
            auto msgSeg = PclUtilities::toMsgPointCloud2(cloudCluster);
            std::string topicSeg("seg" + std::to_string(index));
            if (!pubs_map_seg_[topicSeg])
            {
                pubs_map_seg_[topicSeg] = std::make_unique<ros::Publisher>(
                    node_handle_->advertise<sensor_msgs::PointCloud2>(topicSeg, 10, false));
            }
            msgSeg.header.frame_id = "laser";
            pubs_map_seg_[topicSeg]->publish(msgSeg);
            std::cout << "cluser with topic " << topicSeg << " has size " << cloudCluster->size() << " data points" << std::endl;
#endif
#ifndef DEBUG
            auto msgInliers = PclUtilities::toMsgPointCloud2(inliers);
            std::string topicInliers("inliers" + std::to_string(index));
            if (!pubs_map_inliers_[topicInliers])
            {
                pubs_map_inliers_[topicInliers] = std::make_unique<ros::Publisher>(
                    node_handle_->advertise<sensor_msgs::PointCloud2>(topicInliers, 10, false));
            }
            msgInliers.header.frame_id = "laser";
            pubs_map_inliers_[topicInliers]->publish(msgInliers);
            std::cout << "coeffs with topic " << topicInliers << " has size " << coeffs.size() << std::endl;
            for (const auto& it : coeffs)
            {
                std::cout << it << std::endl;
            }
#endif
            ++index;
        }
        std::cout << "total clusters number: " << clusterIndices.size() << std::endl;
    }

    PLUGINLIB_EXPORT_CLASS(pose_registration_plugins::PlainPoseRegistration, whi_pose_registration::BasePoseRegistration)
} // namespace pose_registration_plugins
