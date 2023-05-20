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
	    std::cout << "\nWHI plain pose registration plugin VERSION 00.01.5" << std::endl;
	    std::cout << "Copyright © 2023-2024 Wheel Hub Intelligent Co.,Ltd. All rights reserved\n" << std::endl;
    }

    void PlainPoseRegistration::initialize(const std::string& LaserTopic)
    {
        /// params
        // common
        node_handle_->param("pose_registration/PlainPose/downsampling", downsampling_, true);
        if (!node_handle_->getParam("pose_registration/PlainPose/downsampleing_coeffs", downsampling_coeffs_))
        {
            for (int i = 0; i < 3; ++i)
            {
                downsampling_coeffs_.push_back(0.01);
            }
        }
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
        node_handle_->param("pose_registration/PlainPose/edge_neighbour", edge_neighbour_, 20);

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
        /// segement
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
        else if (segment_type_ == "edge")
        {
            clusterIndices = PclUtilities<pcl::PointXYZI>::segmentEdgeDetection(pclOperating,
                k_radius_, edge_neighbour_, 1.0);
            
            static PclVisualize<pcl::PointXYZI> viewer;
            viewer.viewCloud(pclOperating, "debug");
        }
#ifndef DEBUG
        std::cout << "total clusters number: " << clusterIndices.size() << std::endl;
        for (int i = 0; i < clusterIndices.size(); ++i)
        {
            std::cout << "cluster " << i << " has points " << clusterIndices[i].indices.size() << std::endl;
        }
#endif

        int index = 0;
        for (const auto& cluster : clusterIndices)
        {
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloudCluster(new pcl::PointCloud<pcl::PointXYZI>());
            for (const auto& idx : cluster.indices)
            {
                // process each point
                cloudCluster->push_back((*pclOperating)[idx]);
            }
            cloudCluster->width = cloudCluster->size();
            cloudCluster->height = pclOperating->height;
            cloudCluster->is_dense = true;

            /// sample consensus
            if (!cloudCluster->empty())
            {
                std::vector<double> coeffs;
                pcl::PointCloud<pcl::PointXYZI>::Ptr inliers;
                PclUtilities<pcl::PointXYZI>::sampleConsensusModelCircle2D(cloudCluster, coeffs, inliers);
#ifndef DEBUG
                auto msgSeg = PclUtilities<pcl::PointXYZI>::toMsgPointCloud2(cloudCluster);
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
                auto msgInliers = PclUtilities<pcl::PointXYZI>::toMsgPointCloud2(inliers);
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
            }

            ++index;
        }
    }

    PLUGINLIB_EXPORT_CLASS(pose_registration_plugins::PlainPoseRegistration, whi_pose_registration::BasePoseRegistration)
} // namespace pose_registration_plugins
