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

#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_circle.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pluginlib/class_list_macros.h>

namespace pose_registration_plugins
{
    PlainPoseRegistration::PlainPoseRegistration()
        : BasePoseRegistration()
        , cloud_projector_(std::make_unique<laser_geometry::LaserProjection>())
    {
        /// node version and copyright announcement
	    std::cout << "\nWHI plain pose registration plugin VERSION 00.01.1" << std::endl;
	    std::cout << "Copyright © 2023-2024 Wheel Hub Intelligent Co.,Ltd. All rights reserved\n" << std::endl;
    }

    void PlainPoseRegistration::initialize(const std::string& LaserTopic)
    {
        topic_laser_scan_ = LaserTopic;
        sub_laser_scan_ = std::make_unique<ros::Subscriber>(node_handle_->subscribe<sensor_msgs::LaserScan>(
		    topic_laser_scan_, 10, std::bind(&PlainPoseRegistration::subCallbackLaserScan, this, std::placeholders::_1)));
    }

    bool PlainPoseRegistration::computeVelocityCommands(geometry_msgs::Twist& CmdVel)
    {

    }

    void PlainPoseRegistration::subCallbackLaserScan(const sensor_msgs::LaserScan::ConstPtr& Laser)
    {
        /// conversion
        // convert sensor_msgs::LaserScan to sensor_msgs::PointCloud2
        sensor_msgs::PointCloud2 msgCloud;
        cloud_projector_->projectLaser(*Laser, msgCloud);
#ifndef DEBUG
        if (!pub_point_cloud_)
        {
            pub_point_cloud_ = std::make_unique<ros::Publisher>(
                node_handle_->advertise<sensor_msgs::PointCloud2>("projected", 10, false));
        }
        pub_point_cloud_->publish(msgCloud);
#endif
        // convert sensor_msgs::PointCloud2 to pcl::PCLPointCloud2
        pcl::PCLPointCloud2 pclCloud2;
        pcl_conversions::toPCL(msgCloud, pclCloud2);
        // convert pcl::PCLPointCloud2 to pcl::PointCloud<pcl::PointXYZ>
        pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(pclCloud2, *pclCloud);

        /// filtering
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloudfiltered(new pcl::PointCloud<pcl::PointXYZ>);
        vg.setInputCloud(pclCloud);
        vg.setLeafSize(0.01f, 0.01f, 0.01f);
        vg.filter(*pclCloudfiltered);
#ifndef DEBUG
        std::cout << "before filtering has: " << pclCloud->size() << " data points" << std::endl;
        std::cout << "after filtering has: " << pclCloudfiltered->size()  << " data points" << std::endl;
        // convert pcl::PointCloud<pcl::PointXYZ> to sensor_msgs::PointCloud2
        sensor_msgs::PointCloud2 msgFiltered;
        pcl::toROSMsg(*pclCloudfiltered, msgFiltered);
        if (!pub_filtered_)
        {
            pub_filtered_ = std::make_unique<ros::Publisher>(
                node_handle_->advertise<sensor_msgs::PointCloud2>("filtered", 10, false));
        }
        msgFiltered.header.frame_id = "laser";
        pub_filtered_->publish(msgFiltered);
#endif

        /// segement
        // Creating the KdTree object for the search method of the extraction
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(pclCloudfiltered);
        std::vector<pcl::PointIndices> clusterIndices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(0.01);
        ec.setMinClusterSize(100);
        ec.setMaxClusterSize(25000);
        ec.setSearchMethod(tree);
        ec.setInputCloud(pclCloudfiltered);
        ec.extract(clusterIndices);
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
#ifndef DEBUG
            // convert pcl::PointCloud<pcl::PointXYZ> to sensor_msgs::PointCloud2
            sensor_msgs::PointCloud2 msgSeg;
            pcl::toROSMsg(*cloudCluster, msgSeg);
            if (!pub_seg_)
            {
                pub_seg_ = std::make_unique<ros::Publisher>(
                    node_handle_->advertise<sensor_msgs::PointCloud2>("seg", 10, false));
            }
            msgSeg.header.frame_id = "laser";
            pub_seg_->publish(msgSeg);
#endif

            std::cout << "pointCloud representing the Cluster: " << cloudCluster->size() << " data points" << std::endl;
        }
        std::cout << "total clusters number: " << clusterIndices.size() << std::endl;

        /// sample consensus
        // created RandomSampleConsensus object and compute the appropriated model
        pcl::SampleConsensusModelCircle2D<pcl::PointXYZ>::Ptr
            modelCircle(new pcl::SampleConsensusModelCircle2D<pcl::PointXYZ>(pclCloud));
        pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(modelCircle);
        ransac.setDistanceThreshold(0.01);
        ransac.computeModel();
        std::vector<int> inliers;
        ransac.getInliers(inliers);
        // copies all inliers of the model computed to another PointCloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr final(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*pclCloud, inliers, *final);

        // convert pcl::PointCloud<pcl::PointXYZ> to sensor_msgs::PointCloud2
        sensor_msgs::PointCloud2 msgInlier;
        pcl::toROSMsg(*final, msgInlier);
#ifndef DEBUG
        if (!pub_inliers_)
        {
            pub_inliers_ = std::make_unique<ros::Publisher>(
                node_handle_->advertise<sensor_msgs::PointCloud2>("inliers", 10, false));
        }
        msgInlier.header.frame_id = "laser";
        pub_inliers_->publish(msgInlier);
#endif
    }

    PLUGINLIB_EXPORT_CLASS(pose_registration_plugins::PlainPoseRegistration, whi_pose_registration::BasePoseRegistration)
} // namespace pose_registration_plugins
