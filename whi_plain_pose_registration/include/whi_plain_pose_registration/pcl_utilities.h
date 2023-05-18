/******************************************************************
utilities of PCL

Features:
- message conversion
- basic point cloud manipulations
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2023-05-18: Initial version
2022-xx-xx: xxx
******************************************************************/
#pragma once
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/search.h>

class PclUtilities
{
public:
    PclUtilities() = delete;
    ~PclUtilities() = delete;

public:
    static sensor_msgs::PointCloud2 msgLaserScanToMsgPointCloud2(const sensor_msgs::LaserScan& MsgLaserScan);
    static pcl::PointCloud<pcl::PointXYZ>::Ptr fromMsgLaserScan(const sensor_msgs::LaserScan& MsgLaserScan);
    static pcl::PointCloud<pcl::PointXYZ>::Ptr fromMsgPointCloud2(const sensor_msgs::PointCloud2& MsgPointCloud2);
    // template<typename K, template<typename> typename T = pcl::PointCloud>
    // T<K>::Ptr fromMsgPointCloud2(const sensor_msgs::PointCloud2& MsgPointCloud2)
    // {
    //     // convert sensor_msgs::PointCloud2 to pcl::PCLPointCloud2
    //     pcl::PCLPointCloud2 pclCloud2;
    //     pcl_conversions::toPCL(MsgPointCloud2, pclCloud2);
    //     // convert pcl::PCLPointCloud2 to pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointXYZRGB>
    //     T<K>::Ptr pclCloud(new T<K>);
    //     pcl::fromPCLPointCloud2(pclCloud2, *pclCloud);

    //     return pclCloud;
    // };
    static sensor_msgs::PointCloud2 toMsgPointCloud2(const pcl::PointCloud<pcl::PointXYZ>::Ptr Src);

    static pcl::PointCloud<pcl::PointXYZ>::Ptr downsampleVoxelGrid(const pcl::PointCloud<pcl::PointXYZ>::Ptr Src,
        double LeafSizeX, double LeafSizeY, double LeafSizeZ);

    static pcl::PointCloud<pcl::Normal>::Ptr estimateNormalKn(const pcl::PointCloud<pcl::PointXYZ>::Ptr Src,
        int KNeighbours);
    static std::pair<pcl::PointCloud<pcl::Normal>::Ptr, pcl::search::Search<pcl::PointXYZ>::Ptr> estimateNormalPairKn(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr Src, int KNeighbours);
    static pcl::PointCloud<pcl::Normal>::Ptr estimateNormalKr(const pcl::PointCloud<pcl::PointXYZ>::Ptr Src,
        double KRadius);
    static std::pair<pcl::PointCloud<pcl::Normal>::Ptr, pcl::search::Search<pcl::PointXYZ>::Ptr> estimateNormalPairKr(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr Src, double KRadius);

    static std::vector<pcl::PointIndices> segmentEuclidean(const pcl::PointCloud<pcl::PointXYZ>::Ptr Src,
        double Tolerance, int MinClusterSize = 100, int MaxClusterSize = 25000);
    static std::vector<pcl::PointIndices> segmentMinCut(const pcl::PointCloud<pcl::PointXYZ>::Ptr Src,
        const pcl::PointXYZ& Center, double Radius, int Neighbours = 5, double Sigma = 0.25, double Weight = 0.8);
    static std::vector<pcl::PointIndices> segmentRegionGrowingKn(const pcl::PointCloud<pcl::PointXYZ>::Ptr Src,
        int KNeighbours, int Neighbours, double Angle, double Curvature,
        int MinClusterSize = 100, int MaxClusterSize = 25000);
    static std::vector<pcl::PointIndices> segmentRegionGrowingKr(const pcl::PointCloud<pcl::PointXYZ>::Ptr Src,
        double KRadius, int Neighbours, double Angle, double Curvature,
        int MinClusterSize = 100, int MaxClusterSize = 25000);
    static std::vector<pcl::PointIndices> segmentRegionGrowingRGB(const pcl::PointCloud<pcl::PointXYZ>::Ptr Src,
        double Distance, double PointColor, double RegionColor, int MinClusterSize = 100, int MaxClusterSize = 25000);

    static bool sampleConsensusModelCircle2D(const pcl::PointCloud<pcl::PointXYZ>::Ptr Src,
        std::vector<double>& Coeffs, pcl::PointCloud<pcl::PointXYZ>::Ptr& Inliers);
};
