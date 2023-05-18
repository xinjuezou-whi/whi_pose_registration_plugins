/******************************************************************
utilities of PCL

Features:
- message conversion
- basic point cloud manipulations
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

******************************************************************/
#include "whi_plain_pose_registration/pcl_utilities.h"

#include <laser_geometry/laser_geometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_circle.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/min_cut_segmentation.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/region_growing_rgb.h>

sensor_msgs::PointCloud2 PclUtilities::msgLaserScanToMsgPointCloud2(const sensor_msgs::LaserScan& MsgLaserScan)
{
    std::unique_ptr<laser_geometry::LaserProjection> projector = std::make_unique<laser_geometry::LaserProjection>();
    sensor_msgs::PointCloud2 msgCloud;
    projector->projectLaser(MsgLaserScan, msgCloud);

    return msgCloud;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PclUtilities::fromMsgLaserScan(const sensor_msgs::LaserScan& MsgLaserScan)
{
    return fromMsgPointCloud2(msgLaserScanToMsgPointCloud2(MsgLaserScan));
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PclUtilities::fromMsgPointCloud2(const sensor_msgs::PointCloud2& MsgPointCloud2)
{
    // convert sensor_msgs::PointCloud2 to pcl::PCLPointCloud2
    pcl::PCLPointCloud2 pclCloud2;
    pcl_conversions::toPCL(MsgPointCloud2, pclCloud2);
    // convert pcl::PCLPointCloud2 to pcl::PointCloud<pcl::PointXYZ>
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pclCloud2, *pclCloud);

    return pclCloud;
}

sensor_msgs::PointCloud2 PclUtilities::toMsgPointCloud2(const pcl::PointCloud<pcl::PointXYZ>::Ptr Src)
{
    sensor_msgs::PointCloud2 msgPointCloud2;
    pcl::toROSMsg(*Src, msgPointCloud2);

    return msgPointCloud2;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr PclUtilities::downsampleVoxelGrid(const pcl::PointCloud<pcl::PointXYZ>::Ptr Src,
    double LeafSizeX, double LeafSizeY, double LeafSizeZ)
{
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloudfiltered(new pcl::PointCloud<pcl::PointXYZ>);
    vg.setInputCloud(Src);
    vg.setLeafSize(float(LeafSizeX), float(LeafSizeY), float(LeafSizeZ));
    vg.filter(*pclCloudfiltered);

    return pclCloudfiltered;
}

pcl::PointCloud<pcl::Normal>::Ptr PclUtilities::estimateNormalKn(const pcl::PointCloud<pcl::PointXYZ>::Ptr Src,
    int KNeighbours)
{
    pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimator;
    normalEstimator.setSearchMethod(tree);
    normalEstimator.setInputCloud(Src);
    normalEstimator.setKSearch(KNeighbours);
    normalEstimator.compute(*normals);

    return normals;
}

std::pair<pcl::PointCloud<pcl::Normal>::Ptr, pcl::search::Search<pcl::PointXYZ>::Ptr> PclUtilities::estimateNormalPairKn(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr Src, int KNeighbours)
{
    pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimator;
    normalEstimator.setSearchMethod(tree);
    normalEstimator.setInputCloud(Src);
    normalEstimator.setKSearch(KNeighbours);
    normalEstimator.compute(*normals);

    return std::make_pair(normals, tree);
}

pcl::PointCloud<pcl::Normal>::Ptr PclUtilities::estimateNormalKr(const pcl::PointCloud<pcl::PointXYZ>::Ptr Src,
    double KRadius)
{
    pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimator;
    normalEstimator.setSearchMethod(tree);
    normalEstimator.setInputCloud(Src);
    normalEstimator.setRadiusSearch(KRadius);
    normalEstimator.compute(*normals);

    return normals;
}

std::pair<pcl::PointCloud<pcl::Normal>::Ptr, pcl::search::Search<pcl::PointXYZ>::Ptr> PclUtilities::estimateNormalPairKr(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr Src, double KRadius)
{
    pcl::search::Search<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimator;
    normalEstimator.setSearchMethod(tree);
    normalEstimator.setInputCloud(Src);
    normalEstimator.setRadiusSearch(KRadius);
    normalEstimator.compute(*normals);

    return std::make_pair(normals, tree);
}

std::vector<pcl::PointIndices> PclUtilities::segmentEuclidean(const pcl::PointCloud<pcl::PointXYZ>::Ptr Src,
    double Tolerance, int MinClusterSize/* = 100*/, int MaxClusterSize/* = 25000*/)
{
    // creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(Src);
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(Tolerance);
    ec.setMinClusterSize(MinClusterSize);
    ec.setMaxClusterSize(MaxClusterSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(Src);

    std::vector<pcl::PointIndices> clusters;
    ec.extract(clusters);

    return clusters;
}

std::vector<pcl::PointIndices> PclUtilities::segmentMinCut(const pcl::PointCloud<pcl::PointXYZ>::Ptr Src,
    const pcl::PointXYZ& Center, double Radius, int Neighbours/* = 5*/, double Sigma/* = 0.25*/, double Weight/* = 0.8*/)
{
    pcl::MinCutSegmentation<pcl::PointXYZ> seg;
    seg.setInputCloud(Src);
    pcl::PointCloud<pcl::PointXYZ>::Ptr foregroundPoints(new pcl::PointCloud<pcl::PointXYZ>());
    foregroundPoints->points.push_back(Center);
    seg.setForegroundPoints(foregroundPoints);
    seg.setSigma(Sigma);
    seg.setRadius(Radius);
    seg.setNumberOfNeighbours(Neighbours);
    seg.setSourceWeight(Weight);

    std::vector<pcl::PointIndices> clusters;
    seg.extract(clusters);

    return clusters;
}

std::vector<pcl::PointIndices> PclUtilities::segmentRegionGrowingKn(const pcl::PointCloud<pcl::PointXYZ>::Ptr Src,
    int KNeighbours, int Neighbours, double Angle, double Curvature,
    int MinClusterSize/* = 100*/, int MaxClusterSize/* = 25000*/)
{
    auto pair = estimateNormalPairKn(Src, KNeighbours);

    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    reg.setMinClusterSize(MinClusterSize);
    reg.setMaxClusterSize(MaxClusterSize);
    reg.setSearchMethod(pair.second);
    reg.setNumberOfNeighbours(Neighbours);
    reg.setInputCloud(Src);
    //reg.setIndices(indices);
    reg.setInputNormals(pair.first);
    reg.setSmoothnessThreshold(Angle);
    reg.setCurvatureThreshold(Curvature);

    std::vector<pcl::PointIndices> clusters;
    reg.extract(clusters);

    return clusters;
}

std::vector<pcl::PointIndices> PclUtilities::segmentRegionGrowingKr(const pcl::PointCloud<pcl::PointXYZ>::Ptr Src,
    double KRadius, int Neighbours, double Angle, double Curvature,
    int MinClusterSize/* = 100*/, int MaxClusterSize/* = 25000*/)
{
    auto pair = estimateNormalPairKr(Src, KRadius);

    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    reg.setMinClusterSize(MinClusterSize);
    reg.setMaxClusterSize(MaxClusterSize);
    reg.setSearchMethod(pair.second);
    reg.setNumberOfNeighbours(Neighbours);
    reg.setInputCloud(Src);
    //reg.setIndices(indices);
    reg.setInputNormals(pair.first);
    reg.setSmoothnessThreshold(Angle);
    reg.setCurvatureThreshold(Curvature);

    std::vector<pcl::PointIndices> clusters;
    reg.extract(clusters);

    return clusters;
}

std::vector<pcl::PointIndices> segmentRegionGrowingRGB(const pcl::PointCloud<pcl::PointXYZ>::Ptr Src,
    double Distance, double PointColor, double RegionColor, int MinClusterSize/* = 100*/, int MaxClusterSize/* = 25000*/)
{
    pcl::PointCloud <pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud <pcl::PointXYZRGB>);
    pcl::copyPointCloud(*Src, *cloud);
    pcl::search::Search<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    pcl::RegionGrowingRGB<pcl::PointXYZRGB> reg;
    reg.setInputCloud(cloud);
    reg.setSearchMethod(tree);
    reg.setDistanceThreshold(float(Distance));
    reg.setPointColorThreshold(float(PointColor));
    reg.setRegionColorThreshold(float(RegionColor));
    reg.setMinClusterSize(MinClusterSize);
    reg.setMaxClusterSize(MaxClusterSize);

    std::vector<pcl::PointIndices> clusters;
    reg.extract(clusters);

    return clusters;
}

bool PclUtilities::sampleConsensusModelCircle2D(const pcl::PointCloud<pcl::PointXYZ>::Ptr Src,
        std::vector<double>& Coeffs, pcl::PointCloud<pcl::PointXYZ>::Ptr& Inliers)
{
    // created RandomSampleConsensus object and compute the appropriated model
    pcl::SampleConsensusModelCircle2D<pcl::PointXYZ>::Ptr
        modelCircle(new pcl::SampleConsensusModelCircle2D<pcl::PointXYZ>(Src));
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(modelCircle);
    ransac.setDistanceThreshold(0.01);
    ransac.computeModel();
    std::vector<int> inliers;
    ransac.getInliers(inliers);

    if (!inliers.empty())
    {
        // copies all inliers of the model computed to another PointCloud
        Inliers.reset(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud(*Src, inliers, *Inliers);
        Eigen::VectorXf coeffs;
        ransac.getModelCoefficients(coeffs);
        for (int i = 0; i < coeffs.size(); ++i)
        {
            Coeffs.push_back(coeffs[i]);
        }

        return true;
    }
    else
    {
        return false;
    }
}
