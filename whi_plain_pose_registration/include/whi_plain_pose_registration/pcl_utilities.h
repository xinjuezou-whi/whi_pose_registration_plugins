/******************************************************************
utilities of PCL

Features:
- message conversion
- basic point cloud manipulations
- xxx

Reference:
- https://github.com/PointCloudLibrary/pcl/tree/master/test for usage
- https://github.com/PointCloudLibrary/pcl/tree/master/examples for usage

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
#include <laser_geometry/laser_geometry.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/search.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_circle.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/organized_edge_detection.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/min_cut_segmentation.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/io/pcd_io.h>

template<typename T = pcl::PointXYZ,
    template<typename...> class P = pcl::PointCloud, template<typename...> class S = pcl::search::Search>
class PclUtilities
{
public:
    static const std::string& VERSION()
    {
        return "00.09";
    }

public:
    PclUtilities() = delete;
    ~PclUtilities() = delete;

public:
    static sensor_msgs::PointCloud2 msgLaserScanToMsgPointCloud2(const sensor_msgs::LaserScan& MsgLaserScan)
    {
        std::unique_ptr<laser_geometry::LaserProjection> projector = std::make_unique<laser_geometry::LaserProjection>();
        sensor_msgs::PointCloud2 msgCloud;
        projector->projectLaser(MsgLaserScan, msgCloud);

        return msgCloud;
    }

    static typename pcl::PointCloud<T>::Ptr fromMsgPointCloud2(const sensor_msgs::PointCloud2& MsgPointCloud2)
    {
        // convert sensor_msgs::PointCloud2 to pcl::PCLPointCloud2
        pcl::PCLPointCloud2 pclCloud2;
        pcl_conversions::toPCL(MsgPointCloud2, pclCloud2);
        // convert pcl::PCLPointCloud2 to pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointXYZRGB>
        typename pcl::PointCloud<T>::Ptr pclCloud(new pcl::PointCloud<T>());
        pcl::fromPCLPointCloud2(pclCloud2, *pclCloud);

#ifdef DEBUG
        std::cout << "check whether has color field" << std::endl;
        for (const auto& it : pclCloud2.fields)
        {
            std::cout << "name " << it.name << std::endl;
            std::cout << "datatype " << it.datatype << std::endl;
        }
#endif

        return pclCloud;
    }

    static typename pcl::PointCloud<T>::Ptr fromMsgLaserScan(const sensor_msgs::LaserScan& MsgLaserScan)
    {
        return fromMsgPointCloud2(msgLaserScanToMsgPointCloud2(MsgLaserScan));
    }

    static sensor_msgs::PointCloud2 toMsgPointCloud2(const typename pcl::PointCloud<T>::Ptr Src)
    {
        sensor_msgs::PointCloud2 msgPointCloud2;
        pcl::toROSMsg(*Src, msgPointCloud2);

        return msgPointCloud2;
    }

    static typename pcl::PointCloud<T>::Ptr loadFrom(const std::string& File)
    {
        typename pcl::PointCloud<T>::Ptr cloud(new pcl::PointCloud<T>);
        if (pcl::io::loadPCDFile<T>(File, *cloud) == -1)
        {
            std::cout << "failed to load pcd file " << File << std::endl;
            return nullptr;
        }
        else
        {
            return cloud;
        }
    }

    static void extractTo(const typename pcl::PointCloud<T>::Ptr Src, const pcl::PointIndices::Ptr Indices,
        typename pcl::PointCloud<T>::Ptr Dst, bool Negative = false)
    {
        // create the filtering object
        pcl::ExtractIndices<T> extract;
        extract.setInputCloud(Src);
        extract.setIndices(Indices);
        extract.setNegative(Negative);
        extract.filter(*Dst);
    }

    static void extractTo(const typename pcl::PointCloud<T>::Ptr Src, const pcl::PointIndices& Indices,
        typename pcl::PointCloud<T>::Ptr Dst, bool Negative = false)
    {
        pcl::PointIndices::Ptr indices(new pcl::PointIndices(Indices));
        extractTo(Src, indices, Dst, Negative);
    }

    static void extractTo(const typename pcl::PointCloud<T>::Ptr Src, std::vector<int>& Indices,
        typename pcl::PointCloud<T>::Ptr Dst, bool Negative = false)
    {
        pcl::PointIndices::Ptr indices(new pcl::PointIndices());
        indices->indices = Indices;
        extractTo(Src, indices, Dst, Negative);
    }

    static typename pcl::PointCloud<T>::Ptr downsampleVoxelGrid(const typename pcl::PointCloud<T>::Ptr Src,
        double LeafSizeX, double LeafSizeY, double LeafSizeZ)
    {
        pcl::VoxelGrid<T> vg;
        typename pcl::PointCloud<T>::Ptr pclCloudfiltered(new pcl::PointCloud<T>());
        vg.setInputCloud(Src);
        vg.setLeafSize(float(LeafSizeX), float(LeafSizeY), float(LeafSizeZ));
        vg.filter(*pclCloudfiltered);

        return pclCloudfiltered;
    }

    static pcl::PointCloud<pcl::Normal>::Ptr estimateNormalKn(const typename pcl::PointCloud<T>::Ptr Src,
        int KNeighbours)
    {
        typename pcl::search::Search<T>::Ptr tree(new pcl::search::KdTree<T>());
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
        pcl::NormalEstimation<T, pcl::Normal> normalEstimator;
        normalEstimator.setSearchMethod(tree);
        normalEstimator.setInputCloud(Src);
        normalEstimator.setKSearch(KNeighbours);
        normalEstimator.compute(*normals);

        return normals;
    }

    static std::pair<pcl::PointCloud<pcl::Normal>::Ptr, typename pcl::search::Search<T>::Ptr>
        estimateNormalPairKn(const typename pcl::PointCloud<T>::Ptr Src, int KNeighbours)
    {
        typename pcl::search::Search<T>::Ptr tree(new pcl::search::KdTree<T>());
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
        pcl::NormalEstimation<T, pcl::Normal> normalEstimator;
        normalEstimator.setSearchMethod(tree);
        normalEstimator.setInputCloud(Src);
        normalEstimator.setKSearch(KNeighbours);
        normalEstimator.compute(*normals);

        return std::make_pair(normals, tree);
    }

    static pcl::PointCloud<pcl::Normal>::Ptr estimateNormalKr(const typename pcl::PointCloud<T>::Ptr Src,
        double KRadius)
    {
        typename pcl::search::Search<T>::Ptr tree(new pcl::search::KdTree<T>());
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
        pcl::NormalEstimation<T, pcl::Normal> normalEstimator;
        normalEstimator.setSearchMethod(tree);
        normalEstimator.setInputCloud(Src);
        normalEstimator.setRadiusSearch(KRadius);
        normalEstimator.compute(*normals);

        return normals;
    }

    static std::pair<pcl::PointCloud<pcl::Normal>::Ptr, typename pcl::search::Search<T>::Ptr>
        estimateNormalPairKr(const typename pcl::PointCloud<T>::Ptr Src, double KRadius)
    {
        typename pcl::search::Search<T>::Ptr tree(new pcl::search::KdTree<T>());
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
        pcl::NormalEstimation<T, pcl::Normal> normalEstimator;
        normalEstimator.setSearchMethod(tree);
        normalEstimator.setInputCloud(Src);
        normalEstimator.setRadiusSearch(KRadius);
        normalEstimator.compute(*normals);

        return std::make_pair(normals, tree);
    }

    static std::vector<pcl::PointIndices> segmentEuclidean(const typename pcl::PointCloud<T>::Ptr Src,
        double Tolerance, int MinClusterSize = 100, int MaxClusterSize = 25000)
    {
        // creating the KdTree object for the search method of the extraction
        typename pcl::search::KdTree<T>::Ptr tree(new pcl::search::KdTree<T>());
        tree->setInputCloud(Src);
        pcl::EuclideanClusterExtraction<T> ec;
        ec.setClusterTolerance(Tolerance);
        ec.setMinClusterSize(MinClusterSize);
        ec.setMaxClusterSize(MaxClusterSize);
        ec.setSearchMethod(tree);
        ec.setInputCloud(Src);

        std::vector<pcl::PointIndices> clusters;
        ec.extract(clusters);

        return clusters;
    }

    static std::vector<pcl::PointIndices> segmentMinCut(const typename pcl::PointCloud<T>::Ptr Src,
        const T& Center, double Radius, int Neighbours = 5, double Sigma = 0.25, double Weight = 0.8)
    {
        pcl::MinCutSegmentation<T> seg;
        seg.setInputCloud(Src);
        typename pcl::PointCloud<T>::Ptr foregroundPoints(new pcl::PointCloud<T>());
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

    static std::vector<pcl::PointIndices> segmentRegionGrowingKn(const typename pcl::PointCloud<T>::Ptr Src,
        int KNeighbours, int Neighbours, double Angle, double Curvature,
        int MinClusterSize = 100, int MaxClusterSize = 25000)
    {
        auto pair = estimateNormalPairKn(Src, KNeighbours);

        pcl::RegionGrowing<T, pcl::Normal> reg;
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

    static std::vector<pcl::PointIndices> segmentRegionGrowingKr(const typename pcl::PointCloud<T>::Ptr Src,
        double KRadius, int Neighbours, double Angle, double Curvature,
        int MinClusterSize = 100, int MaxClusterSize = 25000)
    {
        auto pair = estimateNormalPairKr(Src, KRadius);

        pcl::RegionGrowing<T, pcl::Normal> reg;
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

    static std::vector<pcl::PointIndices> segmentRegionGrowingRGB(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr Src,
        double Distance, double PointColor, double RegionColor, int MinClusterSize = 100, int MaxClusterSize = 25000)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::copyPointCloud(*Src, *cloud);
        pcl::search::Search<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
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

    static std::vector<pcl::PointIndices> segmentConditionalEuclidean(const pcl::PointCloud<pcl::PointXYZI>::Ptr Src,
        double KRadius, double ClusterRadius, double IntensityTolerance,
        int MinClusterSize = 100, int MaxClusterSize = 25000)
    {
        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
        // set up a Normal Estimation class and merge data in cloud_with_normals
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloudWithNormals(new pcl::PointCloud<pcl::PointXYZINormal>);
        pcl::copyPointCloud(*Src, *cloudWithNormals);
        pcl::NormalEstimation<pcl::PointXYZI, pcl::PointXYZINormal> ne;
        ne.setInputCloud(Src);
        ne.setSearchMethod(tree);
        ne.setRadiusSearch(KRadius);
        ne.compute(*cloudWithNormals);
        // set up a Conditional Euclidean Clustering class
        pcl::ConditionalEuclideanClustering<pcl::PointXYZINormal> cec(true);
        cec.setInputCloud(cloudWithNormals);
        cec.setConditionFunction(
            [=](const pcl::PointXYZINormal& PointA, const pcl::PointXYZINormal& PointB, float SquaredDistance) -> bool
            {
                if (std::abs(PointA.intensity - PointB.intensity) < IntensityTolerance)
                {
                    return true;
                }
                else
                {
                    return false;
                }
            });
        cec.setClusterTolerance(ClusterRadius);
        cec.setMinClusterSize(MinClusterSize);
        cec.setMaxClusterSize(MaxClusterSize);
        std::vector<pcl::PointIndices> clusters;
        cec.segment(clusters);

        return clusters;
    }

    static std::vector<pcl::PointIndices> segmentEdgeDetection(const typename pcl::PointCloud<T>::Ptr Src,
        double KRadius, int Neighbour, double Depth)
    {
        auto normal = estimateNormalKr(Src, KRadius);

        pcl::OrganizedEdgeFromNormals<T, pcl::Normal, pcl::Label> oed;
        oed.setInputNormals(normal);
        oed.setInputCloud(Src);
        oed.setEdgeType(oed.EDGELABEL_NAN_BOUNDARY | oed.EDGELABEL_OCCLUDING | oed.EDGELABEL_OCCLUDED |
            oed.EDGELABEL_HIGH_CURVATURE | oed.EDGELABEL_RGB_CANNY);
        oed.setDepthDisconThreshold(Depth);
        oed.setMaxSearchNeighbors(Neighbour);

        pcl::PointCloud<pcl::Label> labels;
        std::vector<pcl::PointIndices> clusters;
        oed.compute(labels, clusters);

        return clusters;
    }

    static std::vector<int> segmentSAC(const typename pcl::PointCloud<T>::Ptr Src, int Model,
        double DistanceThresh)
    {
        // create the segmentation object
        pcl::SACSegmentation<T> seg;
        // optional
        seg.setOptimizeCoefficients(true);
        // mandatory
        // for model enum definition(like pcl::SACMODEL_PLANE), please refer to:
        // https://pointclouds.org/documentation/group__sample__consensus.html
        seg.setModelType(Model);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(DistanceThresh);
        seg.setInputCloud(Src);
        pcl::ModelCoefficients::Ptr coeffs(new pcl::ModelCoefficients());
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
        seg.segment(*inliers, *coeffs);

        return inliers->indices;
    }

    static bool sampleConsensusModelCircle2D(const typename pcl::PointCloud<T>::Ptr Src,
        double DistanceThresh, std::vector<int>& Inliers, std::vector<double>& Coeffs)
    {
        // created RandomSampleConsensus object and compute the appropriated model
        typename pcl::SampleConsensusModelCircle2D<T>::Ptr
            modelCircle(new pcl::SampleConsensusModelCircle2D<T>(Src));
        pcl::RandomSampleConsensus<T> ransac(modelCircle, DistanceThresh);
        if (ransac.computeModel())
        {
            ransac.getInliers(Inliers);
#ifdef DEBUG
            // copies all inliers of the model computed to another PointCloud
            pcl::PointCloud<T>::Ptr inliers(new pcl::PointCloud<T>());
            Inliers.reset(new pcl::PointCloud<T>());
            pcl::copyPointCloud(*Src, Inliers, *inliers);
#endif

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

    static bool sampleConsensusModelLine(const typename pcl::PointCloud<T>::Ptr Src,
        double DistanceThresh, std::vector<int>& Inliers, std::vector<double>& Coeffs)
    {
        // created RandomSampleConsensus object and compute the appropriated model
        typename pcl::SampleConsensusModelLine<T>::Ptr
            modelLine(new pcl::SampleConsensusModelLine<T>(Src));
        pcl::RandomSampleConsensus<T> ransac(modelLine, DistanceThresh);
        if (ransac.computeModel())
        {
            ransac.getInliers(Inliers);

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
};
