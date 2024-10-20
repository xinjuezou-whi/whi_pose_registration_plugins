/******************************************************************
utilities of PCL

Features:
- message conversion
- basic point cloud manipulations
- xxx

Reference:
- https://github.com/PointCloudLibrary/pcl/tree/master/test for usage
- https://github.com/PointCloudLibrary/pcl/tree/master/examples for usage

Written by Xinjue Zou, xinjue.zou.whi@gmail.com, Yue Zhou, sevendull@163.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2023-05-18: Initial version
2024-05-10: Added segment with normal vector
2024-xx-xx: xxx
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
#include <pcl/search/organized.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/features/fpfh_omp.h>
#include <vtkAutoInit.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/registration/ndt.h> // NDT配准算法
#include <pcl/segmentation/impl/extract_clusters.hpp>
#include <pcl/features/don.h>
#include "pcl_visualize.h"
typedef pcl::PointCloud<pcl::PointXYZ> pointcloud;
typedef pcl::PointCloud<pcl::Normal> pointnormal;
typedef pcl::PointCloud<pcl::FPFHSignature33> fpfhFeature;
typedef pcl::PointCloud<pcl::PointXYZ> pointcloud;

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

    static void matrix2angle(Eigen::Matrix4f &result_trans, Eigen::Vector3f &result_angle)
    {
        double ax, ay, az;
        if (result_trans(2, 0) == 1 || result_trans(2, 0) == -1)
        {
            az = 0;
            double dlta;
            dlta = atan2(result_trans(0, 1), result_trans(0, 2));
            if (result_trans(2, 0) == -1)
            {
                ay = M_PI / 2;
                ax = az + dlta;
            }
            else
            {
                ay = -M_PI / 2;
                ax = -az + dlta;
            }
        }
        else
        {
            ay = -asin(result_trans(2, 0));
            ax = atan2(result_trans(2, 1) / cos(ay), result_trans(2, 2) / cos(ay));
            az = atan2(result_trans(1, 0) / cos(ay), result_trans(0, 0) / cos(ay));
        }
        result_angle << ax, ay, az;
    }


    static fpfhFeature::Ptr compute_fpfh_feature(pointcloud::Ptr input_cloud)
    {
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
        //-------------------------法向量估计-----------------------
        pointnormal::Ptr normals(new pointnormal);
        pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n;
        n.setInputCloud(input_cloud);
        n.setNumberOfThreads(8);        // 设置openMP的线程数
        n.setSearchMethod(tree);        // 搜索方式
        n.setKSearch(20);               // K近邻点个数
        n.compute(*normals);
        //-------------------------FPFH估计-------------------------
        fpfhFeature::Ptr fpfh(new fpfhFeature);
        pcl::FPFHEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fest;
        fest.setNumberOfThreads(8);     //指定8核计算
        fest.setInputCloud(input_cloud);//输入点云
        fest.setInputNormals(normals);  //输入法线
        fest.setSearchMethod(tree);     //搜索方式
        fest.setKSearch(20);            //K近邻点个数
        //fest.setRadiusSearch(0.025);  //搜索半径
        fest.compute(*fpfh);            //计算FPFH
        return fpfh;
    }

    //体素下采样
    static pointcloud::Ptr voxel_grid_fiter(pointcloud::Ptr & inCloud , std::vector<float>& samplecoeff)
    {
        pcl::VoxelGrid<pcl::PointXYZ> vs;
        vs.setLeafSize(samplecoeff[0], samplecoeff[1], samplecoeff[2]);  //0.005    // 设置0.001不宜 ,过小，采样点数过多，会出现 错误 翻转现象  
        vs.setInputCloud(inCloud);
        pointcloud::Ptr outCloud(new pointcloud);
        vs.filter(*outCloud);
        return outCloud;
    }

    //配准
    static bool regist_sacia_ndt(pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud,
        pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud, Eigen::Vector3f& euler_angles,
        std::vector<double>& transxy, double& score, std::vector<float>& samplecoeff, int maxiter,
        Eigen::Matrix4f& outmatrix, bool neediter = true)
    {
        pointcloud::Ptr source(new pointcloud);
        pointcloud::Ptr target(new pointcloud);
        source = voxel_grid_fiter(source_cloud,samplecoeff); // 下采样滤波
        target = voxel_grid_fiter(target_cloud,samplecoeff); // 下采样滤波
        printf("[regist_sacia_ndt] downsampled source cloud to: %d\n", source->points.size());
        printf("[regist_sacia_ndt] downsampled target cloud to: %d\n", target->points.size());
        if (source->points.size() < 10 )
        {    
            ROS_ERROR("source sample get count < 10, wrong");
            score =1 ;
            return false;
        }

        // 1、计算源点云和目标点云的FPFH
        fpfhFeature::Ptr source_fpfh = compute_fpfh_feature(source);
        fpfhFeature::Ptr target_fpfh = compute_fpfh_feature(target);
    
        // 2、采样一致性SAC_IA初始配准
        clock_t start, end;
        start = clock();
        pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia;
        sac_ia.setInputSource(source);
        sac_ia.setSourceFeatures(source_fpfh);
        sac_ia.setInputTarget(target);
        sac_ia.setTargetFeatures(target_fpfh);
        sac_ia.setMaximumIterations(maxiter);          //设置之后，精确度更高，可以减少翻转现象 7000
        sac_ia.setMinSampleDistance(0.003f);       // 设置样本之间的最小距离
        sac_ia.setMaxCorrespondenceDistance(0.1);// 设置对应点对之间的最大距离  //0.1
        //sac_ia.setNumberOfSamples(80);       //500   // 设置每次迭代计算中使用的样本数量（可省）,可节省时间
        sac_ia.setCorrespondenceRandomness(6);   // 设置在6个最近特征对应中随机选取一个
        pointcloud::Ptr align(new pointcloud);
        sac_ia.align(*align);
        Eigen::Matrix4f initial_RT = Eigen::Matrix4f::Identity();// 定义初始变换矩阵
        initial_RT = sac_ia.getFinalTransformation();
        cout << "\nSAC_IA has converged:"<< sac_ia.hasConverged() <<", score is " << sac_ia.getFitnessScore() << endl;
        cout << "变换矩阵：\n" << initial_RT << endl;
        score = 1;
        if (sac_ia.hasConverged())
        {
            score =  sac_ia.getFitnessScore();
        }
        //pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        //pcl::transformPointCloud(*source_cloud, *output_cloud, sac_ia.getFinalTransformation());
        
        Eigen::Matrix4f rot_mat = sac_ia.getFinalTransformation();
        matrix2angle(rot_mat,euler_angles);
        transxy.clear();
        transxy.push_back(rot_mat(0,3));
        transxy.push_back(rot_mat(1,3));
        printf("[regist_sacia_ndt] roll: %f, pitch: %f, yaw: %f\n", euler_angles(0), euler_angles(1), euler_angles(2));
        printf("[regist_sacia_ndt] x trans: %f, y trans: %f\n", transxy[0], transxy[1]);
        outmatrix = rot_mat;
        if (!neediter)
        {
            return sac_ia.hasConverged();
        }
        if (sac_ia.hasConverged())
        {
            if (fabs(euler_angles(0)) == 0 && fabs(euler_angles(1)) == 0 )
            {
                ROS_INFO("out right value");
                outmatrix = rot_mat;
                return sac_ia.hasConverged();
            }
            else
            {
                Eigen::Matrix4f out_mat;
                return regist_sacia_ndt(target_cloud, source_cloud,
                    euler_angles, transxy, score, samplecoeff, maxiter, outmatrix);
            }
        }
        else
        {
            return sac_ia.hasConverged();
        }        
    }

    static void segment_don(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& modelcloud,
        std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& outcloudvec, double scale1, double scale2, double threshold,
        double segradius, std::vector<float>& samplecoeff, int maxiter)
    {
        int VISUAL = 1, SAVE = 0;//0表示不显示任何结果，1表示显示每一步的输出，2只显示最终结果

        //pcl::PointCloud<PointXYZ>::Ptr cloud(new pcl::PointCloud<PointXYZ>);
        //pcl::io::loadPCDFile(file1.c_str(), *cloud);

        std::vector<int> indices_src;
        pcl::removeNaNFromPointCloud(*cloud, *cloud, indices_src);

        //创建KD树
        pcl::search::Search<pcl::PointXYZ>::Ptr tree;
        if (cloud->isOrganized())
        {
            tree.reset(new pcl::search::OrganizedNeighbor<pcl::PointXYZ>());
        }
        else
        {
            tree.reset(new pcl::search::KdTree<pcl::PointXYZ>(false));
        }

        tree->setInputCloud(cloud);

        if (scale1 >= scale2)
        {
            cerr << "Error: Large scale must be > small scale!" << endl;
            exit(EXIT_FAILURE);
        }

        //-----------------------计算法线-----------------------------
        pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::PointNormal> ne;
        ne.setInputCloud(cloud);
        ne.setSearchMethod(tree);

        //-----------------------设置视点--------------------------
        //ne.setViewPoint(std::numeric_limits<float>::max(), std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
        ne.setViewPoint(100, 100, 0);

        //-----------------自定义 计算法向量---------------
        cout << "Calculating normals for scale1..." << scale1 << endl;
        pcl::PointCloud<pcl::PointNormal>::Ptr normals_small_scale(new pcl::PointCloud<pcl::PointNormal>);

        int smallscale = static_cast<int>(std::floor(scale1));
        Eigen::Vector3f normalvec(0,0,1);
        for (size_t i=0; i<cloud->points.size (); i++)
        {
            if (i + smallscale >= cloud->points.size())
            {
                pcl::PointNormal new_point;
                new_point.x = cloud->points[i].x;
                new_point.y = cloud->points[i].y;
                new_point.z = cloud->points[i].z;
                new_point.normal_x = normalvec.x();
                new_point.normal_y = normalvec.y();
                new_point.normal_z = normalvec.z();
                normals_small_scale->points.push_back(new_point);
                continue;
            }
            if (i + 1 >= cloud->points.size())
            {
                pcl::PointNormal new_point;
                new_point.x = cloud->points[i].x;
                new_point.y = cloud->points[i].y;
                new_point.z = cloud->points[i].z;
                new_point.normal_x = normalvec.x();
                new_point.normal_y = normalvec.y();
                new_point.normal_z = normalvec.z();
                normals_small_scale->points.push_back(new_point);
                continue;
            }

            Eigen::Vector3f curpoint(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
            Eigen::Vector3f neighborpoint(cloud->points[i+1].x, cloud->points[i+1].y, cloud->points[i+1].z);
            Eigen::Vector3f nextpoint(cloud->points[i+smallscale].x, cloud->points[i+smallscale].y, cloud->points[i+smallscale].z);

            double neighbordis = sqrt( (curpoint.x()-neighborpoint.x())*(curpoint.x()-neighborpoint.x()) +  (curpoint.y()-neighborpoint.y())*(curpoint.y()-neighborpoint.y()) +
            (curpoint.z()-neighborpoint.z())*(curpoint.z()-neighborpoint.z()) );
            if (neighbordis > 0.15)
            {
                pcl::PointNormal new_point;
                new_point.x = cloud->points[i].x;
                new_point.y = cloud->points[i].y;
                new_point.z = cloud->points[i].z;
                new_point.normal_x = normalvec.x();
                new_point.normal_y = normalvec.y();
                new_point.normal_z = normalvec.z();
                normals_small_scale->points.push_back(new_point);
                continue;			
            }

            Eigen::Vector3f curzpoint(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z + 0.3);
            Eigen::Vector3f curvec(nextpoint.x()-curpoint.x(),nextpoint.y()-curpoint.y(),nextpoint.z()-curpoint.z());
            Eigen::Vector3f curzvec(curzpoint.x()-curpoint.x(),curzpoint.y()-curpoint.y(),curzpoint.z()-curpoint.z());
            Eigen::Vector3f crossvec = curvec.cross(curzvec);
            normalvec = crossvec.normalized();

            pcl::PointNormal new_point;
            new_point.x = curpoint.x();
            new_point.y = curpoint.y();
            new_point.z = curpoint.z();
            new_point.normal_x = normalvec.x();
            new_point.normal_y = normalvec.y();
            new_point.normal_z = normalvec.z();
            normals_small_scale->points.push_back(new_point);
        }

        // 大尺度辅助半径
        cout << "Calculating normals for scale2..." << scale2 << endl;
        pcl::PointCloud<pcl::PointNormal>::Ptr normals_large_scale(new pcl::PointCloud<pcl::PointNormal>);
        int largescale = static_cast<int>(std::floor(scale2));
        normalvec << 0.0f, 0.0f, 1.0f;
        for (size_t i=0; i<cloud->points.size (); i++)
        {
            if (i + largescale >= cloud->points.size())
            {
                pcl::PointNormal new_point;
                new_point.x = cloud->points[i].x;
                new_point.y = cloud->points[i].y;
                new_point.z = cloud->points[i].z;
                new_point.normal_x = normalvec.x();
                new_point.normal_y = normalvec.y();
                new_point.normal_z = normalvec.z();
                normals_large_scale->points.push_back(new_point);
                continue;
            }
            if (i + 1 >= cloud->points.size())
            {
                pcl::PointNormal new_point;
                new_point.x = cloud->points[i].x;
                new_point.y = cloud->points[i].y;
                new_point.z = cloud->points[i].z;
                new_point.normal_x = normalvec.x();
                new_point.normal_y = normalvec.y();
                new_point.normal_z = normalvec.z();
                normals_large_scale->points.push_back(new_point);
                continue;
            }

            Eigen::Vector3f curpoint(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
            Eigen::Vector3f neighborpoint(cloud->points[i+1].x, cloud->points[i+1].y, cloud->points[i+1].z);
            Eigen::Vector3f nextpoint(cloud->points[i+largescale].x, cloud->points[i+largescale].y, cloud->points[i+largescale].z);

            double neighbordis = sqrt( (curpoint.x()-neighborpoint.x())*(curpoint.x()-neighborpoint.x()) +  (curpoint.y()-neighborpoint.y())*(curpoint.y()-neighborpoint.y()) +
            (curpoint.z()-neighborpoint.z())*(curpoint.z()-neighborpoint.z()) );
            if (neighbordis > 0.15)
            {
                pcl::PointNormal new_point;
                new_point.x = cloud->points[i].x;
                new_point.y = cloud->points[i].y;
                new_point.z = cloud->points[i].z;
                new_point.normal_x = normalvec.x();
                new_point.normal_y = normalvec.y();
                new_point.normal_z = normalvec.z();
                normals_large_scale->points.push_back(new_point);
                continue;			
            }

            Eigen::Vector3f curzpoint(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z + 0.3);
            Eigen::Vector3f curvec(nextpoint.x()-curpoint.x(),nextpoint.y()-curpoint.y(),nextpoint.z()-curpoint.z());
            Eigen::Vector3f curzvec(curzpoint.x()-curpoint.x(),curzpoint.y()-curpoint.y(),curzpoint.z()-curpoint.z());
            Eigen::Vector3f crossvec = curvec.cross(curzvec);
            normalvec = crossvec.normalized();

            pcl::PointNormal new_point;
            new_point.x = curpoint.x();
            new_point.y = curpoint.y();
            new_point.z = curpoint.z();
            new_point.normal_x = normalvec.x();
            new_point.normal_y = normalvec.y();
            new_point.normal_z = normalvec.z();
            normals_large_scale->points.push_back(new_point);
        }

        //---------------------为DON创造输出点云------------------------------
        pcl::PointCloud<pcl::PointNormal>::Ptr doncloud(new pcl::PointCloud<pcl::PointNormal>);
        pcl::copyPointCloud<pcl::PointXYZ, pcl::PointNormal>(*cloud, *doncloud);

        cout << "Calculating DoN... " << endl;
        //------------------------创造DoN算子-------------------------------------
        pcl::DifferenceOfNormalsEstimation<pcl::PointXYZ, pcl::PointNormal, pcl::PointNormal> don;
        don.setInputCloud(cloud);
        don.setNormalScaleLarge(normals_large_scale);
        don.setNormalScaleSmall(normals_small_scale);

        if (!don.initCompute())
        {
            cerr << "Error: Could not intialize DoN feature operator" << endl;
            exit(EXIT_FAILURE);
        }
        //----------------------计算DoN特征向量-------------------------
        don.computeFeature(*doncloud);//对输入点集，计算DON特征向量，并输出

        //------------------输出一些不同的曲率--------------------------------
        {
            cout << "You may have some sense about the input threshold（curvature） next time for your data" << endl;
            int size_cloud = doncloud->size();
            cout << size_cloud << endl;
            int step = size_cloud / 10;
            for (int i = 0; i < size_cloud; i += step)cout << " " << doncloud->points[i].curvature << " " << endl;

        }

        //-------------------------按大小滤波-----------------------------
        cout << "Filtering out DoN mag <= " << threshold << "..." << endl;
        // 创建条件滤波函数
        pcl::ConditionOr<pcl::PointNormal>::Ptr range_cond(new pcl::ConditionOr<pcl::PointNormal>());//确定点是否属于满足设定的条件
        range_cond->addComparison(pcl::FieldComparison<pcl::PointNormal>::ConstPtr(
            new pcl::FieldComparison<pcl::PointNormal>("curvature", pcl::ComparisonOps::GT, threshold))
        );//添加比较条件
        // 创建滤波
        pcl::ConditionalRemoval<pcl::PointNormal> condrem;
        condrem.setInputCloud(doncloud);
        condrem.setCondition(range_cond);
        pcl::PointCloud<pcl::PointNormal>::Ptr doncloud_filtered(new pcl::PointCloud<pcl::PointNormal>);
        //设置输入点云
        condrem.filter(*doncloud_filtered);
        doncloud = doncloud_filtered;
        cout << "Filtered Pointcloud: " << doncloud->points.size() << " data points." << endl;

        cout << "Clustering using EuclideanClusterExtraction with tolerance <= " << segradius << "..." << endl;

        pcl::search::KdTree<pcl::PointNormal>::Ptr segtree(new pcl::search::KdTree<pcl::PointNormal>);
        segtree->setInputCloud(doncloud);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointNormal> ec;

        ec.setClusterTolerance(segradius);
        ec.setMinClusterSize(50);
        ec.setMaxClusterSize(3000);
        ec.setSearchMethod(segtree);
        ec.setInputCloud(doncloud);
        ec.extract(cluster_indices);
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr find_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud <pcl::PointXYZ>::Ptr tmp_xyz(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::copyPointCloud<pcl::PointNormal, pcl::PointXYZ>(*doncloud, *tmp_xyz);
      
        {
            printf("[segment_don] tmp_xyz .point.size = %d\n" , tmp_xyz->points.size());
            int ci = 0;
            double minscore = 1.0;

            for (const auto& cluster : cluster_indices)
            {
                ci++;
                pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFeatures(new pcl::PointCloud<pcl::PointXYZ>());
                //PclUtilities<pcl::PointXYZI>::extractTo(pclOperating, cluster, cloudFeatures);
                pcl::PointIndices::Ptr indices(new pcl::PointIndices(cluster));
                pcl::ExtractIndices<pcl::PointXYZ> extract;
                extract.setInputCloud(tmp_xyz);
                extract.setIndices(indices);
                extract.setNegative(false);
                extract.filter(*cloudFeatures);
                //*out_cloud += *cloudFeatures;
                printf("[segment_don] extractTo cluster indices = %d, cloudFeatures->size() = %d\n",
                    ci, cloudFeatures->size());

                Eigen::Vector3f euler_angles ;
                double score;
                std::vector<double> transxy;
                Eigen::Matrix4f outmat;
                regist_sacia_ndt(modelcloud, cloudFeatures, euler_angles, transxy,score, samplecoeff, maxiter, outmat, false);
                printf("[segment_don] segment find cloud, ci = %d\n", ci);
                if (score < minscore)
                {
                    //minscore = score;
                    //*find_cloud = *cloudFeatures;
                    //ROS_INFO("find cloud, ci=%d ",ci);
                }
                double score_thres = 5e-4;
                if (score < score_thres)
                {
                    outcloudvec.push_back(cloudFeatures);
                    printf("[segment_don] find cloud, ci = %d\n",ci);
                }

            }
            //*outcloud = *find_cloud;
        }
        
    }

    static void segment_Eucli_sel(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudFeatures, pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& outcloud,double feature_segment_distance_thresh_,int feature_min_size_,int feature_max_size_,std::vector<float>& samplecoeff,int maxiter)
    {

        std::vector<pcl::PointIndices> featureIndices;
        featureIndices = segmentEuclidean(cloudFeatures,feature_segment_distance_thresh_, feature_min_size_, feature_max_size_);
        std::cout << "total feature number from features: " << featureIndices.size() << std::endl;
        for (int i = 0; i < featureIndices.size(); ++i)
        {
            std::cout << "feature " << i << " has points " << featureIndices[i].indices.size() << std::endl;
        }
        int ci = 0;
        double minscore = 1.0;
        pcl::PointCloud<pcl::PointXYZ>::Ptr find_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& feature : featureIndices)
        {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFeature(new pcl::PointCloud<pcl::PointXYZ>());
            extractTo(cloudFeatures, feature, cloudFeature);

            Eigen::Vector3f euler_angles ;
            double score;
            std::vector<double> transxy;
            Eigen::Matrix4f outmat;
            regist_sacia_ndt(target_cloud, cloudFeature, euler_angles,transxy, score, samplecoeff, maxiter, outmat);
            printf("[segment_Eucli_sel] segment find cloud, ci = %d\n", ci);
            find_cloud->points.clear();
            if(score < minscore)
            {
                minscore = score;
                *find_cloud = *cloudFeature;
                printf("[segment_Eucli_sel] find cloud, ci = %d\n", ci);
            }

        }
        *outcloud = *find_cloud;

    }
};

// 最小包围圆算法 

    typedef long long ll;
    const int maxn = 1e6 + 10;
    const double eps = 1e-8;
    int cmp(double x) 
    {
        if (fabs(x) < eps) return 0;
        if (x > 0) return 1;
        return -1;
    }

    struct mypoint
    {
        double x, y;
        mypoint() {}
        mypoint(double a, double b) : x(a), y(b) {}
        friend mypoint operator-(const mypoint& a, const mypoint& b) 
        {
            return mypoint(a.x - b.x, a.y - b.y);
        }
        double norm()
        {
            return sqrt(pow(x, 2) + pow(y, 2));
        }
    };
    double dist(const mypoint& a, const mypoint& b) 
    {
        return (a - b).norm();
    }

    void circle_center(mypoint p0, mypoint p1, mypoint& cp) 
    {
        cp.x = (p0.x + p1.x) / 2;
        cp.y = (p0.y + p1.y) / 2;
    }

    void circle_center(mypoint p0, mypoint p1, mypoint p2, mypoint& cp) 
    {
        double a1 = p1.x - p0.x,b1 = p1.y - p0.y,c1 = (p1.x * p1.x - p0.x * p0.x + p1.y * p1.y - p0.y * p0.y) / 2;
        double a2 = p2.x - p0.x,b2 = p2.y - p0.y,c2 = (p2.x * p2.x - p0.x * p0.x + p2.y * p2.y - p0.y * p0.y) / 2;
        cp.x = (c1 * b2 - b1 * c2) / (a1 * b2 - b1 * a2);
        cp.y = (a1 * c2 - c1 * a2) / (a1 * b2 - b1 * a2);
    }

    //double radius; point center;
    bool point_in(const mypoint& p, double radius, mypoint center) 
    {
        return cmp(dist(p, center) - radius) < 0;
    }

    void min_circle_cover(const std::vector<mypoint>& List, double& radius, mypoint& center)
    {
        radius = 0;
        center = List[0];
        for (int i = 1; i < List.size(); i++) 
        if (!point_in(List[i],radius,center)) 
        {
            radius = 0;
            center = List[0];
            for (int i = 1; i < List.size(); i++) 
            if (!point_in(List[i],radius,center)) 
            {
                center = List[i], radius = 0;
                for (int j = 0; j < i; j++) 
                if (!point_in(List[j],radius,center)) 
                {
                    circle_center(List[i], List[j], center);
                    radius = dist(List[j], center);
                    for (int k = 0; k < j; k++) 
                    if (!point_in(List[k],radius,center))
                    {
                        circle_center(List[i], List[j], List[k], center);
                        radius = dist(List[k], center);
                    }
                }
            }
        }
    }
