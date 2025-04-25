/******************************************************************
visualization of PCL to solve that the PCLVisualizer cannot run across
multiple threads

Features:
- queue event
- one visualizer per thread
- xxx

Limits:
- Only one instance should be initiated otherwise the OpenGL would crash,
  it is a PCL issue
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2023-05-20: Initial version
2022-xx-xx: xxx
******************************************************************/
#pragma once
#include "event_queue.h"

#include <pcl/visualization/pcl_visualizer.h>

#include <thread>
#include <atomic>

template<typename T = pcl::PointXYZ, template<typename...> class P = pcl::PointCloud>
class PclVisualize
{
public:
    static const std::string& VERSION()
    {
        return "00.03";
    }

public:
    PclVisualize()
        : queue_(std::make_unique<EventQueue<void>>(5, false))
    {
        // spawn the event queue thread
		th_spinner_ = std::thread(std::bind(&PclVisualize::thViewerSpin, this));
    };
    ~PclVisualize()
    {
        viewer_->close();

        terminated_.store(true);
        th_spinner_.join();
    };

public:
    void viewCloud(const typename pcl::PointCloud<T>::Ptr Src, const std::string& CloudID,
        int PointSize = 1, std::array<double, 3> PointColor = {0.0, 1.0, 0.0})
    {
        queue_->produce(std::bind(&PclVisualize::view, this, Src, CloudID, PointSize, PointColor), "view");
    };

    void viewCloud02(const typename pcl::PointCloud<T>::Ptr Cur, const std::string& CurCloudID,
        const typename pcl::PointCloud<T>::Ptr Src, const std::string& CloudID,
        const typename pcl::PointCloud<T>::Ptr Tar, const std::string& TarCloudID,
        const typename pcl::PointCloud<T>::Ptr offset, const std::string& offsetID,
        int PointSize = 1, std::array<double, 3> PointColor = {0.0, 1.0, 0.0})
    {
        queue_->produce(std::bind(&PclVisualize::view02, this,Cur ,CurCloudID , Src, CloudID, Tar, TarCloudID, offset, offsetID, PointSize, PointColor), "view");
    };    

    void addviewCloud(const typename pcl::PointCloud<T>::Ptr Src, const std::string& CloudID,
        int PointSize = 1, std::array<double, 3> PointColor = {0.0, 1.0, 0.0})
    {
        queue_->produce(std::bind(&PclVisualize::addview, this, Src, CloudID, PointSize, PointColor), "view");
    };    

    void addviewVector(std::vector< typename pcl::PointCloud<T>::Ptr >  pointV, std::vector< std::string > cloudIDV,
        std::vector<int> PointSizeV, std::vector< std::array<double, 3> > PointColorV)
    {
        queue_->produce(std::bind(&PclVisualize::viewVector, this, pointV, cloudIDV, PointSizeV, PointColorV), "view");
    }


private:
    // NOTE: ONLY ONE INSTANCE SHOULD BE INITIATED
    // otherwise the OpenGL would complain and crash, it is a PCL issue
    std::shared_ptr<void> view(const typename pcl::PointCloud<T>::Ptr Src, const std::string& CloudID,
        int PointSize, std::array<double, 3> PointColor)
    {
        if (!viewer_)
        {
            viewer_.reset(new pcl::visualization::PCLVisualizer("3D Viewer"));
            viewer_->setBackgroundColor(1.0, 1.0, 1.0);
        }
        else
        {
            viewer_->removeAllPointClouds();
            viewer_->removeAllShapes();
        }

        if (viewer_->contains(CloudID))
        {
            viewer_->updatePointCloud<T>(Src, CloudID);
        }
        else
        {
            viewer_->addPointCloud<T>(Src, CloudID);
        }
        viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, PointSize, CloudID);
        viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
            PointColor[0], PointColor[1], PointColor[2], CloudID);

        return nullptr;
    }

    std::shared_ptr<void> view02(const typename pcl::PointCloud<T>::Ptr Cur, const std::string& CurCloudID,
        const typename pcl::PointCloud<T>::Ptr Src, const std::string& CloudID,
        const typename pcl::PointCloud<T>::Ptr Tar, const std::string& TarCloudID,
        const typename pcl::PointCloud<T>::Ptr offset, const std::string& offsetID,
        int PointSize, std::array<double, 3> PointColor)
    {
        if (!viewer_)
        {
            viewer_.reset(new pcl::visualization::PCLVisualizer("3D Viewer"));
            viewer_->setBackgroundColor(1.0, 1.0, 1.0);
        }

        if (viewer_->contains(CloudID))
        {
            viewer_->updatePointCloud<T>(Src, CloudID);
        }
        else
        {
            viewer_->addPointCloud<T>(Src, CloudID);
        }
        if (viewer_->contains(TarCloudID))
        {
            viewer_->updatePointCloud<T>(Tar, TarCloudID);
        }
        else
        {
            viewer_->addPointCloud<T>(Tar, TarCloudID);
        }       
        if (viewer_->contains(CurCloudID))
        {
            viewer_->updatePointCloud<T>(Cur, CurCloudID);
        }
        else
        {
            viewer_->addPointCloud<T>(Cur, CurCloudID);
        }  
        if (viewer_->contains(offsetID))
        {
            viewer_->updatePointCloud<T>(offset, offsetID);
        }
        else
        {
            viewer_->addPointCloud<T>(offset, offsetID);
        }                      

        viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, PointSize, CloudID);
        viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
            0.0, 1.0, 0.0, CloudID);
        viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, PointSize, CurCloudID);
        viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
            0.0, 0.0, 1.0, CurCloudID);               
        viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, PointSize, TarCloudID);
        viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
            1.0, 0.0, 0.0, TarCloudID);
        viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, PointSize, offsetID);
        viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
            1.0, 0.0, 0.0, offsetID);
        
        return nullptr;
    }

    std::shared_ptr<void> addview(const typename pcl::PointCloud<T>::Ptr Src, const std::string& CloudID,
        int PointSize, std::array<double, 3> PointColor)
    {
        if (!viewer_)
        {
            viewer_.reset(new pcl::visualization::PCLVisualizer("3D Viewer"));
            viewer_->setBackgroundColor(1.0, 1.0, 1.0);
        }

        if (viewer_->contains(CloudID))
        {
            viewer_->updatePointCloud<T>(Src, CloudID);
        }
        else
        {
            viewer_->addPointCloud<T>(Src, CloudID);
        }
        viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, PointSize, CloudID);
        viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
            0.0, 0.0, 0.0, CloudID);

        return nullptr;
    }    

    std::shared_ptr<void> viewVector(std::vector< typename pcl::PointCloud<T>::Ptr >  pointV, std::vector< std::string > cloudIDV,
        std::vector<int> PointSizeV, std::vector< std::array<double, 3> > PointColorV)
    {

        if (!viewer_)
        {
            viewer_.reset(new pcl::visualization::PCLVisualizer("3D Viewer"));
            viewer_->setBackgroundColor(1.0, 1.0, 1.0);
        }
        else
        {
            viewer_->removeAllPointClouds();
            viewer_->removeAllShapes();
        }

        if (pointV.size() != cloudIDV.size())
        {
            printf("pointV.size() != cloudIDV.size() , maybe wrong , please check vectors");
            return nullptr;
        }
        for (int i = 0; i < pointV.size(); i++)
        {

            if (viewer_->contains(cloudIDV[i]))
            {
                viewer_->updatePointCloud<T>(pointV[i], cloudIDV[i]);
            }
            else
            {
                viewer_->addPointCloud<T>(pointV[i], cloudIDV[i]);
            }
        }

        if (PointSizeV.size() != PointColorV.size())
        {
            printf("PointSizeV.size() != PointColorV.size() , maybe wrong , please check vectors");
            return nullptr;
        }

        for (int i = 0; i< PointSizeV.size(); i++)
        {
            viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, PointSizeV[i], cloudIDV[i]);
            viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
                PointColorV[i][0], PointColorV[i][1], PointColorV[i][2], cloudIDV[i]);
        }
        return nullptr;
    }


    void thViewerSpin()
    {
        while (!terminated_.load())
        {
            EventQueue<void>::EventFunc eventViewer(queue_->consume("view"));
	        if (eventViewer != nullptr)
	        {
		        // invoke the event means executing the action binded with it
		        eventViewer();
	        }

            if (viewer_)
            {
                viewer_->spinOnce(1);
            }
        }
    };

private:
    EventQueue<void>::UniquePtr queue_{ nullptr };
    pcl::visualization::PCLVisualizer::Ptr viewer_{ nullptr };
    std::thread th_spinner_;
    std::atomic_bool terminated_{ false };
};
