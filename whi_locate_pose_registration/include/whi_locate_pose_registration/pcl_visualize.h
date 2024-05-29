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
        return "00.02";
    }

public:
    PclVisualize()
        : queue_(std::make_unique<EventQueue>(false))
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
        int PointSize = 1, std::array<double, 3> PointColor = {0.0, 1.0, 0.0})
    {
        queue_->produce(std::bind(&PclVisualize::view02, this,Cur ,CurCloudID , Src, CloudID, Tar, TarCloudID, PointSize, PointColor), "view");
    };    


private:
    // NOTE: ONLY ONE INSTANCE SHOULD BE INITIATED
    // otherwise the OpenGL would complain and crash, it is a PCL issue
    void view(const typename pcl::PointCloud<T>::Ptr Src, const std::string& CloudID,
        int PointSize, std::array<double, 3> PointColor)
    {
        if (!viewer_)
        {
            viewer_.reset(new pcl::visualization::PCLVisualizer("3D Viewer"));
            viewer_->setBackgroundColor(0.0, 0.0, 0.0);
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
    }

    void view02(const typename pcl::PointCloud<T>::Ptr Cur, const std::string& CurCloudID,
        const typename pcl::PointCloud<T>::Ptr Src, const std::string& CloudID,
    const typename pcl::PointCloud<T>::Ptr Tar, const std::string& TarCloudID,
        int PointSize, std::array<double, 3> PointColor)
    {
        if (!viewer_)
        {
            viewer_.reset(new pcl::visualization::PCLVisualizer("3D Viewer"));
            viewer_->setBackgroundColor(0.0, 0.0, 0.0);
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


        viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, PointSize, CloudID);
        viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
            0.0, 1.0, 0.0, CloudID);
        viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, PointSize, CurCloudID);
        viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
            0.0, 0.0, 1.0, CurCloudID);               
        viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, PointSize, TarCloudID);
        viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,
            1.0, 0.0, 0.0, TarCloudID);            
    }    

    void thViewerSpin()
    {
        while (!terminated_.load())
        {
            // handle send goal in callback done logic
            EventQueue::EventFunc eventViewer(queue_->consume("view"));
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
    EventQueue::UniquePtr queue_{ nullptr };
    pcl::visualization::PCLVisualizer::Ptr viewer_{ nullptr };
    std::thread th_spinner_;
    std::atomic_bool terminated_{ false };
};
