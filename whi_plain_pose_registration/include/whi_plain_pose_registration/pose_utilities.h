/******************************************************************
utilities of poses operation

Features:
- converstion between Eular and Quaternion
- transform
- xxx

Written by Xinjue Zou, xinjue.zou@outlook.com

GNU General Public License, check LICENSE for more information.
All text above must be included in any redistribution.

Changelog:
2023-05-22: Initial version
2022-xx-xx: xxx
******************************************************************/
#pragma once
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf/LinearMath/Matrix3x3.h>

class PoseUtilities
{
public:
    PoseUtilities() = delete;
    ~PoseUtilities() = delete;

public:
    static geometry_msgs::Quaternion fromEuler(double Roll, double Pitch, double Yaw)
    {
        tf2::Quaternion orientation;
		orientation.setRPY(Roll, Pitch, Yaw);

		return tf2::toMsg(orientation);
    }

    static std::array<double, 3> toEuler(const geometry_msgs::Quaternion& Orientation)
    {
        tf::Quaternion quat(Orientation.x, Orientation.y, Orientation.z, Orientation.w);
        double roll = 0.0, pitch = 0.0, yaw = 0.0;
  		tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

        return { roll, pitch, yaw };
    }

    static geometry_msgs::TransformStamped getTransform(const geometry_msgs::Pose& Parent,
        const geometry_msgs::Pose& Child, const std::string FrameParent = "", const std::string FrameChild = "")
    {
        // convert the poses to tf2::Transform objects
        tf2::Transform transformParent;
        tf2::Transform transformChild;
        tf2::fromMsg(Parent, transformParent);
        tf2::fromMsg(Child, transformChild);
        // calculate the transform between the two poses
        tf2::Transform transform = transformParent.inverse() * transformChild;
        // convert the transform back to geometry_msgs::TransformStamped
        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = FrameParent;
        transformStamped.child_frame_id = FrameChild;
        transformStamped.transform = tf2::toMsg(transform);

        return transformStamped;
    }

    static geometry_msgs::Pose applyTransform(const geometry_msgs::Pose& Src, geometry_msgs::TransformStamped& Transform)
    {
        // apply the transform to the pose
        geometry_msgs::Pose transformedPose;
        tf2::doTransform(Src, transformedPose, Transform);

        return transformedPose;
    }
};
