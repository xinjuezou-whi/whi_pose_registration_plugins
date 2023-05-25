/******************************************************************
utilities of poses operation

Features:
- convertion between Euler and quaternion
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
#include <tf2/LinearMath/Matrix3x3.h>
#include <angles/angles.h>

class PoseUtilities
{
public:
    static const std::string& VERSION()
    {
        return "00.04";
    }

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

    static std::array<double, 3> toEuler(const tf2::Quaternion& Quaternion)
    {
        double roll = 0.0, pitch = 0.0, yaw = 0.0;
  		tf2::Matrix3x3(Quaternion).getRPY(roll, pitch, yaw);

        return { roll, pitch, yaw };
    }

    static std::array<double, 3> toEuler(const geometry_msgs::Quaternion& Orientation)
    {
        tf2::Quaternion quaternion(Orientation.x, Orientation.y, Orientation.z, Orientation.w);

        return toEuler(quaternion);
    }

    static geometry_msgs::TransformStamped convert(const geometry_msgs::Pose& Src)
    {
        geometry_msgs::TransformStamped transform;
        transform.transform.translation.x = Src.position.x;
        transform.transform.translation.y = Src.position.y;
        transform.transform.translation.z = Src.position.z;
        transform.transform.rotation = Src.orientation;

        return transform;
    }

    static geometry_msgs::Pose convert(const geometry_msgs::TransformStamped& Src)
    {
        geometry_msgs::Pose pose;
        pose.position.x = Src.transform.translation.x;
        pose.position.y = Src.transform.translation.y;
        pose.position.z = Src.transform.translation.z;
        pose.orientation = Src.transform.rotation;

        return pose;
    }

    static geometry_msgs::Quaternion getRelativeRotation(const geometry_msgs::Quaternion& From,
        const geometry_msgs::Quaternion& To)
    {
        tf2::Quaternion quatFrom(From.x, From.y, From.z, From.w);
        tf2::Quaternion quatTo(To.x, To.y, To.z, To.w);

        return tf2::toMsg(quatTo * quatFrom.inverse());
    }

    static geometry_msgs::TransformStamped getTransform(const geometry_msgs::Pose& From,
        const geometry_msgs::Pose& To, const std::string FrameFrom = "", const std::string FrameTo = "")
    {
        // convert the poses to tf2::Transform objects
        tf2::Transform transformFrom;
        tf2::Transform transformTo;
        tf2::fromMsg(From, transformFrom);
        tf2::fromMsg(To, transformTo);
        // calculate the transform between the two poses
        tf2::Transform transform = transformTo * transformFrom.inverse();
        // convert the transform back to geometry_msgs::TransformStamped
        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = FrameFrom;
        transformStamped.child_frame_id = FrameTo;
        transformStamped.transform = tf2::toMsg(transform);

        return transformStamped;
    }

    static geometry_msgs::Pose applyTransform(const geometry_msgs::Pose& Src,
        const geometry_msgs::TransformStamped& Transform)
    {
        // apply the transform to the pose
        geometry_msgs::Pose transformedPose;
        tf2::doTransform(Src, transformedPose, Transform);

        return transformedPose;
    }

    static double distance(const geometry_msgs::Pose& PoseA, const geometry_msgs::Pose& PoseB)
    {
        return sqrt(pow(PoseA.position.x - PoseB.position.x, 2) + 
            pow(PoseA.position.y - PoseB.position.y, 2) + pow(PoseA.position.z - PoseB.position.z, 2));
    }

    static geometry_msgs::Point createVector2D(const geometry_msgs::Point& Start, const geometry_msgs::Point& End)
	{
        geometry_msgs::Point point;
        point.x = End.x - Start.x;
        point.y = End.y - Start.y;
		return point;
	}

    static geometry_msgs::Point createVector2D(const geometry_msgs::Point& Start, double Length, double Theta)
	{
        geometry_msgs::Point point;
        point.x = Length * sin(Theta) + Start.x;
        point.y = Length * cos(Theta) + Start.y;
		return point;
	}

    static double angleBetweenVectors2D(const geometry_msgs::Point& From, const geometry_msgs::Point& To)
	{
		double dot = From.x * To.x + From.y * To.y;
		double magFrom = sqrt(From.x * From.x + From.y * From.y);
		double magTo = sqrt(To.x * To.x + To.y * To.y);

		// // axis of cross product
		// double z = From.x * To.y - From.y * To.x;

		// return signOf(z) * acos(dot / (magFrom * magTo));
        return acos(dot / (magFrom * magTo));
	}

#ifdef DEBUG
    static void UT()
    {
        geometry_msgs::Pose poseRef;
        poseRef.orientation = PoseUtilities::fromEuler(0, 0, angles::from_degrees(90));
        geometry_msgs::Pose poseTo;
        poseTo.position.x = -1;
        poseTo.position.y = 2;
        poseTo.position.z = -3;
        poseTo.orientation = PoseUtilities::fromEuler(0, 0, angles::from_degrees(45));
        auto euler = PoseUtilities::toEuler(PoseUtilities::getRelativeRotation(poseTo.orientation, poseRef.orientation));
        std::cout << "relative rotation: " << angles::to_degrees(euler[0]) << ","
            << angles::to_degrees(euler[1]) << "," << angles::to_degrees(euler[2]) << std::endl;
        auto trans = PoseUtilities::getTransform(poseTo, poseRef);
        std::cout << "translation x:" << trans.transform.translation.x << ", y:" << trans.transform.translation.y <<
            ", z:" << trans.transform.translation.z << std::endl;
        auto rotation = PoseUtilities::toEuler(trans.transform.rotation);
        std::cout << "rotation r:" << angles::to_degrees(rotation[0]) << ", p:" << angles::to_degrees(rotation[1]) <<
            ", y:" << angles::to_degrees(rotation[2]) << std::endl;
        auto transback = PoseUtilities::applyTransform(poseTo, trans);
        std::cout << "transform back x:" << transback.position.x << ", y:" << transback.position.y <<
            ", z:" << transback.position.z << std::endl;
        auto oriback = PoseUtilities::toEuler(transback.orientation);
        std::cout << "transform back roll:" << angles::to_degrees(oriback[0]) << ", pitch:" <<
            angles::to_degrees(oriback[1]) << ", yaw: " << angles::to_degrees(oriback[2]) << std::endl;
    }
#endif
};
