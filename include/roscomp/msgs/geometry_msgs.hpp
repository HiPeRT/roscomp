#pragma once

#include "roscomp/typesupport_id.hpp"

#if TKROS_VERSION == 1
#include <geometry_msgs/Accel.h>
#include <geometry_msgs/AccelStamped.h>
#include <geometry_msgs/AccelWithCovariance.h>
#include <geometry_msgs/AccelWithCovarianceStamped.h>
#include <geometry_msgs/Inertia.h>
#include <geometry_msgs/InertiaStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/WrenchStamped.h>

#elif TKROS_VERSION == 2

#include <geometry_msgs/msg/accel.hpp>
#include <geometry_msgs/msg/accel_stamped.hpp>
#include <geometry_msgs/msg/accel_with_covariance.hpp>
#include <geometry_msgs/msg/accel_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/inertia.hpp>
#include <geometry_msgs/msg/inertia_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/polygon.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/quaternion_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>

#endif

namespace roscomp {
namespace geometry_msgs {

#if TKROS_VERSION == 1

typedef ::geometry_msgs::Accel Accel;
typedef ::geometry_msgs::AccelStamped AccelStamped;
typedef ::geometry_msgs::AccelWithCovariance AccelWithCovariance;
typedef ::geometry_msgs::AccelWithCovarianceStamped AccelWithCovarianceStamped;
typedef ::geometry_msgs::Inertia Inertia;
typedef ::geometry_msgs::InertiaStamped InertiaStamped;
typedef ::geometry_msgs::Point Point;
typedef ::geometry_msgs::Point32 Point32;
typedef ::geometry_msgs::PointStamped PointStamped;
typedef ::geometry_msgs::Polygon Polygon;
typedef ::geometry_msgs::PolygonStamped PolygonStamped;
typedef ::geometry_msgs::Pose Pose;
typedef ::geometry_msgs::Pose2D Pose2D;
typedef ::geometry_msgs::PoseArray PoseArray;
typedef ::geometry_msgs::PoseStamped PoseStamped;
typedef ::geometry_msgs::PoseWithCovariance PoseWithCovariance;
typedef ::geometry_msgs::PoseWithCovarianceStamped PoseWithCovarianceStamped;
typedef ::geometry_msgs::Quaternion Quaternion;
typedef ::geometry_msgs::QuaternionStamped QuaternionStamped;
typedef ::geometry_msgs::Transform Transform;
typedef ::geometry_msgs::TransformStamped TransformStamped;
typedef ::geometry_msgs::Twist Twist;
typedef ::geometry_msgs::TwistStamped TwistStamped;
typedef ::geometry_msgs::TwistWithCovariance TwistWithCovariance;
typedef ::geometry_msgs::TwistWithCovarianceStamped TwistWithCovarianceStamped;
typedef ::geometry_msgs::Vector3 Vector3;
typedef ::geometry_msgs::Vector3Stamped Vector3Stamped;
typedef ::geometry_msgs::Wrench Wrench;
typedef ::geometry_msgs::WrenchStamped WrenchStamped;

#elif TKROS_VERSION == 2

typedef ::geometry_msgs::msg::Accel Accel;
typedef ::geometry_msgs::msg::AccelStamped AccelStamped;
typedef ::geometry_msgs::msg::AccelWithCovariance AccelWithCovariance;
typedef ::geometry_msgs::msg::AccelWithCovarianceStamped AccelWithCovarianceStamped;
typedef ::geometry_msgs::msg::Inertia Inertia;
typedef ::geometry_msgs::msg::InertiaStamped InertiaStamped;
typedef ::geometry_msgs::msg::Point Point;
typedef ::geometry_msgs::msg::Point32 Point32;
typedef ::geometry_msgs::msg::PointStamped PointStamped;
typedef ::geometry_msgs::msg::Polygon Polygon;
typedef ::geometry_msgs::msg::PolygonStamped PolygonStamped;
typedef ::geometry_msgs::msg::Pose Pose;
typedef ::geometry_msgs::msg::Pose2D Pose2D;
typedef ::geometry_msgs::msg::PoseArray PoseArray;
typedef ::geometry_msgs::msg::PoseStamped PoseStamped;
typedef ::geometry_msgs::msg::PoseWithCovariance PoseWithCovariance;
typedef ::geometry_msgs::msg::PoseWithCovarianceStamped PoseWithCovarianceStamped;
typedef ::geometry_msgs::msg::Quaternion Quaternion;
typedef ::geometry_msgs::msg::QuaternionStamped QuaternionStamped;
typedef ::geometry_msgs::msg::Transform Transform;
typedef ::geometry_msgs::msg::TransformStamped TransformStamped;
typedef ::geometry_msgs::msg::Twist Twist;
typedef ::geometry_msgs::msg::TwistStamped TwistStamped;
typedef ::geometry_msgs::msg::TwistWithCovariance TwistWithCovariance;
typedef ::geometry_msgs::msg::TwistWithCovarianceStamped TwistWithCovarianceStamped;
typedef ::geometry_msgs::msg::Vector3 Vector3;
typedef ::geometry_msgs::msg::Vector3Stamped Vector3Stamped;
typedef ::geometry_msgs::msg::Wrench Wrench;
typedef ::geometry_msgs::msg::WrenchStamped WrenchStamped;

#endif

} // namespace geometry_msgs
} // namespace roscomp
