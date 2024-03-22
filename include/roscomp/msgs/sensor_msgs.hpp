#pragma once

#include "roscomp/typesupport_id.hpp"

#if TKROS_VERSION == 1

#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/ChannelFloat32.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/FluidPressure.h>
#include <sensor_msgs/Illuminance.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/JoyFeedback.h>
#include <sensor_msgs/JoyFeedbackArray.h>
#include <sensor_msgs/LaserEcho.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/MultiDOFJointState.h>
#include <sensor_msgs/MultiEchoLaserScan.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
#include <sensor_msgs/Range.h>
#include <sensor_msgs/RegionOfInterest.h>
#include <sensor_msgs/RelativeHumidity.h>
#include <sensor_msgs/Temperature.h>
#include <sensor_msgs/TimeReference.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#elif TKROS_VERSION == 2

#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/channel_float32.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>
#include <sensor_msgs/msg/illuminance.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/joy_feedback.hpp>
#include <sensor_msgs/msg/joy_feedback_array.hpp>
#include <sensor_msgs/msg/laser_echo.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <sensor_msgs/msg/multi_dof_joint_state.hpp>
#include <sensor_msgs/msg/multi_echo_laser_scan.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/nav_sat_status.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <sensor_msgs/msg/region_of_interest.hpp>
#include <sensor_msgs/msg/relative_humidity.hpp>
#include <sensor_msgs/msg/temperature.hpp>
#include <sensor_msgs/msg/time_reference.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#endif

namespace roscomp {
namespace sensor_msgs {

#if TKROS_VERSION == 1

typedef ::sensor_msgs::BatteryState BatteryState;
typedef ::sensor_msgs::CameraInfo CameraInfo;
typedef ::sensor_msgs::ChannelFloat32 ChannelFloat32;
typedef ::sensor_msgs::CompressedImage CompressedImage;
typedef ::sensor_msgs::FluidPressure FluidPressure;
typedef ::sensor_msgs::Illuminance Illuminance;
typedef ::sensor_msgs::Image Image;
typedef ::sensor_msgs::Imu Imu;
typedef ::sensor_msgs::JointState JointState;
typedef ::sensor_msgs::Joy Joy;
typedef ::sensor_msgs::JoyFeedback JoyFeedback;
typedef ::sensor_msgs::JoyFeedbackArray JoyFeedbackArray;
typedef ::sensor_msgs::LaserEcho LaserEcho;
typedef ::sensor_msgs::LaserScan LaserScan;
typedef ::sensor_msgs::MagneticField MagneticField;
typedef ::sensor_msgs::MultiDOFJointState MultiDOFJointState;
typedef ::sensor_msgs::MultiEchoLaserScan MultiEchoLaserScan;
typedef ::sensor_msgs::NavSatFix NavSatFix;
typedef ::sensor_msgs::NavSatStatus NavSatStatus;
typedef ::sensor_msgs::PointCloud PointCloud;
typedef ::sensor_msgs::PointCloud2 PointCloud2;
typedef ::sensor_msgs::PointField PointField;
typedef ::sensor_msgs::Range Range;
typedef ::sensor_msgs::RegionOfInterest RegionOfInterest;
typedef ::sensor_msgs::RelativeHumidity RelativeHumidity;
typedef ::sensor_msgs::Temperature Temperature;
typedef ::sensor_msgs::TimeReference TimeReference;

#elif TKROS_VERSION == 2

typedef ::sensor_msgs::msg::BatteryState BatteryState;
typedef ::sensor_msgs::msg::CameraInfo CameraInfo;
typedef ::sensor_msgs::msg::ChannelFloat32 ChannelFloat32;
typedef ::sensor_msgs::msg::CompressedImage CompressedImage;
typedef ::sensor_msgs::msg::FluidPressure FluidPressure;
typedef ::sensor_msgs::msg::Illuminance Illuminance;
typedef ::sensor_msgs::msg::Image Image;
typedef ::sensor_msgs::msg::Imu Imu;
typedef ::sensor_msgs::msg::JointState JointState;
typedef ::sensor_msgs::msg::Joy Joy;
typedef ::sensor_msgs::msg::JoyFeedback JoyFeedback;
typedef ::sensor_msgs::msg::JoyFeedbackArray JoyFeedbackArray;
typedef ::sensor_msgs::msg::LaserEcho LaserEcho;
typedef ::sensor_msgs::msg::LaserScan LaserScan;
typedef ::sensor_msgs::msg::MagneticField MagneticField;
typedef ::sensor_msgs::msg::MultiDOFJointState MultiDOFJointState;
typedef ::sensor_msgs::msg::MultiEchoLaserScan MultiEchoLaserScan;
typedef ::sensor_msgs::msg::NavSatFix NavSatFix;
typedef ::sensor_msgs::msg::NavSatStatus NavSatStatus;
typedef ::sensor_msgs::msg::PointCloud PointCloud;
typedef ::sensor_msgs::msg::PointCloud2 PointCloud2;
typedef ::sensor_msgs::msg::PointField PointField;
typedef ::sensor_msgs::msg::Range Range;
typedef ::sensor_msgs::msg::RegionOfInterest RegionOfInterest;
typedef ::sensor_msgs::msg::RelativeHumidity RelativeHumidity;
typedef ::sensor_msgs::msg::Temperature Temperature;
typedef ::sensor_msgs::msg::TimeReference TimeReference;

#endif

} // namespace sensor_msgs
} // namespace roscomp
