#pragma once

#include "roscomp/typesupport_id.hpp"

#if TKROS_VERSION == 1
#include <visualization_msgs/ImageMarker.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerControl.h>
#include <visualization_msgs/InteractiveMarkerFeedback.h>
#include <visualization_msgs/InteractiveMarkerInit.h>
#include <visualization_msgs/InteractiveMarkerPose.h>
#include <visualization_msgs/InteractiveMarkerUpdate.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/MenuEntry.h>

#elif TKROS_VERSION == 2
#include <visualization_msgs/msg/image_marker.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
#include <visualization_msgs/msg/interactive_marker_init.hpp>
#include <visualization_msgs/msg/interactive_marker_pose.hpp>
#include <visualization_msgs/msg/interactive_marker_update.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/menu_entry.hpp>

#endif

namespace roscomp {
namespace visualization_msgs {

#if TKROS_VERSION == 1

typedef ::visualization_msgs::ImageMarker ImageMarker;
typedef ::visualization_msgs::InteractiveMarker InteractiveMarker;
typedef ::visualization_msgs::InteractiveMarkerControl InteractiveMarkerControl;
typedef ::visualization_msgs::InteractiveMarkerFeedback InteractiveMarkerFeedback;
typedef ::visualization_msgs::InteractiveMarkerInit InteractiveMarkerInit;
typedef ::visualization_msgs::InteractiveMarkerPose InteractiveMarkerPose;
typedef ::visualization_msgs::InteractiveMarkerUpdate InteractiveMarkerUpdate;
typedef ::visualization_msgs::Marker Marker;
typedef ::visualization_msgs::MarkerArray MarkerArray;
typedef ::visualization_msgs::MenuEntry MenuEntry;

#elif TKROS_VERSION == 2

typedef ::visualization_msgs::msg::ImageMarker ImageMarker;
typedef ::visualization_msgs::msg::InteractiveMarker InteractiveMarker;
typedef ::visualization_msgs::msg::InteractiveMarkerControl InteractiveMarkerControl;
typedef ::visualization_msgs::msg::InteractiveMarkerFeedback InteractiveMarkerFeedback;
typedef ::visualization_msgs::msg::InteractiveMarkerInit InteractiveMarkerInit;
typedef ::visualization_msgs::msg::InteractiveMarkerPose InteractiveMarkerPose;
typedef ::visualization_msgs::msg::InteractiveMarkerUpdate InteractiveMarkerUpdate;
typedef ::visualization_msgs::msg::Marker Marker;
typedef ::visualization_msgs::msg::MarkerArray MarkerArray;
typedef ::visualization_msgs::msg::MenuEntry MenuEntry;

#endif

} // namespace visualization_msgs
} // namespace roscomp