
#pragma once

#include "roscomp/typesupport_id.hpp"

#if TKROS_VERSION == 1

#include <nav_msgs/GridCells.h>
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

#elif TKROS_VERSION == 2

#include <nav_msgs/msg/grid_cells.hpp>
#include <nav_msgs/msg/map_meta_data.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

#endif

namespace roscomp {
namespace nav_msgs {

#if TKROS_VERSION == 1

typedef ::nav_msgs::GridCells GridCells;
typedef ::nav_msgs::MapMetaData MapMetaData;
typedef ::nav_msgs::OccupancyGrid OccupancyGrid;
typedef ::nav_msgs::Odometry Odometry;
typedef ::nav_msgs::Path Path;

#elif TKROS_VERSION == 2

typedef ::nav_msgs::msg::GridCells GridCells;
typedef ::nav_msgs::msg::MapMetaData MapMetaData;
typedef ::nav_msgs::msg::OccupancyGrid OccupancyGrid;
typedef ::nav_msgs::msg::Odometry Odometry;
typedef ::nav_msgs::msg::Path Path;

#endif

} // namespace nav_msgs
} // namespace roscomp
