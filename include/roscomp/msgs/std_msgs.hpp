
#pragma once

#include "roscomp/typesupport_id.hpp"

#if TKROS_VERSION == 1

#include <std_msgs/Bool.h>
#include <std_msgs/Byte.h>
#include <std_msgs/ByteMultiArray.h>
#include <std_msgs/Char.h>
#include <std_msgs/ColorRGBA.h>
#include <std_msgs/Duration.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Header.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Int64MultiArray.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int8MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/MultiArrayLayout.h>
#include <std_msgs/String.h>
#include <std_msgs/Time.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/UInt16MultiArray.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/UInt32MultiArray.h>
#include <std_msgs/UInt64.h>
#include <std_msgs/UInt64MultiArray.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/UInt8MultiArray.h>

#elif TKROS_VERSION == 2

#include <builtin_interfaces/msg/duration.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/byte.hpp>
#include <std_msgs/msg/byte_multi_array.hpp>
#include <std_msgs/msg/char.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/int64.hpp>
#include <std_msgs/msg/int64_multi_array.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/int8_multi_array.hpp>
#include <std_msgs/msg/multi_array_dimension.hpp>
#include <std_msgs/msg/multi_array_layout.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/u_int16.hpp>
#include <std_msgs/msg/u_int16_multi_array.hpp>
#include <std_msgs/msg/u_int32.hpp>
#include <std_msgs/msg/u_int32_multi_array.hpp>
#include <std_msgs/msg/u_int64.hpp>
#include <std_msgs/msg/u_int64_multi_array.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>

#endif

namespace roscomp {
namespace std_msgs {

#if TKROS_VERSION == 1

typedef ::std_msgs::Bool Bool;
typedef ::std_msgs::Byte Byte;
typedef ::std_msgs::ByteMultiArray ByteMultiArray;
typedef ::std_msgs::Char Char;
typedef ::std_msgs::ColorRGBA ColorRGBA;
typedef ::std_msgs::Duration Duration;
typedef ::std_msgs::Empty Empty;
typedef ::std_msgs::Float32 Float32;
typedef ::std_msgs::Float32MultiArray Float32MultiArray;
typedef ::std_msgs::Float64 Float64;
typedef ::std_msgs::Float64MultiArray Float64MultiArray;
typedef ::std_msgs::Header Header;
typedef ::std_msgs::Int16 Int16;
typedef ::std_msgs::Int16MultiArray Int16MultiArray;
typedef ::std_msgs::Int32 Int32;
typedef ::std_msgs::Int32MultiArray Int32MultiArray;
typedef ::std_msgs::Int64 Int64;
typedef ::std_msgs::Int64MultiArray Int64MultiArray;
typedef ::std_msgs::Int8 Int8;
typedef ::std_msgs::Int8MultiArray Int8MultiArray;
typedef ::std_msgs::MultiArrayDimension MultiArrayDimension;
typedef ::std_msgs::MultiArrayLayout MultiArrayLayout;
typedef ::std_msgs::String String;
typedef ::std_msgs::Time Time;
typedef ::std_msgs::UInt16 UInt16;
typedef ::std_msgs::UInt16MultiArray UInt16MultiArray;
typedef ::std_msgs::UInt32 UInt32;
typedef ::std_msgs::UInt32MultiArray UInt32MultiArray;
typedef ::std_msgs::UInt64 UInt64;
typedef ::std_msgs::UInt64MultiArray UInt64MultiArray;
typedef ::std_msgs::UInt8 UInt8;
typedef ::std_msgs::UInt8MultiArray UInt8MultiArray;

#elif TKROS_VERSION == 2

typedef ::std_msgs::msg::Bool Bool;
typedef ::std_msgs::msg::Byte Byte;
typedef ::std_msgs::msg::ByteMultiArray ByteMultiArray;
typedef ::std_msgs::msg::Char Char;
typedef ::std_msgs::msg::ColorRGBA ColorRGBA;
typedef ::builtin_interfaces::msg::Duration Duration;
typedef ::std_msgs::msg::Empty Empty;
typedef ::std_msgs::msg::Float32 Float32;
typedef ::std_msgs::msg::Float32MultiArray Float32MultiArray;
typedef ::std_msgs::msg::Float64 Float64;
typedef ::std_msgs::msg::Float64MultiArray Float64MultiArray;
typedef ::std_msgs::msg::Header Header;
typedef ::std_msgs::msg::Int16 Int16;
typedef ::std_msgs::msg::Int16MultiArray Int16MultiArray;
typedef ::std_msgs::msg::Int32 Int32;
typedef ::std_msgs::msg::Int32MultiArray Int32MultiArray;
typedef ::std_msgs::msg::Int64 Int64;
typedef ::std_msgs::msg::Int64MultiArray Int64MultiArray;
typedef ::std_msgs::msg::Int8 Int8;
typedef ::std_msgs::msg::Int8MultiArray Int8MultiArray;
typedef ::std_msgs::msg::MultiArrayDimension MultiArrayDimension;
typedef ::std_msgs::msg::MultiArrayLayout MultiArrayLayout;
typedef ::std_msgs::msg::String String;
typedef ::builtin_interfaces::msg::Time Time;
typedef ::std_msgs::msg::UInt16 UInt16;
typedef ::std_msgs::msg::UInt16MultiArray UInt16MultiArray;
typedef ::std_msgs::msg::UInt32 UInt32;
typedef ::std_msgs::msg::UInt32MultiArray UInt32MultiArray;
typedef ::std_msgs::msg::UInt64 UInt64;
typedef ::std_msgs::msg::UInt64MultiArray UInt64MultiArray;
typedef ::std_msgs::msg::UInt8 UInt8;
typedef ::std_msgs::msg::UInt8MultiArray UInt8MultiArray;

#endif

} // namespace std_msgs
} // namespace roscomp
