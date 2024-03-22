#pragma once

#include "roscomp/rosversion.hpp"
#include "roscomp/typesupport_id.hpp"

#if TKROS_VERSION == 1
#define LZ4_stream_t LZ4_stream_t_deprecated
#define LZ4_resetStream LZ4_resetStream_deprecated
#define LZ4_createStream LZ4_createStream_deprecated
#define LZ4_freeStream LZ4_freeStream_deprecated
#define LZ4_loadDict LZ4_loadDict_deprecated
#define LZ4_compress_fast_continue LZ4_compress_fast_continue_deprecated
#define LZ4_saveDict LZ4_saveDict_deprecated
#define LZ4_streamDecode_t LZ4_streamDecode_t_deprecated
#define LZ4_compress_continue LZ4_compress_continue_deprecated
#define LZ4_compress_limitedOutput_continue LZ4_compress_limitedOutput_continue_deprecated
#define LZ4_createStreamDecode LZ4_createStreamDecode_deprecated
#define LZ4_freeStreamDecode LZ4_freeStreamDecode_deprecated
#define LZ4_setStreamDecode LZ4_setStreamDecode_deprecated
#define LZ4_decompress_safe_continue LZ4_decompress_safe_continue_deprecated
#define LZ4_decompress_fast_continue LZ4_decompress_fast_continue_deprecated
#include <rosbag/bag.h>
#include <rosbag/view.h>
#undef LZ4_stream_t
#undef LZ4_resetStream
#undef LZ4_createStream
#undef LZ4_freeStream
#undef LZ4_loadDict
#undef LZ4_compress_fast_continue
#undef LZ4_saveDict
#undef LZ4_streamDecode_t
#undef LZ4_compress_continue
#undef LZ4_compress_limitedOutput_continue
#undef LZ4_createStreamDecode
#undef LZ4_freeStreamDecode
#undef LZ4_setStreamDecode
#undef LZ4_decompress_safe_continue
#undef LZ4_decompress_fast_continue

#elif TKROS_VERSION == 2

#if TKROS_DISTRO == TKROS_DISTRO_dashing
#include <regex>

#include <rosbag2/converter_interfaces/serialization_format_converter.hpp>
#include <rosbag2/sequential_reader.hpp>
#include <rosbag2/typesupport_helpers.hpp>

#elif TKROS_DISTRO >= TKROS_DISTRO_foxy
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_cpp/typesupport_helpers.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>
#include <rosbag2_storage/ros_helper.hpp>

#endif

#include "roscomp/time.hpp"
#endif

#ifdef TKROS_VERSION

namespace roscomp {
namespace bag {

#if TKROS_VERSION == 1
enum mode : uint32_t
{
    Write = rosbag::bagmode::Write,
    Read = rosbag::bagmode::Read,
    Append = rosbag::bagmode::Append
};

typedef rosbag::Bag Bag;
typedef rosbag::MessageInstance MessageInstance;
typedef rosbag::View View;
typedef rosbag::TopicQuery TopicQuery;

#elif TKROS_VERSION == 2
enum class mode
{
    Write = 1,
    Read = 2,
    Append = 4
};

class TopicQuery
{
  public:
    TopicQuery(std::vector<std::string> const& filter);
#if TKROS_DISTRO == TKROS_DISTRO_dashing
    std::vector<std::string> topic_filter_;
#elif TKROS_DISTRO >= TKROS_DISTRO_foxy
    rosbag2_storage::StorageFilter topic_filter_;
#endif
};

class Bag
{
  public:
    Bag();

    void open(std::string, mode);
    void close();

    bool has_next() const;
    void read_next();
    std::string const& topic_name() const;
    roscomp::time::Time const& timestamp();

    void set_filter(const TopicQuery&);

    template<typename MessageT>
    std::shared_ptr<MessageT> instantiate()
    {
#if TKROS_DISTRO == TKROS_DISTRO_dashing
        const std::string message_type = std::regex_replace(
          std::string(rosidl_generator_traits::data_type<MessageT>()), std::regex(":{2,2}"), "/");

        auto type_support = rosbag2::get_typesupport(message_type, typesupport_id_);
#elif TKROS_DISTRO >= TKROS_DISTRO_foxy
        const std::string message_type = rosidl_generator_traits::name<MessageT>();
        auto typesupport_library =
          rosbag2_cpp::get_typesupport_library(message_type, typesupport_id_);
        auto type_support =
          rosbag2_cpp::get_typesupport_handle(message_type, typesupport_id_, typesupport_library);
#endif

        ros_generic_message_->time_stamp = serialized_message_->time_stamp;

        MessageT message;
        ros_generic_message_->message = &message;

        cdr_deserializer_->deserialize(serialized_message_, type_support, ros_generic_message_);

        return std::make_shared<MessageT>(message);
    }

    template<typename MessageT>
    void write(std::string const& topic, ::builtin_interfaces::msg::Time const& time, MessageT msg)
    {
        std::string message_type = rosidl_generator_traits::name<MessageT>();
        std::unordered_map<std::string, std::vector<std::string>>::iterator topic_exists =
          topics_with_types_.find(topic);

        if (topic_exists == topics_with_types_.end()) {
            topics_with_types_.insert(
              std::pair<std::string, std::vector<std::string>>(topic, { message_type }));
            writer_->create_topic({ topic, message_type, rmw_get_serialization_format(), "" });

        } else if (std::find(topic_exists->second.begin(),
                             topic_exists->second.end(),
                             message_type) == topic_exists->second.end()) {

            topic_exists->second.push_back(message_type);
            writer_->create_topic({ topic, message_type, rmw_get_serialization_format(), "" });
        }

#if TKROS_DISTRO >= TKROS_DISTRO_foxy
        auto typesupport_library =
          rosbag2_cpp::get_typesupport_library(message_type, typesupport_id_);
        auto type_support =
          rosbag2_cpp::get_typesupport_handle(message_type, typesupport_id_, typesupport_library);

        ros_generic_message_->message = &msg;

        serialized_message_ = std::make_shared<rosbag2_storage::SerializedBagMessage>();
        serialized_message_->serialized_data = rosbag2_storage::make_empty_serialized_message(0);
        serialized_message_->time_stamp = roscomp::time::to_nanosec(roscomp::time::Time(time));
        serialized_message_->topic_name = const_cast<char*>(topic.c_str());
        cdr_serializer_->serialize(ros_generic_message_, type_support, serialized_message_);

        writer_->write(serialized_message_);
#endif
    }

    template<typename MessageT>
    void write(std::string const& topic,
               ::builtin_interfaces::msg::Time const& time,
               std::shared_ptr<MessageT> msg)
    {
        write(topic, time, *msg);
    }

  private:
    const std::string typesupport_id_ = "rosidl_typesupport_cpp";
    roscomp::time::Time time_tmp_;

    // general bag stuff
    std::shared_ptr<rosbag2_storage::SerializedBagMessage> serialized_message_;

#if TKROS_DISTRO == TKROS_DISTRO_dashing
    rosbag2::StorageOptions storage_options_;
#elif TKROS_DISTRO == TKROS_DISTRO_foxy
    rosbag2_cpp::StorageOptions storage_options_;
#elif TKROS_DISTRO == TKROS_DISTRO_galactic
    rosbag2_storage::StorageOptions storage_options_;
#endif

#if TKROS_DISTRO == TKROS_DISTRO_dashing
    rosbag2::ConverterOptions converter_options_;
    rosbag2::SerializationFormatConverterFactory factory_;
    std::shared_ptr<rosbag2_introspection_message_t> ros_generic_message_;
#elif TKROS_DISTRO >= TKROS_DISTRO_foxy
    rosbag2_cpp::ConverterOptions converter_options_;
    rosbag2_cpp::SerializationFormatConverterFactory factory_;
    std::shared_ptr<rosbag2_cpp::rosbag2_introspection_message_t> ros_generic_message_;
#endif

    // reader
#if TKROS_DISTRO == TKROS_DISTRO_dashing
    std::unique_ptr<rosbag2::SequentialReader> reader_;
    std::unique_ptr<rosbag2::converter_interfaces::SerializationFormatDeserializer>
      cdr_deserializer_;
    std::vector<std::string> topic_filter_;

#elif TKROS_DISTRO >= TKROS_DISTRO_foxy
    std::unique_ptr<rosbag2_cpp::readers::SequentialReader> reader_;
    std::unique_ptr<rosbag2_cpp::converter_interfaces::SerializationFormatDeserializer>
      cdr_deserializer_;
    rosbag2_storage::StorageFilter storage_filter_;

#endif

    // writer
#if TKROS_DISTRO >= TKROS_DISTRO_foxy
    std::unique_ptr<rosbag2_cpp::writers::SequentialWriter> writer_;
    std::unique_ptr<rosbag2_cpp::converter_interfaces::SerializationFormatSerializer>
      cdr_serializer_;
#endif
    std::unordered_map<std::string, std::vector<std::string>> topics_with_types_;
};

class MessageInstance
{
  public:
    MessageInstance(Bag& bag_);

    std::string const& getTopic() const;
    roscomp::time::Time const& getTime() const;

    template<typename T>
    std::shared_ptr<T> instantiate() const
    {
        return bag_.instantiate<T>();
    }

  private:
    Bag& bag_;
};

class View //: public std::iterator<std::input_iterator_tag, Bag>
{
  public:
    View(Bag& bag);
    View(Bag& bag, const TopicQuery& topic_filter);
    View(const View& v);
    ~View();
    View begin();
    View end();
    View& operator++();
    bool operator==(const View& rhs) const;
    bool operator!=(const View& rhs) const;
    MessageInstance& operator*();
    MessageInstance const& operator*() const;
    MessageInstance* operator->();
    MessageInstance const* operator->() const;

  private:
    MessageInstance msg_instance_;
    Bag& bag_;
};

#endif

} // namespace bag
} // namespace roscomp

#endif