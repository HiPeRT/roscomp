#include <iostream>

#include <rmw/rmw.h>

#include "roscomp/bag.hpp"
#include "roscomp/time.hpp"

namespace roscomp {
namespace bag {

TopicQuery::TopicQuery(std::vector<std::string> const& filter)
{
#if TKROS_DISTRO == TKROS_DISTRO_dashing
    topic_filter_ = filter;
#elif TKROS_DISTRO >= TKROS_DISTRO_foxy
    topic_filter_.topics = filter;
#endif
}

// Bag class
Bag::Bag()
  : serialized_message_(nullptr)
  , reader_(nullptr)
  , writer_(nullptr)
{
    cdr_deserializer_ = factory_.load_deserializer(rmw_get_serialization_format());
    cdr_serializer_ = factory_.load_serializer(rmw_get_serialization_format());

#if TKROS_DISTRO == TKROS_DISTRO_dashing
    ros_generic_message_ = std::make_shared<rosbag2_introspection_message_t>();
#elif TKROS_DISTRO >= TKROS_DISTRO_foxy
    ros_generic_message_ = std::make_shared<rosbag2_cpp::rosbag2_introspection_message_t>();
#endif
    ros_generic_message_->message = nullptr;
    ros_generic_message_->allocator = rcutils_get_default_allocator();
}

void Bag::open(std::string bag_file, mode bag_mode)
{
    storage_options_ = { bag_file, "sqlite3" };
    converter_options_ = { rmw_get_serialization_format(), rmw_get_serialization_format() };

    if (bag_mode == mode::Write) {
#if TKROS_DISTRO == TKROS_DISTRO_dashing
        std::cerr << "unimplemented\n";
        exit(-1);
#elif TKROS_DISTRO >= TKROS_DISTRO_foxy
        writer_ = std::unique_ptr<rosbag2_cpp::writers::SequentialWriter>(
          new rosbag2_cpp::writers::SequentialWriter{});
#endif
        writer_->open(storage_options_, converter_options_);

    } else if (bag_mode == mode::Read) {
#if TKROS_DISTRO == TKROS_DISTRO_dashing
        reader_ = std::unique_ptr<rosbag2::SequentialReader>(new rosbag2::SequentialReader{});
#elif TKROS_DISTRO >= TKROS_DISTRO_foxy
        reader_ = std::unique_ptr<rosbag2_cpp::readers::SequentialReader>(
          new rosbag2_cpp::readers::SequentialReader{});
#endif
        reader_->open(storage_options_, converter_options_);
    } else {
        std::cerr << "unimplemented\n";
        exit(-1);
    }
}

void Bag::close()
{
    reader_.reset();
    writer_.reset();
}

bool Bag::has_next() const
{
    return reader_->has_next();
}

void Bag::read_next()
{
    serialized_message_ = reader_->read_next();
#if TKROS_DISTRO == TKROS_DISTRO_dashing
    while (std::find(topic_filter_.begin(), topic_filter_.end(), serialized_message_->topic_name) ==
           topic_filter_.end())
        serialized_message_ = reader_->read_next();
#endif
}

std::string const& Bag::topic_name() const
{
    return serialized_message_->topic_name;
}

void Bag::set_filter(const TopicQuery& storage_filter)
{
#if TKROS_DISTRO == TKROS_DISTRO_dashing
    topic_filter_ = storage_filter.topic_filter_;
#elif TKROS_DISTRO >= TKROS_DISTRO_foxy
    reader_->set_filter(storage_filter.topic_filter_);
#endif
}

roscomp::time::Time const& Bag::timestamp()
{
    time_tmp_ = roscomp::time::Time(serialized_message_->time_stamp);
    return time_tmp_;
}

MessageInstance::MessageInstance(Bag& bag)
  : bag_(bag)
{}

std::string const& MessageInstance::getTopic() const
{
    return bag_.topic_name();
}

roscomp::time::Time const& MessageInstance::getTime() const
{
    return bag_.timestamp();
}

MessageInstance& View::operator*()
{
    return msg_instance_;
}

MessageInstance const& View::operator*() const
{
    return msg_instance_;
}

MessageInstance* View::operator->()
{
    return &msg_instance_;
}

MessageInstance const* View::operator->() const
{
    return &msg_instance_;
}

// View iterator
View::View(Bag& bag)
  : msg_instance_(bag)
  , bag_(bag)
{
    bag_.read_next();
}

View::View(Bag& bag, const TopicQuery& topic_filter)
  : msg_instance_(bag)
  , bag_(bag)
{
    bag_.set_filter(topic_filter);
    bag_.read_next();
}

View::View(const View& v)
  : msg_instance_(v.msg_instance_)
  , bag_(v.bag_)
{}

View::~View() {}

View View::begin()
{
    return *this;
}

View View::end()
{
    return *this;
}

View& View::operator++()
{
    bag_.read_next();
    return *this;
}

bool View::operator==(const View& rhs) const
{
    (void)rhs;
    return bag_.has_next();
}

bool View::operator!=(const View& rhs) const
{
    (void)rhs;
    return bag_.has_next();
}

} // namespace bag
} // namespace roscomp
