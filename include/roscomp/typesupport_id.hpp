#pragma once

#include <memory>

namespace roscomp {

#if TKROS_VERSION == 1
template<typename MessageT>
using CallbackPtr = const typename MessageT::ConstPtr&;
template<typename MessageT>
using ZeroCopyPtr = typename MessageT::Ptr;

template<typename ServiceT>
using ConstRequestPtr = const std::shared_ptr<typename ServiceT::Request>;
template<typename ServiceT>
using RequestPtr = std::shared_ptr<typename ServiceT::Request>;
template<typename ServiceT>
using ResponsePtr = std::shared_ptr<typename ServiceT::Response>;

#elif TKROS_VERSION == 2
template<typename MessageT>
using CallbackPtr = typename MessageT::SharedPtr;
template<typename MessageT>
using ZeroCopyPtr = typename MessageT::UniquePtr;

template<typename ServiceT>
using ConstRequestPtr = const typename ServiceT::Request::SharedPtr;
template<typename ServiceT>
using RequestPtr = typename ServiceT::Request::SharedPtr;
template<typename ServiceT>
using ResponsePtr = typename ServiceT::Response::SharedPtr;

#endif

} // namespace roscomp
