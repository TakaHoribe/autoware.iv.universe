#pragma once

#include <memory>

namespace autoware_vector_map {
namespace data {

template <class T>
using Ptr = std::shared_ptr<T>;

template <class T>
using ConstPtr = std::shared_ptr<const T>;

using Id = int64_t;
constexpr Id invalid_id = 0;

}  // namespace data
}  // namespace autoware_vector_map

#include "geometry.h"

#include "vector_map/attributes.h"
#include "vector_map/features.h"
#include "vector_map/relationships.h"
