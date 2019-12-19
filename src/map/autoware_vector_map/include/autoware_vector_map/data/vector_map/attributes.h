#pragma once

#include <string>

#include <autoware_vector_map/data/data.h>

namespace autoware_vector_map {
namespace data {

template <class T_Feature>
struct Attribute {
  using FeatureType = T_Feature;

  Id id;
};

}  // namespace data
}  // namespace autoware_vector_map
