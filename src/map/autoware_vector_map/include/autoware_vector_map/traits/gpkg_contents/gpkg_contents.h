#pragma once

#include <autoware_vector_map/data/data.h>

#define AUTOWARE_VECTOR_MAP_REGISTER_GPKG_CONTENT(CONTENT, TABLE_NAME) \
  template <>                                                          \
  constexpr const char* gpkg_content<data::CONTENT>::class_name() {    \
    return #CONTENT;                                                   \
  }                                                                    \
                                                                       \
  template <>                                                          \
  constexpr const char* gpkg_content<data::CONTENT>::table_name() {    \
    return #TABLE_NAME;                                                \
  }

#define AUTOWARE_VECTOR_MAP_REGISTER_GPKG_MEMBER(FEATURE, INDEX, VARIABLE, IF_NULL) \
  template <>                                                                       \
  template <>                                                                       \
  struct gpkg_content<data::FEATURE>::member_def<INDEX> {                           \
    using type = decltype(data::FEATURE::VARIABLE);                                 \
    static constexpr const char* name = #VARIABLE;                                  \
    static constexpr auto reference = &data::FEATURE::VARIABLE;                     \
    static type if_null() { return IF_NULL; }                                       \
  };

namespace autoware_vector_map {
namespace traits {

template <class T>
struct gpkg_content {
  static constexpr const char* class_name();
  static constexpr const char* table_name();

  // The order of member_def<N> should be the same as the table's one
  template <size_t N>
  struct member_def;
};

}  // namespace traits
}  // namespace autoware_vector_map

#include "vector_map/attributes.h"
#include "vector_map/features.h"
#include "vector_map/relationships.h"
