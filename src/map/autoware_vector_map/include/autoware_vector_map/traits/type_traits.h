#pragma once

#include <autoware_vector_map/future/void_t.h>

namespace autoware_vector_map {
namespace traits {

template <class T, class = std::void_t<>>
struct has_geometry : std::false_type {};

template <class T>
struct has_geometry<T, std::void_t<decltype(std::declval<T>().geometry)>> : std::true_type {};

template <class T, size_t N>
using member = typename gpkg_content<T>::template member_def<N>;

template <class T, size_t N, class = std::void_t<>>
struct has_member_n : std::false_type {};

template <class T, size_t N>
struct has_member_n<T, N, std::void_t<typename member<T, N>::type>> : std::true_type {};

}  // namespace traits
}  // namespace autoware_vector_map
