#pragma once

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/geometries.hpp>
#include <boost/geometry/geometries/register/box.hpp>
#include <boost/geometry/geometries/register/linestring.hpp>
#include <boost/geometry/geometries/register/multi_linestring.hpp>
#include <boost/geometry/geometries/register/multi_point.hpp>
#include <boost/geometry/geometries/register/multi_polygon.hpp>
#include <boost/geometry/geometries/register/point.hpp>
#include <boost/geometry/geometries/register/ring.hpp>
#include <boost/geometry/geometries/register/segment.hpp>

#include <autoware_vector_map/data/data.h>

// clang-format off
BOOST_GEOMETRY_REGISTER_POINT_3D(autoware_vector_map::data::Point, double, cs::cartesian, x(), y(), z())
BOOST_GEOMETRY_REGISTER_LINESTRING(autoware_vector_map::data::LineString)
BOOST_GEOMETRY_REGISTER_RING(autoware_vector_map::data::LinearRing)
BOOST_GEOMETRY_REGISTER_MULTI_POLYGON(autoware_vector_map::data::Polygon)
// clang-format on
