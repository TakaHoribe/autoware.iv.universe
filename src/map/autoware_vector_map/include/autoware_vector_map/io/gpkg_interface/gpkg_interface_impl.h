#pragma once

#include <autoware_vector_map/io/gpkg_interface.h>

#include <memory>
#include <string>
#include <vector>

#include <fmt/format.h>

#include <ogrsf_frmts.h>

#include <autoware_vector_map/bridge/ogr/feature_conversion.h>
#include <autoware_vector_map/traits/gpkg_contents.h>

namespace autoware_vector_map {
namespace io {

using fmt::literals::operator""_a;

template <class T>
ConstPtr<T> GpkgInterface::getFeatureById(const Id id) {
  // Use raw pointer because reusing the same resource
  OGRLayer* table_layer = dataset_->GetLayerByName(traits::gpkg_content<T>::table_name());
  if (!table_layer) {
    return nullptr;
  }

  const auto ogr_feature =
      std::shared_ptr<OGRFeature>(table_layer->GetFeature(static_cast<GIntBig>(id)));
  if (!ogr_feature) {
    return nullptr;
  }

  return std::make_shared<T>(bridge::fromOgrFeature<T>(ogr_feature.get()));
}

template <class T>
std::vector<ConstPtr<T>> GpkgInterface::getFeaturesByIds(const std::vector<Id>& ids) {
  std::stringstream ss_ids;
  for (auto itr = std::begin(ids); itr != std::end(ids); ++itr) {
    if (itr != std::begin(ids)) {
      ss_ids << ", ";
    }

    ss_ids << *itr;
  }

  OGRLayer* table_layer = dataset_->GetLayerByName(traits::gpkg_content<T>::table_name());
  const auto sql =
      fmt::format("SELECT {} FROM {} WHERE {} in ({})", bridge::createFeatureQuery<T>(table_layer),
                  traits::gpkg_content<T>::table_name(), table_layer->GetFIDColumn(), ss_ids.str());

  return getFeaturesBySql<T>(sql.c_str());
}

template <class T>
std::vector<ConstPtr<T>> GpkgInterface::getAllFeatures() {
  OGRLayer* table_layer = dataset_->GetLayerByName(traits::gpkg_content<T>::table_name());
  const auto sql = fmt::format("SELECT {} FROM {}", bridge::createFeatureQuery<T>(table_layer),
                               traits::gpkg_content<T>::table_name());
  return getFeaturesBySql<T>(sql.c_str());
}

template <class T, RelationSide S, class U>
std::vector<ConstPtr<U>> GpkgInterface::getRelatedFeaturesById(
    const Id id, const std::function<bool(const T&)> predicate) {
  OGRLayer* relationship_table_layer =
      dataset_->GetLayerByName(traits::gpkg_content<T>::table_name());

  const auto relationship_sql = fmt::format(
      "SELECT {} FROM {} WHERE {} = {}", bridge::createFeatureQuery<T>(relationship_table_layer),
      traits::gpkg_content<T>::table_name(),
      traits::gpkg_relationship<T>::template this_feature_id_name<S>(), id);

  std::vector<ConstPtr<T>> relationship_features = getFeaturesBySql<T>(relationship_sql.c_str());

  std::vector<Id> feature_ids;
  for (const auto relationship_feature : relationship_features) {
    if (predicate && !predicate(*relationship_feature)) {
      continue;
    }

    feature_ids.push_back(
        traits::gpkg_relationship<T>::template related_feature_id<S>(*relationship_feature));
  }

  return getFeaturesByIds<U>(feature_ids);
}

template <class T>
std::vector<ConstPtr<T>> GpkgInterface::findFeaturesByRange(const Point3d& p, const double range) {
  OGRLayer* table_layer = dataset_->GetLayerByName(traits::gpkg_content<T>::table_name());

  const auto condition = fmt::format(
      "Intersects(Buffer(GeomFromText('POINTZ({x} {y} {z})'), {range}), "
      "CastAutomagic({geometry_name})) = 1",
      "x"_a = p.x(), "y"_a = p.y(), "z"_a = p.z(), "range"_a = range,
      "geometry_name"_a = table_layer->GetGeometryColumn());

  const auto sql =
      fmt::format("SELECT {} FROM {} WHERE {}", bridge::createFeatureQuery<T>(table_layer),
                  traits::gpkg_content<T>::table_name(), condition);

  return getFeaturesBySql<T>(sql.c_str());
}

template <class T>
std::vector<ConstPtr<T>> GpkgInterface::getFeaturesBySql(const char* sql) {
  const auto result_layer = std::shared_ptr<OGRLayer>(dataset_->ExecuteSQL(sql, nullptr, "sql"));
  return getFeaturesByLayer<T>(result_layer.get());
}

template <class T>
std::vector<ConstPtr<T>> GpkgInterface::getFeaturesByLayer(OGRLayer* layer) {
  std::vector<ConstPtr<T>> features;
  features.reserve(layer->GetFeatureCount());

  while (auto ogr_feature = layer->GetNextFeature()) {
    features.push_back(getFeatureById<T>(static_cast<Id>(ogr_feature->GetFID())));
  }

  return features;
}

}  // namespace io
}  // namespace autoware_vector_map
