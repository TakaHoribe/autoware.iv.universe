#pragma once

#include <memory>
#include <vector>

#include <autoware_vector_map/core.h>
#include <autoware_vector_map/traits/gpkg_contents/gpkg_contents.h>

class OGRLayer;
class GDALDataset;

namespace autoware_vector_map {
namespace io {
namespace gpkg_loader {

using autoware_vector_map::traits::gpkg_attribute;
using autoware_vector_map::traits::gpkg_content;
using autoware_vector_map::traits::gpkg_feature;
using autoware_vector_map::traits::gpkg_relationship;
using autoware_vector_map::traits::RelationSide;

class GpkgLoader {
 public:
  explicit GpkgLoader(const char* gpkg_path);
  explicit GpkgLoader(const std::vector<uint8_t>& bin_data);

  void toFile(const char* gpkg_path);
  std::vector<uint8_t> toBinary();

  template <class T>
  ConstPtr<T> getFeatureById(const Id id);

  template <class T>
  ConstPtr<std::vector<T>> getFeaturesByIds(const std::vector<Id>& ids);

  template <class T, RelationSide S,
            class U = typename gpkg_relationship<T>::template RelatedFeature<S>::type,
            class P = std::function<bool(const T&)>>
  ConstPtr<std::vector<U>> getRelatedFeaturesById(const Id id, const P& predicate = nullptr);

  template <class T>
  ConstPtr<std::vector<T>> getAllFeatures();

  template <class T>
  ConstPtr<std::vector<T>> findFeaturesByRange(const Point3d& p, const double range);

 private:
  std::unique_ptr<GDALDataset> dataset_;

  template <class T>
  ConstPtr<std::vector<T>> getFeaturesBySql(const char* sql);

  template <class T>
  ConstPtr<std::vector<T>> getFeaturesByLayer(OGRLayer* layer);
};

}  // namespace gpkg_loader
}  // namespace io
}  // namespace autoware_vector_map

#include "gpkg_loader_impl.h"
