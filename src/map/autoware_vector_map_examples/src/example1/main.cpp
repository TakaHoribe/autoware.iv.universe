#include <ros/ros.h>

#include <autoware_vector_map/autoware_vector_map.h>
#include <autoware_vector_map/bridge/boost/geometry_registration.h>
#include <autoware_vector_map/bridge/ogr/geometry_converter.h>
#include <autoware_vector_map/util/to_debug_string.h>

#include <common.h>

using namespace autoware_vector_map::io::gpkg_loader;
using namespace autoware_vector_map::data;
using namespace autoware_vector_map::traits;
using namespace autoware_vector_map::util;
using namespace autoware_vector_map::bridge::ogr;

void example(const std::unique_ptr<GpkgLoader>& gpkg_loader) {
  // getFeatureById
  {
    using Feature = Lane;

    std::cout << "###### " << gpkg_content<Feature>::class_name() << " ######" << std::endl;

    for (Id id : {-1, 0, 1, 3, 1000}) {
      const auto feature = gpkg_loader->getFeatureById<Feature>(id);
      if (feature) {
        std::cout << toDebugString(*feature) << std::endl;
      }
    }
  }

  // getFeaturesByIds
  {
    using Feature = Lane;
    const std::vector<Id> ids{-1, 0, 2, 4, 1000};

    std::cout << "###### " << gpkg_content<Feature>::class_name() << " ######" << std::endl;

    const auto features = gpkg_loader->getFeaturesByIds<Feature>(ids);
    for (const auto& feature : *features) {
      std::cout << toDebugString(feature) << std::endl;
    }
  }

  // getAllFeatures
  {
    using Feature = LaneSection;

    std::cout << "###### " << gpkg_content<Feature>::class_name() << " ######" << std::endl;

    const auto features = gpkg_loader->getAllFeatures<Feature>();
    for (const auto& feature : *features) {
      std::cout << toDebugString(feature) << std::endl;
    }
  }

  // findFeaturesByRange
  {
    using Feature = Lane;
    const Point p(80721.15, 7393.34, 0);
    const double range = 5.0;

    std::cout << "###### " << gpkg_content<Feature>::class_name() << " ######" << std::endl;

    const auto features = gpkg_loader->findFeaturesByRange<Feature>(p, range);
    for (const auto& feature : *features) {
      std::cout << toDebugString(feature) << std::endl;
    }
  }

  // getRelatedFeaturesById
  {
    using Relationship = LaneConnection;
    const auto lane = *gpkg_loader->getFeatureById<Lane>(1);

    std::cout << "###### " << gpkg_content<Relationship>::class_name() << " ######" << std::endl;

    const auto features =
        gpkg_loader->getRelatedFeaturesById<Relationship, RelationSide::Left>(lane.id);
    for (const auto& feature : *features) {
      std::cout << toDebugString(feature) << std::endl;
    }
  }

  // getRelatedFeaturesById(with predicate)
  {
    using Relationship = AdjacentLane;
    const auto lane = *gpkg_loader->getFeatureById<Lane>(30);
    const auto predicate = [](const Relationship& r) { return r.type == "left"; };

    std::cout << "###### " << gpkg_content<Relationship>::class_name() << " ######" << std::endl;

    const auto features =
        gpkg_loader->getRelatedFeaturesById<Relationship, RelationSide::Left>(lane.id, predicate);
    for (const auto& feature : *features) {
      std::cout << toDebugString(feature) << std::endl;
    }
  }

  // boost::geometry
  {
    std::cout << "###### boost::geometry ######" << std::endl;

    const auto lane_1 = *gpkg_loader->getFeatureById<Lane>(1);
    const auto lane_2 = *gpkg_loader->getFeatureById<Lane>(2);

    std::cout << boost::geometry::distance(lane_1.geometry.at(0), lane_2.geometry) << std::endl
              << std::endl;
  }
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "example1");

  ros::NodeHandle nh_("");
  ros::NodeHandle private_nh_("~");

  std::string vector_map_path;
  private_nh_.getParam("vector_map_path", vector_map_path);

  const auto gpkg_loader = std::make_unique<GpkgLoader>(vector_map_path.c_str());

  example(gpkg_loader);
}
