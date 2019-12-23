#include <examples_common.h>

#include <autoware_vector_map/api/vecor_map_api.h>

#include <autoware_vector_map/util/helper_functions.h>

namespace autoware_vector_map {

using util::contains_all;
using util::equal;
using util::sorted;
using util::to_ids;

void example(const std::string vector_map_path) {
  api::VectorMapApi vmap(new io::GpkgInterface(vector_map_path.c_str()));

  // findNearestLane
  const Point3d p(80720.70642536, 7406.76297997, 0);
  const double range = 5.0;
  const auto nearest_lane = vmap.findNearestLane(p, range);
  assert(nearest_lane->id == 66);

  // getNextLanes / getPrevLanes
  const auto next_lanes = vmap.getNextLanes(*nearest_lane);
  const auto prev_lanes = vmap.getPrevLanes(*nearest_lane);
  assert(equal(sorted(to_ids(next_lanes)), {69}));
  assert(equal(sorted(to_ids(prev_lanes)), {}));

  // getRelatedStopLines / getRelatedCrosswalks
  const auto lane_50 = vmap.getFeatureById<Lane>(50);
  const auto stop_lines = vmap.getRelatedStopLines(*lane_50);
  const auto crosswalks = vmap.getRelatedCrosswalks(*lane_50);
  assert(equal(sorted(to_ids(stop_lines)), {3}));
  assert(equal(sorted(to_ids(crosswalks)), {11, 14}));

  // TODO: findLaneRoute

  std::cout << "finish!" << std::endl;
}

}  // namespace autoware_vector_map

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "example1");

  ros::NodeHandle nh_("");
  ros::NodeHandle private_nh_("~");

  std::string vector_map_path;
  private_nh_.getParam("vector_map_path", vector_map_path);

  autoware_vector_map::example(vector_map_path);
}
