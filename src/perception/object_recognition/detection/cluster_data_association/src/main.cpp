#include <ros/ros.h>
#include <cluster_data_association/node.hpp>

int main(int argc, char** argv) {
  ros::init(argc, argv, "cluster_data_association_node");
  cluster_data_association::ClusterDataAssociationNode node;
  ros::spin();
  return 0;
}
