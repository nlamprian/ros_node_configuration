#include <ros_node_configuration/devices_node.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "devices_node");
  DevicesNode node;
  ros::spin();
  return 0;
}
