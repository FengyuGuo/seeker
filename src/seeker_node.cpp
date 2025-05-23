#include <nodelet/loader.h>
#include <ros/ros.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "seeker_node");
  nodelet::Loader nodelet;
  nodelet::M_string remap(ros::names::getRemappings());
  nodelet::V_string nargv;
  std::string nodelet_name = ros::this_node::getName();
  ROS_INFO_STREAM("Started " << nodelet_name << " nodelet.");
  nodelet.load(nodelet_name, "seeker/SeekRosNode", remap,
               nargv);
  ros::spin();
  return 0;
}
