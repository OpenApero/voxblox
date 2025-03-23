#include "voxblox_ros/esdf_server.h"

#include <gflags/gflags.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "voxblox");
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InstallFailureSignalHandler();
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  voxblox::EsdfServer node(nh, nh_private);

  // TODO (kaixqu): Make this a parameter.
  int num_threads = 8;
  ros::AsyncSpinner spinner(num_threads);
  spinner.start();

  // ros::waitForShutdown() blocks until ROS is shut down
  ros::waitForShutdown();
  return 0;
}
