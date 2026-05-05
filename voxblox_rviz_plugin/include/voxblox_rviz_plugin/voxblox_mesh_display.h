#ifndef VOXBLOX_RVIZ_PLUGIN_VOXBLOX_MESH_DISPLAY_H_
#define VOXBLOX_RVIZ_PLUGIN_VOXBLOX_MESH_DISPLAY_H_

#include <atomic>
#include <memory>

#include <ros/ros.h>
#include <rviz/message_filter_display.h>
#include <std_srvs/Trigger.h>
#include <voxblox_msgs/Mesh.h>

#include "voxblox_rviz_plugin/voxblox_mesh_visual.h"

namespace voxblox_rviz_plugin {

class VoxbloxMeshVisual;

class VoxbloxMeshDisplay
    : public rviz::MessageFilterDisplay<voxblox_msgs::Mesh> {
  Q_OBJECT
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  VoxbloxMeshDisplay();
  virtual ~VoxbloxMeshDisplay();

 protected:
  virtual void onInitialize();

  virtual void reset();

  virtual void update(float wall_dt, float ros_dt);

 private:
  void processMessage(const voxblox_msgs::Mesh::ConstPtr& msg);
  bool resetServiceCallback(std_srvs::Trigger::Request& request,
                            std_srvs::Trigger::Response& response);

  std::unique_ptr<VoxbloxMeshVisual> visual_;
  ros::NodeHandle nh_;
  ros::ServiceServer reset_service_;
  std::atomic_bool reset_requested_;
};

}  // namespace voxblox_rviz_plugin

#endif  // VOXBLOX_RVIZ_PLUGIN_VOXBLOX_MESH_DISPLAY_H_
