#include "voxblox_rviz_plugin/voxblox_mesh_display.h"

#include <mutex>
#include <set>

#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>

#include <tf/transform_listener.h>

#include <rviz/frame_manager.h>
#include <rviz/visualization_manager.h>

namespace voxblox_rviz_plugin {

namespace {
// A node can advertise a service name only once, so the mesh displays in one
// RViz share a reset service, advertised while any of them exists.
std::mutex displays_mutex;
std::set<VoxbloxMeshDisplay*> displays;
ros::ServiceServer reset_service;
}  // namespace

VoxbloxMeshDisplay::VoxbloxMeshDisplay() : reset_requested_(false) {}

void VoxbloxMeshDisplay::onInitialize() {
  MFDClass::onInitialize();

  std::lock_guard<std::mutex> lock(displays_mutex);
  if (displays.empty()) {
    ros::NodeHandle nh;
    reset_service =
        nh.advertiseService("voxblox_mesh_display/reset",
                            &VoxbloxMeshDisplay::resetServiceCallback);
  }
  displays.insert(this);
}

VoxbloxMeshDisplay::~VoxbloxMeshDisplay() {
  std::lock_guard<std::mutex> lock(displays_mutex);
  displays.erase(this);
  if (displays.empty()) {
    // Unadvertise now, while ROS is up, rather than at static destruction.
    reset_service = ros::ServiceServer();
  }
}

void VoxbloxMeshDisplay::reset() {
  MFDClass::reset();
  reset_requested_.store(false);
  visual_.reset();
}

void VoxbloxMeshDisplay::update(float wall_dt, float ros_dt) {
  MFDClass::update(wall_dt, ros_dt);

  if (reset_requested_.exchange(false)) {
    visual_.reset();
  }
}

bool VoxbloxMeshDisplay::resetServiceCallback(
    std_srvs::Trigger::Request& /*request*/,
    std_srvs::Trigger::Response& response) {
  std::lock_guard<std::mutex> lock(displays_mutex);
  for (VoxbloxMeshDisplay* display : displays) {
    display->reset_requested_.store(true);
  }
  response.success = true;
  response.message = "Voxblox mesh display reset requested.";
  return true;
}

void VoxbloxMeshDisplay::processMessage(
    const voxblox_msgs::Mesh::ConstPtr& msg) {
  // Here we call the rviz::FrameManager to get the transform from the
  // fixed frame to the frame in the header of this Imu message.  If
  // it fails, we can't do anything else so we return.
  Ogre::Quaternion orientation;
  Ogre::Vector3 position;
  if (!context_->getFrameManager()->getTransform(
          msg->header.frame_id, msg->header.stamp, position, orientation)) {
    ROS_DEBUG("Error transforming from frame '%s' to frame '%s'",
              msg->header.frame_id.c_str(), qPrintable(fixed_frame_));
    return;
  }

  if (visual_ == nullptr) {
    visual_.reset(
        new VoxbloxMeshVisual(context_->getSceneManager(), scene_node_));
  }

  // Now set or update the contents of the chosen visual.
  visual_->setMessage(msg);
  visual_->setFramePosition(position);
  visual_->setFrameOrientation(orientation);
}

}  // namespace voxblox_rviz_plugin

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(voxblox_rviz_plugin::VoxbloxMeshDisplay, rviz::Display)
