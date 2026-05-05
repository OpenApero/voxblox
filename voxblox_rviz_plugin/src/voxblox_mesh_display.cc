#include "voxblox_rviz_plugin/voxblox_mesh_display.h"

#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>

#include <tf/transform_listener.h>

#include <rviz/frame_manager.h>
#include <rviz/visualization_manager.h>

namespace voxblox_rviz_plugin {

VoxbloxMeshDisplay::VoxbloxMeshDisplay() : reset_requested_(false) {}

void VoxbloxMeshDisplay::onInitialize() {
  MFDClass::onInitialize();

  reset_service_ = nh_.advertiseService(
      "voxblox_mesh_display/reset",
      &VoxbloxMeshDisplay::resetServiceCallback, this);
}

VoxbloxMeshDisplay::~VoxbloxMeshDisplay() {}

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
  reset_requested_.store(true);
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
