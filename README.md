# Voxblox (OpenApero fork)

[![Build Test](https://github.com/OpenApero/voxblox/actions/workflows/build_test.yml/badge.svg)](https://github.com/OpenApero/voxblox/actions/workflows/build_test.yml)

This is the [OpenApero](https://github.com/OpenApero/openapero) fork of
[ethz-asl/voxblox](https://github.com/ethz-asl/voxblox), maintained for
[open_apero](https://github.com/OpenApero/openapero). Everything below this
notice is upstream documentation and still applies.

## What this fork adds

* `TsdfServer` accessors for the region of the map touched since the last
  reset, the total map region, and the optical-frame poses that were actually
  integrated: `getUpdatedMapRegion`, `getTotalMapRegion`,
  `getMappedOpticalFrameTransforms`, `resetMapUpdateStatus`
  (`voxblox_ros/include/voxblox_ros/tsdf_server.h`).
* Two extra `Block::Status` bits, `kBbox` and `kCache`, for out-of-tree
  consumers that need their own updated-block bookkeeping
  (`voxblox/include/voxblox/core/block.h`).
* Insertion of a point cloud together with an exact-time-synchronized sensor
  transform, instead of a TF lookup: `insertSyncPointcloud`
  (`voxblox_ros/include/voxblox_ros/tsdf_server.h`).
* A per-insertion `weight_scaling_factor` on the TSDF integrators, so
  low-confidence points can be integrated without overwriting real
  measurements (`voxblox/include/voxblox/integrator/tsdf_integrator.h`).
* A planar `observation_boundary` option on the TSDF integrators, as an
  alternative to the spherical max-range boundary
  (`voxblox/include/voxblox/integrator/tsdf_integrator.h`).
* `integrator_threads` and `mesh_integrator_threads` parameters, defaulting to
  the CPU allocation of the process rather than the machine's core count
  (`voxblox_ros/include/voxblox_ros/ros_params.h`,
  `voxblox/include/voxblox/utils/cpu_count.h`).
* A `voxblox_mesh_display/reset` service on the RViz mesh display, so the
  display can be cleared without user interaction
  (`voxblox_rviz_plugin/src/voxblox_mesh_display.cc`).
* Mesh-layer and mesh-integrator accessors plus `hasMeshSubscribers()` on
  `TsdfServer`, and `MeshLayer::clearAllMeshes()`, which empties the meshes but
  keeps the blocks so remote displays are told to drop them
  (`voxblox_ros/include/voxblox_ros/tsdf_server.h`,
  `voxblox/include/voxblox/mesh/mesh_layer.h`).
* An `optical_frame_transforms` field on the `voxblox_msgs/Layer` message
  (`voxblox_msgs/msg/Layer.msg`).

## Installation

The ten catkin build dependencies are vendored as git submodules under
`deps/`, so clone recursively into the `src/` directory of your catkin
workspace:

```bash
cd ~/catkin_ws/src
git clone --recursive https://github.com/OpenApero/voxblox.git
```

Then build as usual:

```bash
cd ~/catkin_ws
catkin build voxblox_ros
```

Do not additionally fetch the dependencies with `wstool`; doing so would give
the workspace two copies of each catkin package. The upstream readthedocs
installation page describes upstream's `wstool` route, which does not apply
here; its apt dependency list still does.

---

![voxblox_small](https://cloud.githubusercontent.com/assets/5616392/15180357/536a8776-1781-11e6-8c1d-f2dfa34b1408.gif)

Voxblox is a volumetric mapping library based mainly on Truncated Signed Distance Fields (TSDFs). It varies from other SDF libraries in the following ways:
 * CPU-only, can be run single-threaded or multi-threaded for some integrators
 * Support for multiple different layer types (containing different types of voxels)
 * Serialization using protobufs
 * Different ways of handling weighting during merging
 * Different ways of inserting pose information about scans
 * Tight ROS integration (in voxblox_ros package)
 * Easily extensible with whatever integrators you want
 * Features an implementation of building Euclidean Signed Distance Fields (ESDFs, EDTs) directly from TSDFs.

**If you're looking for skeletonization/sparse topology or planning applications, please refer to the [mav_voxblox_planning](https://github.com/ethz-asl/mav_voxblox_planning) repo.**
**If you want to create ground truth maps from meshes or gazebo environments, please check out the [voxblox_ground_truth](https://github.com/ethz-asl/voxblox_ground_truth) pakage!**

![example_gif](http://i.imgur.com/2wLztFm.gif)

# Documentation
* All voxblox documentation can be found on [our readthedocs page](https://voxblox.readthedocs.io/en/latest/index.html)

## Table of Contents
* [Paper and Video](#paper-and-video)
* [Credits](#credits)
* [Example Outputs](https://voxblox.readthedocs.io/en/latest/pages/Example-Outputs.html)
* [Performance](https://voxblox.readthedocs.io/en/latest/pages/Performance.html)
* [Installation](#installation) (this fork; the upstream readthedocs installation page does not apply)
* [Running Voxblox](https://voxblox.readthedocs.io/en/latest/pages/Running-Voxblox.html)
* [Using Voxblox for Planning](https://voxblox.readthedocs.io/en/latest/pages/Using-Voxblox-for-Planning.html)
* [Transformations in Voxblox](https://voxblox.readthedocs.io/en/latest/pages/Transformations.html)
* [Contributing to Voxblox](https://voxblox.readthedocs.io/en/latest/pages/Modifying-and-Contributing.html)
* [Library API](https://voxblox.readthedocs.io/en/latest/api/library_root.html)

# Paper and Video
A video showing sample output from voxblox can be seen [here](https://www.youtube.com/watch?v=PlqT5zNsvwM).
A video of voxblox being used for online planning on-board a multicopter can be seen [here](https://youtu.be/lrGSwAPzMOQ).

If using voxblox for scientific publications, please cite the following paper, available [here](http://helenol.github.io/publications/iros_2017_voxblox.pdf):

Helen Oleynikova, Zachary Taylor, Marius Fehr, Juan Nieto, and Roland Siegwart, “**Voxblox: Incremental 3D Euclidean Signed Distance Fields for On-Board MAV Planning**”, in *IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*, 2017.

```latex
@inproceedings{oleynikova2017voxblox,
  author={Oleynikova, Helen and Taylor, Zachary and Fehr, Marius and Siegwart, Roland and  Nieto, Juan},
  booktitle={IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  title={Voxblox: Incremental 3D Euclidean Signed Distance Fields for On-Board MAV Planning},
  year={2017}
}
```

# Credits
This library was written primarily by Helen Oleynikova and Marius Fehr, with significant contributions from Zachary Taylor, Alexander Millane, and others. The marching cubes meshing and ROS mesh generation were taken or heavily derived from [open_chisel](https://github.com/personalrobotics/OpenChisel). We've retained the copyright headers for the relevant files.

![offline_manifold](https://i.imgur.com/pvHhVsL.png)
