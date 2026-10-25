# Voxblox (OpenApero fork)

[![Build Test](https://github.com/OpenApero/voxblox/actions/workflows/build_test.yml/badge.svg)](https://github.com/OpenApero/voxblox/actions/workflows/build_test.yml)

This is the [OpenApero](https://github.com/OpenApero/openapero) fork of
[ethz-asl/voxblox](https://github.com/ethz-asl/voxblox), maintained for
[open_apero](https://github.com/OpenApero/openapero). Open Apero 1.0.0 builds
against the tag `open_apero-1.0.0`. Everything below the horizontal rule is
upstream documentation; it still applies except where the behaviour changes
listed here say otherwise.

## What this fork adds

* `TsdfServer` accessors for the region of the map touched since the last
  reset, the total map region, and the optical-frame poses the point clouds
  were integrated at (ICP-refined when ICP is on):
  `getUpdatedMapRegion`, `getTotalMapRegion`,
  `getMappedOpticalFrameTransforms`, `resetMapUpdateStatus`
  (`voxblox_ros/include/voxblox_ros/tsdf_server.h`).
* Two extra `Block::Status` bits, `kBbox` and `kCache`, for out-of-tree
  consumers that need their own updated-block bookkeeping
  (`voxblox/include/voxblox/core/block.h`).
* Insertion of a point cloud together with an exact-time-synchronized sensor
  transform, instead of a TF lookup: `insertSyncPointcloud`, fed by the
  `sync_pointcloud` and `sync_camera_transform` topics, which queue
  `sync_queue_size` messages (default 10)
  (`voxblox_ros/include/voxblox_ros/tsdf_server.h`).
* A per-insertion `weight_scaling_factor` on the TSDF integrators, so
  low-confidence points can be integrated without overwriting real
  measurements (`voxblox/include/voxblox/integrator/tsdf_integrator.h`).
* A planar `observation_boundary` option on the TSDF integrators, as an
  alternative to the spherical max-range boundary
  (`voxblox/include/voxblox/integrator/tsdf_integrator.h`).
* `integrator_threads` and `mesh_integrator_threads` parameters, defaulting to
  the CPU allocation of the process rather than the machine's core count,
  which a non-positive value also selects
  (`voxblox_ros/include/voxblox_ros/ros_params.h`,
  `voxblox/include/voxblox/utils/cpu_count.h`).
* A `voxblox_mesh_display/reset` service on the RViz mesh display, so the
  mesh displays of an RViz instance can be cleared without user interaction
  (`voxblox_rviz_plugin/src/voxblox_mesh_display.cc`).
* Mesh-layer and mesh-integrator accessors plus `hasMeshSubscribers()` on
  `TsdfServer`, and `MeshLayer::clearAllMeshes()`, which empties the meshes but
  keeps the blocks so remote displays are told to drop them
  (`voxblox_ros/include/voxblox_ros/tsdf_server.h`,
  `voxblox/include/voxblox/mesh/mesh_layer.h`).
* An `optical_frame_transforms` field on the `voxblox_msgs/Layer` message
  (`voxblox_msgs/msg/Layer.msg`), which carries each integrated sensor pose
  once: `esdf_server` sends it with the first ESDF layer whose update covered
  it, or with the TSDF layer when no ESDF layer goes out, and `tsdf_server`
  with the next TSDF layer.

## Behaviour changes relative to upstream

* **`voxblox_msgs/Layer` is not wire-compatible with upstream.** The new
  `optical_frame_transforms` field changes the message's MD5 sum, so the
  fork's nodes and stock voxblox nodes cannot exchange `tsdf_map_*` or
  `esdf_map_*` layers. Build every node that exchanges layers from this fork.
* `TsdfServer::publishMap` publishes only when new data was integrated since
  the previous publish or a new subscriber needs the whole map.
* The `EsdfServer` update timer skips cycles in which no point cloud was
  integrated and no TSDF layer received. `EsdfServer::publishMap` returns early unless the ESDF was
  recomputed since it was last published or a new subscriber needs the whole
  map, and then publishes the TSDF layer as well, controlled by
  `publish_tsdf_map` alone. Upstream published the TSDF
  from `esdf_server` only when `publish_esdf_map` was set.
* After a layer is published, the `kMap` update bits of its blocks are reset,
  so the next incremental message carries only blocks changed since then.
  Upstream never reset them, so incremental messages kept every block ever
  updated.
* Layers received on `tsdf_map_in` / `esdf_map_in` mark every received block
  as updated for all consumers (mesh, ESDF, updated region, publishing) and
  count as new data for the ESDF update and map publishing, as local
  integration does.
* The `esdf_map_in` subscriber queue is 10 messages instead of 1.
* `TsdfServer::updateMesh` publishes (and caches) the mesh on `~mesh` only
  while that topic has subscribers, and the `~mesh_pcl`
  (`pcl_msgs/PolygonMesh`) publisher is removed. `clear()` publishes the emptied mesh blocks before
  erasing them, so remote displays drop them.
* With `allow_clear` enabled, the TSDF integrators warn once if none of the
  first 100 point clouds contained a clearing point (a point beyond
  `max_ray_length_m`), which usually means the sensor's far-depth readings do
  not exceed that range.
* Integrator, mesh-integrator and ICP thread counts default to the CPU
  allocation of the process instead of `std::thread::hardware_concurrency()`.

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
