============
Installation
============

These instructions are for the OpenApero fork of voxblox
(https://github.com/OpenApero/voxblox), which vendors its catkin build
dependencies as git submodules under ``deps/``.

Install `ROS Noetic <http://wiki.ros.org/noetic/Installation/Ubuntu/>`_ on
Ubuntu 20.04, the only distribution this fork is built and tested on.

First install additional system dependencies::

	sudo apt-get install python3-catkin-tools ros-noetic-cmake-modules protobuf-compiler autoconf libprotobuf-dev protobuf-c-compiler

If you don't have a catkin workspace yet, set it up as follows::

	mkdir -p ~/catkin_ws/src
	cd ~/catkin_ws
	catkin init
	catkin config --extend /opt/ros/noetic
	catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
	catkin config --merge-devel

Clone the repository recursively, so the dependencies under ``deps/`` are
fetched with it::

	cd ~/catkin_ws/src/
	git clone --recursive https://github.com/OpenApero/voxblox.git

If you already cloned without ``--recursive``, fetch the dependencies with::

	cd ~/catkin_ws/src/voxblox
	git submodule update --init --recursive

Do not additionally fetch the dependencies with ``wstool``: that would give the
workspace two copies of each catkin package.

Compile::

	cd ~/catkin_ws/
	catkin build voxblox_ros
