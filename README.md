# multiple-kinects-auto-calibration
Compilation has been tested on ros-kinetic and ros-noetic

Master thesis work performing live calibration of multiple 3D cameras with low overlapping area in an indoor environment by matching large planes such as ground, tables, walls etc.

This repo includes ROS nodes for:

- Pointcloud preprocessing (subsampling, cutting and transformation)
- Plane detection
- Several 3D keypoints extraction
- Plane matching
- Keypoints matching
- ICP pointcloud registration
- Plane matching registration
- Keypoints registration
- Mixed KP and plane registration

![Calibration Result](https://raw.githubusercontent.com/smaillot/master-thesis/master/images/planes_scene.png)
