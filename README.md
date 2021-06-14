# obstacle_det
gives the nearest point by segmenting floor from point cloud message

## Description

Subscribes to /xtion/depth_registered/points topic, which is a point cloud. Then this cloud will be downsampled using voxel filter. Downsampled cloud is then segmented for a plane using RANSAC algorithm. and remaining points are used to find the nearest point of an obstacle present on the robot's path.

## Dependencies
- PCL
- c++11 compiler.
