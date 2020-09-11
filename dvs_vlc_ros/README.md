# What is this package?
ROS package for VLC-based localization with Event-Based Camera (EBS).  
It detects the VLC-based LEDs in field of view, receive an ID information from markers, and localize itself.

# Requirements
- Eigen
- OpenCV
- ROS (Tested on Kinetic with Ubuntu 16.04)
- cfg/calibration.txt (which should include intrinsic parameters)
- cfg/markers.txt (which should include markers' position in world frame)

