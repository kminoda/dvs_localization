# dvs_localization
A repository of my implementation for event-based localization with VLC markers.
I used prophesee camera and their drivers for this experiment.

## ./calibration
Execute calibration. You should blink checkerboards on PC or whatever. Then, collect event list from several perspectives.

## ./vlc
This contain two components: detector and encoder.
Detector detects VLC-LEDs according to their frequencies.
Encoder encodes messages from VLC.
The results should go into `./localization/data`

## ./localization
Pipeline for offline localization.
It calls functions from `./vlc` to locate and encode messages from marker, and then estimate it's pose.
This only supports localization upon 4-DoF.

## ./dvs_vlc_ros
ROS package for online localization based on VLC-markers with Event-Based Camera.