# VINS-Mah

ROS 2 port of [VINS-Mah](https://github.com/hyq123-cmd/VINS-Mah), a visual-inertial
odometry system for dynamic environments built on VINS-Fusion.

The port remains under development. We reorganized the code, configuration and
launch files to follow ROS package conventions and are testing Zenoh as middleware.

Mahalanobis outlier rejection and pose-covariance recovery remain disabled as
inherited from upstream. The pose-covariance output is not populated. This port
has not reproduced the published VINS-Mah benchmarks.
