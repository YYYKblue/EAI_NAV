# Upstream provenance

This package is the minimal runtime subset of `yocs_velocity_smoother` from
[`yujinrobot/yujin_ocs`](https://github.com/yujinrobot/yujin_ocs), commit
`17337e5a2d0a0f3711c55e272e656eb59174d657` (package version 0.12.1).

Only the nodelet source/header, dynamic-reconfigure schema, nodelet manifest,
runtime launch file, and BSD license are retained. Tests, IDE metadata, example
configuration, and unrelated YOCS packages are intentionally omitted.

Local changes are limited to ROS Noetic build metadata, the Python 3 shebang
used by the dynamic-reconfigure generator, and replacing the ECL worker-thread
wrapper with the C++11 standard-library equivalent.
