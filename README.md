# Install Instructions (WIP)

This instruction is in progress, there could be error messages for missing dependencies.

### ROS2-Humble
1. Follow instructions here [https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

### Clone this repo
1. clone the repo and `cd UE5Sim_colcon_ws`
2. try `colcon build` (it should fail)

### ROSBridge w/ TCP
1. Clone this repo under `src` [https://github.com/tsender/rosbridge_suite](https://github.com/tsender/rosbridge_suite)

### Open3D
Refer to [https://www.open3d.org/docs/release/getting_started.html](https://www.open3d.org/docs/release/getting_started.html) for more details.
1. Get `open3d-devel-linux-x86_64-cxx11-abi-cuda-0.18.0.tar.xz` from [https://github.com/isl-org/Open3D/releases/tag/v0.18.0](https://github.com/isl-org/Open3D/releases/tag/v0.18.0)
2. Extract to a folder
3. Navigate to `lib/cmake/Open3D` folder, comment out line 17-19 in `Open3DConfig.cmake`.
4. Change `set(Open3D_DIR <dir>)` to the path in step 3 for `tomato_xarm6`, `benchbot_xarm_cpp`, and `benchbot_xarm6_stereo` packages
