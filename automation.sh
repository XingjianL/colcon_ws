kill -9 $(pgrep benchbot_xarm6_cpp)
RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run benchbot_xarm6_cpp benchbot_xarm6_cpp \
-- --light-temp 6500,0.3,1 --both --closest --bench-sample-gap 1 --arm_random_locations 0 --arm-sample-gap 11 --pred "0,0,10" --pcg-seed-incr 0 --ros-args --log-level error
kill -9 $(pgrep benchbot_xarm6_cpp)
RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run benchbot_xarm6_cpp benchbot_xarm6_cpp \
-- --light-temp 6500,0.3,1 --both --closest --bench-sample-gap 1 --arm_random_locations 0 --arm-sample-gap 14 --pcg-seed-incr 0 

kill -9 $(pgrep benchbot_xarm6_cpp)
RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run benchbot_xarm6_cpp benchbot_xarm6_cpp \
-- --light-temp 6500,0.3,1 --both --closest --bench-sample-gap 1 --arm_random_locations 0 --arm-sample-gap 11 --pred "0,0,10" --pcg-seed-incr 17
kill -9 $(pgrep benchbot_xarm6_cpp)
RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run benchbot_xarm6_cpp benchbot_xarm6_cpp \
-- --light-temp 6500,0.3,1 --both --closest --bench-sample-gap 1 --arm_random_locations 0 --arm-sample-gap 32 --pcg-seed-incr 0 

kill -9 $(pgrep benchbot_xarm6_cpp)
RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run benchbot_xarm6_cpp benchbot_xarm6_cpp \
-- --light-temp 6500,0.3,1 --both --closest --bench-sample-gap 1 --arm_random_locations 0 --arm-sample-gap 11 --pred "0,0,10" --pcg-seed-incr 2
kill -9 $(pgrep benchbot_xarm6_cpp)
RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run benchbot_xarm6_cpp benchbot_xarm6_cpp \
-- --light-temp 6500,0.3,1 --both --closest --bench-sample-gap 1 --arm_random_locations 0 --arm-sample-gap 34 --pcg-seed-incr 0 