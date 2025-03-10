RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_xarm6 -- --reset-time --light-temp 10000,5,1 --both --seed 508 --split-height-leaf 300,16 --preprocess Lab --percent-healthy 0.1 --pcg-seed-incr 1
RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_xarm6 -- --reset-time --light-temp 10000,4,1 --both --seed 518 --split-height-leaf 300,16 --preprocess Lab --percent-healthy 0.1 --pcg-seed-incr 1
RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_xarm6 -- --reset-time --light-temp 10000,3,1 --both --seed 528 --split-height-leaf 300,16 --preprocess Lab --percent-healthy 0.1 --pcg-seed-incr 1
RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_xarm6 -- --reset-time --light-temp 10000,2,1 --both --seed 538 --split-height-leaf 300,16 --preprocess Lab --percent-healthy 0.1 --pcg-seed-incr 1
RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_xarm6 -- --reset-time --light-temp 10000,1,1 --both --seed 548 --split-height-leaf 300,16 --preprocess Lab --percent-healthy 0.1 --pcg-seed-incr 1
RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_xarm6 -- --reset-time --light-temp 10000,0,1 --both --seed 558 --split-height-leaf 300,16 --preprocess Lab --percent-healthy 0.1 --pcg-seed-incr 1
RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_xarm6 -- --reset-time --light-temp 10000,0,1 --both --seed 568 --split-height-leaf 300,16 --preprocess Lab --percent-healthy 0.1 --pcg-seed-incr 1
RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_xarm6 -- --reset-time --light-temp 10000,1,1 --both --seed 578 --split-height-leaf 300,16 --preprocess Lab --percent-healthy 0.1 --pcg-seed-incr 1
RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_xarm6 -- --reset-time --light-temp 10000,2,1 --both --seed 588 --split-height-leaf 300,16 --preprocess Lab --percent-healthy 0.1 --pcg-seed-incr 1
RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_xarm6 -- --reset-time --light-temp 10000,3,1 --both --seed 598 --split-height-leaf 300,16 --preprocess Lab --percent-healthy 0.1 --pcg-seed-incr 1

# kill -9 $(pgrep benchbot_xarm6_cpp)
# RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run benchbot_xarm6_cpp benchbot_xarm6_cpp \
# -- --light-temp 6500,0.3,1 --both --closest --bench-sample-gap 1 --arm_random_locations 0 --arm-sample-gap 11 --pred "0,0,10" --pcg-seed-incr 0 --ros-args --log-level error
# kill -9 $(pgrep benchbot_xarm6_cpp)
# RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run benchbot_xarm6_cpp benchbot_xarm6_cpp \
# -- --light-temp 6500,0.3,1 --both --closest --bench-sample-gap 1 --arm_random_locations 0 --arm-sample-gap 14 --pcg-seed-incr 0 

# kill -9 $(pgrep benchbot_xarm6_cpp)
# RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run benchbot_xarm6_cpp benchbot_xarm6_cpp \
# -- --light-temp 6500,0.3,1 --both --closest --bench-sample-gap 1 --arm_random_locations 0 --arm-sample-gap 11 --pred "0,0,10" --pcg-seed-incr 17
# kill -9 $(pgrep benchbot_xarm6_cpp)
# RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run benchbot_xarm6_cpp benchbot_xarm6_cpp \
# -- --light-temp 6500,0.3,1 --both --closest --bench-sample-gap 1 --arm_random_locations 0 --arm-sample-gap 32 --pcg-seed-incr 0 

# kill -9 $(pgrep benchbot_xarm6_cpp)
# RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run benchbot_xarm6_cpp benchbot_xarm6_cpp \
# -- --light-temp 6500,0.3,1 --both --closest --bench-sample-gap 1 --arm_random_locations 0 --arm-sample-gap 11 --pred "0,0,10" --pcg-seed-incr 2
# kill -9 $(pgrep benchbot_xarm6_cpp)
# RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run benchbot_xarm6_cpp benchbot_xarm6_cpp \
# -- --light-temp 6500,0.3,1 --both --closest --bench-sample-gap 1 --arm_random_locations 0 --arm-sample-gap 34 --pcg-seed-incr 0 