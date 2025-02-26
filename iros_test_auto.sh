# for i in $(seq 1 10);
# do
#     RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run benchbot_xarm6_cpp benchbot_xarm6_cpp \
#     -- --light-temp 6500,0.3,1 --both --closest --bench-sample-gap 1 --arm_random_locations 0 --arm-sample-gap 40 --pred --pcg-seed-incr 1

#     RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run benchbot_xarm6_cpp benchbot_xarm6_cpp \
#     -- --light-temp 6500,0.3,1 --both --closest --bench-sample-gap 1 --arm_random_locations 0 --arm-sample-gap 40 --pcg-seed-incr 0 
# done
ros2 run benchbot_xarm6_cpp benchbot_xarm6_cpp -- --light-temp 6500,0.3,1 --both --closest --bench-sample-gap 1 --arm_random_locations 0 --arm-sample-gap 11 --pcg-seed-incr 0 --ros-args --log-level info
for i in $(seq 1 2);
do
    # kill -9 $(pgrep benchbot_xarm6_cpp)
    # ros2 run benchbot_xarm6_cpp benchbot_xarm6_cpp \
    # -- --light-temp 6500,0.3,1 --both --closest --bench-sample-gap 1 --arm_random_locations 0 --arm-sample-gap 11 --pred "0,1,100" --pcg-seed-incr 1
    
    # kill -9 $(pgrep benchbot_xarm6_cpp)
    # ros2 run benchbot_xarm6_cpp benchbot_xarm6_cpp \
    # -- --light-temp 6500,0.3,1 --both --closest --bench-sample-gap 1 --arm_random_locations 0 --arm-sample-gap 11 --pred "0,1,1000" --pcg-seed-incr 0
    
    # kill -9 $(pgrep benchbot_xarm6_cpp)
    # ros2 run benchbot_xarm6_cpp benchbot_xarm6_cpp \
    # -- --light-temp 6500,0.3,1 --both --closest --bench-sample-gap 1 --arm_random_locations 0 --arm-sample-gap 11 --pred "0,1,10000" --pcg-seed-incr 0
    
    # kill -9 $(pgrep benchbot_xarm6_cpp)
    # sleep 1
    # wait
    # ros2 run benchbot_xarm6_cpp benchbot_xarm6_cpp \
    # -- --light-temp 6500,0.3,1 --both --closest --bench-sample-gap 1 --arm_random_locations 0 --arm-sample-gap 11 --pred "0,0,10" --pcg-seed-incr 1 --ros-args --log-level info

    # kill -9 $(pgrep benchbot_xarm6_cpp)
    # sleep 1
    # wait
    # ros2 run benchbot_xarm6_cpp benchbot_xarm6_cpp \
    # -- --light-temp 6500,0.3,1 --both --closest --bench-sample-gap 1 --arm_random_locations 0 --arm-sample-gap 11 --pred "0,0,10" --nbv-color-id 16 --pcg-seed-incr 0 --ros-args --log-level info

    # kill -9 $(pgrep benchbot_xarm6_cpp)
    # sleep 1
    # wait
    # ros2 run benchbot_xarm6_cpp benchbot_xarm6_cpp \
    # -- --light-temp 6500,0.3,1 --both --closest --bench-sample-gap 1 --arm_random_locations 0 --arm-sample-gap 11 --pred "0,0,10" --nbv-color-id 20 --pcg-seed-incr 0 --ros-args --log-level info

    # kill -9 $(pgrep benchbot_xarm6_cpp)
    # sleep 1
    # wait
    # ros2 run benchbot_xarm6_cpp benchbot_xarm6_cpp \
    # -- --light-temp 6500,0.3,1 --both --closest --bench-sample-gap 1 --arm_random_locations 0 --arm-sample-gap 11 --pred "0,0,10" --nbv-color-id 26 --pcg-seed-incr 0 --ros-args --log-level info
    
    # kill -9 $(pgrep benchbot_xarm6_cpp)
    # sleep 1
    # wait
    # ros2 run benchbot_xarm6_cpp benchbot_xarm6_cpp \
    # -- --light-temp 6500,0.3,1 --both --closest --bench-sample-gap 1 --arm_random_locations 0 --arm-sample-gap 11 --pcg-seed-incr 0 --ros-args --log-level info
done
# kill -9 $(pgrep benchbot_xarm6_cpp)
# RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run benchbot_xarm6_cpp benchbot_xarm6_cpp \
# -- --light-temp 6500,0.3,1 --both --closest --bench-sample-gap 1 --arm_random_locations 0 --arm-sample-gap 12 --pcg-seed-incr 0 
# for i in $(seq 2 4 39);
# do
#     kill -9 $(pgrep benchbot_xarm6_cpp)
#     sleep 1
#     wait
#     RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run benchbot_xarm6_cpp benchbot_xarm6_cpp \
#     -- --light-temp 7500,0.3,1 --both --closest --bench-sample-gap 1 --arm_random_locations 0 --arm-sample-gap "$i" --pcg-seed-incr 0 --ros-args --log-level error
# done
# for j in $(seq 8 60);
# do
#     kill -9 $(pgrep benchbot_xarm6_cpp)
#     sleep 1
#     wait
#     RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run benchbot_xarm6_cpp benchbot_xarm6_cpp \
#     -- --light-temp 7500,0.3,1 --both --closest --bench-sample-gap 1 --arm_random_locations 0 --arm-sample-gap 11 --pred "0,0,10" --pcg-seed-incr 1 --ros-args --log-level info
#     for i in $(seq 2 4 39);
#     do
#         kill -9 $(pgrep benchbot_xarm6_cpp)
#         sleep 1
#         wait
#         RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run benchbot_xarm6_cpp benchbot_xarm6_cpp \
#         -- --light-temp 7500,0.3,1 --both --closest --bench-sample-gap 1 --arm_random_locations 0 --arm-sample-gap "$i" --pcg-seed-incr 0 --ros-args --log-level error
#     done
# done
