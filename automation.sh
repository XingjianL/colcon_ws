# RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_xarm6 -- --reset-time --light-temp 3000 --both --closest --seed 0
# for i in $(seq 1 11);
# do
#     echo $i
#     #ros2 run benchbot_xarm6_cpp benchbot_xarm6_cpp
#     #ros2 run benchbot_xarm6_stereo benchbot_xarm6_stereo
#     RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_xarm6 -- --light-temp 3000 --both --closest --seed $i
# done

#RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_xarm6 -- --reset-time --light-temp 6500 --both --closest --seed 12
# for i in $(seq 1 11);
# do
#     echo $((12+i))
#     #ros2 run benchbot_xarm6_cpp benchbot_xarm6_cpp
#     #ros2 run benchbot_xarm6_stereo benchbot_xarm6_stereo
#     RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_xarm6 -- --light-temp 6500 --both --closest --seed $((12+i))
# done

# RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_xarm6 -- --reset-time --light-temp 10000 --both --closest --seed 24

RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run benchbot_xarm6_cpp benchbot_xarm6_cpp -- --reset-time --light-temp 6500,0.3,1 --both --closest --bench-sample-gap 1 --arm_random_locations 0 --arm-sample-gap 2 --pcg-seed-incr 6
for i in $(seq 1 4);
do
    kill -9 $(pgrep benchbot_xarm6_cpp)
    RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run benchbot_xarm6_cpp benchbot_xarm6_cpp -- --light-temp 6500,0.3,1 --both --closest --bench-sample-gap 1 --arm_random_locations 0 --arm-sample-gap 2 #% --ros-args --log-level error 
done

RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run benchbot_xarm6_cpp benchbot_xarm6_cpp -- --reset-time --light-temp 6500,0.3,1 --both --closest --bench-sample-gap 1 --arm_random_locations 0 --arm-sample-gap 3 --pcg-seed-incr 6
for i in $(seq 1 4);
do
    kill -9 $(pgrep benchbot_xarm6_cpp)
    RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run benchbot_xarm6_cpp benchbot_xarm6_cpp -- --light-temp 6500,0.3,1 --both --closest --bench-sample-gap 1 --arm_random_locations 0 --arm-sample-gap 3 #% --ros-args --log-level error 
done

RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run benchbot_xarm6_cpp benchbot_xarm6_cpp -- --reset-time --light-temp 6500,0.3,1 --both --closest --bench-sample-gap 1 --arm_random_locations 0 --arm-sample-gap 4 --pcg-seed-incr 5
for i in $(seq 1 4);
do
    kill -9 $(pgrep benchbot_xarm6_cpp)
    RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run benchbot_xarm6_cpp benchbot_xarm6_cpp -- --light-temp 6500,0.3,1 --both --closest --bench-sample-gap 1 --arm_random_locations 0 --arm-sample-gap 4 #% --ros-args --log-level error 
done

RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run benchbot_xarm6_cpp benchbot_xarm6_cpp -- --reset-time --light-temp 6500,0.3,1 --both --closest --bench-sample-gap 1 --arm_random_locations 0 --arm-sample-gap 5 --pcg-seed-incr 5
for i in $(seq 1 4);
do
    kill -9 $(pgrep benchbot_xarm6_cpp)
    RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run benchbot_xarm6_cpp benchbot_xarm6_cpp -- --light-temp 6500,0.3,1 --both --closest --bench-sample-gap 1 --arm_random_locations 0 --arm-sample-gap 5 #% --ros-args --log-level error 
done

RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run benchbot_xarm6_cpp benchbot_xarm6_cpp -- --reset-time --light-temp 6500,0.3,1 --both --closest --bench-sample-gap 1 --arm_random_locations 0 --arm-sample-gap 6 --pcg-seed-incr 5
for i in $(seq 1 4);
do
    kill -9 $(pgrep benchbot_xarm6_cpp)
    RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run benchbot_xarm6_cpp benchbot_xarm6_cpp -- --light-temp 6500,0.3,1 --both --closest --bench-sample-gap 1 --arm_random_locations 0 --arm-sample-gap 6 #% --ros-args --log-level error 
done

RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run benchbot_xarm6_cpp benchbot_xarm6_cpp -- --reset-time --light-temp 6500,0.3,1 --both --closest --bench-sample-gap 1 --arm_random_locations 0 --arm-sample-gap 7 --pcg-seed-incr 5
for i in $(seq 1 4);
do
    kill -9 $(pgrep benchbot_xarm6_cpp)
    RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run benchbot_xarm6_cpp benchbot_xarm6_cpp -- --light-temp 6500,0.3,1 --both --closest --bench-sample-gap 1 --arm_random_locations 0 --arm-sample-gap 7 #% --ros-args --log-level error 
done

RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run benchbot_xarm6_cpp benchbot_xarm6_cpp -- --reset-time --light-temp 6500,0.3,1 --both --closest --bench-sample-gap 1 --arm_random_locations 0 --arm-sample-gap 8 --pcg-seed-incr 5
for i in $(seq 1 4);
do
    kill -9 $(pgrep benchbot_xarm6_cpp)
    RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run benchbot_xarm6_cpp benchbot_xarm6_cpp -- --light-temp 6500,0.3,1 --both --closest --bench-sample-gap 1 --arm_random_locations 0 --arm-sample-gap 8 #% --ros-args --log-level error 
done

RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run benchbot_xarm6_cpp benchbot_xarm6_cpp -- --reset-time --light-temp 6500,0.3,1 --both --closest --bench-sample-gap 1 --arm_random_locations 0 --arm-sample-gap 9 --pcg-seed-incr 5
for i in $(seq 1 4);
do
    kill -9 $(pgrep benchbot_xarm6_cpp)
    RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run benchbot_xarm6_cpp benchbot_xarm6_cpp -- --light-temp 6500,0.3,1 --both --closest --bench-sample-gap 1 --arm_random_locations 0 --arm-sample-gap 9 #% --ros-args --log-level error 
done

RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run benchbot_xarm6_cpp benchbot_xarm6_cpp -- --reset-time --light-temp 6500,0.3,1 --both --closest --bench-sample-gap 1 --arm_random_locations 0 --arm-sample-gap 10 --pcg-seed-incr 5
for i in $(seq 1 4);
do
    kill -9 $(pgrep benchbot_xarm6_cpp)
    RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run benchbot_xarm6_cpp benchbot_xarm6_cpp -- --light-temp 6500,0.3,1 --both --closest --bench-sample-gap 1 --arm_random_locations 0 --arm-sample-gap 10 #% --ros-args --log-level error 
done

RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run benchbot_xarm6_cpp benchbot_xarm6_cpp -- --reset-time --light-temp 6500,0.3,1 --both --closest --bench-sample-gap 1 --arm_random_locations 0 --arm-sample-gap 11 --pcg-seed-incr 5
for i in $(seq 1 4);
do
    kill -9 $(pgrep benchbot_xarm6_cpp)
    RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run benchbot_xarm6_cpp benchbot_xarm6_cpp -- --light-temp 6500,0.3,1 --both --closest --bench-sample-gap 1 --arm_random_locations 0 --arm-sample-gap 11 #% --ros-args --log-level error 
done

# ---

# for i in $(seq 1 21);
# do
#     kill -9 $(pgrep tomato_xarm6)
#     RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_xarm6 -- --reset-time --light-temp 6500,40,-3 --both --closest --seed 508 --disease-filter 2 --split-height-leaf 300,16 --preprocess Lab --percent-healthy 0.1 --move-robot "[{name: BenchBot, x: 100, y: 200}, {name: Spider, x: 300, y: 350}, {name: Husky, x: 200, y: 525}]"
#     kill -9 $(pgrep tomato_xarm6)
# done

# RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_xarm6 -- --reset-time --light-temp 6500,40,-3 --both --closest --seed 506 --disease-filter 0,2,3,4,6,7,8,9 --split-height-leaf 300,16 --preprocess Lab
# RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_xarm6 -- --reset-time --light-temp 6500,40,-3 --both --closest --seed 507 --disease-filter 1,2,3,4,6,7,8,9 --split-height-leaf 300,16 --preprocess Lab

# RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_xarm6 -- --reset-time --light-temp 6500,40,-3 --both --closest --seed 508 --disease-filter 0,2,3,4,6,7,8,9 --split-height-leaf 75,16 --preprocess Lab
# RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_xarm6 -- --reset-time --light-temp 6500,40,-3 --both --closest --seed 509 --disease-filter 1,2,3,4,6,7,8,9 --split-height-leaf 75,16 --preprocess Lab

# RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_xarm6 -- --reset-time --light-temp 6500,40,-3 --both --closest --seed 510 --disease-filter 0,2,3,4,6,7,8,9 --split-height-leaf 75,16,125,8 --preprocess Lab
# RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_xarm6 -- --reset-time --light-temp 6500,40,-3 --both --closest --seed 511 --disease-filter 1,2,3,4,6,7,8,9 --split-height-leaf 75,16,125,8 --preprocess Lab

# RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_xarm6 -- --reset-time --light-temp 10000,40,-3 --both --closest --seed 506 --disease-filter 0,2,3,4,6,7,8,9 --split-height-leaf 300,16 --preprocess Lab
# RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_xarm6 -- --reset-time --light-temp 10000,40,-3 --both --closest --seed 507 --disease-filter 1,2,3,4,6,7,8,9 --split-height-leaf 300,16 --preprocess Lab

# RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_xarm6 -- --reset-time --light-temp 10000,40,-3 --both --closest --seed 508 --disease-filter 0,2,3,4,6,7,8,9 --split-height-leaf 75,16 --preprocess Lab
# RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_xarm6 -- --reset-time --light-temp 10000,40,-3 --both --closest --seed 509 --disease-filter 1,2,3,4,6,7,8,9 --split-height-leaf 75,16 --preprocess Lab

# RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_xarm6 -- --reset-time --light-temp 10000,40,-3 --both --closest --seed 510 --disease-filter 0,2,3,4,6,7,8,9 --split-height-leaf 75,16,125,8 --preprocess Lab
# RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_xarm6 -- --reset-time --light-temp 10000,40,-3 --both --closest --seed 511 --disease-filter 1,2,3,4,6,7,8,9 --split-height-leaf 75,16,125,8 --preprocess Lab

# RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_xarm6 -- --reset-time --light-temp 0,0,1 --both --closest --seed 506 --disease-filter 0,2,3,4,6,7,8,9 --split-height-leaf 300,16 --preprocess Lab
# RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_xarm6 -- --reset-time --light-temp 0,0,1 --both --closest --seed 507 --disease-filter 1,2,3,4,6,7,8,9 --split-height-leaf 300,16 --preprocess Lab

# RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_xarm6 -- --reset-time --light-temp 0,0,1 --both --closest --seed 508 --disease-filter 0,2,3,4,6,7,8,9 --split-height-leaf 75,16 --preprocess Lab
# RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_xarm6 -- --reset-time --light-temp 0,0,1 --both --closest --seed 509 --disease-filter 1,2,3,4,6,7,8,9 --split-height-leaf 75,16 --preprocess Lab

# RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_xarm6 -- --reset-time --light-temp 0,0,1 --both --closest --seed 510 --disease-filter 0,2,3,4,6,7,8,9 --split-height-leaf 75,16,125,8 --preprocess Lab
# RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_xarm6 -- --reset-time --light-temp 0,0,1 --both --closest --seed 511 --disease-filter 1,2,3,4,6,7,8,9 --split-height-leaf 75,16,125,8 --preprocess Lab