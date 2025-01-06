# RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_xarm6 -- --reset-time --light-temp 3000 --both --seed 0
# for i in $(seq 1 11);
# do
#     echo $i
#     #ros2 run benchbot_xarm6_cpp benchbot_xarm6_cpp
#     #ros2 run benchbot_xarm6_stereo benchbot_xarm6_stereo
#     RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_xarm6 -- --light-temp 3000 --both --seed $i
# done

#RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_xarm6 -- --reset-time --light-temp 6500 --both --seed 12
# for i in $(seq 1 11);
# do
#     echo $((12+i))
#     #ros2 run benchbot_xarm6_cpp benchbot_xarm6_cpp
#     #ros2 run benchbot_xarm6_stereo benchbot_xarm6_stereo
#     RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_xarm6 -- --light-temp 6500 --both --seed $((12+i))
# done

# RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_xarm6 -- --reset-time --light-temp 10000 --both --seed 24
for i in $(seq 1 11);
do
    kill -9 $(pgrep tomato_xarm6)
    RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_xarm6 -- --reset-time --light-temp 0,0,1 --both --seed 508 --disease-filter 2 --split-height-leaf 300,16 --preprocess Lab --percent-healthy 0.1 --move-robot "[{name: BenchBot, x: 100, y: 200}, {name: Spider, x: 300, y: 350}, {name: Husky, x: 200, y: 525}]"
    kill -9 $(pgrep tomato_xarm6)
done

# RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_xarm6 -- --reset-time --light-temp 6500,40,-3 --both --seed 506 --disease-filter 0,2,3,4,6,7,8,9 --split-height-leaf 300,16 --preprocess Lab
# RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_xarm6 -- --reset-time --light-temp 6500,40,-3 --both --seed 507 --disease-filter 1,2,3,4,6,7,8,9 --split-height-leaf 300,16 --preprocess Lab

# RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_xarm6 -- --reset-time --light-temp 6500,40,-3 --both --seed 508 --disease-filter 0,2,3,4,6,7,8,9 --split-height-leaf 75,16 --preprocess Lab
# RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_xarm6 -- --reset-time --light-temp 6500,40,-3 --both --seed 509 --disease-filter 1,2,3,4,6,7,8,9 --split-height-leaf 75,16 --preprocess Lab

# RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_xarm6 -- --reset-time --light-temp 6500,40,-3 --both --seed 510 --disease-filter 0,2,3,4,6,7,8,9 --split-height-leaf 75,16,125,8 --preprocess Lab
# RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_xarm6 -- --reset-time --light-temp 6500,40,-3 --both --seed 511 --disease-filter 1,2,3,4,6,7,8,9 --split-height-leaf 75,16,125,8 --preprocess Lab

# RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_xarm6 -- --reset-time --light-temp 10000,40,-3 --both --seed 506 --disease-filter 0,2,3,4,6,7,8,9 --split-height-leaf 300,16 --preprocess Lab
# RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_xarm6 -- --reset-time --light-temp 10000,40,-3 --both --seed 507 --disease-filter 1,2,3,4,6,7,8,9 --split-height-leaf 300,16 --preprocess Lab

# RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_xarm6 -- --reset-time --light-temp 10000,40,-3 --both --seed 508 --disease-filter 0,2,3,4,6,7,8,9 --split-height-leaf 75,16 --preprocess Lab
# RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_xarm6 -- --reset-time --light-temp 10000,40,-3 --both --seed 509 --disease-filter 1,2,3,4,6,7,8,9 --split-height-leaf 75,16 --preprocess Lab

# RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_xarm6 -- --reset-time --light-temp 10000,40,-3 --both --seed 510 --disease-filter 0,2,3,4,6,7,8,9 --split-height-leaf 75,16,125,8 --preprocess Lab
# RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_xarm6 -- --reset-time --light-temp 10000,40,-3 --both --seed 511 --disease-filter 1,2,3,4,6,7,8,9 --split-height-leaf 75,16,125,8 --preprocess Lab

# RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_xarm6 -- --reset-time --light-temp 0,0,1 --both --seed 506 --disease-filter 0,2,3,4,6,7,8,9 --split-height-leaf 300,16 --preprocess Lab
# RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_xarm6 -- --reset-time --light-temp 0,0,1 --both --seed 507 --disease-filter 1,2,3,4,6,7,8,9 --split-height-leaf 300,16 --preprocess Lab

# RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_xarm6 -- --reset-time --light-temp 0,0,1 --both --seed 508 --disease-filter 0,2,3,4,6,7,8,9 --split-height-leaf 75,16 --preprocess Lab
# RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_xarm6 -- --reset-time --light-temp 0,0,1 --both --seed 509 --disease-filter 1,2,3,4,6,7,8,9 --split-height-leaf 75,16 --preprocess Lab

# RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_xarm6 -- --reset-time --light-temp 0,0,1 --both --seed 510 --disease-filter 0,2,3,4,6,7,8,9 --split-height-leaf 75,16,125,8 --preprocess Lab
# RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_xarm6 -- --reset-time --light-temp 0,0,1 --both --seed 511 --disease-filter 1,2,3,4,6,7,8,9 --split-height-leaf 75,16,125,8 --preprocess Lab