RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_xarm6 -- --reset-time --light-temp 3000 --both --seed 0
for i in $(seq 1 11);
do
    echo $i
    #ros2 run benchbot_xarm6_cpp benchbot_xarm6_cpp
    #ros2 run benchbot_xarm6_stereo benchbot_xarm6_stereo
    RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_xarm6 -- --light-temp 3000 --both --seed $i
done

RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_xarm6 -- --reset-time --light-temp 6500 --both --seed 12
for i in $(seq 1 11);
do
    echo $((12+i))
    #ros2 run benchbot_xarm6_cpp benchbot_xarm6_cpp
    #ros2 run benchbot_xarm6_stereo benchbot_xarm6_stereo
    RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_xarm6 -- --light-temp 6500 --both --seed $((12+i))
done

RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_xarm6 -- --reset-time --light-temp 10000 --both --seed 24
for i in $(seq 1 11);
do
    echo $((24+i))
    #ros2 run benchbot_xarm6_cpp benchbot_xarm6_cpp
    #ros2 run benchbot_xarm6_stereo benchbot_xarm6_stereo
    RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run tomato_xarm6 tomato_xarm6 -- --light-temp 10000 --both --seed $((24+i))
done