





for i in $(seq 1 10);
do
    RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run benchbot_xarm6_cpp benchbot_xarm6_cpp \
    -- --light-temp 6500,0.3,1 --both --closest --bench-sample-gap 1 --arm_random_locations 0 --arm-sample-gap 40 --pred --pcg-seed-incr 1

    RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run benchbot_xarm6_cpp benchbot_xarm6_cpp \
    -- --light-temp 6500,0.3,1 --both --closest --bench-sample-gap 1 --arm_random_locations 0 --arm-sample-gap 40 --pcg-seed-incr 0 
done
for i in $(seq 1 10);
do
    RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run benchbot_xarm6_cpp benchbot_xarm6_cpp \
    -- --light-temp 6500,0.3,1 --both --closest --bench-sample-gap 1 --arm_random_locations 0 --arm-sample-gap 20 --pred --pcg-seed-incr 1

    RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run benchbot_xarm6_cpp benchbot_xarm6_cpp \
    -- --light-temp 6500,0.3,1 --both --closest --bench-sample-gap 1 --arm_random_locations 0 --arm-sample-gap 20 --pcg-seed-incr 0 
done
for i in $(seq 1 10);
do
    RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run benchbot_xarm6_cpp benchbot_xarm6_cpp \
    -- --light-temp 6500,0.3,1 --both --closest --bench-sample-gap 1 --arm_random_locations 0 --arm-sample-gap 15 --pred --pcg-seed-incr 1

    RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run benchbot_xarm6_cpp benchbot_xarm6_cpp \
    -- --light-temp 6500,0.3,1 --both --closest --bench-sample-gap 1 --arm_random_locations 0 --arm-sample-gap 15 --pcg-seed-incr 0 
done
for i in $(seq 1 10);
do
    RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run benchbot_xarm6_cpp benchbot_xarm6_cpp \
    -- --light-temp 6500,0.3,1 --both --closest --bench-sample-gap 1 --arm_random_locations 0 --arm-sample-gap 11 --pred --pcg-seed-incr 1

    RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run benchbot_xarm6_cpp benchbot_xarm6_cpp \
    -- --light-temp 6500,0.3,1 --both --closest --bench-sample-gap 1 --arm_random_locations 0 --arm-sample-gap 11 --pcg-seed-incr 0 
done
for i in $(seq 1 10);
do
    RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run benchbot_xarm6_cpp benchbot_xarm6_cpp \
    -- --light-temp 6500,0.3,1 --both --closest --bench-sample-gap 1 --arm_random_locations 0 --arm-sample-gap 5 --pred --pcg-seed-incr 1

    RCUTILS_LOGGING_USE_ROSOUT=1 RCUTILS_LOGGING_BUFFERED_STREAM=1 ros2 run benchbot_xarm6_cpp benchbot_xarm6_cpp \
    -- --light-temp 6500,0.3,1 --both --closest --bench-sample-gap 1 --arm_random_locations 0 --arm-sample-gap 5 --pcg-seed-incr 0 
done