# only moving the robots, does not include feedback

# move benchbot camera above the weeds
ros2 topic pub /ue5/BenchBot/planar_robot_tf geometry_msgs/msg/Transform "{translation: {x: 1.0, y: 6.5, z: 0.25}, rotation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}" --once
# wait for ros image to update in RViz
sleep 2
# move spider robot above the weeds
ros2 topic pub /ue5/Spider/planar_robot_tf geometry_msgs/msg/Transform "{translation: {x: 1.5, y: 3.5, z: 0.0}, rotation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}" --once