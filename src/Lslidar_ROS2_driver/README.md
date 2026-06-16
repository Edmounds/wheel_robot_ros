ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '/home/cqc/cartographer/maps/n10p.pbstream', include_unfinished_submaps: true}"


ros2 launch cartographer_config cartographer_localization.launch.py \
load_state_filename:=/home/cqc/cartographer/maps/n10p.pbstream \
load_frozen_state:=true
