#!/bin/bash

export CYCLONEDDS_URI=file://$HOME/cyclone_config.xml
export ROS_DOMAIN_ID=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

case $1 in
  1)
    cd ~/ardupilot
    source ~/venv-ardupilot/bin/activate
    python3 Tools/autotest/sim_vehicle.py \
      -v ArduCopter \
      --no-rebuild \
      -I 0 \
      --no-wsl2-network \
      --add-param-file=$HOME/ardupilot/sitl_params.parm
    ;;
  2)
    source /opt/ros/jazzy/setup.bash
    source ~/ros2_px/install/setup.bash
    export ROS_DOMAIN_ID=1
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    export CYCLONEDDS_URI=file://$HOME/cyclone_config.xml
    ros2 run mavros mavros_node \
      --ros-args \
      -p fcu_url:=udp://127.0.0.1:14550@14555 \
      -p target_system_id:=1 \
      -p target_component_id:=1 \
      -p system_id:=255 \
      --log-level mavros:=WARN &
    sleep 10
    ros2 service call /mavros/set_stream_rate \
      mavros_msgs/srv/StreamRate \
      "{stream_id: 0, message_rate: 50, on_off: true}"
    wait
    ;;
  3)
    source /opt/ros/jazzy/setup.bash
    source ~/ros2_px/install/setup.bash
    ros2 launch drone_mission sitl.launch.py \
      run_label:=${2:-sitl_test} \
      shaper_type:=${3:-none} \
      rope_length:=${4:-1.0}
    ;;
  4)
    source /opt/ros/jazzy/setup.bash
    source ~/ros2_px/install/setup.bash
    echo "Waiting 20s for SITL to initialize..."
    sleep 20
    echo "Setting GUIDED mode..."
    ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'GUIDED'}"
    sleep 2
    echo "Arming..."
    ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"
    sleep 2
    echo "Taking off..."
    ros2 service call /mavros/cmd/takeoff mavros_msgs/srv/CommandTOL \
      "{altitude: 10.0, min_pitch: 0.0, yaw: 0.0, latitude: 0.0, longitude: 0.0}"
    sleep 15
    echo "Sending start signal..."
    ros2 topic pub /mission/start std_msgs/msg/String "{data: start}" --once
    ;;
esac
