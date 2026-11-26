#!/bin/bash
# Starts this robot's swarm software in a tmux session. Run on every robot.
#
#   ./remora_robot.sh
#
# The robot runs only itself: robot_id comes from ros2_configs/.../system_config.yaml
# ('auto' = this machine's hostname, e.g. remora3), so the same script works on every robot.
#
# All robots and the operator laptop must be on the same network and use the same
# ROS_DOMAIN_ID (override with: ROS_DOMAIN_ID=5 ./remora_robot.sh).
#
# Start / stop the swarm from the operator laptop (not from the robots):
#   start: ros2 topic pub /swarm_command communication_interfaces/msg/Int8Message '{data: 1}'
#          (keep it running: patterns started by a /pattern_command switch wait for it again)
#   stop:  Ctrl-C the publisher above, then
#          ros2 topic pub --once /swarm_command communication_interfaces/msg/Int8Message '{data: 0}'
#   switch a robot: ros2 topic pub --once /pattern_command std_msgs/msg/String '{data: remora3:random_walk_pattern}'

SESSION="remora"
WS="$HOME/remora_ws"
DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

# Network discovery across the subnet (the Unity script restricted it to localhost).
# FASTRTPS_DEFAULT_PROFILES_FILE is cleared in case a localhost profile is set elsewhere.
FULL_SETUP="source /opt/ros/jazzy/setup.bash && source $WS/install/setup.bash && \
export ROS_DOMAIN_ID=$DOMAIN_ID && \
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET && \
unset FASTRTPS_DEFAULT_PROFILES_FILE"

# Clean up a previous run, including pattern processes left behind by the supervisor.
tmux kill-session -t "$SESSION" 2>/dev/null
pkill -f "remora_controller" 2>/dev/null
pkill -f "lib/ros2swarm/" 2>/dev/null

echo "REMORA: starting on $(hostname), ROS_DOMAIN_ID=$DOMAIN_ID"

tmux new-session -d -s "$SESSION"
tmux set-option -t "$SESSION" pane-border-status top
tmux set-option -t "$SESSION" pane-border-format ' #{pane_index}: #{pane_title} '

# --- PANE 0: CONTROLLER ---
# Hardware protection, pattern supervisor + status publisher, GNSS range sensor, all for this robot.
tmux select-pane -t "$SESSION:0.0" -T "Controller"
tmux send-keys -t "$SESSION:0.0" "$FULL_SETUP" C-m
tmux send-keys -t "$SESSION:0.0" "ros2 launch $WS/ros2_launch/sw_system/remora_controller/swarm_controller_dynamic_switch.launch.py" C-m

# --- PANE 1: HARDWARE DRIVERS (placeholder) ---
# GNSS, IMU and motor drivers go here once their launch files exist in ros2_launch/hw_drivers/.
# The controller needs <ns>/gnss and <ns>/imu, and its velocity output must reach the motors.
# tmux split-window -h -t "$SESSION"
# tmux select-pane -t "$SESSION" -T "Drivers"
# tmux send-keys -t "$SESSION" "$FULL_SETUP" C-m
# tmux send-keys -t "$SESSION" "ros2 launch $WS/ros2_launch/hw_drivers/<drivers>.launch.py" C-m

# --- PANE 2: MONITOR ---
tmux split-window -h -t "$SESSION"
tmux select-pane -t "$SESSION" -T "Monitor"
tmux send-keys -t "$SESSION" "$FULL_SETUP" C-m

tmux select-layout -t "$SESSION" tiled

# Attach only when run from a terminal (e.g. over SSH), so it also works from a startup service.
if [ -t 1 ]; then
    tmux attach-session -t "$SESSION"
else
    echo "REMORA: running in tmux session '$SESSION' (attach with: tmux attach -t $SESSION)"
fi
