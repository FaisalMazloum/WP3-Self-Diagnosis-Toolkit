# #!/bin/bash


# # Accept session name as argument, or use default
# SESSION_NAME="${1:-remora_unity_session}"
# # Start or attach to named tmux session
# tmux new-session -d -s "$SESSION_NAME" || tmux attach-session -t "$SESSION_NAME"


# # 1. Absolute Paths - Double check these exist!
# ROS_SETUP="source /opt/ros/jazzy/setup.bash"
# WS_SETUP="source /home/$USER/remora_ws/install/setup.bash"
# FULL_SETUP="$ROS_SETUP && $WS_SETUP"
# SESSION="remora_swarm"

# COOLDOWN=30 # 10 seconds original, but added 20s for cleanup

# # 2. Kill everything before starting (The Reset)
# tmux kill-session -t $SESSION 2>/dev/null
# pkill -f "remora_controller"
# pkill -f "ros2 topic pub"

# # 3. Create the session in the background
# tmux new-session -d -s $SESSION

# # --- PANE 0: TCP ENDPOINT ---
# tmux send-keys -t $SESSION "$FULL_SETUP" C-m
# tmux send-keys -t $SESSION "while true; do sleep 10; echo '--- Starting TCP ---'; timeout --signal=SIGINT 40s ros2 launch remora_controller tcp_endpoint.launch.py; echo '--- Cooling Down ---'; sleep $COOLDOWN; done" C-m

# # --- PANE 1: CONTROLLER (Right) ---
# tmux split-window -h -t $SESSION
# tmux send-keys -t $SESSION "$FULL_SETUP" C-m
# tmux send-keys -t $SESSION "while true; do sleep 15; echo '--- Starting Controller ---'; timeout --signal=SIGINT 35s ros2 launch remora_controller swarm_controller.launch.py robot_count:=10 pattern:=aggregation_pattern; sleep $COOLDOWN; done" C-m

# # --- PANE 2: PUBLISHER (Bottom Left) ---
# tmux select-pane -t 0
# tmux split-window -v -t $SESSION
# tmux send-keys -t $SESSION "$FULL_SETUP" C-m
# tmux send-keys -t $SESSION "while true; do sleep 20; echo '--- Publishing ---'; timeout --signal=SIGINT 30s ros2 topic pub /swarm_command communication_interfaces/msg/Int8Message '{data: 1}'; sleep $COOLDOWN; done" C-m

# # --- PANE 3: MONITOR (Bottom Right) ---
# tmux select-pane -t 1
# tmux split-window -v -t $SESSION
# tmux send-keys -t $SESSION "echo 'System Monitor Active'; bash" C-m

# # 4. Attach to the session
# tmux attach-session -t $SESSION



#!/bin/bash

SESSION_NAME="${1:-remora_unity_session}"
ROBOT_COUNT="${2:-10}"
# Number of DDS participants estimated from robot count
DDS_PARTICIPANTS=$((2 * ROBOT_COUNT + 20))

# Generated Fast DDS configuration
DDS_XML="$HOME/remora_localhost_dynamic.xml"

# Generate Fast DDS configuration dynamically
cat > "$DDS_XML" <<EOF
<?xml version="1.0" encoding="UTF-8"?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">

    <transport_descriptors>
        <transport_descriptor>
            <transport_id>remora_localhost_udp</transport_id>
            <type>UDPv4</type>

            <interfaceWhiteList>
                <address>127.0.0.1</address>
            </interfaceWhiteList>

            <maxInitialPeersRange>${DDS_PARTICIPANTS}</maxInitialPeersRange>
        </transport_descriptor>
    </transport_descriptors>

    <participant profile_name="remora_localhost"
                 is_default_profile="true">
        <rtps>
            <builtin>
                <initialPeersList>
                    <locator>
                        <udpv4>
                            <address>127.0.0.1</address>
                        </udpv4>
                    </locator>
                </initialPeersList>

                <metatrafficUnicastLocatorList>
                    <locator/>
                </metatrafficUnicastLocatorList>
            </builtin>

            <useBuiltinTransports>false</useBuiltinTransports>

            <userTransports>
                <transport_id>remora_localhost_udp</transport_id>
            </userTransports>
        </rtps>
    </participant>
</profiles>
EOF

echo "REMORA: $ROBOT_COUNT robots"
echo "DDS discovery range: $DDS_PARTICIPANTS"

tmux new-session -d -s "$SESSION_NAME" || tmux attach-session -t "$SESSION_NAME"

ROS_SETUP="source /opt/ros/jazzy/setup.bash"
WS_SETUP="source /home/$USER/remora_ws/install/setup.bash"

FULL_SETUP="$ROS_SETUP && $WS_SETUP && \
export ROS_AUTOMATIC_DISCOVERY_RANGE=SYSTEM_DEFAULT && \
export FASTRTPS_DEFAULT_PROFILES_FILE=$DDS_XML"
SESSION="remora"

tmux kill-session -t $SESSION 2>/dev/null
pkill -f "remora_controller"
pkill -f "ros2 topic pub"

tmux new-session -d -s $SESSION
tmux set-option -t $SESSION pane-border-status top
tmux set-option -t $SESSION pane-border-format ' #{pane_index}: #{pane_title} '

# --- PANE 0: TCP ENDPOINT ---
tmux select-pane -t $SESSION -T "TCP Endpoint"
tmux send-keys -t $SESSION "$FULL_SETUP" C-m
# tmux send-keys -t $SESSION "sleep 10; echo '--- Starting TCP ---'; ros2 launch remora_controller tcp_endpoint.launch.py" C-m
# tmux send-keys -t $SESSION "sleep 10; echo '--- Starting TCP ---'; ros2 launch remora_controller tcp_endpoint.launch.py ip:=192.168.8.188" C-m
# tmux send-keys -t $SESSION "sleep 0; echo '--- Starting TCP ---'; ros2 launch remora_controller tcp_endpoint.launch.py" C-m
# tmux send-keys -t $SESSION "sleep 0; echo '--- Starting TCP ---'; ros2 launch $HOME/remora_ws/ros2_launch/sw_simulation/tcp_endpoint.launch.py" C-m
tmux send-keys -t $SESSION "sleep 0; echo '--- Starting TCP ---'; ros2 launch $HOME/remora_ws/ros2_launch/sw_simulation/tcp_endpoint.launch.py" C-m

# --- PANE 1: CONTROLLER (Right) ---
tmux split-window -h -t $SESSION
tmux select-pane -t $SESSION -T "Controller"
tmux send-keys -t $SESSION "$FULL_SETUP" C-m
# tmux send-keys -t $SESSION "sleep 15; echo '--- Starting Controller ---'; ros2 launch remora_controller swarm_controller.launch.py robot_count:=10 pattern:=adaptive_sampling_pattern" C-m
# tmux send-keys -t $SESSION "sleep 15; echo '--- Starting Controller ---'; ros2 launch remora_controller swarm_controller_timed_switch.launch.py robot_count:=10 pattern_a:=dispersion_pattern pattern_b:=aggregation_pattern switch_duration:=30.0" C-m
# tmux send-keys -t $SESSION "sleep 15; echo '--- Starting Controller ---'; ros2 launch remora_controller swarm_controller_staggered_switch.launch.py robot_count:=10 pattern_a:=random_walk_pattern pattern_b:=aggregation_pattern switch_duration:=30.0 stagger_duration:=15.0" C-m
# tmux send-keys -t $SESSION "sleep 0; echo '--- Starting Controller ---'; ros2 launch remora_controller swarm_controller_dynamic_switch.launch.py robot_count:=${ROBOT_COUNT} initial_pattern:=drive_pattern" C-m
# tmux send-keys -t $SESSION "sleep 0; echo '--- Starting Controller ---'; ros2 launch $HOME/remora_ws/ros2_launch/sw_system/swarm_controller_dynamic_switch.launch.py robot_id:=remora0 robot_count:=${ROBOT_COUNT} initial_pattern:=drive_pattern" C-m
tmux send-keys -t $SESSION "sleep 0; echo '--- Starting Controller ---'; ros2 launch $HOME/remora_ws/ros2_launch/sw_system/remora_controller/swarm_controller_dynamic_switch.launch.py robot_id:=central robot_count:=${ROBOT_COUNT} initial_pattern:=drive_pattern" C-m
# tmux send-keys -t $SESSION "sleep 0; echo '--- Starting Controller ---'; ros2 launch $HOME/remora_ws/ros2_launch/sw_system/remora_controller/swarm_controller_dynamic_switch.launch.py robot_id:=central" C-m

# --- PANE 2: PUBLISHER (Bottom Left) ---
tmux select-pane -t $SESSION:0.0
tmux split-window -v -t $SESSION
tmux select-pane -t $SESSION -T "Publisher"
tmux send-keys -t $SESSION "$FULL_SETUP" C-m
tmux send-keys -t $SESSION "sleep 20; echo '--- Publishing ---'; ros2 topic pub /swarm_command communication_interfaces/msg/Int8Message '{data: 1}'" C-m

# # --- PANE 3: GNSS Range Sensor ---
# tmux split-window -v -t $SESSION
# tmux select-pane -t $SESSION -T "GNSS Range Sensor"
# tmux send-keys -t $SESSION "$FULL_SETUP" C-m
# tmux send-keys -t $SESSION "sleep 0; echo '--- Starting GNSS to Laserscan ---'; ros2 launch remora_controller gnss_range_sensor.launch.py robot_count:=${ROBOT_COUNT}" C-m

# --- PANE 4: MONITOR (Bottom Left) ---
tmux split-window -v -t $SESSION
tmux select-pane -t $SESSION -T "Monitor"
tmux send-keys -t $SESSION "echo 'System Monitor Active'; bash" C-m


# Arrange all panes evenly
tmux select-layout -t $SESSION tiled

tmux attach-session -t $SESSION
