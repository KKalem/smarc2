#! /bin/bash
ROBOT_NAME=floatsam_usv
SESSION=${ROBOT_NAME}_bringup
USE_SIM_TIME=true

# WASP / MQTT settings
AGENT_TYPE=subsurface
PULSE_RATE=0.5
CONTEXT=tuper
BT_LOG_MODE=compact
DOMAIN=surface

if [ "$USE_SIM_TIME" = "true" ]; then
    REALSIM=simulation
    LINK_SUFFIX="_gt"
else
    REALSIM=real
    LINK_SUFFIX=""
fi

# --- Vehicle health publisher (simulation uses publisher) ---
tmux -2 new-session -d -s $SESSION -n 'vehicle_health'
tmux select-window -t $SESSION:0
if [ "$REALSIM" = "real" ]; then
    tmux send-keys "sleep 5; echo 'In real mode: launch vehicle health checker if available'" C-m
else
    tmux send-keys "ros2 topic pub -r 1 /$ROBOT_NAME/smarc/vehicle_health std_msgs/msg/Int8 '{data: 0}'" C-m
fi

# --- MQTT bridge ---
tmux new-window -t $SESSION:1 -n 'mqtt_bridge'
tmux select-window -t $SESSION:1
tmux send-keys "sleep 3; ros2 launch str_json_mqtt_bridge waraps_bridge.launch broker_addr:=20.240.40.232 broker_port:=1884 robot_name:=$ROBOT_NAME domain:=$DOMAIN realsim:=$REALSIM use_sim_time:=$USE_SIM_TIME context:=$CONTEXT" C-m

# --- Topic bridge for floatsam ---
tmux new-window -t $SESSION:2 -n 'topic_bridge'
tmux select-window -t $SESSION:2
tmux send-keys "sleep 3; ros2 launch floatsam_topic_bridge floatsam_bridge.launch.py" C-m

# --- Controllers window (main controllers + description if any) ---
tmux new-window -t $SESSION:3 -n 'controllers'
tmux select-window -t $SESSION:3
tmux select-pane -t $SESSION:3.0
tmux split-window -v -t $SESSION:3.0
tmux select-layout -t $SESSION:3 tiled
tmux select-pane -t $SESSION:3.0
tmux send-keys "sleep 2; ros2 launch floatsam_controllers floatsam_controllers_launch.py" C-m

# --- Servers / action servers ---
tmux new-window -t $SESSION:4 -n 'servers'
tmux select-window -t $SESSION:4
tmux select-pane -t $SESSION:4.0
tmux split-window -h -t $SESSION:4.0
tmux select-pane -t $SESSION:4.0
tmux send-keys "sleep 4; ros2 launch floatsam_move_to floatsam_move_to.launch.py" C-m


# --- Behavior tree (WASP BT) ---
tmux new-window -t $SESSION:5 -n 'bt'
tmux select-window -t $SESSION:5
tmux send-keys "sleep 2; ros2 launch wasp_bt wasp_bt.launch robot_name:=$ROBOT_NAME agent_type:=$AGENT_TYPE pulse_rate:=$PULSE_RATE use_sim_time:=$USE_SIM_TIME bt_log_mode:=$BT_LOG_MODE" C-m


# Logging window.
tmux new-window -t $SESSION:7 -n 'logging'
tmux select-window -t $SESSION:7

# Set default window and attach
tmux select-window -t $SESSION:0
tmux -2 attach-session -t $SESSION
