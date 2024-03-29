#!/bin/bash

setup_bash_path='/home/antoine/workspace/catkin_ws/devel/setup.bash'

session="self-racing-cars-all"

tmux new-session -d -s $session


# splitting the pane 0 vertically
tmux split-window -t 0 -v
# splitting the pane 0 vertically
tmux split-window -t 0 -v
# splitting the pane 0 vertically
tmux split-window -t 0 -v
tmux select-layout even-vertical
# splitting the pane 0 horizontally
tmux split-window -t 0 -h
# splitting the pane 2 horizontally
tmux split-window -t 2 -h
# splitting the pane 4 horizontally
tmux split-window -t 4 -h
# splitting the pane 6 horizontally
tmux split-window -t 6 -h


# roscore
tmux select-pane -t 0
tmux send-keys 'roscore' C-m
# sleeping 10 seconds so that roscore has time to start
echo "Waiting 10 seconds for roscore to start..."
sleep 10

# gps publisher
tmux select-pane -t 1
tmux send-keys "cd '/home/antoine/workspace/catkin_ws/src/autonomous_software_pkg/src'" C-m
tmux send-keys "source $setup_bash_path" C-m
tmux send-keys "rosrun autonomous_software_pkg gps_publisher.py" C-m

# vehicle state publisher
tmux select-pane -t 2
tmux send-keys "cd '/home/antoine/workspace/catkin_ws/src/autonomous_software_pkg/src'" C-m
tmux send-keys "source $setup_bash_path" C-m
tmux send-keys "rosrun autonomous_software_pkg vehicle_state_publisher.py" C-m

# pane for the controller
tmux select-pane -t 3
tmux send-keys "cd '/home/antoine/workspace/catkin_ws/src/autonomous_software_pkg/src'" C-m
tmux send-keys "source $setup_bash_path" C-m
tmux send-keys "rosrun autonomous_software_pkg controller.py" C-m

# rosserial arduino
tmux select-pane -t 4
tmux send-keys "cd '/home/antoine/workspace/catkin_ws/src/autonomous_software_pkg/src'" C-m
tmux send-keys "source $setup_bash_path" C-m
tmux send-keys "rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 _baud:=57600" C-m

# rostopic echo /arduino_logging
tmux select-pane -t 5
tmux send-keys "cd '/home/antoine/workspace/catkin_ws/src/autonomous_software_pkg/src'" C-m
tmux send-keys "source $setup_bash_path" C-m
tmux send-keys "rostopic echo /arduino_logging" C-m

# attaching the session
tmux attach-session -t $session
