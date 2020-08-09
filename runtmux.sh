#/bin/bash
cd /home/upsquared/landrov
export PATH="/home/upsquared/miniconda3/bin:$PATH"
tmux kill-session -t landrov
tmux new-session -d -s landrov

tmux split-window -h
tmux select-pane -t 0
tmux send-keys "printf '\033]2;controler\033\\'" ENTER
tmux send-keys "python controler.py" ENTER
tmux split-window -v
tmux send-keys "printf '\033]2;video_server.py\033\\'" ENTER
tmux send-keys "python video_server.py" ENTER
tmux select-pane -t 2
tmux send-keys "cd web/html && python ../webserver.py" ENTER
tmux split-window -v
tmux send-keys "python web/websoc_server.py" ENTER




