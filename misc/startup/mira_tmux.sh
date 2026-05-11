#!/bin/bash

############################################################
# HELPERS
############################################################

wait_for_prompt() {
    local pane="$1"
    local pattern="$2"
    local timeout="${3:-60}"
    local elapsed=0

    while ! tmux capture-pane -p -t "$pane" | grep -qE "$pattern"; do
        sleep 1
        elapsed=$((elapsed + 1))

        if [ "$elapsed" -ge "$timeout" ]; then
            echo "Timeout waiting for '$pattern' on $pane" >&2
            return 1
        fi
    done
}

############################################################
# CREATE ONBOARD SESSION IF IT DOESN'T EXIST
############################################################

if tmux has-session -t mira_onboard 2>/dev/null; then
    echo "mira_onboard already exists"

else

    ########################################################
    # CREATE SESSION
    ########################################################

    tmux new-session -d -s mira_onboard

    ########################################################
    # CREATE 4-PANE LAYOUT
    ########################################################

    tmux split-window -h -t mira_onboard
    tmux split-window -v -t mira_onboard:0.0
    tmux split-window -v -t mira_onboard:0.1

    tmux select-layout -t mira_onboard tiled

    ########################################################
    # PANES USED FOR SSH
    ########################################################

    PANES=(0.0 0.1 0.2)

    ########################################################
    # SSH INTO DEVICE
    ########################################################

    for pane in "${PANES[@]}"; do

        tmux send-keys -t "mira_onboard:${pane}" \
        'sshpass -p "root" ssh -o StrictHostKeyChecking=no -X dntorin@192.168.2.6' C-m

    done

    ########################################################
    # WAIT FOR SSH PROMPTS
    ########################################################

    for pane in "${PANES[@]}"; do

        wait_for_prompt \
        "mira_onboard:${pane}" \
        'dntorin@[^ ]*:[^$]*\$' \
        30

    done

    ########################################################
    # ENTER DOCKER
    ########################################################

    for pane in "${PANES[@]}"; do

        tmux send-keys -t "mira_onboard:${pane}" \
        "cd mira && make docker" C-m

    done

    ########################################################
    # WAIT FOR DOCKER PROMPTS
    ########################################################

    for pane in "${PANES[@]}"; do

        wait_for_prompt \
        "mira_onboard:${pane}" \
        'root@[^ ]*:/workspace#' \
        120

    done

    ########################################################
    # RUN TARGETS
    ########################################################

    # ALT MASTER
    tmux send-keys -t mira_onboard:0.0 \
    "make alt_master" C-m

    # ZED
    tmux send-keys -t mira_onboard:0.1 \
    "make camera_zed" C-m

    # BOTTOM CAM
    tmux send-keys -t mira_onboard:0.2 \
    "make camera_bottomcam" C-m

fi

############################################################
# CREATE TELEOP SESSION IF IT DOESN'T EXIST
############################################################

if tmux has-session -t mira_teleop 2>/dev/null; then

    echo "mira_teleop already exists"

else

    tmux new-session -d -s mira_teleop

    tmux send-keys -t mira_teleop \
    "cd ~/DNT/mira && make teleop" C-m

fi