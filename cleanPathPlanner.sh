#/bin/bash
ssh admin@10.43.29.2 "
    if [ -e "/home/lvuser/deploy/pathplanner/autos/*" ]; then
        rm "/home/lvuser/deploy/pathplanner/autos/*
    fi
"
