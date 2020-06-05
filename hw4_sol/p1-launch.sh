#!/bin/bash
if [ ! "$BASH_VERSION" ] ; then
    exec /bin/bash "$0" "$@"
fi

# launch simulation and control first
./hw4-p1-sol-interfaces 1 &
HW4P1SOL_PID=$!

# trap ctrl-c and call ctrl_c()
trap ctrl_c INT

function ctrl_c() {
    kill -2 $HW4P1SOL_PID
}

sleep 2 

# launch interfaces server
python3 interface/server.py hw4-p1-interfaces.html &
SERVER_PID=$!

# wait for simviz to quit
wait $HW4P1SOL_PID

# onnce simviz dies, kill interfaces server
for pid in $(ps -ef | grep interface/server.py | awk '{print $2}'); do kill -9 $pid; done
