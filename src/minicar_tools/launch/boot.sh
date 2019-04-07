#!/usr/bin/env bash

roscore &
sleep 3
rostopic pub /test std_msgs/String "data: 'Hello'"  -r 10 &
disown
