#!/bin/bash

export DISPLAY=192.168.168.134:0.0
cd /root/v4l2-proxy/
ENABLE_SYSTEM_CMDS=1 node /root/v4l2-proxy/control-sink.js

exit 0
