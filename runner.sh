#!/bin/bash

if pidof -x "v4l2-capture-test" >/dev/null; then
  killall v4l2-capture-test
  if pidof -x "v4l2-capture-test" >/dev/null; then
    sleep 5
    killall v4l2-capture-test
  fi
fi
~/v4l2-video-capture-testing-program/hdmi-setup.sh >/dev/null 2>&1
sleep 15
~/v4l2-video-capture-testing-program/hdmi-setup.sh >/dev/null 2>&1
sleep 2
# Change below if you want to capture from more than one device!
~/v4l2-video-capture-testing-program/bin/ARM/Debug/v4l2-capture-test /dev/video0 /dev/fb0
#~/projects/v4l2-capture-test/bin/ARM/Debug/v4l2-capture-test /dev/video0 /dev/video1 /dev/fb0
