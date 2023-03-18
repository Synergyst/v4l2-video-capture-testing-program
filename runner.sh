#!/bin/bash

if pidof -x "v4l2-capture-test" >/dev/null; then
  killall v4l2-capture-test
#  killall ffmpeg
  if pidof -x "v4l2-capture-test" >/dev/null; then
    sleep 5
    killall v4l2-capture-test
#    killall ffmpeg
  fi
fi
~/projects/v4l2-capture-test/hdmi-setup.sh >/dev/null 2>&1
sleep 15
~/projects/v4l2-capture-test/hdmi-setup.sh >/dev/null 2>&1
#~/projects/v4l2-capture-test/setup-uvc.sh stop >/dev/null 2>&1
#sleep 1
#~/projects/v4l2-capture-test/setup-uvc.sh stop >/dev/null 2>&1
#sleep 1
#uvc-setup.sh start lowresall
#uvc-setup.sh start mjpeglowres
#uvc-setup.sh start yuyvlowres
#~/projects/v4l2-capture-test/setup-uvc.sh start yuyv >/dev/null 2>&1
#~/projects/v4l2-capture-test/setup-uvc.sh start mjpeg >/dev/null 2>&1
#setup-uvc.sh start yuyvhighres
#uvc-setup.sh start mjpeg
#uvc-setup.sh start all
#sleep 1
#if pidof -x "uvc-gadget" >/dev/null; then
#  echo "uvc-gadget is already running"
#else
#  (uvc-gadget uvc.0 >/dev/null 2>&1) &
#fi
#sleep 3
#~/projects/v4l2-capture-test/bin/ARM/Debug/v4l2-capture-test /dev/video1 1280 720
#~/projects/v4l2-capture-test/bin/ARM/Debug/v4l2-capture-test /dev/video1 1280 720 |ffmpeg -hide_banner -loglevel error -f rawvideo -pixel_format yuyv422 -video_size 1280x720 -i pipe:0 -f v4l2 -c:v copy /dev/video2
~/projects/v4l2-capture-test/bin/ARM/Debug/v4l2-capture-test /dev/video0 /dev/video1 /dev/fb0