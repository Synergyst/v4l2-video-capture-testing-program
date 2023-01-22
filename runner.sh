#!/bin/bash

~/projects/v4l2-capture-test/hdmi-setup.sh >/dev/null 2>&1
sleep 3
~/projects/v4l2-capture-test/setup-uvc.sh stop >/dev/null 2>&1
sleep 3
~/projects/v4l2-capture-test/setup-uvc.sh stop >/dev/null 2>&1
sleep 3
#uvc-setup.sh start lowresall
#uvc-setup.sh start mjpeglowres
#uvc-setup.sh start yuyvlowres
~/projects/v4l2-capture-test/setup-uvc.sh start yuyv >/dev/null 2>&1
#setup-uvc.sh start yuyvhighres
#uvc-setup.sh start mjpeg
#uvc-setup.sh start all
sleep 3
(uvc-gadget uvc.0 >/dev/null 2>&1) &
sleep 3
#~/projects/v4l2-capture-test/bin/ARM/Debug/v4l2-capture-test /dev/video1 1280 720