#!/bin/bash

~/projects/v4l2-capture-test/hdmi-setup.sh
sleep 3
~/projects/v4l2-capture-test/setup-uvc.sh stop
sleep 3
~/projects/v4l2-capture-test/setup-uvc.sh stop
sleep 3
#uvc-setup.sh start lowresall
#uvc-setup.sh start mjpeglowres
#uvc-setup.sh start yuyvlowres
~/projects/v4l2-capture-test/setup-uvc.sh start yuyv
#setup-uvc.sh start yuyvhighres
#uvc-setup.sh start mjpeg
#uvc-setup.sh start all
sleep 3
(uvc-gadget uvc.0) &
sleep 3
~/projects/v4l2-capture-test/bin/ARM/Debug/v4l2-capture-test /dev/video1 1280 720