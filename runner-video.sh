#!/bin/bash


cd /root/v4l2-video-capture-testing-program
/root/v4l2-video-capture-testing-program/v4l2-capture-test --devices /dev/video0 --port 1337 --fps 60 --mjpeg --bgr --jpeg-quality 50 --encode-threads 12 --lazy --lazy-threshold 600

exit 0
