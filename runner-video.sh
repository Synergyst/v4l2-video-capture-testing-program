#!/bin/bash


cd /root/v4l2-video-capture-testing-program
#/root/v4l2-video-capture-testing-program/v4l2-capture-test --devices /dev/video0 --port 1337 --fps 60 --mjpeg --bgr --jpeg-quality 60 --encode-threads 2 --no-lazy --lazy-threshold 600
#/root/v4l2-video-capture-testing-program/v4l2-capture-test --devices /dev/video0 --port 1337 --fps 60 --mjpeg --bgr --jpeg-quality 60 --encode-threads 2 --no-lazy
/root/v4l2-video-capture-testing-program/v4l2-capture-test --devices /dev/video1 --port 1337 --bgr

exit 0
