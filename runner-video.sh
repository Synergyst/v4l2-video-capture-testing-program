#!/bin/bash

./v4l2-capture-test --devices /dev/video0 --port 1337 --fps 60 --mjpeg --bgr --jpeg-quality 70 --encode-threads 8 --lazy --lazy-threshold 600

exit 0
