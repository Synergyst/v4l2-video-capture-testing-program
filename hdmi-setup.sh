#!/bin/bash

ls /dev/video*
v4l2-ctl --list-devices
for m in {0..1} ; do
  v4l2-ctl -d /dev/video$m --set-edid=file=`ls ~/projects/v4l2-capture-test/1080P50EDID.txt` --fix-edid-checksums
  sleep 2
  v4l2-ctl -d /dev/video$m --query-dv-timings
  sleep 1
  v4l2-ctl -d /dev/video$m --set-dv-bt-timings query
  sleep 1
  v4l2-ctl -d /dev/video$m -V
  sleep 1
  #v4l2-ctl -d /dev/video$m -v pixelformat=RGB3
  v4l2-ctl -d /dev/video$m -v pixelformat=UYVY
  sleep 2
done
for m in {0..1} ; do
  echo -n "/dev/video$m:"
  v4l2-ctl -d /dev/video$m --log-status
done