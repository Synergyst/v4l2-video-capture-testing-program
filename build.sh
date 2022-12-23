#/bin/bash

g++ -o ~/projects/v4l2-video-capture-testing-program/bin/ARM/Debug/v4l2-video-capture-testing-program -std=c++20 -lv4l2 -fopenmp v4l2-video-capture-testing-program.cpp v4l2cap.cpp v4l2capalt.cpp
if [[ $? -eq 0 ]]; then
  echo "Success"
  exit 0
else
  echo "Build failed"
  exit 1
fi