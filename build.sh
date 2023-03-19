#/bin/bash

export LD_LIBRARY_PATH=/usr/lib/aarch64-linux-gnu:$LD_LIBRARY_PATH

g++ -o ~/projects/v4l2-capture-test/bin/ARM/Debug/v4l2-capture-test -O3 -std=gnu++20 -fopenmp v4l2-capture-test.cpp -I/usr/include/aarch64-linux-gnu -lm -ldl -lpthread `pkg-config --libs libavutil libavcodec libavformat libavdevice libswscale libv4l2 tbb`
if [[ $? -eq 0 ]]; then
  echo "Successfully built: v4l2-capture-test"
else
  echo "Build failed (v4l2-capture-test)"
  exit 1
fi