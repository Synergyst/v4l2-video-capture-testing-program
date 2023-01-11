#/bin/bash

# ~/projects/v4l2-video-capture-testing-program/bin/ARM/Debug/v4l2-video-capture-testing-program | ffmpeg -y -hide_banner -probesize 256M -f rawvideo -pixel_format gray -video_size 640x360 -framerate 10 -thread_queue_size 4096 -i pipe:0 -pixel_format yuyv422 -video_size 640x360 -framerate 10 -thread_queue_size 4096 -i /dev/video2 -filter_complex "[1:v]chromakey=0xffffff:0.01:0.01[ckout];[0:v][ckout]overlay[out]" -map [out] -f rawvideo -pixel_format uyvy422 pipe:1 | ffplay -hide_banner -loglevel error -f rawvideo -pixel_format yuva420p -video_size 640x360 -framerate 10 -i pipe:0
# ~/projects/v4l2-video-capture-testing-program/bin/ARM/Debug/v4l2-video-capture-testing-program | ffmpeg -y -hide_banner -probesize 256M -f rawvideo -pixel_format gray -video_size 640x360 -framerate 10 -thread_queue_size 4096 -i pipe:0 -pixel_format yuyv422 -video_size 640x360 -framerate 10 -thread_queue_size 4096 -i /dev/video2 -shortest -filter_complex "[1:v]colorkey=0x000000:0.01:0.5[ckout];[0:v][ckout]overlay[out]" -map [out] -f rawvideo -pixel_format uyvy422 pipe:1 | ffplay -hide_banner -loglevel error -f rawvideo -pixel_format yuva420p -video_size 640x360 -framerate 10 -i pipe:0
# ~/projects/v4l2-video-capture-testing-program/bin/ARM/Debug/v4l2-video-capture-testing-program | ffmpeg -y -hide_banner -probesize 512M -f rawvideo -pixel_format gray -video_size 640x360 -framerate 10 -thread_queue_size 4096 -i pipe:0 -pixel_format uyvy422 -video_size 1920x1080 -framerate 10 -thread_queue_size 4096 -i /dev/video0 -shortest -filter_complex "[1:v]colorkey=0x000000:0.01:0.5[ckout];[0:v][ckout]overlay[out]" -map [out] -f rawvideo -pix_fmt gray pipe:1 | ffplay -hide_banner -loglevel error -f rawvideo -pixel_format gray -video_size 640x360 -framerate 10 -i pipe:0

mkdir -p ~/projects/v4l2-video-capture-testing-program/bin/ARM/Debug/
g++ -shared -o ~/projects/v4l2-video-capture-testing-program/bin/ARM/Debug/libv4l2edid.so -fPIC -O3 -std=c++20 -lv4l2 v4l2edid.cpp
if [[ $? -eq 0 ]]; then
  echo "Successfully compiled EDID library"
  echo "We need to copy the libv4l2edid.so library file to /usr/local/lib/ and then run ldconfig"
  sudo cp ~/projects/v4l2-video-capture-testing-program/bin/ARM/Debug/libv4l2edid.so /usr/local/lib/ && sudo ldconfig
else
  echo "Build failed (libv4l2edid.so)"
  exit 1
fi
#g++ -shared -o ~/projects/v4l2-video-capture-testing-program/bin/ARM/Debug/libimgproc.so -O3 -std=c++20 -fopenmp imgproc.cpp
#g++ -shared -o ~/projects/v4l2-video-capture-testing-program/bin/ARM/Debug/libimgproc.so -fPIC -O3 -std=c++20 -fopenmp imgproc.cpp
#if [[ $? -eq 0 ]]; then
#  echo "Successfully compiled shared library"
#  echo "We need to copy the libimgproc.so library file to /usr/local/lib/ and then run ldconfig"
#  sudo cp ~/projects/v4l2-video-capture-testing-program/bin/ARM/Debug/libimgproc.so /usr/local/lib/ && sudo ldconfig
#else
#  echo "Build failed (libimgproc.so)"
#  exit 1
#fi
g++ -shared -o ~/projects/v4l2-video-capture-testing-program/bin/ARM/Debug/libv4l2cap.so -fPIC -O3 -std=c++20 -fopenmp v4l2cap.cpp -ldl -lv4l2
if [[ $? -eq 0 ]]; then
  echo "Successfully compiled shared library"
  echo "We need to copy the libv4l2cap.so library file to /usr/local/lib/ and then run ldconfig"
  sudo cp ~/projects/v4l2-video-capture-testing-program/bin/ARM/Debug/libv4l2cap.so /usr/local/lib/ && sudo ldconfig
else
  echo "Build failed (libv4l2cap.so)"
  exit 1
fi
g++ -fPIC -o ~/projects/v4l2-video-capture-testing-program/bin/ARM/Debug/v4l2-video-capture-testing-program -O3 -std=c++20 -fopenmp v4l2-video-capture-testing-program.cpp -ldl -lv4l2cap -lv4l2
if [[ $? -eq 0 ]]; then
  echo "Successfully compiled capture test program"
else
  echo "Build failed (v4l2-video-capture-testing-program)"
  exit 1
fi
rustc -o ~/projects/v4l2-video-capture-testing-program/bin/ARM/Debug/v4l2-video-capture-testing-program-porting v4l2-video-capture-testing-program-porting.rs
if [[ $? -eq 0 ]]; then
  echo "Successfully compiled Rust program (WIP, no functionality yet)"
else
  echo "Build failed (v4l2-video-capture-testing-program-porting)"
  exit 1
fi