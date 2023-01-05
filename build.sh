#/bin/bash

# ~/projects/v4l2-video-capture-testing-program/bin/ARM/Debug/v4l2-video-capture-testing-program | ffmpeg -y -hide_banner -probesize 256M -f rawvideo -pixel_format gray -video_size 640x360 -framerate 10 -thread_queue_size 4096 -i pipe:0 -pixel_format yuyv422 -video_size 640x360 -framerate 10 -thread_queue_size 4096 -i /dev/video2 -filter_complex "[1:v]chromakey=0xffffff:0.01:0.01[ckout];[0:v][ckout]overlay[out]" -map [out] -f rawvideo -pixel_format uyvy422 pipe:1 | ffplay -hide_banner -loglevel error -f rawvideo -pixel_format yuva420p -video_size 640x360 -framerate 10 -i pipe:0
# ~/projects/v4l2-video-capture-testing-program/bin/ARM/Debug/v4l2-video-capture-testing-program | ffmpeg -y -hide_banner -probesize 256M -f rawvideo -pixel_format gray -video_size 640x360 -framerate 10 -thread_queue_size 4096 -i pipe:0 -pixel_format yuyv422 -video_size 640x360 -framerate 10 -thread_queue_size 4096 -i /dev/video2 -shortest -filter_complex "[1:v]colorkey=0x000000:0.01:0.5[ckout];[0:v][ckout]overlay[out]" -map [out] -f rawvideo -pixel_format uyvy422 pipe:1 | ffplay -hide_banner -loglevel error -f rawvideo -pixel_format yuva420p -video_size 640x360 -framerate 10 -i pipe:0
# ~/projects/v4l2-video-capture-testing-program/bin/ARM/Debug/v4l2-video-capture-testing-program | ffmpeg -y -hide_banner -probesize 512M -f rawvideo -pixel_format gray -video_size 640x360 -framerate 10 -thread_queue_size 4096 -i pipe:0 -pixel_format uyvy422 -video_size 1920x1080 -framerate 10 -thread_queue_size 4096 -i /dev/video0 -shortest -filter_complex "[1:v]colorkey=0x000000:0.01:0.5[ckout];[0:v][ckout]overlay[out]" -map [out] -f rawvideo -pix_fmt gray pipe:1 | ffplay -hide_banner -loglevel error -f rawvideo -pixel_format gray -video_size 640x360 -framerate 10 -i pipe:0

mkdir -p ~/projects/v4l2-video-capture-testing-program/bin/ARM/Debug/
g++ -o ~/projects/v4l2-video-capture-testing-program/bin/ARM/Debug/v4l2-video-capture-testing-program -O3 -std=c++20 -lv4l2 -fopenmp v4l2-video-capture-testing-program.cpp v4l2cap.cpp v4l2capalt.cpp
if [[ $? -eq 0 ]]; then
  echo "Success"
  exit 0
else
  echo "Build failed"
  exit 1
fi