#/bin/bash

# ~/projects/v4l2-video-capture-testing-program/bin/ARM/Debug/v4l2-video-capture-testing-program | ffmpeg -y -hide_banner -probesize 256M -f rawvideo -pixel_format gray -video_size 640x360 -framerate 10 -thread_queue_size 4096 -i pipe:0 -pixel_format yuyv422 -video_size 640x360 -framerate 10 -thread_queue_size 4096 -i /dev/video2 -filter_complex "[1:v]chromakey=0xffffff:0.01:0.01[ckout];[0:v][ckout]overlay[out]" -map [out] -f rawvideo -pixel_format uyvy422 pipe:1 | ffplay -hide_banner -loglevel error -f rawvideo -pixel_format yuva420p -video_size 640x360 -framerate 10 -i pipe:0
# ~/projects/v4l2-video-capture-testing-program/bin/ARM/Debug/v4l2-video-capture-testing-program | ffmpeg -y -hide_banner -probesize 256M -f rawvideo -pixel_format gray -video_size 640x360 -framerate 10 -thread_queue_size 4096 -i pipe:0 -pixel_format yuyv422 -video_size 640x360 -framerate 10 -thread_queue_size 4096 -i /dev/video2 -shortest -filter_complex "[1:v]colorkey=0x000000:0.01:0.5[ckout];[0:v][ckout]overlay[out]" -map [out] -f rawvideo -pixel_format uyvy422 pipe:1 | ffplay -hide_banner -loglevel error -f rawvideo -pixel_format yuva420p -video_size 640x360 -framerate 10 -i pipe:0
# ~/projects/v4l2-video-capture-testing-program/bin/ARM/Debug/v4l2-video-capture-testing-program | ffmpeg -y -hide_banner -probesize 512M -f rawvideo -pixel_format gray -video_size 640x360 -framerate 10 -thread_queue_size 4096 -i pipe:0 -pixel_format uyvy422 -video_size 1920x1080 -framerate 10 -thread_queue_size 4096 -i /dev/video0 -shortest -filter_complex "[1:v]colorkey=0x000000:0.01:0.5[ckout];[0:v][ckout]overlay[out]" -map [out] -f rawvideo -pix_fmt gray pipe:1 | ffplay -hide_banner -loglevel error -f rawvideo -pixel_format gray -video_size 640x360 -framerate 10 -i pipe:0

export PATH=/usr/local/cuda-10.2/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda-10.2/lib64:$LD_LIBRARY_PATH
#echo $LD_LIBRARY_PATH
#export LD_LIBRARY_PATH=/usr/local/lib/:/usr/local/cuda-10.2/lib64:$LD_LIBRARY_PATH
#echo $LD_LIBRARY_PATH
#g++ -shared -o ~/projects/v4l2-capture-test/bin/ARM/Debug/libv4l2edid.so -fPIC -O3 -std=gnu++20 -lv4l2 v4l2edid.cpp
#if [[ $? -eq 0 ]]; then
#  echo "Successfully built: libv4l2edid.so"
#  #echo "We need to copy the libv4l2edid.so library file to /usr/local/lib/ and then run ldconfig"
#  sudo cp ~/projects/v4l2-capture-test/bin/ARM/Debug/libv4l2edid.so /usr/local/lib/ && sudo ldconfig
#else
#  echo "Build failed (libv4l2edid.so)"
#  exit 1
#fi
g++ -o ~/projects/v4l2-capture-test/bin/ARM/Debug/v4l2-capture-test -O3 -std=gnu++20 -fopenmp v4l2-capture-test.cpp -ldl -lv4l2 -lpthread
#g++-8 -I/usr/local/cuda-10.2/include -I/usr/local/cuda-10.2/samples/common/inc -L/usr/local/cuda-10.2/lib64 -o ~/projects/v4l2-capture-test/bin/ARM/Debug/v4l2-capture-test -O3 -std=gnu++2a -fopenmp v4l2-capture-test.cpp -ldl -lv4l2 -lpthread -lcudart
#nvcc -I/usr/local/cuda-10.2/include -I/usr/local/cuda-10.2/samples/common/inc -L/usr/local/cuda-10.2/lib64 -o ~/projects/v4l2-capture-test/bin/ARM/Debug/v4l2-capture-test -O3 v4l2-capture.cu -ldl -lv4l2 -lpthread -lcudart
#make
if [[ $? -eq 0 ]]; then
  echo "Successfully built: v4l2-capture-test"
else
  echo "Build failed (v4l2-capture-test)"
  exit 1
fi
#~/projects/v4l2-capture-test/bin/ARM/Debug/v4l2-capture-test /dev/video0 640 360 | ffplay -hide_banner -loglevel error -f rawvideo -video_size 640x360 -pixel_format gray -i pipe:0
#rustc -o ~/projects/v4l2-capture-test/bin/ARM/Debug/v4l2-video-capture-testing-program-porting v4l2-video-capture-testing-program-porting.rs
#if [[ $? -eq 0 ]]; then
#  echo "Successfully built: v4l2-video-capture-testing-program-porting"
#else
#  echo "Build failed (v4l2-video-capture-testing-program-porting)"
#  exit 1
#fi