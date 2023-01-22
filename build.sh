#/bin/bash

export PATH=/usr/local/cuda-10.2/bin:$PATH
export LD_LIBRARY_PATH=/usr/lib/aarch64-linux-gnu:/usr/local/cuda-10.2/lib64:/usr/local/lib:/usr/local/cuda-10.2/targets/aarch64-linux/lib:$LD_LIBRARY_PATH

#(git clone git://git.ideasonboard.org/uvc-gadget.git && cd uvc-gadget && make && make install)
#if [[ $? -eq 0 ]]; then
#  echo "Successfully built (uvc-gadget)"
#else
#  echo "Build failed (uvc-gadget), perhaps we already built it?"
#fi

#g++ -shared -o ~/projects/v4l2-capture-test/bin/ARM/Debug/libv4l2edid.so -fPIC -O3 -std=gnu++20 -lv4l2 v4l2edid.cpp
#if [[ $? -eq 0 ]]; then
#  echo "Successfully built: libv4l2edid.so"
#  #echo "We need to copy the libv4l2edid.so library file to /usr/local/lib/ and then run ldconfig"
#  sudo cp ~/projects/v4l2-capture-test/bin/ARM/Debug/libv4l2edid.so /usr/local/lib/ && sudo ldconfig
#else
#  echo "Build failed (libv4l2edid.so)"
#  exit 1
#fi

if ! command -v /usr/local/cuda-10.2/bin/nvcc; then
  g++ -o ~/projects/v4l2-capture-test/bin/ARM/Debug/v4l2-capture-test -O3 -std=gnu++20 -fopenmp v4l2-capture-test.cpp -I/usr/include/aarch64-linux-gnu -lm -ldl -lpthread `pkg-config --libs libavutil libavcodec libavformat libavdevice libswscale libv4l2`
else
  /usr/local/cuda-10.2/bin/nvcc -I/usr/local/cuda-10.2/include -I/usr/local/cuda-10.2/samples/common/inc -L/usr/local/cuda-10.2/lib64 -o ~/projects/v4l2-capture-test/bin/ARM/Debug/v4l2-capture-test -O3 v4l2-capture.cu -ldl -lv4l2 -lpthread -lcudart
fi
#apt-get install cuda-toolkit-10-2 cuda-tools-10-2 cuda-compiler-10-2 cuda-samples-10-2 cuda-libraries-10-2 cuda-driver-dev-10-2 libcudnn8-dev libcudnn8 cuda-*dev* mlocate dos2unix
if [[ $? -eq 0 ]]; then
  echo "Successfully built: v4l2-capture-test"
else
  echo "Build failed (v4l2-capture-test)"
  exit 1
fi

# ~/projects/v4l2-capture-test/bin/ARM/Debug/v4l2-capture-test /dev/video1 1280 720 |ffmpeg -hide_banner -loglevel error -f rawvideo -pixel_format yuyv422 -video_size 1280x720 -i pipe:0 -f rawvideo -c:v copy /dev/video2

arm-linux-gnueabihf-gcc -g -I/opt/vc/include -pipe -o v4l2-mmal-uvc.o -c v4l2-mmal-uvc.c && arm-linux-gnueabihf-gcc -o v4l2-mmal-uvc v4l2-mmal-uvc.o -L/opt/vc/lib -lrt -lbcm_host -lvcos -lvchiq_arm -pthread -lmmal_core -lmmal_util -lmmal_vc_client -lvcsm
if [[ $? -eq 0 ]]; then
  echo "Successfully built: v4l2-mmal-uvc"
else
  echo "Build failed (v4l2-mmal-uvc)"
  exit 1
fi

#~/projects/v4l2-capture-test/v4l2-mmal-uvc

#rustc -o ~/projects/v4l2-capture-test/bin/ARM/Debug/v4l2-video-capture-testing-program-porting v4l2-video-capture-testing-program-porting.rs
#if [[ $? -eq 0 ]]; then
#  echo "Successfully built: v4l2-video-capture-testing-program-porting"
#else
#  echo "Build failed (v4l2-video-capture-testing-program-porting)"
#  exit 1
#fi